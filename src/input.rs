//! Zero-dependency Linux evdev input manager for:
//! - USB-C Host RC radios (EdgeTX/OpenTX, RadioMaster Pocket, ELRS USB Joystick)
//! - Bluetooth controllers (Xbox, PlayStation, Switch Pro, 8BitDo, ELRS BLE Joystick)
//! - CardputerZero built-in TCA8418 keyboard (`/dev/input/by-path/platform-3f804000.i2c-event`)
//!
//! Features:
//! - Scored evdev controller discovery (skips touchpads, IMUs, and non-gamepad nodes)
//! - Interactive 4-step "Detect Sticks" calibration wizard (`C`)
//! - Interactive 3-step "Bind AUX Switches/Buttons" wizard (`B`) for Arm/Disarm, Flight Mode, and Turtle Mode
//!   supporting BOTH EV_ABS switch channels (EdgeTX CH5-CH8) AND EV_KEY buttons!
//! - Fn-free menu navigation (F/X/Z/C, Arrows, Right-Stick)
//! - CardputerZero short-Esc / long-Esc exit semantics

use crate::config::{AuxBinding, Config, FlightMode, StickMode, ThrottleMode};
use crate::hidraw::{probe_hidraw_sidecar, HidrawSidecar};
use crate::imu::ImuSensor;
use crate::linux_sys::*;
use crate::math::clampf;
use crate::physics::StickInput;
use std::ffi::CString;
use std::fs;
use std::path::Path;
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

#[derive(Copy, Clone, Debug, Default)]
pub struct AxisCal {
    pub present: bool,
    pub min: i32,
    pub max: i32,
    pub value: i32,
}

impl AxisCal {
    /// Returns normalized value in -1.0 ..= +1.0 with 4% center deadzone.
    pub fn norm(&self) -> f32 {
        if !self.present || self.max <= self.min {
            return 0.0;
        }
        let span = (self.max - self.min) as f32;
        let v = ((self.value - self.min) as f32 / span) * 2.0 - 1.0;
        let v = clampf(v, -1.0, 1.0);
        if v.abs() < 0.04 { 0.0 } else { v }
    }

    /// Raw normalized value without deadzone (used for calibration & switch detection).
    pub fn raw_norm(&self) -> f32 {
        if !self.present || self.max <= self.min {
            return 0.0;
        }
        let span = (self.max - self.min) as f32;
        clampf(((self.value - self.min) as f32 / span) * 2.0 - 1.0, -1.0, 1.0)
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ControllerKind {
    #[allow(dead_code)]
    None,
    RcRadio, // EdgeTX / OpenTX / ExpressLRS
    Gamepad, // Xbox / PS / Switch / 8BitDo / Generic
}

/// Active flight control source driving the quadcopter and OSD stick layout.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FlightControlSource {
    /// Physical USB-C or Bluetooth RC radio / gamepad.
    Controller,
    /// CardputerZero onboard BMI270 IMU simplified flight (no controller connected).
    ImuSimplified,
    /// Neither controller nor IMU detected (e.g. CardputerZero Lite without controller).
    None,
}

const MAX_KEY_CODE: usize = 768; // Covers keyboard, BTN_JOYSTICK (0x120..), BTN_GAMEPAD (0x130..), BTN_TRIGGER_HAPPY (0x2c0..)

#[inline]
fn set_key_bit(keys_down: &mut [u64; MAX_KEY_CODE / 64], code: u16, down: bool) {
    let idx = code as usize;
    if idx < MAX_KEY_CODE {
        if down {
            keys_down[idx / 64] |= 1u64 << (idx % 64);
        } else {
            keys_down[idx / 64] &= !(1u64 << (idx % 64));
        }
    }
}

#[inline]
fn unpack_key_bitmap(key_bits: &[u8; MAX_KEY_CODE / 8], keys_down: &mut [u64; MAX_KEY_CODE / 64]) {
    for word in 0..(MAX_KEY_CODE / 64) {
        let base = word * 8;
        keys_down[word] = u64::from_le_bytes([
            key_bits[base],
            key_bits[base + 1],
            key_bits[base + 2],
            key_bits[base + 3],
            key_bits[base + 4],
            key_bits[base + 5],
            key_bits[base + 6],
            key_bits[base + 7],
        ]);
    }
}

/// Resynchronize all evdev axis values and button states directly from the kernel via
/// `EVIOCGABS` and `EVIOCGKEY` (called on initial open and after any `SYN_DROPPED`).
/// Skips axes sourced from a `/dev/hidraw*` sidecar (`hidraw_axis_mask`) so recovered
/// channels like ELRS BLE CH8 (`ABS_RUDDER` / `A7+`) are never overwritten with zeros.
fn resync_device_state(
    fd: i32,
    axes: &mut [AxisCal; ABS_MAX + 1],
    keys_down: &mut [u64; MAX_KEY_CODE / 64],
    hidraw_axis_mask: u64,
) {
    for code in 0..=ABS_MAX {
        if (hidraw_axis_mask & (1u64 << code)) != 0 {
            continue;
        }
        if axes[code].present {
            let mut info = InputAbsInfo::default();
            if unsafe { ioctl(fd, eviocgabs(code as u32), &mut info) } >= 0
                && info.maximum > info.minimum
            {
                axes[code].value = info.value;
                axes[code].min = info.minimum;
                axes[code].max = info.maximum;
            }
        }
    }
    let mut key_bits = [0u8; MAX_KEY_CODE / 8];
    if unsafe { ioctl(fd, eviocgkey(key_bits.len()), key_bits.as_mut_ptr()) } >= 0 {
        unpack_key_bitmap(&key_bits, keys_down);
    }
}

/// Best-effort one-shot helper to stop lingering BlueZ discovery scans when a Bluetooth
/// controller connects. On Raspberry Pi CM0 / CardputerZero's shared 2.4 GHz radio, an
/// active BLE scan left on after pairing periodically stalls BLE HID packets for 100–500 ms.
fn stop_bt_discovery_async() {
    let _ = thread::Builder::new()
        .name("bt-scan-off".to_string())
        .spawn(|| {
            let _ = Command::new("bluetoothctl")
                .args(["scan", "off"])
                .stdin(Stdio::null())
                .stdout(Stdio::null())
                .stderr(Stdio::null())
                .status();
        });
}

struct GamepadLiveState {
    axes: [AxisCal; ABS_MAX + 1],
    keys_down: [u64; MAX_KEY_CODE / 64],
    pressed_buttons: Vec<u16>,
    hat_up: bool,
    hat_down: bool,
    hat_left: bool,
    hat_right: bool,
    disconnected: bool,
}

/// Dedicated zero-contention background evdev + optional hidraw drain loop.
///
/// Sits on `poll(2)` (0% CPU when idle, <0.1 ms wakeup when a Bluetooth/USB HID packet
/// arrives) and drains up to 256 evdev events per `read(2)` syscall plus any sibling
/// `/dev/hidraw*` sidecar packets into thread-local state BEFORE acquiring the shared
/// mutex once per wakeup. This guarantees:
/// - The 60 Hz render thread never blocks waiting on `read(2)` or `ioctl(2)` syscalls.
/// - Bursty BLE packets always collapse straight to the newest stick/switch sample.
/// - `SYN_DROPPED` discards partial frames until `SYN_REPORT` and resyncs lock-free.
/// - ExpressLRS BLE CH8 (duplicate `Slider` 0x36) stays live on `ABS_RUDDER` (`A7+`).
fn gamepad_reader_thread(
    fd: i32,
    mut hidraw: Option<HidrawSidecar>,
    mut local_axes: [AxisCal; ABS_MAX + 1],
    mut local_keys_down: [u64; MAX_KEY_CODE / 64],
    shared: Arc<Mutex<GamepadLiveState>>,
    stop: Arc<AtomicBool>,
) {
    let mut ev_buf = [InputEvent::default(); 256];
    let ev_size = std::mem::size_of::<InputEvent>();
    let buf_bytes = ev_buf.len() * ev_size;
    let mut raw_buf = [0u8; 64];

    let mut local_pressed: Vec<u16> = Vec::with_capacity(16);
    let mut dropping = false;

    while !stop.load(Ordering::Relaxed) {
        let mut pfds = [
            PollFd {
                fd,
                events: POLLIN,
                revents: 0,
            },
            PollFd {
                fd: hidraw.as_ref().map(|h| h.fd).unwrap_or(-1),
                events: POLLIN,
                revents: 0,
            },
        ];
        let nfds = if hidraw.is_some() { 2 } else { 1 };
        let pr = unsafe { poll(pfds.as_mut_ptr(), nfds, 15) };
        if pr < 0 {
            let err = std::io::Error::last_os_error().raw_os_error().unwrap_or(0);
            if err == 4 {
                // EINTR
                continue;
            }
            if let Ok(mut st) = shared.lock() {
                st.disconnected = true;
            }
            break;
        }
        if pr == 0 {
            continue;
        }

        if (pfds[0].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0
            && (pfds[0].revents & POLLIN) == 0
        {
            if let Ok(mut st) = shared.lock() {
                st.disconnected = true;
            }
            break;
        }

        let hidraw_mask = hidraw.as_ref().map(|h| h.axis_mask).unwrap_or(0);
        let mut disconnected = false;
        let mut dirty = false;
        let mut hat_up = false;
        let mut hat_down = false;
        let mut hat_left = false;
        let mut hat_right = false;
        local_pressed.clear();

        // 1. Drain ALL queued evdev events into thread-local state (zero mutex contention!)
        if (pfds[0].revents & POLLIN) != 0 {
            loop {
                let n =
                    unsafe { read(fd, ev_buf.as_mut_ptr() as *mut std::ffi::c_void, buf_bytes) };
                if n > 0 {
                    let count = (n as usize) / ev_size;
                    for ev in &ev_buf[..count] {
                        if ev.type_ == EV_SYN && ev.code == SYN_DROPPED {
                            dropping = true;
                            continue;
                        }
                        if dropping {
                            if ev.type_ == EV_SYN && ev.code == SYN_REPORT {
                                dropping = false;
                                resync_device_state(
                                    fd,
                                    &mut local_axes,
                                    &mut local_keys_down,
                                    hidraw_mask,
                                );
                                dirty = true;
                            }
                            continue;
                        }
                        match ev.type_ {
                            EV_ABS => {
                                let c = ev.code as usize;
                                if c <= ABS_MAX && (hidraw_mask & (1u64 << c)) == 0 {
                                    local_axes[c].value = ev.value;
                                    if !local_axes[c].present {
                                        local_axes[c].present = true;
                                        local_axes[c].min = -32768;
                                        local_axes[c].max = 32767;
                                    }
                                    dirty = true;
                                }
                                if ev.code == ABS_HAT0Y {
                                    if ev.value < 0 {
                                        hat_up = true;
                                        dirty = true;
                                    }
                                    if ev.value > 0 {
                                        hat_down = true;
                                        dirty = true;
                                    }
                                } else if ev.code == ABS_HAT0X {
                                    if ev.value < 0 {
                                        hat_left = true;
                                        dirty = true;
                                    }
                                    if ev.value > 0 {
                                        hat_right = true;
                                        dirty = true;
                                    }
                                }
                            }
                            EV_KEY => {
                                let pressed = ev.value == 1;
                                let held = ev.value != 0;
                                set_key_bit(&mut local_keys_down, ev.code, held);
                                if pressed && local_pressed.len() < 32 {
                                    local_pressed.push(ev.code);
                                }
                                dirty = true;
                            }
                            _ => {}
                        }
                    }
                    if count < ev_buf.len() {
                        break;
                    }
                } else if n == 0 {
                    disconnected = true;
                    break;
                } else {
                    let err = std::io::Error::last_os_error().raw_os_error().unwrap_or(0);
                    if err == 11 || err == 4 {
                        // EAGAIN / EINTR -> fully drained
                        break;
                    }
                    // ENODEV (19), EIO (5), EBADF (9) -> device disconnected
                    disconnected = true;
                    break;
                }
            }

            if dropping && !disconnected {
                dropping = false;
                resync_device_state(fd, &mut local_axes, &mut local_keys_down, hidraw_mask);
                dirty = true;
            }
        }

        // 2. Drain ALL queued sibling /dev/hidraw* packets, keeping the very latest sample
        if let Some(ref sidecar) = hidraw {
            let mut drop_hidraw = false;
            if (pfds[1].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0
                && (pfds[1].revents & POLLIN) == 0
            {
                drop_hidraw = true;
            } else if (pfds[1].revents & POLLIN) != 0 {
                loop {
                    let n = unsafe {
                        read(
                            sidecar.fd,
                            raw_buf.as_mut_ptr() as *mut std::ffi::c_void,
                            raw_buf.len(),
                        )
                    };
                    if n > 0 {
                        let len = (n as usize).min(raw_buf.len());
                        if sidecar.apply_report(&raw_buf[..len], &mut local_axes) {
                            dirty = true;
                        }
                    } else if n == 0 {
                        drop_hidraw = true;
                        break;
                    } else {
                        let err = std::io::Error::last_os_error().raw_os_error().unwrap_or(0);
                        if err == 11 || err == 4 {
                            // EAGAIN / EINTR -> fully drained
                            break;
                        }
                        drop_hidraw = true;
                        break;
                    }
                }
            }
            if drop_hidraw {
                if let Some(old) = hidraw.take() {
                    unsafe { close(old.fd) };
                }
            }
        }

        // 3. Publish latest drained snapshot in a single sub-microsecond critical section
        if dirty || disconnected {
            if let Ok(mut st) = shared.lock() {
                if dirty {
                    st.axes = local_axes;
                    st.keys_down = local_keys_down;
                    for &code in &local_pressed {
                        if st.pressed_buttons.len() < 32 {
                            st.pressed_buttons.push(code);
                        }
                    }
                    st.hat_up |= hat_up;
                    st.hat_down |= hat_down;
                    st.hat_left |= hat_left;
                    st.hat_right |= hat_right;
                }
                if disconnected {
                    st.disconnected = true;
                }
            }
        }

        if disconnected {
            break;
        }
    }

    if let Some(sidecar) = hidraw {
        unsafe { close(sidecar.fd) };
    }
    unsafe { close(fd) };
}

pub struct GamepadDevice {
    shared: Arc<Mutex<GamepadLiveState>>,
    stop: Arc<AtomicBool>,
    reader: Option<JoinHandle<()>>,
    pub path: String,
    pub name: String,
    pub kind: ControllerKind,
    #[allow(dead_code)]
    pub bustype: u16,
    pub axes: [AxisCal; ABS_MAX + 1],
    /// Bitset of currently held EV_KEY button codes (0..768)
    pub keys_down: [u64; MAX_KEY_CODE / 64],
    /// All EV_KEY button codes pressed during the current frame (edge-triggered)
    pub frame_btns_pressed: Vec<u16>,
    /// Most recent EV_KEY button pressed this frame (edge-triggered, used by AUX wizard)
    pub frame_btn_pressed: Option<u16>,
    /// Last button code ever pressed (for live diagnostics on Controller screen)
    pub last_btn_seen: Option<u16>,
}

impl GamepadDevice {
    fn spawn(
        fd: i32,
        path: String,
        name: String,
        kind: ControllerKind,
        bustype: u16,
        mut axes: [AxisCal; ABS_MAX + 1],
    ) -> Self {
        // Request monotonic timestamps on evdev (best-effort; ignored on older kernels)
        let clk = CLOCK_MONOTONIC;
        let _ = unsafe { ioctl(fd, eviocsclockid(), &clk) };

        // Probe sibling /dev/hidraw* to recover duplicate/dropped HID axes (e.g. ELRS BLE CH8 -> A7+)
        let hidraw = probe_hidraw_sidecar(&path, &axes);
        let hidraw_mask = if let Some(ref sidecar) = hidraw {
            for ax in &sidecar.axes {
                axes[ax.evdev_code] = AxisCal {
                    present: true,
                    min: ax.logical_min,
                    max: ax.logical_max,
                    value: ax.logical_min + (ax.logical_max - ax.logical_min) / 2,
                };
            }
            sidecar.axis_mask
        } else {
            0
        };

        let mut keys_down = [0u64; MAX_KEY_CODE / 64];
        resync_device_state(fd, &mut axes, &mut keys_down, hidraw_mask);

        if bustype == BUS_BLUETOOTH {
            stop_bt_discovery_async();
        }

        let shared = Arc::new(Mutex::new(GamepadLiveState {
            axes,
            keys_down,
            pressed_buttons: Vec::with_capacity(8),
            hat_up: false,
            hat_down: false,
            hat_left: false,
            hat_right: false,
            disconnected: false,
        }));
        let stop = Arc::new(AtomicBool::new(false));
        let bg_shared = Arc::clone(&shared);
        let bg_stop = Arc::clone(&stop);
        let reader = thread::spawn(move || {
            gamepad_reader_thread(fd, hidraw, axes, keys_down, bg_shared, bg_stop)
        });

        Self {
            shared,
            stop,
            reader: Some(reader),
            path,
            name,
            kind,
            bustype,
            axes,
            keys_down,
            frame_btns_pressed: Vec::with_capacity(8),
            frame_btn_pressed: None,
            last_btn_seen: None,
        }
    }

    #[inline]
    pub fn is_key_down(&self, code: u16) -> bool {
        let idx = code as usize;
        if idx < MAX_KEY_CODE {
            (self.keys_down[idx / 64] & (1u64 << (idx % 64))) != 0
        } else {
            false
        }
    }

    #[inline]
    pub fn was_btn_pressed(&self, code: u16) -> bool {
        self.frame_btns_pressed.contains(&code)
    }
}

impl Drop for GamepadDevice {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        if let Some(handle) = self.reader.take() {
            let _ = handle.join();
        }
    }
}

#[derive(Copy, Clone, Debug, Default)]
pub struct MenuActions {
    pub up: bool,
    pub down: bool,
    pub left: bool,
    pub right: bool,
    pub select: bool,
    pub back: bool,
    pub tab_prev: bool,
    pub tab_next: bool,
    pub respawn: bool,
    pub calibrate: bool,
    pub recenter_imu: bool,
    pub bind_aux: bool,
    pub next_device: bool,
    pub exit_app: bool,
}

/// Live radio AUX switch and button inputs evaluated every frame during flight.
#[derive(Copy, Clone, Debug, Default)]
pub struct AuxInputs {
    /// True only when `cfg.aux_arm` is bound to an active axis/button on a connected controller.
    pub arm_aux_connected: bool,
    /// Latched ARM switch state (Some(true)=ARM, Some(false)=DISARM) if bound to an EV_ABS switch.
    pub arm_switch: Option<bool>,
    /// Edge-triggered toggle for ARM (bound EV_KEY button or keyboard Tab).
    pub arm_toggle: bool,
    /// Latched FLIGHT MODE switch state (supports 2-pos Acro/Angle and 3-pos Acro/Horizon/Angle).
    pub mode_switch: Option<FlightMode>,
    /// Edge-triggered toggle for Flight Mode (bound EV_KEY button or keyboard M).
    pub mode_toggle: bool,
    /// Latched TURTLE switch state (Some(true)=TURTLE ON) if bound to an EV_ABS switch.
    pub turtle_switch: Option<bool>,
    /// Edge-triggered toggle for Turtle Mode (bound EV_KEY button or keyboard T).
    pub turtle_toggle: bool,
}

/// Interactive 4-step stick calibration wizard.
#[derive(Clone, Debug)]
pub struct CalibWizard {
    /// 0 = Throttle UP, 1 = Yaw RIGHT, 2 = Pitch FORWARD, 3 = Roll RIGHT
    pub step: usize,
    pub baseline: [f32; ABS_MAX + 1],
    pub used_axes: [Option<u16>; 4],
    pub step_started: Instant,
    pub message: String,
}

impl CalibWizard {
    pub fn step_prompt(&self) -> &'static str {
        match self.step {
            0 => "1/4: MOVE THROTTLE UP",
            1 => "2/4: MOVE YAW RIGHT",
            2 => "3/4: MOVE PITCH FORWARD",
            _ => "4/4: MOVE ROLL RIGHT",
        }
    }
}

/// Interactive 3-step AUX switch/button binding wizard (Arm, Mode, Turtle).
#[derive(Clone, Debug)]
pub struct AuxWizard {
    /// 0 = ARM/DISARM, 1 = FLIGHT MODE (Acro/Angle), 2 = TURTLE MODE
    pub step: usize,
    pub baseline: [f32; ABS_MAX + 1],
    pub stick_axes: [usize; 4],
    pub step_started: Instant,
    pub message: String,
}

impl AuxWizard {
    pub fn step_prompt(&self) -> &'static str {
        match self.step {
            0 => "1/3: FLIP ARM SWITCH OR BTN",
            1 => "2/3: FLIP MODE SWITCH OR BTN",
            _ => "3/3: FLIP TURTLE SWITCH/BTN",
        }
    }
}

fn is_hat_axis(code: usize) -> bool {
    code == ABS_HAT0X as usize || code == ABS_HAT0Y as usize
}

#[inline]
fn set_dir_bit(mask: &mut u8, bit: u8, held: bool) {
    if held {
        *mask |= 1u8 << bit;
    } else {
        *mask &= !(1u8 << bit);
    }
}

pub struct InputManager {
    pub gamepad: Option<GamepadDevice>,
    pub imu: ImuSensor,
    keyboards: Vec<(i32, String)>,
    last_scan: Instant,
    device_skip_idx: usize,
    pub candidate_count: usize,
    pub calib: Option<CalibWizard>,
    pub aux_wiz: Option<AuxWizard>,

    // Keyboard flight state:
    // Left gimbal = CardputerZero arrow keys (F/X/Z/C, Arrows, ;/./,//, WASD) -> Throttle / Yaw
    kb_l_up: u8,
    kb_l_down: u8,
    kb_l_left: u8,
    kb_l_right: u8,
    // Right gimbal = K (pitch fwd), N (pitch back), B (roll left), M (roll right)
    kb_r_up: bool,
    kb_r_down: bool,
    kb_r_left: bool,
    kb_r_right: bool,
    kb_throttle: f32,

    // Keyboard edge-triggered AUX toggles
    kb_arm_toggle: bool,
    kb_mode_toggle: bool,
    kb_turtle_toggle: bool,

    // CardputerZero Esc short/long press tracking
    esc_down_at: Option<Instant>,
    esc_long_fired: bool,

    // Edge-triggered menu actions & stick nav latch
    pending: MenuActions,
    stick_nav_latched_x: bool,
    stick_nav_latched_y: bool,
}

impl InputManager {
    pub fn new() -> Self {
        let imu = ImuSensor::probe_default();
        let mut mgr = Self {
            gamepad: None,
            imu,
            keyboards: Vec::new(),
            last_scan: Instant::now() - Duration::from_secs(10),
            device_skip_idx: 0,
            candidate_count: 0,
            calib: None,
            aux_wiz: None,
            kb_l_up: 0,
            kb_l_down: 0,
            kb_l_left: 0,
            kb_l_right: 0,
            kb_r_up: false,
            kb_r_down: false,
            kb_r_left: false,
            kb_r_right: false,
            kb_throttle: 0.0,
            kb_arm_toggle: false,
            kb_mode_toggle: false,
            kb_turtle_toggle: false,
            esc_down_at: None,
            esc_long_fired: false,
            pending: MenuActions::default(),
            stick_nav_latched_x: false,
            stick_nav_latched_y: false,
        };
        mgr.scan_devices();
        mgr.imu.set_active(mgr.gamepad.is_none());
        mgr
    }

    /// Construct a deterministic mock ELRS radio for `--screenshot` / `--preview` UI renders.
    pub fn demo_radio_for_preview() -> Self {
        let mut mgr = Self::new();
        let mut axes = [AxisCal::default(); ABS_MAX + 1];
        for &(code, val) in &[
            (ABS_X as usize, 1180),      // Roll slightly right
            (ABS_Y as usize, 1260),      // Pitch slightly forward
            (ABS_Z as usize, 980),       // Throttle ~48%
            (ABS_RX as usize, 1024),     // Yaw centered
            (ABS_RY as usize, 2047),     // CH5 ARM switch
            (ABS_RZ as usize, 1024),     // CH6 MODE switch
            (ABS_RUDDER as usize, 0),    // CH8 TURTLE switch
        ] {
            axes[code] = AxisCal {
                present: true,
                min: 0,
                max: 2047,
                value: val,
            };
        }
        mgr.gamepad = Some(GamepadDevice {
            shared: Arc::new(Mutex::new(GamepadLiveState {
                axes,
                keys_down: [0u64; MAX_KEY_CODE / 64],
                pressed_buttons: Vec::new(),
                hat_up: false,
                hat_down: false,
                hat_left: false,
                hat_right: false,
                disconnected: false,
            })),
            stop: Arc::new(AtomicBool::new(true)),
            reader: None,
            path: "/dev/input/event0".to_string(),
            name: "RadioMaster Pocket ELRS".to_string(),
            kind: ControllerKind::RcRadio,
            bustype: BUS_BLUETOOTH,
            axes,
            keys_down: [0u64; MAX_KEY_CODE / 64],
            frame_btns_pressed: Vec::new(),
            frame_btn_pressed: None,
            last_btn_seen: None,
        });
        mgr.candidate_count = 1;
        mgr
    }

    #[inline]
    pub fn has_controller(&self) -> bool {
        self.gamepad.is_some()
    }

    #[inline]
    #[allow(dead_code)]
    pub fn has_imu(&self) -> bool {
        self.imu.is_available()
    }

    #[inline]
    pub fn is_cardputer_lite(&self) -> bool {
        self.imu.is_lite
    }

    #[inline]
    pub fn control_source(&self) -> FlightControlSource {
        if self.gamepad.is_some() {
            FlightControlSource::Controller
        } else if self.imu.is_available() {
            FlightControlSource::ImuSimplified
        } else {
            FlightControlSource::None
        }
    }

    /// True on launch when no controller is connected and no usable IMU is available
    /// (or the device explicitly identifies as CardputerZero Lite).
    #[inline]
    pub fn needs_controller_warning(&self) -> bool {
        self.gamepad.is_none() && (!self.imu.is_available() || self.imu.is_lite)
    }

    /// Capture current handheld pose as neutral level for IMU simplified flight.
    pub fn recenter_imu(&mut self) {
        self.imu.recenter();
    }

    /// Vertical climb/descend command in IMU simplified mode: `-1.0` descend, `0.0` auto-hover, `+1.0` climb.
    /// Driven by Left Gimbal Up/Down (CardputerZero F/X, Arrows Up/Down, W/S, ;/.).
    #[inline]
    pub fn imu_climb_cmd(&self) -> f32 {
        let up = self.kb_l_up != 0;
        let down = self.kb_l_down != 0;
        (up as i32 - down as i32) as f32
    }

    /// Reset software keyboard throttle to 0 (e.g. on respawn).
    pub fn reset_kb_throttle(&mut self) {
        self.kb_throttle = 0.0;
    }

    pub fn controller_name(&self) -> String {
        match &self.gamepad {
            Some(g) => Self::friendly_name(&g.name),
            None if self.imu.is_available() => "CardputerZero IMU".to_string(),
            None if self.imu.is_lite => "Controller Required!".to_string(),
            None => "No BT/USB Controller".to_string(),
        }
    }

    /// Shorten verbose USB/BT HID descriptor names so they fit cleanly on the 1.9" screen.
    fn friendly_name(raw: &str) -> String {
        let lower = raw.to_ascii_lowercase();
        let is_edgetx_family = lower.contains("opentx") || lower.contains("edgetx") || lower.contains("radiomaster");

        let mut s = raw.trim().to_string();
        for prefix in ["OpenTX ", "EdgeTX ", "Microsoft ", "Sony Interactive Entertainment "] {
            if let Some(rest) = s.strip_prefix(prefix) {
                s = rest.to_string();
            }
        }
        s = s.replace("RadioMaster", "RM");
        s = s.replace("Radiomaster", "RM");
        for suffix in [" Joystick", " Gamepad", " Controller"] {
            if s.len() > 16 {
                if let Some(trimmed) = s.strip_suffix(suffix) {
                    s = trimmed.to_string();
                }
            }
        }
        if is_edgetx_family && !s.to_ascii_lowercase().contains("edgetx") {
            format!("{} (EdgeTX)", s.trim())
        } else {
            s.trim().to_string()
        }
    }

    #[allow(dead_code)]
    pub fn controller_kind(&self) -> ControllerKind {
        match &self.gamepad {
            Some(g) => g.kind,
            None => ControllerKind::None,
        }
    }

    /// Cycle to the next detected evdev controller.
    pub fn cycle_device(&mut self) {
        self.device_skip_idx = self.device_skip_idx.wrapping_add(1);
        self.gamepad = None;
        self.scan_devices();
    }

    /// Start the interactive 4-step stick calibration wizard.
    pub fn start_calibration(&mut self) {
        let Some(ref gp) = self.gamepad else { return };
        self.aux_wiz = None;
        let mut baseline = [0.0f32; ABS_MAX + 1];
        for i in 0..=ABS_MAX {
            baseline[i] = gp.axes[i].raw_norm();
        }
        self.calib = Some(CalibWizard {
            step: 0,
            baseline,
            used_axes: [None; 4],
            step_started: Instant::now(),
            message: "Hold sticks still, then push target axis!".to_string(),
        });
    }

    /// Start the interactive 3-step AUX switch/button binding wizard (Arm, Mode, Turtle).
    pub fn start_aux_wizard(&mut self, cfg: &Config) {
        let Some(ref gp) = self.gamepad else { return };
        self.calib = None;
        let (auto_codes, _) = Self::resolve_auto_axes(gp);
        let stick_axes = [
            cfg.axis_thr.map(|c| c as usize).unwrap_or(auto_codes[0]),
            cfg.axis_yaw.map(|c| c as usize).unwrap_or(auto_codes[1]),
            cfg.axis_pit.map(|c| c as usize).unwrap_or(auto_codes[2]),
            cfg.axis_rol.map(|c| c as usize).unwrap_or(auto_codes[3]),
        ];
        let mut baseline = [0.0f32; ABS_MAX + 1];
        for i in 0..=ABS_MAX {
            baseline[i] = gp.axes[i].raw_norm();
        }
        self.aux_wiz = Some(AuxWizard {
            step: 0,
            baseline,
            stick_axes,
            step_started: Instant::now(),
            message: "Flip switch to ON, press btn, or Enter=Skip".to_string(),
        });
    }

    /// Update 4-step stick calibration wizard.
    pub fn update_calibration(&mut self, cfg: &mut Config) -> bool {
        let Some(ref mut wiz) = self.calib else { return false };
        let Some(ref gp) = self.gamepad else {
            self.calib = None;
            return false;
        };

        if wiz.step_started.elapsed() < Duration::from_millis(450) {
            for i in 0..=ABS_MAX {
                wiz.baseline[i] = gp.axes[i].raw_norm();
            }
            return false;
        }

        let mut best_code: Option<usize> = None;
        let mut best_delta = 0.0f32;
        for code in 0..=ABS_MAX {
            if !gp.axes[code].present || is_hat_axis(code) {
                continue;
            }
            if wiz.used_axes.iter().any(|&u| u == Some(code as u16)) {
                continue;
            }
            let delta = gp.axes[code].raw_norm() - wiz.baseline[code];
            if delta.abs() > best_delta.abs() {
                best_delta = delta;
                best_code = Some(code);
            }
        }

        if let Some(code) = best_code {
            if best_delta.abs() >= 0.45 {
                let inverted = best_delta < 0.0;
                wiz.used_axes[wiz.step] = Some(code as u16);
                match wiz.step {
                    0 => {
                        cfg.axis_thr = Some(code as u16);
                        cfg.invert_throttle = inverted;
                        wiz.message = format!("THR -> ABS_{code} (inv:{})", inverted as u8);
                    }
                    1 => {
                        cfg.axis_yaw = Some(code as u16);
                        cfg.invert_yaw = inverted;
                        wiz.message = format!("YAW -> ABS_{code} (inv:{})", inverted as u8);
                    }
                    2 => {
                        cfg.axis_pit = Some(code as u16);
                        cfg.invert_pitch = inverted;
                        wiz.message = format!("PIT -> ABS_{code} (inv:{})", inverted as u8);
                    }
                    _ => {
                        cfg.axis_rol = Some(code as u16);
                        cfg.invert_roll = inverted;
                        cfg.save();
                        self.calib = None;
                        return true;
                    }
                }
                wiz.step += 1;
                wiz.step_started = Instant::now();
                for i in 0..=ABS_MAX {
                    wiz.baseline[i] = gp.axes[i].raw_norm();
                }
            }
        }
        false
    }

    /// Update 3-step AUX switch/button binding wizard. Pass `skip=true` when user presses Enter to skip a step.
    pub fn update_aux_wizard(&mut self, cfg: &mut Config, skip: bool) -> bool {
        let Some(ref mut wiz) = self.aux_wiz else { return false };
        let Some(ref gp) = self.gamepad else {
            self.aux_wiz = None;
            return false;
        };

        if skip {
            match wiz.step {
                0 => {
                    cfg.aux_arm = AuxBinding::None;
                    wiz.message = "ARM skipped (Auto-Arm)".to_string();
                }
                1 => {
                    cfg.aux_mode = AuxBinding::None;
                    wiz.message = "MODE switch skipped".to_string();
                }
                _ => {
                    cfg.aux_turtle = AuxBinding::None;
                    cfg.save();
                    self.aux_wiz = None;
                    return true;
                }
            }
            wiz.step += 1;
            wiz.step_started = Instant::now();
            for i in 0..=ABS_MAX {
                wiz.baseline[i] = gp.axes[i].raw_norm();
            }
            return false;
        }

        if wiz.step_started.elapsed() < Duration::from_millis(400) {
            for i in 0..=ABS_MAX {
                wiz.baseline[i] = gp.axes[i].raw_norm();
            }
            return false;
        }

        let mut detected: Option<AuxBinding> = None;

        // 1. Check any EV_KEY button pressed on the controller
        if let Some(btn) = gp.frame_btn_pressed {
            detected = Some(AuxBinding::Button { code: btn });
        }

        // 2. Check any non-stick EV_ABS switch axis deflected by >= 0.45
        if detected.is_none() {
            let mut best_code: Option<usize> = None;
            let mut best_delta = 0.0f32;
            for code in 0..=ABS_MAX {
                if !gp.axes[code].present || is_hat_axis(code) || wiz.stick_axes.contains(&code) {
                    continue;
                }
                let delta = gp.axes[code].raw_norm() - wiz.baseline[code];
                if delta.abs() > best_delta.abs() {
                    best_delta = delta;
                    best_code = Some(code);
                }
            }
            if let Some(code) = best_code {
                if best_delta.abs() >= 0.45 {
                    let invert = best_delta < 0.0;
                    detected = Some(AuxBinding::Axis {
                        code: code as u16,
                        invert,
                    });
                }
            }
        }

        if let Some(binding) = detected {
            match wiz.step {
                0 => {
                    cfg.aux_arm = binding;
                    wiz.message = format!("ARM -> {}", binding.short_label());
                }
                1 => {
                    cfg.aux_mode = binding;
                    wiz.message = format!("MODE -> {}", binding.short_label());
                }
                _ => {
                    cfg.aux_turtle = binding;
                    cfg.save();
                    self.aux_wiz = None;
                    return true;
                }
            }
            wiz.step += 1;
            wiz.step_started = Instant::now();
            for i in 0..=ABS_MAX {
                wiz.baseline[i] = gp.axes[i].raw_norm();
            }
        }
        false
    }

    /// Compact live string of active non-hat EV_ABS axes + last pressed button for Controller diagnostics.
    pub fn raw_axes_summary(&self) -> String {
        let Some(ref gp) = self.gamepad else {
            return "No evdev ABS axes".to_string();
        };
        let mut active: Vec<(usize, f32)> = Vec::new();
        for code in 0..=ABS_MAX {
            if gp.axes[code].present && !is_hat_axis(code) {
                active.push((code, gp.axes[code].raw_norm()));
                if active.len() >= 8 {
                    break;
                }
            }
        }
        let compact = active.len() > 6;
        let mut parts: Vec<String> = active
            .into_iter()
            .map(|(code, v)| {
                if compact {
                    // Compact 1-digit format so all 8 RC axes (0..7) fit on the 320px display
                    let rounded = v.round() as i32;
                    format!("{code}:{rounded:+}")
                } else {
                    format!("{code}:{v:+.1}")
                }
            })
            .collect();
        if let Some(b) = gp.last_btn_seen {
            parts.push(format!("B{b}"));
        }
        if parts.is_empty() {
            "No ABS axes".to_string()
        } else {
            parts.join(" ")
        }
    }

    /// Scan `/dev/input/event*` for gamepads/RC radios and CardputerZero keyboard.
    pub fn scan_devices(&mut self) {
        self.last_scan = Instant::now();

        if self.keyboards.is_empty() {
            let cardputer_kb = "/dev/input/by-path/platform-3f804000.i2c-event";
            if Path::new(cardputer_kb).exists() {
                if let Some(fd) = Self::open_nonblock(cardputer_kb) {
                    self.keyboards.push((fd, cardputer_kb.to_string()));
                }
            }
        }

        let Ok(entries) = fs::read_dir("/dev/input") else { return };
        let mut paths: Vec<String> = entries
            .flatten()
            .filter_map(|e| {
                let name = e.file_name().to_string_lossy().to_string();
                if name.starts_with("event") {
                    Some(format!("/dev/input/{name}"))
                } else {
                    None
                }
            })
            .collect();
        paths.sort();

        struct Candidate {
            score: i32,
            fd: i32,
            path: String,
            name: String,
            kind: ControllerKind,
            bustype: u16,
            axes: [AxisCal; ABS_MAX + 1],
        }

        let mut candidates: Vec<Candidate> = Vec::new();

        for path in paths {
            if let Some(ref gp) = self.gamepad {
                if gp.path == path {
                    continue;
                }
            }

            let Some(fd) = Self::open_nonblock(&path) else { continue };

            let mut name_buf = [0u8; 128];
            let r = unsafe { ioctl(fd, eviocgname(name_buf.len()), name_buf.as_mut_ptr()) };
            let name = if r > 0 {
                let end = name_buf.iter().position(|&b| b == 0).unwrap_or(name_buf.len());
                String::from_utf8_lossy(&name_buf[..end]).trim().to_string()
            } else {
                path.clone()
            };

            let mut ev_bits = [0u8; 4];
            let _ = unsafe { ioctl(fd, eviocgbit(0, ev_bits.len()), ev_bits.as_mut_ptr()) };
            let has_abs = (ev_bits[0] & (1 << EV_ABS)) != 0;
            let has_key = (ev_bits[0] & (1 << EV_KEY)) != 0;

            let lower = name.to_ascii_lowercase();
            let is_ignored = lower.contains("imu")
                || lower.contains("accel")
                || lower.contains("gyro")
                || lower.contains("bmi270")
                || lower.contains("touchpad")
                || lower.contains("motion")
                || lower.contains("sensor")
                || lower.contains("mouse")
                || lower.contains("tca8418");

            if has_abs && !is_ignored && self.gamepad.is_none() {
                let mut abs_bits = [0u8; 8];
                let _ = unsafe { ioctl(fd, eviocgbit(EV_ABS as u32, abs_bits.len()), abs_bits.as_mut_ptr()) };
                let mut axes = [AxisCal::default(); ABS_MAX + 1];
                let mut stick_axes = 0i32;
                for code in 0..=ABS_MAX {
                    if (abs_bits[code / 8] & (1 << (code % 8))) != 0 {
                        let mut info = InputAbsInfo::default();
                        if unsafe { ioctl(fd, eviocgabs(code as u32), &mut info) } >= 0 {
                            if info.maximum > info.minimum {
                                axes[code] = AxisCal {
                                    present: true,
                                    min: info.minimum,
                                    max: info.maximum,
                                    value: info.value,
                                };
                                if !is_hat_axis(code) {
                                    stick_axes += 1;
                                }
                            }
                        }
                    }
                }

                if stick_axes >= 2 {
                    let mut id = InputId::default();
                    let _ = unsafe { ioctl(fd, eviocgid(), &mut id) };

                    let is_rc = lower.contains("edgetx")
                        || lower.contains("opentx")
                        || lower.contains("radiomaster")
                        || lower.contains("elrs")
                        || lower.contains("expresslrs")
                        || lower.contains("frsky")
                        || lower.contains("jumper")
                        || lower.contains("tbs")
                        || lower.contains("betaflight");
                    let is_pad = lower.contains("xbox")
                        || lower.contains("wireless controller")
                        || lower.contains("dualsense")
                        || lower.contains("dualshock")
                        || lower.contains("8bitdo")
                        || lower.contains("pro controller")
                        || lower.contains("gamepad")
                        || lower.contains("joystick");

                    let mut score = stick_axes * 10;
                    if stick_axes >= 4 { score += 50; }
                    if is_rc || is_pad { score += 40; }
                    if has_key { score += 15; }

                    candidates.push(Candidate {
                        score,
                        fd,
                        path,
                        name,
                        kind: if is_rc { ControllerKind::RcRadio } else { ControllerKind::Gamepad },
                        bustype: id.bustype,
                        axes,
                    });
                    continue;
                }
            }

            if has_key && self.keyboards.is_empty() {
                if lower.contains("keyboard") || lower.contains("keypad") || lower.contains("tca8418") {
                    self.keyboards.push((fd, path));
                    continue;
                }
            }

            unsafe { close(fd) };
        }

        if self.gamepad.is_none() && !candidates.is_empty() {
            candidates.sort_by(|a, b| b.score.cmp(&a.score));
            self.candidate_count = candidates.len();
            let pick = self.device_skip_idx % candidates.len();
            let chosen = candidates.swap_remove(pick);
            for c in candidates {
                unsafe { close(c.fd) };
            }
            self.gamepad = Some(GamepadDevice::spawn(
                chosen.fd,
                chosen.path,
                chosen.name,
                chosen.kind,
                chosen.bustype,
                chosen.axes,
            ));
        } else if self.gamepad.is_none() && !self.imu.is_available() && !self.imu.is_lite {
            let probed = ImuSensor::probe_default();
            if probed.is_available() {
                self.imu = probed;
            }
        }
    }

    fn open_nonblock(path: &str) -> Option<i32> {
        let cpath = CString::new(path).ok()?;
        let fd = unsafe { open(cpath.as_ptr(), O_RDONLY | O_NONBLOCK) };
        if fd >= 0 { Some(fd) } else { None }
    }

    fn is_bound_button(cfg: &Config, code: u16) -> bool {
        matches!(cfg.aux_arm, AuxBinding::Button { code: c } if c == code)
            || matches!(cfg.aux_mode, AuxBinding::Button { code: c } if c == code)
            || matches!(cfg.aux_turtle, AuxBinding::Button { code: c } if c == code)
    }

    /// Poll all input devices; call once per frame.
    pub fn poll(&mut self, dt: f32, in_menu: bool, cfg: &Config) {
        self.pending = MenuActions::default();
        self.kb_arm_toggle = false;
        self.kb_mode_toggle = false;
        self.kb_turtle_toggle = false;

        if self.gamepad.is_none() && self.last_scan.elapsed() > Duration::from_millis(1500) {
            self.scan_devices();
        }

        let wizard_active = self.calib.is_some() || self.aux_wiz.is_some();

        // 1. Snapshot latest state from the dedicated background gamepad reader thread
        //    (zero syscalls on the 60 Hz render thread — evdev is drained continuously).
        let mut disconnect = false;
        if let Some(ref mut gp) = self.gamepad {
            gp.frame_btns_pressed.clear();
            gp.frame_btn_pressed = None;

            let (hat_up, hat_down, hat_left, hat_right) = if let Ok(mut st) = gp.shared.lock() {
                gp.axes = st.axes;
                gp.keys_down = st.keys_down;
                gp.frame_btns_pressed.extend(st.pressed_buttons.drain(..));
                disconnect = st.disconnected;
                let hats = (st.hat_up, st.hat_down, st.hat_left, st.hat_right);
                st.hat_up = false;
                st.hat_down = false;
                st.hat_left = false;
                st.hat_right = false;
                hats
            } else {
                (false, false, false, false)
            };

            if in_menu && !wizard_active {
                if hat_up { self.pending.up = true; }
                if hat_down { self.pending.down = true; }
                if hat_left { self.pending.left = true; }
                if hat_right { self.pending.right = true; }
            }

            if let Some(&last_btn) = gp.frame_btns_pressed.last() {
                gp.frame_btn_pressed = Some(last_btn);
                gp.last_btn_seen = Some(last_btn);
            }

            for &code in &gp.frame_btns_pressed {
                // Only map generic menu/respawn shortcuts if button isn't bound to an AUX flight switch
                let skip_shortcut = wizard_active || (!in_menu && Self::is_bound_button(cfg, code));
                if !skip_shortcut {
                    match code {
                        BTN_SOUTH | BTN_TRIGGER => if in_menu { self.pending.select = true; },
                        BTN_EAST | BTN_THUMB | BTN_START | BTN_MODE => self.pending.back = true,
                        BTN_SELECT | BTN_NORTH => self.pending.respawn = true,
                        BTN_TL => {
                            if in_menu {
                                self.pending.tab_prev = true;
                            } else {
                                self.pending.respawn = true;
                            }
                        }
                        BTN_TR => {
                            if in_menu {
                                self.pending.tab_next = true;
                            }
                        }
                        _ => {}
                    }
                }
            }
        }
        if disconnect {
            self.gamepad = None;
            self.calib = None;
            self.aux_wiz = None;
        }

        // 1b. Keep IMU background thread active only when no physical controller is connected.
        let use_imu = self.gamepad.is_none();
        self.imu.set_active(use_imu);
        if use_imu {
            self.imu.poll();
        }

        // 2. Poll keyboards (batched reads)
        let mut kb_events: Vec<(u16, i32)> = Vec::new();
        let mut kb_buf = [InputEvent::default(); 32];
        let ev_size = std::mem::size_of::<InputEvent>();
        let kb_bytes = kb_buf.len() * ev_size;
        for (fd, _) in &self.keyboards {
            loop {
                let n = unsafe { read(*fd, kb_buf.as_mut_ptr() as *mut std::ffi::c_void, kb_bytes) };
                if n > 0 {
                    let count = (n as usize) / ev_size;
                    for ev in &kb_buf[..count] {
                        if ev.type_ == EV_KEY {
                            kb_events.push((ev.code, ev.value));
                        }
                    }
                    if count < kb_buf.len() {
                        break;
                    }
                } else {
                    break;
                }
            }
        }
        for (code, val) in kb_events {
            self.process_key_event(code, val, in_menu);
        }

        // CardputerZero long-Esc (>= 700ms) for clean exit to APPLaunch
        if let Some(t0) = self.esc_down_at {
            if !self.esc_long_fired && t0.elapsed() >= Duration::from_millis(700) {
                self.esc_long_fired = true;
                self.pending.exit_app = true;
            }
        }

        // Keyboard left-gimbal throttle (CardputerZero F/X, Arrows Up/Down, W/S, ;/.)
        if self.kb_l_up != 0 {
            self.kb_throttle = (self.kb_throttle + 1.1 * dt).min(1.0);
        } else if self.kb_l_down != 0 {
            self.kb_throttle = (self.kb_throttle - 1.3 * dt).max(0.0);
        }

        // Edge-triggered RIGHT-STICK ONLY menu navigation
        if in_menu && !wizard_active && self.gamepad.is_some() {
            let sticks = self.flight_sticks(cfg);
            if sticks.pitch.abs() < 0.35 {
                self.stick_nav_latched_y = false;
            } else if !self.stick_nav_latched_y {
                if sticks.pitch > 0.70 {
                    self.pending.up = true;
                    self.stick_nav_latched_y = true;
                } else if sticks.pitch < -0.70 {
                    self.pending.down = true;
                    self.stick_nav_latched_y = true;
                }
            }
            if sticks.roll.abs() < 0.35 {
                self.stick_nav_latched_x = false;
            } else if !self.stick_nav_latched_x {
                if sticks.roll > 0.70 {
                    self.pending.right = true;
                    self.stick_nav_latched_x = true;
                } else if sticks.roll < -0.70 {
                    self.pending.left = true;
                    self.stick_nav_latched_x = true;
                }
            }
        }
    }

    /// Process a single Linux evdev keyboard event.
    ///
    /// Flight keyboard mapping (Mode 2):
    /// - Left Gimbal (Throttle / Yaw): CardputerZero arrow keys (`F`/`X`/`Z`/`C`, `Up`/`Down`/`Left`/`Right`)
    /// - Right Gimbal (Pitch / Roll): `K` (pitch forward), `N` (pitch back), `B` (roll left), `M` (roll right)
    /// - Flight shortcuts: `Space` = zero IMU, `E` = cycle flight mode, `T` = turtle, `R` = respawn, `Tab` = arm/disarm
    fn process_key_event(&mut self, code: u16, val: i32, in_menu: bool) {
        let pressed = val == 1;
        let held = val != 0;
        match code {
            KEY_ESC => {
                if val == 1 {
                    self.esc_down_at = Some(Instant::now());
                    self.esc_long_fired = false;
                } else if val == 0 {
                    if !self.esc_long_fired && self.esc_down_at.is_some() {
                        self.pending.back = true;
                    }
                    self.esc_down_at = None;
                    self.esc_long_fired = false;
                }
            }
            // Left Gimbal Up (Throttle+ / IMU Climb) + Menu Up
            KEY_F => {
                set_dir_bit(&mut self.kb_l_up, 0, held);
                if pressed && in_menu { self.pending.up = true; }
            }
            KEY_UP => {
                set_dir_bit(&mut self.kb_l_up, 1, held);
                if pressed && in_menu { self.pending.up = true; }
            }
            // Left Gimbal Down (Throttle- / IMU Descend) + Menu Down
            KEY_X => {
                set_dir_bit(&mut self.kb_l_down, 0, held);
                if pressed && in_menu { self.pending.down = true; }
            }
            KEY_DOWN => {
                set_dir_bit(&mut self.kb_l_down, 1, held);
                if pressed && in_menu { self.pending.down = true; }
            }
            // Left Gimbal Left (Yaw Left) + Menu Left
            KEY_Z => {
                set_dir_bit(&mut self.kb_l_left, 0, held);
                if pressed && in_menu { self.pending.left = true; }
            }
            KEY_LEFT => {
                set_dir_bit(&mut self.kb_l_left, 1, held);
                if pressed && in_menu { self.pending.left = true; }
            }
            // Left Gimbal Right (Yaw Right) + Menu Right / Controller Calibrate
            KEY_C => {
                set_dir_bit(&mut self.kb_l_right, 0, held);
                if pressed && in_menu {
                    self.pending.calibrate = true;
                    self.pending.right = true;
                }
            }
            KEY_RIGHT => {
                set_dir_bit(&mut self.kb_l_right, 1, held);
                if pressed && in_menu { self.pending.right = true; }
            }
            // Right Gimbal: K (Pitch Forward), N (Pitch Back), B (Roll Left), M (Roll Right)
            KEY_K => {
                self.kb_r_up = held;
            }
            KEY_N => {
                self.kb_r_down = held;
                if pressed && in_menu { self.pending.next_device = true; }
            }
            KEY_B => {
                self.kb_r_left = held;
                if pressed && in_menu { self.pending.bind_aux = true; }
            }
            KEY_M => {
                self.kb_r_right = held;
            }
            // Menu / Flight utility keys
            KEY_ENTER => {
                if pressed { self.pending.select = true; }
            }
            KEY_SPACE => {
                if pressed {
                    if in_menu {
                        self.pending.select = true;
                    } else {
                        self.pending.recenter_imu = true;
                    }
                }
            }
            KEY_BACKSPACE => {
                if pressed { self.pending.back = true; }
            }
            KEY_R => {
                if pressed { self.pending.respawn = true; }
            }
            KEY_TAB => {
                if pressed {
                    if in_menu {
                        self.pending.tab_next = true;
                    } else {
                        self.kb_arm_toggle = true;
                    }
                }
            }
            KEY_E => {
                if pressed && !in_menu { self.kb_mode_toggle = true; }
            }
            KEY_T => {
                if pressed && !in_menu { self.kb_turtle_toggle = true; }
            }
            _ => {}
        }
    }

    pub fn take_menu_actions(&mut self) -> MenuActions {
        let a = self.pending;
        self.pending = MenuActions::default();
        a
    }

    /// Evaluate bound radio AUX switches/buttons + keyboard flight shortcuts.
    pub fn aux_inputs(&self, cfg: &Config) -> AuxInputs {
        let mut out = AuxInputs {
            arm_toggle: self.kb_arm_toggle,
            mode_toggle: self.kb_mode_toggle,
            turtle_toggle: self.kb_turtle_toggle,
            ..AuxInputs::default()
        };

        let Some(ref gp) = self.gamepad else {
            return out;
        };

        let eval_axis = |code: u16, invert: bool| -> bool {
            let c = (code as usize).min(ABS_MAX);
            if !gp.axes[c].present {
                return false;
            }
            let v = gp.axes[c].raw_norm();
            if invert { v < -0.25 } else { v > 0.25 }
        };

        match cfg.aux_arm {
            AuxBinding::Axis { code, invert } => {
                let c = (code as usize).min(ABS_MAX);
                if gp.axes[c].present {
                    out.arm_aux_connected = true;
                    out.arm_switch = Some(eval_axis(code, invert));
                }
            }
            AuxBinding::Button { code } => {
                out.arm_aux_connected = true;
                if gp.was_btn_pressed(code) {
                    out.arm_toggle = true;
                }
            }
            AuxBinding::None => {}
        }

        match cfg.aux_mode {
            AuxBinding::Axis { code, invert } => {
                let c = (code as usize).min(ABS_MAX);
                if gp.axes[c].present {
                    let mut v = gp.axes[c].raw_norm();
                    if invert {
                        v = -v;
                    }
                    let fm = if v > 0.35 {
                        FlightMode::Angle
                    } else if v < -0.35 {
                        FlightMode::Acro
                    } else {
                        FlightMode::Horizon
                    };
                    out.mode_switch = Some(fm);
                }
            }
            AuxBinding::Button { code } => {
                if gp.was_btn_pressed(code) {
                    out.mode_toggle = true;
                }
            }
            AuxBinding::None => {}
        }

        match cfg.aux_turtle {
            AuxBinding::Axis { code, invert } => {
                let c = (code as usize).min(ABS_MAX);
                if gp.axes[c].present {
                    out.turtle_switch = Some(eval_axis(code, invert));
                }
            }
            AuxBinding::Button { code } => {
                // Works identically for both momentary (SD/SH) and latching (SA) radio buttons!
                out.turtle_switch = Some(gp.is_key_down(code));
            }
            AuxBinding::None => {}
        }

        out
    }

    /// Auto-resolve (thr, yaw, pit, rol) axis codes and default Y-inversions for a controller.
    fn resolve_auto_axes(gp: &GamepadDevice) -> ([usize; 4], [bool; 4]) {
        let has = |c: u16| gp.axes[c as usize].present;

        if gp.kind == ControllerKind::RcRadio {
            let rol = ABS_X as usize;
            let pit = ABS_Y as usize;
            // EdgeTX USB & ExpressLRS BLE expose CH1-CH4 on X/Y/Z/RX (or RZ) and use
            // ABS_THROTTLE (6) / ABS_RUDDER (7) for CH7 (SC) and CH8 (SD / Crash Flip).
            // Prefer Z & RX so CH7/CH8 AUX switches are never shadowed by stick_axes;
            // fall back to ABS_THROTTLE / ABS_RUDDER only on legacy sim dongles lacking Z/RX.
            let thr = if has(ABS_Z) {
                ABS_Z as usize
            } else if has(ABS_THROTTLE) {
                ABS_THROTTLE as usize
            } else {
                ABS_Z as usize
            };
            let yaw = if has(ABS_RX) {
                ABS_RX as usize
            } else if has(ABS_RZ) {
                ABS_RZ as usize
            } else if has(ABS_RUDDER) {
                ABS_RUDDER as usize
            } else {
                ABS_RX as usize
            };
            return ([thr, yaw, pit, rol], [false, false, false, false]);
        }

        if has(ABS_X) && has(ABS_Y) && has(ABS_RX) && has(ABS_RY) {
            return (
                [ABS_Y as usize, ABS_X as usize, ABS_RY as usize, ABS_RX as usize],
                [true, false, true, false],
            );
        }

        if has(ABS_X) && has(ABS_Y) && has(ABS_Z) && has(ABS_RZ) {
            return (
                [ABS_Y as usize, ABS_X as usize, ABS_RZ as usize, ABS_Z as usize],
                [true, false, true, false],
            );
        }

        let mut found = [0usize; 4];
        let mut count = 0usize;
        for c in 0..=ABS_MAX {
            if gp.axes[c].present && !is_hat_axis(c) {
                found[count] = c;
                count += 1;
                if count == 4 {
                    break;
                }
            }
        }
        ([found[1], found[0], found[3], found[2]], [true, false, true, false])
    }

    /// Compute normalized FPV flight stick input from controller or keyboard fallback.
    pub fn flight_sticks(&self, cfg: &Config) -> StickInput {
        if let Some(ref gp) = self.gamepad {
            let (auto_codes, auto_inv) = Self::resolve_auto_axes(gp);
            let manual = cfg.axis_thr.is_some()
                || cfg.axis_yaw.is_some()
                || cfg.axis_pit.is_some()
                || cfg.axis_rol.is_some();

            let read_axis = |opt: Option<u16>, fallback: usize, default_inv: bool, user_inv: bool| -> f32 {
                let code = opt.map(|c| c as usize).unwrap_or(fallback).min(ABS_MAX);
                let mut v = gp.axes[code].norm();
                let inv = if manual { user_inv } else { default_inv ^ user_inv };
                if inv {
                    v = -v;
                }
                v
            };

            let mut raw_thr = read_axis(cfg.axis_thr, auto_codes[0], auto_inv[0], cfg.invert_throttle);
            let yaw = read_axis(cfg.axis_yaw, auto_codes[1], auto_inv[1], cfg.invert_yaw);
            let mut pitch = read_axis(cfg.axis_pit, auto_codes[2], auto_inv[2], cfg.invert_pitch);
            let roll = read_axis(cfg.axis_rol, auto_codes[3], auto_inv[3], cfg.invert_roll);

            if !manual && cfg.stick_mode == StickMode::Mode1 {
                std::mem::swap(&mut raw_thr, &mut pitch);
            }

            let throttle = match cfg.throttle_mode {
                ThrottleMode::RcFull => clampf((raw_thr + 1.0) * 0.5, 0.0, 1.0),
                ThrottleMode::GamepadCenter => {
                    if raw_thr >= 0.0 {
                        0.34 + raw_thr * 0.66
                    } else {
                        (0.34 * (1.0 + raw_thr)).max(0.0)
                    }
                }
            };
            StickInput { throttle, yaw, pitch, roll }
        } else if self.imu.is_available() {
            // Simplified CardputerZero IMU flight:
            // - Left Gimbal (CardputerZero F/X/Z/C, Arrows, W/S/A/D): Climb/Descend + Yaw
            // - Right Gimbal: CardputerZero IMU Tilt (+ optional K/N/B/M keyboard override)
            let (imu_roll, imu_pitch) = self.imu.tilt_sticks();
            let kb_yaw = ((self.kb_l_right != 0) as i32 - (self.kb_l_left != 0) as i32) as f32 * 0.80;
            let kb_pitch = (self.kb_r_up as i32 - self.kb_r_down as i32) as f32 * 0.80;
            let kb_roll = (self.kb_r_right as i32 - self.kb_r_left as i32) as f32 * 0.80;
            let pitch = clampf(imu_pitch + kb_pitch, -1.0, 1.0);
            let roll = clampf(imu_roll + kb_roll, -1.0, 1.0);
            let yaw = clampf(kb_yaw + imu_roll * 0.60, -1.0, 1.0);
            let throttle = clampf(0.5 + 0.5 * self.imu_climb_cmd(), 0.0, 1.0);
            StickInput {
                throttle,
                yaw,
                pitch,
                roll,
            }
        } else {
            // Keyboard flight:
            // - Left Gimbal: CardputerZero arrow keys (F/X/Z/C, Arrows, ;/./,//, WASD) -> Throttle / Yaw
            // - Right Gimbal: K (pitch fwd), N (pitch back), B (roll left), M (roll right)
            let yaw = ((self.kb_l_right != 0) as i32 - (self.kb_l_left != 0) as i32) as f32 * 0.75;
            let pitch = (self.kb_r_up as i32 - self.kb_r_down as i32) as f32 * 0.75;
            let roll = (self.kb_r_right as i32 - self.kb_r_left as i32) as f32 * 0.75;
            StickInput {
                throttle: self.kb_throttle,
                yaw,
                pitch,
                roll,
            }
        }
    }
}

impl Drop for InputManager {
    fn drop(&mut self) {
        for (fd, _) in &self.keyboards {
            unsafe { close(*fd) };
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_eviocgkey_bitmap_unpack_matches_set_key_bit() {
        let codes = [KEY_ESC, BTN_TRIGGER, BTN_SOUTH, BTN_NORTH, 0x2c0];
        let mut raw_bits = [0u8; MAX_KEY_CODE / 8];
        for &c in &codes {
            let idx = c as usize;
            raw_bits[idx / 8] |= 1u8 << (idx % 8);
        }
        let mut unpacked = [0u64; MAX_KEY_CODE / 64];
        unpack_key_bitmap(&raw_bits, &mut unpacked);

        let mut expected = [0u64; MAX_KEY_CODE / 64];
        for &c in &codes {
            set_key_bit(&mut expected, c, true);
        }
        assert_eq!(unpacked, expected);

        // Clearing one bit works cleanly
        set_key_bit(&mut unpacked, BTN_SOUTH, false);
        assert_eq!(
            (unpacked[BTN_SOUTH as usize / 64] & (1u64 << (BTN_SOUTH as usize % 64))),
            0
        );
        assert_ne!(
            (unpacked[BTN_TRIGGER as usize / 64] & (1u64 << (BTN_TRIGGER as usize % 64))),
            0
        );
    }

    #[test]
    fn test_control_source_and_launch_warning_logic() {
        use std::time::{SystemTime, UNIX_EPOCH};

        let nanos = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let root = std::env::temp_dir().join(format!("stickzero_input_imu_{nanos}"));
        let dev = root.join("iio:device0");
        fs::create_dir_all(&dev).unwrap();
        fs::write(dev.join("name"), "bmi270\n").unwrap();
        fs::write(dev.join("in_accel_scale"), "0.001\n").unwrap();
        fs::write(dev.join("in_accel_x_raw"), "0\n").unwrap();
        fs::write(dev.join("in_accel_y_raw"), "0\n").unwrap();
        fs::write(dev.join("in_accel_z_raw"), "9810\n").unwrap();

        let cfg = Config::default();

        // 1. No controller + BMI270 IMU present -> ImuSimplified, no launch warning
        let mut mgr = InputManager::new();
        mgr.gamepad = None;
        mgr.imu = ImuSensor::probe_from_root(&root, false);
        assert_eq!(mgr.control_source(), FlightControlSource::ImuSimplified);
        assert!(!mgr.needs_controller_warning());
        assert_eq!(mgr.controller_name(), "CardputerZero IMU");

        // Simulate banking right + CardputerZero Up arrow (F) pressed
        mgr.imu.latest.roll_rad = 20.0_f32.to_radians();
        mgr.process_key_event(KEY_F, 1, false);
        let sticks = mgr.flight_sticks(&cfg);
        assert!(sticks.roll > 0.45, "roll={}", sticks.roll);
        assert!(sticks.yaw > 0.25, "coordinated bank-to-turn yaw={}", sticks.yaw);
        assert_eq!(mgr.imu_climb_cmd(), 1.0);
        assert_eq!(sticks.throttle, 1.0);

        // Space in flight triggers IMU recenter, not select
        mgr.process_key_event(KEY_SPACE, 1, false);
        let act = mgr.take_menu_actions();
        assert!(act.recenter_imu && !act.select);

        // 2. No controller + CardputerZero Lite -> None, launch warning required
        mgr.imu = ImuSensor::probe_from_root(&root, true);
        assert_eq!(mgr.control_source(), FlightControlSource::None);
        assert!(mgr.needs_controller_warning());
        assert!(mgr.is_cardputer_lite());

        // 3. No controller + no IMU -> None, launch warning required
        let empty = root.join("empty_iio");
        fs::create_dir_all(&empty).unwrap();
        mgr.imu = ImuSensor::probe_from_root(&empty, false);
        assert_eq!(mgr.control_source(), FlightControlSource::None);
        assert!(mgr.needs_controller_warning());
        assert!(!mgr.is_cardputer_lite());

        drop(mgr);
        let _ = fs::remove_dir_all(&root);
    }

    #[test]
    fn test_cardputer_keyboard_gimbals_arrows_left_kbnm_right() {
        let cfg = Config::default();
        let mut mgr = InputManager::new();
        mgr.gamepad = None;
        mgr.imu = ImuSensor::probe_from_root(Path::new("/nonexistent_iio_dir"), false);

        // Left Gimbal: CardputerZero arrow keys F/X/Z/C and hardware Up/Down/Left/Right
        mgr.process_key_event(KEY_F, 1, false);
        mgr.kb_throttle = (mgr.kb_throttle + 0.5).min(1.0);
        mgr.process_key_event(KEY_C, 1, false); // Yaw right (must NOT trigger calibrate in flight!)
        assert!(!mgr.take_menu_actions().calibrate);
        let s1 = mgr.flight_sticks(&cfg);
        assert!(s1.throttle > 0.45);
        assert!(s1.yaw > 0.5, "C should yaw right, got {}", s1.yaw);

        mgr.process_key_event(KEY_C, 0, false);
        mgr.process_key_event(KEY_Z, 1, false); // Yaw left
        assert!(mgr.flight_sticks(&cfg).yaw < -0.5);
        mgr.process_key_event(KEY_Z, 0, false);
        mgr.process_key_event(KEY_F, 0, false);

        // Hardware arrow keys also drive Left Gimbal
        mgr.process_key_event(KEY_LEFT, 1, false);
        assert!(mgr.flight_sticks(&cfg).yaw < -0.5);
        mgr.process_key_event(KEY_LEFT, 0, false);
        mgr.process_key_event(KEY_RIGHT, 1, false);
        assert!(mgr.flight_sticks(&cfg).yaw > 0.5);
        mgr.process_key_event(KEY_RIGHT, 0, false);

        // Right Gimbal: K (pitch fwd), N (pitch back), B (roll left), M (roll right)
        mgr.process_key_event(KEY_K, 1, false);
        assert!(mgr.flight_sticks(&cfg).pitch > 0.5, "K should pitch forward");
        mgr.process_key_event(KEY_K, 0, false);

        mgr.process_key_event(KEY_N, 1, false);
        assert!(mgr.flight_sticks(&cfg).pitch < -0.5, "N should pitch back");
        assert!(!mgr.take_menu_actions().next_device);
        mgr.process_key_event(KEY_N, 0, false);

        mgr.process_key_event(KEY_B, 1, false);
        assert!(mgr.flight_sticks(&cfg).roll < -0.5, "B should roll left");
        assert!(!mgr.take_menu_actions().bind_aux);
        mgr.process_key_event(KEY_B, 0, false);

        mgr.process_key_event(KEY_M, 1, false);
        assert!(mgr.flight_sticks(&cfg).roll > 0.5, "M should roll right");
        assert!(!mgr.kb_mode_toggle, "M must roll right, not toggle flight mode");
        mgr.process_key_event(KEY_M, 0, false);
    }

    #[test]
    fn test_rc_radio_aux_wizard_binds_a4_a5_and_recovered_a7_sd() {
        let mut axes = [AxisCal::default(); ABS_MAX + 1];
        for c in 0..=7 {
            axes[c] = AxisCal {
                present: true,
                min: 0,
                max: 32767,
                value: if c >= 4 { 0 } else { 16384 }, // AUX switches resting LOW (-1.0)
            };
        }

        let gp = GamepadDevice {
            shared: Arc::new(Mutex::new(GamepadLiveState {
                axes,
                keys_down: [0u64; MAX_KEY_CODE / 64],
                pressed_buttons: Vec::new(),
                hat_up: false,
                hat_down: false,
                hat_left: false,
                hat_right: false,
                disconnected: false,
            })),
            stop: Arc::new(AtomicBool::new(true)),
            reader: None,
            path: "/dev/input/event99".to_string(),
            name: "ExpressLRS Joystick".to_string(),
            kind: ControllerKind::RcRadio,
            bustype: BUS_BLUETOOTH,
            axes,
            keys_down: [0u64; MAX_KEY_CODE / 64],
            frame_btns_pressed: Vec::new(),
            frame_btn_pressed: None,
            last_btn_seen: None,
        };

        // 1. Auto-axes on 8-ch ELRS/EdgeTX radio must pick CH1-4 (0..=3) and NEVER shadow CH7(6)/CH8(7)!
        let (auto_codes, _) = InputManager::resolve_auto_axes(&gp);
        assert_eq!(auto_codes, [ABS_Z as usize, ABS_RX as usize, ABS_Y as usize, ABS_X as usize]);
        assert!(!auto_codes.contains(&(ABS_THROTTLE as usize)));
        assert!(!auto_codes.contains(&(ABS_RUDDER as usize)));

        let mut cfg = Config::default();
        let mut mgr = InputManager::new();
        mgr.gamepad = Some(gp);

        // 2. Live diagnostics string must include axis 7 (SD) even with 8 active axes
        let summary = mgr.raw_axes_summary();
        assert!(summary.contains("7:-1"), "expected axis 7 in summary, got: {summary}");

        // 3. AUX wizard binds SA (A4+), SB (A5+), and SD (A7+) cleanly!
        mgr.start_aux_wizard(&cfg);
        let past = Instant::now() - Duration::from_secs(1);

        // Step 0: ARM -> flip axis 4 HIGH
        mgr.aux_wiz.as_mut().unwrap().step_started = past;
        mgr.gamepad.as_mut().unwrap().axes[4].value = 32767;
        assert!(!mgr.update_aux_wizard(&mut cfg, false));
        assert_eq!(cfg.aux_arm.short_label(), "A4+");

        // Step 1: MODE -> flip axis 5 HIGH
        mgr.aux_wiz.as_mut().unwrap().step_started = past;
        mgr.gamepad.as_mut().unwrap().axes[5].value = 32767;
        assert!(!mgr.update_aux_wizard(&mut cfg, false));
        assert_eq!(cfg.aux_mode.short_label(), "A5+");

        // Step 2: CRASH FLIP (SD / AUX 4) -> flip recovered axis 7 HIGH
        mgr.aux_wiz.as_mut().unwrap().step_started = past;
        mgr.gamepad.as_mut().unwrap().axes[7].value = 32767;
        assert!(mgr.update_aux_wizard(&mut cfg, false));
        assert_eq!(cfg.aux_turtle.short_label(), "A7+");

        // 4. Live evaluation matches USB behavior identically
        let aux = mgr.aux_inputs(&cfg);
        assert!(aux.arm_aux_connected);
        assert_eq!(aux.arm_switch, Some(true));
        assert_eq!(aux.mode_switch, Some(FlightMode::Angle));
        assert_eq!(aux.turtle_switch, Some(true));

        mgr.gamepad.as_mut().unwrap().axes[7].value = 0;
        assert_eq!(mgr.aux_inputs(&cfg).turtle_switch, Some(false));
    }
}
