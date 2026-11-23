//! Zero-dependency Linux evdev input manager for:
//! - Bluetooth controllers (Xbox, PlayStation, Switch Pro, 8BitDo, ELRS BLE Joystick)
//! - USB-C Host controllers (EdgeTX/OpenTX USB Joystick, generic HID gamepads)
//! - CardputerZero built-in TCA8418 keyboard (`/dev/input/by-path/platform-3f804000.i2c-event`)
//!
//! Includes hot-plug detection, automatic axis range calibration via EVIOCGABS,
//! stick-based menu navigation, and CardputerZero short-Esc / long-Esc exit semantics.

use crate::config::{Config, StickMode, ThrottleMode};
use crate::linux_sys::*;
use crate::math::clampf;
use crate::physics::StickInput;
use std::ffi::CString;
use std::fs;
use std::path::Path;
use std::time::{Duration, Instant};

#[derive(Copy, Clone, Debug, Default)]
pub struct AxisCal {
    pub present: bool,
    pub min: i32,
    pub max: i32,
    pub value: i32,
}

impl AxisCal {
    /// Returns normalized value in -1.0 ..= +1.0
    pub fn norm(&self) -> f32 {
        if !self.present || self.max <= self.min {
            return 0.0;
        }
        let span = (self.max - self.min) as f32;
        let v = ((self.value - self.min) as f32 / span) * 2.0 - 1.0;
        let v = clampf(v, -1.0, 1.0);
        if v.abs() < 0.03 { 0.0 } else { v }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ControllerKind {
    None,
    RcRadio, // EdgeTX / OpenTX / ExpressLRS
    Gamepad, // Xbox / PS / Switch / 8BitDo / Generic
}

pub struct GamepadDevice {
    fd: i32,
    pub path: String,
    pub name: String,
    pub kind: ControllerKind,
    pub axes: [AxisCal; ABS_MAX + 1],
    pub btn_south: bool,
    pub btn_east: bool,
    pub btn_start: bool,
    pub btn_select: bool,
    pub btn_tl: bool,
    pub btn_tr: bool,
}

impl Drop for GamepadDevice {
    fn drop(&mut self) {
        if self.fd >= 0 {
            unsafe { close(self.fd) };
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
    pub respawn: bool,
    pub exit_app: bool,
}

pub struct InputManager {
    pub gamepad: Option<GamepadDevice>,
    keyboards: Vec<(i32, String)>,
    last_scan: Instant,

    // Keyboard flight state
    kb_w: bool,
    kb_s: bool,
    kb_a: bool,
    kb_d: bool,
    kb_up: bool,
    kb_down: bool,
    kb_left: bool,
    kb_right: bool,
    kb_throttle: f32,

    // CardputerZero Esc short/long press tracking
    esc_down_at: Option<Instant>,
    esc_long_fired: bool,

    // Edge-triggered menu actions
    pending: MenuActions,
    stick_nav_cooldown: Instant,
}

impl InputManager {
    pub fn new() -> Self {
        let mut mgr = Self {
            gamepad: None,
            keyboards: Vec::new(),
            last_scan: Instant::now() - Duration::from_secs(10),
            kb_w: false,
            kb_s: false,
            kb_a: false,
            kb_d: false,
            kb_up: false,
            kb_down: false,
            kb_left: false,
            kb_right: false,
            kb_throttle: 0.0,
            esc_down_at: None,
            esc_long_fired: false,
            pending: MenuActions::default(),
            stick_nav_cooldown: Instant::now(),
        };
        mgr.scan_devices();
        mgr
    }

    pub fn controller_name(&self) -> &str {
        match &self.gamepad {
            Some(g) => &g.name,
            None => "No BT/USB Controller",
        }
    }

    pub fn controller_kind(&self) -> ControllerKind {
        match &self.gamepad {
            Some(g) => g.kind,
            None => ControllerKind::None,
        }
    }

    /// Scan `/dev/input/event*` for gamepads/RC radios and CardputerZero keyboard.
    pub fn scan_devices(&mut self) {
        self.last_scan = Instant::now();

        // 1. Open CardputerZero built-in keyboard if we don't have keyboards yet
        if self.keyboards.is_empty() {
            let cardputer_kb = "/dev/input/by-path/platform-3f804000.i2c-event";
            if Path::new(cardputer_kb).exists() {
                if let Some(fd) = Self::open_nonblock(cardputer_kb) {
                    self.keyboards.push((fd, cardputer_kb.to_string()));
                }
            }
        }

        // 2. Scan /dev/input/event*
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

        for path in paths {
            // Skip already-open gamepad
            if let Some(ref gp) = self.gamepad {
                if gp.path == path {
                    continue;
                }
            }

            let Some(fd) = Self::open_nonblock(&path) else { continue };

            // Query device name
            let mut name_buf = [0u8; 128];
            let r = unsafe { ioctl(fd, eviocgname(name_buf.len()), name_buf.as_mut_ptr()) };
            let name = if r > 0 {
                let end = name_buf.iter().position(|&b| b == 0).unwrap_or(name_buf.len());
                String::from_utf8_lossy(&name_buf[..end]).trim().to_string()
            } else {
                path.clone()
            };

            // Check capabilities: EV_ABS + EV_KEY
            let mut ev_bits = [0u8; 4];
            let _ = unsafe { ioctl(fd, eviocgbit(0, ev_bits.len()), ev_bits.as_mut_ptr()) };
            let has_abs = (ev_bits[0] & (1 << EV_ABS)) != 0;
            let has_key = (ev_bits[0] & (1 << EV_KEY)) != 0;

            if has_abs && self.gamepad.is_none() {
                let mut abs_bits = [0u8; 8];
                let _ = unsafe { ioctl(fd, eviocgbit(EV_ABS as u32, abs_bits.len()), abs_bits.as_mut_ptr()) };
                let mut axes = [AxisCal::default(); ABS_MAX + 1];
                let mut axis_count = 0usize;
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
                                axis_count += 1;
                            }
                        }
                    }
                }

                // Require at least 2 axes and ignore IMU / accelerometer devices
                let lower = name.to_ascii_lowercase();
                let is_imu = lower.contains("imu") || lower.contains("accel") || lower.contains("bmi270") || lower.contains("touch");
                if axis_count >= 2 && !is_imu {
                    let is_rc = lower.contains("edgetx")
                        || lower.contains("opentx")
                        || lower.contains("radiomaster")
                        || lower.contains("elrs")
                        || lower.contains("expresslrs")
                        || lower.contains("frsky")
                        || lower.contains("jumper")
                        || lower.contains("tbs")
                        || lower.contains("joystick");
                    self.gamepad = Some(GamepadDevice {
                        fd,
                        path,
                        name,
                        kind: if is_rc { ControllerKind::RcRadio } else { ControllerKind::Gamepad },
                        axes,
                        btn_south: false,
                        btn_east: false,
                        btn_start: false,
                        btn_select: false,
                        btn_tl: false,
                        btn_tr: false,
                    });
                    continue;
                }
            }

            // Fallback: open any keyboard if we didn't find CardputerZero's dedicated i2c-event path
            if has_key && self.keyboards.is_empty() {
                let lower = name.to_ascii_lowercase();
                if lower.contains("keyboard") || lower.contains("keypad") || lower.contains("tca8418") {
                    self.keyboards.push((fd, path));
                    continue;
                }
            }

            unsafe { close(fd) };
        }
    }

    fn open_nonblock(path: &str) -> Option<i32> {
        let cpath = CString::new(path).ok()?;
        let fd = unsafe { open(cpath.as_ptr(), O_RDONLY | O_NONBLOCK) };
        if fd >= 0 { Some(fd) } else { None }
    }

    /// Poll all input devices; call once per frame.
    pub fn poll(&mut self, dt: f32, in_menu: bool) {
        self.pending = MenuActions::default();

        // Periodically hot-plug scan if no controller is connected
        if self.gamepad.is_none() && self.last_scan.elapsed() > Duration::from_millis(1500) {
            self.scan_devices();
        }

        // 1. Poll gamepad / RC controller
        let mut disconnect = false;
        if let Some(ref mut gp) = self.gamepad {
            let mut ev = InputEvent::default();
            let ev_size = std::mem::size_of::<InputEvent>();
            loop {
                let n = unsafe { read(gp.fd, &mut ev as *mut _ as *mut std::ffi::c_void, ev_size) };
                if n == ev_size as isize {
                    match ev.type_ {
                        EV_ABS => {
                            let c = ev.code as usize;
                            if c <= ABS_MAX {
                                gp.axes[c].value = ev.value;
                                if !gp.axes[c].present {
                                    gp.axes[c].present = true;
                                    gp.axes[c].min = -32768;
                                    gp.axes[c].max = 32767;
                                }
                            }
                            // D-pad hat navigation
                            if ev.code == ABS_HAT0Y {
                                if ev.value < 0 { self.pending.up = true; }
                                if ev.value > 0 { self.pending.down = true; }
                            } else if ev.code == ABS_HAT0X {
                                if ev.value < 0 { self.pending.left = true; }
                                if ev.value > 0 { self.pending.right = true; }
                            }
                        }
                        EV_KEY => {
                            let pressed = ev.value == 1;
                            let held = ev.value != 0;
                            match ev.code {
                                BTN_SOUTH | BTN_TRIGGER => {
                                    gp.btn_south = held;
                                    if pressed { self.pending.select = true; }
                                }
                                BTN_EAST | BTN_THUMB => {
                                    gp.btn_east = held;
                                    if pressed { self.pending.back = true; }
                                }
                                BTN_START | BTN_MODE => {
                                    gp.btn_start = held;
                                    if pressed { self.pending.back = true; }
                                }
                                BTN_SELECT | BTN_NORTH | BTN_TL => {
                                    gp.btn_select = held;
                                    if pressed { self.pending.respawn = true; }
                                }
                                BTN_TR => {
                                    gp.btn_tr = held;
                                }
                                _ => {}
                            }
                        }
                        _ => {}
                    }
                } else if n == 0 {
                    disconnect = true;
                    break;
                } else {
                    // EAGAIN or error
                    break;
                }
            }
        }
        if disconnect {
            self.gamepad = None;
        }

        // 2. Poll keyboards
        let mut kb_events: Vec<(u16, i32)> = Vec::new();
        for (fd, _) in &self.keyboards {
            let mut ev = InputEvent::default();
            let ev_size = std::mem::size_of::<InputEvent>();
            loop {
                let n = unsafe { read(*fd, &mut ev as *mut _ as *mut std::ffi::c_void, ev_size) };
                if n == ev_size as isize {
                    if ev.type_ == EV_KEY {
                        kb_events.push((ev.code, ev.value));
                    }
                } else {
                    break;
                }
            }
        }
        for (code, val) in kb_events {
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
                KEY_UP | KEY_I => {
                    self.kb_up = held;
                    if pressed { self.pending.up = true; }
                }
                KEY_DOWN | KEY_K => {
                    self.kb_down = held;
                    if pressed { self.pending.down = true; }
                }
                KEY_LEFT | KEY_J => {
                    self.kb_left = held;
                    if pressed { self.pending.left = true; }
                }
                KEY_RIGHT | KEY_L => {
                    self.kb_right = held;
                    if pressed { self.pending.right = true; }
                }
                KEY_W => {
                    self.kb_w = held;
                    if pressed && in_menu { self.pending.up = true; }
                }
                KEY_S => {
                    self.kb_s = held;
                    if pressed && in_menu { self.pending.down = true; }
                }
                KEY_A => {
                    self.kb_a = held;
                    if pressed && in_menu { self.pending.left = true; }
                }
                KEY_D => {
                    self.kb_d = held;
                    if pressed && in_menu { self.pending.right = true; }
                }
                KEY_ENTER | KEY_SPACE => {
                    if pressed { self.pending.select = true; }
                }
                KEY_BACKSPACE => {
                    if pressed { self.pending.back = true; }
                }
                KEY_R => {
                    if pressed { self.pending.respawn = true; }
                }
                _ => {}
            }
        }

        // Check CardputerZero long-Esc (>= 700ms) for clean exit to APPLaunch
        if let Some(t0) = self.esc_down_at {
            if !self.esc_long_fired && t0.elapsed() >= Duration::from_millis(700) {
                self.esc_long_fired = true;
                self.pending.exit_app = true;
            }
        }

        // Update keyboard throttle smoothly
        if self.kb_w {
            self.kb_throttle = (self.kb_throttle + 1.1 * dt).min(1.0);
        } else if self.kb_s {
            self.kb_throttle = (self.kb_throttle - 1.3 * dt).max(0.0);
        }

        // Stick-based menu navigation (so user can navigate menus purely with BT/USB controller!)
        if in_menu && self.stick_nav_cooldown.elapsed() > Duration::from_millis(180) {
            if let Some(ref gp) = self.gamepad {
                let (lx, ly, rx, ry) = Self::extract_raw_sticks(gp);
                let nav_y = if ry.abs() > ly.abs() { ry } else { ly };
                let nav_x = if rx.abs() > lx.abs() { rx } else { lx };
                if nav_y > 0.65 {
                    self.pending.up = true;
                    self.stick_nav_cooldown = Instant::now();
                } else if nav_y < -0.65 {
                    self.pending.down = true;
                    self.stick_nav_cooldown = Instant::now();
                } else if nav_x > 0.65 {
                    self.pending.right = true;
                    self.stick_nav_cooldown = Instant::now();
                } else if nav_x < -0.65 {
                    self.pending.left = true;
                    self.stick_nav_cooldown = Instant::now();
                }
            }
        }
    }

    pub fn take_menu_actions(&mut self) -> MenuActions {
        let a = self.pending;
        self.pending = MenuActions::default();
        a
    }

    /// Extract (left_x, left_y, right_x, right_y) in -1..+1 (up = +1, right = +1).
    pub fn extract_raw_sticks(gp: &GamepadDevice) -> (f32, f32, f32, f32) {
        match gp.kind {
            ControllerKind::RcRadio => {
                // EdgeTX / ELRS standard AETR mapping:
                // ABS_X = Aileron (right X), ABS_Y = Elevator (right Y),
                // ABS_Z / ABS_THROTTLE = Throttle (left Y), ABS_RX / ABS_RUDDER = Rudder (left X)
                let rx = gp.axes[ABS_X as usize].norm();
                let ry = gp.axes[ABS_Y as usize].norm();
                let ly = if gp.axes[ABS_THROTTLE as usize].present {
                    gp.axes[ABS_THROTTLE as usize].norm()
                } else {
                    gp.axes[ABS_Z as usize].norm()
                };
                let lx = if gp.axes[ABS_RUDDER as usize].present {
                    gp.axes[ABS_RUDDER as usize].norm()
                } else if gp.axes[ABS_RX as usize].present {
                    gp.axes[ABS_RX as usize].norm()
                } else {
                    gp.axes[ABS_RZ as usize].norm()
                };
                (lx, ly, rx, ry)
            }
            _ => {
                // Standard Bluetooth/USB Gamepad (Xbox, PS, Switch Pro, 8BitDo):
                // Left stick = ABS_X, ABS_Y (inverted Y: up is negative on Linux evdev!)
                // Right stick = ABS_RX, ABS_RY (or ABS_Z, ABS_RZ on some controllers)
                let lx = gp.axes[ABS_X as usize].norm();
                let ly = -gp.axes[ABS_Y as usize].norm();
                let (rx, ry) = if gp.axes[ABS_RX as usize].present && gp.axes[ABS_RY as usize].present {
                    (gp.axes[ABS_RX as usize].norm(), -gp.axes[ABS_RY as usize].norm())
                } else {
                    (gp.axes[ABS_Z as usize].norm(), -gp.axes[ABS_RZ as usize].norm())
                };
                (lx, ly, rx, ry)
            }
        }
    }

    /// Compute normalized FPV flight stick input from controller or keyboard fallback.
    pub fn flight_sticks(&self, cfg: &Config) -> StickInput {
        if let Some(ref gp) = self.gamepad {
            let (lx, ly, rx, ry) = Self::extract_raw_sticks(gp);
            let (raw_thr, yaw, mut pitch, roll) = match cfg.stick_mode {
                StickMode::Mode2 => (ly, lx, ry, rx),
                StickMode::Mode1 => (ry, lx, ly, rx),
            };
            let raw_thr = if cfg.invert_throttle { -raw_thr } else { raw_thr };
            if cfg.invert_pitch {
                pitch = -pitch;
            }
            let throttle = match cfg.throttle_mode {
                ThrottleMode::RcFull => clampf((raw_thr + 1.0) * 0.5, 0.0, 1.0),
                ThrottleMode::GamepadCenter => {
                    // Spring-centered gamepad stick: center (0) = ~0.34 hover throttle,
                    // pushing up goes smoothly to 1.0, pulling down goes to 0.0!
                    if raw_thr >= 0.0 {
                        0.34 + raw_thr * 0.66
                    } else {
                        (0.34 * (1.0 + raw_thr)).max(0.0)
                    }
                }
            };
            StickInput { throttle, yaw, pitch, roll }
        } else {
            // CardputerZero built-in keyboard flight fallback
            let yaw = (self.kb_d as i32 - self.kb_a as i32) as f32 * 0.75;
            let pitch = (self.kb_up as i32 - self.kb_down as i32) as f32 * 0.75;
            let roll = (self.kb_right as i32 - self.kb_left as i32) as f32 * 0.75;
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
