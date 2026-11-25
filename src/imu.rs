//! Zero-dependency Linux IIO sysfs driver for the CardputerZero onboard Bosch BMI270 IMU.
//!
//! Hardware facts verified on M5Stack CardputerZero:
//! - Full CardputerZero carries a Bosch BMI270 6-axis IMU exposed via Linux IIO sysfs
//!   (`/sys/bus/iio/devices/iio:device*`, typically `name = "bmi270"`).
//! - CardputerZero Lite omits the BMI270 IMU (and IMX219 camera).
//! - BMI270 raw axes in device body frame:
//!     +raw_x -> bottom edge (keyboard side)
//!     +raw_y -> right edge
//!     +raw_z -> out of the LCD screen
//!   Mapped to screen coordinates:
//!     screen_x = +raw_y  (right)
//!     screen_y = -raw_x  (up / top edge)
//!     screen_z = +raw_z  (out of screen; +9.81 m/s^2 when lying flat face-up)
//!
//! Reads happen on a dedicated background thread via `pread(2)` at offset 0 so the 60 Hz
//! render/physics loop never blocks on I2C transactions. When an external USB-C/BT controller
//! is connected, the thread drops to idle sleep (~0% CPU).

use crate::math::clampf;
use std::fs::{self, File};
use std::os::unix::fs::FileExt;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::Duration;

/// Deadzone around neutral hold angle (degrees) where tilt output is 0.
const TILT_DEADZONE_DEG: f32 = 3.0;
/// Tilt angle beyond neutral (degrees) producing full 1.0 stick deflection.
const TILT_FULL_DEG: f32 = 28.0;
/// Exponential smoothing factor for accelerometer vector at ~80 Hz polling.
const ACCEL_LPF_ALPHA: f32 = 0.28;
/// Consecutive read failures before marking the IMU stream unavailable.
const MAX_READ_FAILURES: u32 = 40;

#[derive(Copy, Clone, Debug, Default)]
pub struct ImuSample {
    /// Filtered screen-frame acceleration (m/s^2): +x right, +y top edge, +z out of screen.
    pub accel: [f32; 3],
    /// Pitch angle in radians (top edge UP is positive; flat face-up is 0).
    pub pitch_rad: f32,
    /// Roll angle in radians (right edge DOWN is positive; flat face-up is 0).
    pub roll_rad: f32,
    /// True once at least one valid sample has been read and sensor is healthy.
    pub valid: bool,
}

#[derive(Clone, Debug)]
struct DiscoveredImu {
    dir: PathBuf,
    name: String,
    accel_scale: f32,
    initial_accel: [f32; 3],
}

/// Map raw BMI270 sensor coordinates `(raw_x, raw_y, raw_z)` to CardputerZero screen frame:
/// `+x = right edge`, `+y = top edge`, `+z = out of screen`.
#[inline]
pub fn map_bmi270_to_screen(raw: [f32; 3]) -> [f32; 3] {
    [raw[1], -raw[0], raw[2]]
}

/// Compute gimbal-lock-free `(pitch_rad, roll_rad)` from screen-frame proper acceleration:
/// - `pitch_rad > 0` when top edge is tilted UP (holding screen toward viewer).
/// - `roll_rad > 0` when right edge is tilted DOWN (banking right).
#[inline]
pub fn screen_accel_to_angles(screen: [f32; 3]) -> (f32, f32) {
    let sx = screen[0];
    let sy = screen[1];
    let sz = screen[2];
    let pitch_rad = sy.atan2(sx.hypot(sz));
    let roll_rad = (-sx).atan2(sy.hypot(sz));
    (pitch_rad, roll_rad)
}

/// Shape an angular tilt delta (in radians) into a smooth normalized stick value `-1.0 ..= +1.0`.
#[inline]
pub fn shape_tilt_axis(delta_rad: f32) -> f32 {
    let deg = delta_rad.to_degrees();
    let mag = deg.abs();
    if mag <= TILT_DEADZONE_DEG {
        return 0.0;
    }
    let span = (TILT_FULL_DEG - TILT_DEADZONE_DEG).max(1.0);
    let lin = clampf((mag - TILT_DEADZONE_DEG) / span, 0.0, 1.0);
    // Gentle cubic expo: silky precision near level, crisp authority at full tilt.
    let curved = 0.70 * lin + 0.30 * lin * lin * lin;
    curved.copysign(deg)
}

fn read_text_trimmed(path: &Path) -> Option<String> {
    let raw = fs::read_to_string(path).ok()?;
    let trimmed = raw.trim().trim_end_matches('\0').trim();
    if trimmed.is_empty() {
        None
    } else {
        Some(trimmed.to_string())
    }
}

fn read_f32_file(path: &Path) -> Option<f32> {
    let s = read_text_trimmed(path)?;
    let v = s.parse::<f32>().ok()?;
    if v.is_finite() { Some(v) } else { None }
}

/// Refresh-read a single signed/float sysfs value at offset 0 using `pread(2)`.
fn pread_f32(file: &File) -> Option<f32> {
    let mut buf = [0u8; 48];
    let n = file.read_at(&mut buf, 0).ok()?;
    if n == 0 {
        return None;
    }
    let s = std::str::from_utf8(&buf[..n]).ok()?.trim();
    let v = s.parse::<f32>().ok()?;
    if v.is_finite() { Some(v) } else { None }
}

/// Detect whether the device explicitly identifies itself as CardputerZero Lite.
pub fn detect_cardputer_lite() -> bool {
    if let Ok(val) = std::env::var("STICKZERO_FORCE_LITE") {
        let lower = val.trim().to_ascii_lowercase();
        if lower == "1" || lower == "true" || lower == "yes" {
            return true;
        }
    }

    const MODEL_PATHS: [&str; 4] = [
        "/proc/device-tree/model",
        "/sys/firmware/devicetree/base/model",
        "/etc/cardputerzero-model",
        "/sys/class/dmi/id/product_name",
    ];
    for p in MODEL_PATHS {
        if let Some(model) = read_text_trimmed(Path::new(p)) {
            let lower = model.to_ascii_lowercase();
            if lower.contains("cardputer") && lower.contains("lite") {
                return true;
            }
        }
    }
    false
}

fn try_probe_iio_dir(dir: &Path, require_known_imu: bool) -> Option<DiscoveredImu> {
    let name = read_text_trimmed(&dir.join("name")).unwrap_or_default();
    let lower = name.to_ascii_lowercase();
    let is_bmi270 = lower.contains("bmi270");
    let has_gyro = dir.join("in_anglvel_x_raw").exists();
    let is_known_imu = is_bmi270
        || lower.contains("imu")
        || lower.contains("mpu")
        || lower.contains("lsm6")
        || lower.contains("icm")
        || lower.contains("bmi160")
        || has_gyro;

    if require_known_imu && !is_known_imu {
        return None;
    }

    let accel_scale = read_f32_file(&dir.join("in_accel_scale"))?;
    if accel_scale <= 0.0 {
        return None;
    }

    let rx = read_f32_file(&dir.join("in_accel_x_raw"))?;
    let ry = read_f32_file(&dir.join("in_accel_y_raw"))?;
    let rz = read_f32_file(&dir.join("in_accel_z_raw"))?;
    let raw = [rx * accel_scale, ry * accel_scale, rz * accel_scale];
    let norm_sq = raw[0] * raw[0] + raw[1] * raw[1] + raw[2] * raw[2];
    // Real accelerometer under Earth gravity reads ~9.81 m/s^2; reject dead zeroed nodes.
    if !norm_sq.is_finite() || norm_sq < 0.25 {
        return None;
    }

    let display_name = if name.is_empty() {
        dir.file_name()
            .map(|n| n.to_string_lossy().to_string())
            .unwrap_or_else(|| "bmi270".to_string())
    } else {
        name
    };

    Some(DiscoveredImu {
        dir: dir.to_path_buf(),
        name: display_name,
        accel_scale,
        initial_accel: map_bmi270_to_screen(raw),
    })
}

fn discover_imu_in_root(root: &Path) -> Option<DiscoveredImu> {
    if let Ok(direct) = std::env::var("STICKZERO_IMU_DEV") {
        let trimmed = direct.trim();
        if !trimmed.is_empty() {
            return try_probe_iio_dir(Path::new(trimmed), false);
        }
    }

    let entries = fs::read_dir(root).ok()?;
    let mut dirs: Vec<PathBuf> = entries.flatten().map(|e| e.path()).collect();
    dirs.sort();

    // Pass 1: Strongly prefer BMI270 (CardputerZero onboard IMU).
    for dir in &dirs {
        if let Some(name) = read_text_trimmed(&dir.join("name")) {
            if name.to_ascii_lowercase().contains("bmi270") {
                if let Some(found) = try_probe_iio_dir(dir, false) {
                    return Some(found);
                }
            }
        }
    }

    // Pass 2: Fallback to any readable 6-axis IMU node under IIO root.
    for dir in &dirs {
        if let Some(found) = try_probe_iio_dir(dir, true) {
            return Some(found);
        }
    }

    None
}

fn imu_reader_thread(
    dir: PathBuf,
    scale: f32,
    shared: Arc<Mutex<ImuSample>>,
    active: Arc<AtomicBool>,
    stop: Arc<AtomicBool>,
) {
    let Ok(fx) = File::open(dir.join("in_accel_x_raw")) else {
        if let Ok(mut st) = shared.lock() {
            st.valid = false;
        }
        return;
    };
    let Ok(fy) = File::open(dir.join("in_accel_y_raw")) else {
        if let Ok(mut st) = shared.lock() {
            st.valid = false;
        }
        return;
    };
    let Ok(fz) = File::open(dir.join("in_accel_z_raw")) else {
        if let Ok(mut st) = shared.lock() {
            st.valid = false;
        }
        return;
    };

    let mut filtered = shared.lock().map(|s| s.accel).unwrap_or([0.0, 0.0, 9.81]);
    let mut failures = 0u32;

    while !stop.load(Ordering::Relaxed) {
        if !active.load(Ordering::Relaxed) {
            thread::sleep(Duration::from_millis(120));
            continue;
        }

        if let (Some(rx), Some(ry), Some(rz)) = (pread_f32(&fx), pread_f32(&fy), pread_f32(&fz)) {
            let screen = map_bmi270_to_screen([rx * scale, ry * scale, rz * scale]);
            let norm_sq = screen[0] * screen[0] + screen[1] * screen[1] + screen[2] * screen[2];
            if norm_sq.is_finite() && norm_sq >= 0.25 {
                failures = 0;
                filtered[0] += (screen[0] - filtered[0]) * ACCEL_LPF_ALPHA;
                filtered[1] += (screen[1] - filtered[1]) * ACCEL_LPF_ALPHA;
                filtered[2] += (screen[2] - filtered[2]) * ACCEL_LPF_ALPHA;
                let (pitch_rad, roll_rad) = screen_accel_to_angles(filtered);
                if let Ok(mut st) = shared.lock() {
                    st.accel = filtered;
                    st.pitch_rad = pitch_rad;
                    st.roll_rad = roll_rad;
                    st.valid = true;
                }
            } else {
                failures = failures.saturating_add(1);
            }
        } else {
            failures = failures.saturating_add(1);
        }

        if failures >= MAX_READ_FAILURES {
            if let Ok(mut st) = shared.lock() {
                st.valid = false;
            }
            break;
        }

        thread::sleep(Duration::from_millis(12));
    }
}

/// Manages CardputerZero onboard IMU discovery, background polling, neutral pose calibration,
/// and simplified tilt-to-stick conversion.
pub struct ImuSensor {
    shared: Arc<Mutex<ImuSample>>,
    active: Arc<AtomicBool>,
    stop: Arc<AtomicBool>,
    reader: Option<JoinHandle<()>>,
    pub is_lite: bool,
    pub device_name: String,
    #[allow(dead_code)]
    pub device_path: String,
    pub latest: ImuSample,
    pub zero_pitch_rad: f32,
    pub zero_roll_rad: f32,
    zeroed: bool,
}

impl ImuSensor {
    pub fn probe_default() -> Self {
        let root = std::env::var("STICKZERO_IIO_ROOT")
            .ok()
            .filter(|s| !s.trim().is_empty())
            .map(PathBuf::from)
            .unwrap_or_else(|| PathBuf::from("/sys/bus/iio/devices"));
        let is_lite = detect_cardputer_lite();
        Self::probe_from_root(&root, is_lite)
    }

    pub fn probe_from_root(root: &Path, is_lite: bool) -> Self {
        // CardputerZero Lite explicitly has no onboard IMU; do not enable IMU mode on Lite.
        let discovered = if is_lite {
            None
        } else {
            discover_imu_in_root(root)
        };

        if let Some(info) = discovered {
            let (pitch_rad, roll_rad) = screen_accel_to_angles(info.initial_accel);
            let initial = ImuSample {
                accel: info.initial_accel,
                pitch_rad,
                roll_rad,
                valid: true,
            };
            let shared = Arc::new(Mutex::new(initial));
            let active = Arc::new(AtomicBool::new(true));
            let stop = Arc::new(AtomicBool::new(false));
            let bg_dir = info.dir.clone();
            let bg_shared = Arc::clone(&shared);
            let bg_active = Arc::clone(&active);
            let bg_stop = Arc::clone(&stop);
            let reader = thread::spawn(move || {
                imu_reader_thread(bg_dir, info.accel_scale, bg_shared, bg_active, bg_stop)
            });

            let mut sensor = Self {
                shared,
                active,
                stop,
                reader: Some(reader),
                is_lite,
                device_name: info.name,
                device_path: info.dir.to_string_lossy().to_string(),
                latest: initial,
                zero_pitch_rad: 0.0,
                zero_roll_rad: 0.0,
                zeroed: false,
            };
            sensor.recenter();
            sensor
        } else {
            Self {
                shared: Arc::new(Mutex::new(ImuSample::default())),
                active: Arc::new(AtomicBool::new(false)),
                stop: Arc::new(AtomicBool::new(true)),
                reader: None,
                is_lite,
                device_name: "None".to_string(),
                device_path: String::new(),
                latest: ImuSample::default(),
                zero_pitch_rad: 0.0,
                zero_roll_rad: 0.0,
                zeroed: false,
            }
        }
    }

    /// True if a healthy onboard IMU is detected and usable for simplified flight.
    #[inline]
    pub fn is_available(&self) -> bool {
        !self.is_lite && self.reader.is_some() && self.latest.valid
    }

    /// Enable or pause background IIO polling (paused automatically while an external controller is connected).
    pub fn set_active(&self, enabled: bool) {
        self.active.store(enabled, Ordering::Relaxed);
    }

    /// Snapshot the latest filtered IMU reading without blocking.
    pub fn poll(&mut self) {
        if self.reader.is_none() {
            return;
        }
        if let Ok(st) = self.shared.lock() {
            self.latest = *st;
        }
        if self.latest.valid && !self.zeroed {
            self.recenter();
        }
    }

    /// Capture the current handheld orientation as neutral (level hover) reference.
    pub fn recenter(&mut self) {
        if !self.latest.valid {
            return;
        }
        // Clamp neutral reference to sane handheld viewing poses (-50 deg .. +72 deg pitch, +/-30 deg roll)
        self.zero_pitch_rad = clampf(self.latest.pitch_rad, -0.87, 1.26);
        self.zero_roll_rad = clampf(self.latest.roll_rad, -0.52, 0.52);
        self.zeroed = true;
    }

    /// Tilt angle relative to neutral center in degrees: `(pitch_fwd_deg, roll_right_deg)`.
    /// Positive `pitch_fwd_deg` = tilted forward (fly forward); positive `roll_right_deg` = right edge down (bank right).
    pub fn relative_tilt_deg(&self) -> (f32, f32) {
        if !self.is_available() {
            return (0.0, 0.0);
        }
        let pitch_fwd_rad = self.zero_pitch_rad - self.latest.pitch_rad;
        let roll_right_rad = self.latest.roll_rad - self.zero_roll_rad;
        (pitch_fwd_rad.to_degrees(), roll_right_rad.to_degrees())
    }

    /// Convert current relative IMU tilt into normalized `(roll, pitch)` stick commands in `-1.0 ..= +1.0`.
    pub fn tilt_sticks(&self) -> (f32, f32) {
        if !self.is_available() {
            return (0.0, 0.0);
        }
        let pitch_fwd_rad = self.zero_pitch_rad - self.latest.pitch_rad;
        let roll_right_rad = self.latest.roll_rad - self.zero_roll_rad;
        (shape_tilt_axis(roll_right_rad), shape_tilt_axis(pitch_fwd_rad))
    }

    /// Compact live diagnostic readout for the Controller screen.
    pub fn summary(&self) -> String {
        if self.is_lite {
            "CardputerZero Lite (no IMU)".to_string()
        } else if !self.is_available() {
            "No IMU sensor detected".to_string()
        } else {
            let (p_deg, r_deg) = self.relative_tilt_deg();
            format!("{} P:{:+5.1} R:{:+5.1} deg", self.device_name, p_deg, r_deg)
        }
    }
}

impl Drop for ImuSensor {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        if let Some(handle) = self.reader.take() {
            let _ = handle.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{SystemTime, UNIX_EPOCH};

    fn make_temp_iio_dir(tag: &str) -> PathBuf {
        let nanos = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let dir = std::env::temp_dir().join(format!("stickzero_imu_test_{tag}_{nanos}"));
        fs::create_dir_all(&dir).unwrap();
        dir
    }

    fn write_fake_bmi270(root: &Path, raw_x: i32, raw_y: i32, raw_z: i32) -> PathBuf {
        let dev = root.join("iio:device0");
        fs::create_dir_all(&dev).unwrap();
        fs::write(dev.join("name"), "bmi270\n").unwrap();
        fs::write(dev.join("in_accel_scale"), "0.001\n").unwrap();
        fs::write(dev.join("in_accel_x_raw"), format!("{raw_x}\n")).unwrap();
        fs::write(dev.join("in_accel_y_raw"), format!("{raw_y}\n")).unwrap();
        fs::write(dev.join("in_accel_z_raw"), format!("{raw_z}\n")).unwrap();
        dev
    }

    #[test]
    fn test_bmi270_axis_mapping_and_tilt_signs() {
        // CardputerZero flat face-up: raw = (0, 0, +9.81) -> screen = (0, 0, +9.81)
        let flat = map_bmi270_to_screen([0.0, 0.0, 9.81]);
        let (p0, r0) = screen_accel_to_angles(flat);
        assert!(p0.abs() < 1e-4 && r0.abs() < 1e-4);

        // Held tilted 30 deg toward user (top edge UP):
        // screen_y = +g*sin(30), screen_z = +g*cos(30) => raw_x = -screen_y, raw_y = 0, raw_z = screen_z
        let tilt30 = 30.0_f32.to_radians();
        let held = map_bmi270_to_screen([-9.81 * tilt30.sin(), 0.0, 9.81 * tilt30.cos()]);
        let (p_held, r_held) = screen_accel_to_angles(held);
        assert!((p_held.to_degrees() - 30.0).abs() < 0.2, "got {}", p_held.to_degrees());
        assert!(r_held.abs() < 1e-4);

        // Tilt top edge forward by 15 deg (from 30 deg down to 15 deg): delta pitch_fwd = +15 deg -> positive stick!
        let tilt15 = 15.0_f32.to_radians();
        let fwd = map_bmi270_to_screen([-9.81 * tilt15.sin(), 0.0, 9.81 * tilt15.cos()]);
        let (p_fwd, _) = screen_accel_to_angles(fwd);
        let pitch_cmd = shape_tilt_axis(p_held - p_fwd);
        assert!(pitch_cmd > 0.35 && pitch_cmd < 0.65, "forward tilt pitch_cmd={pitch_cmd}");

        // Tilt right edge DOWN by 20 deg: screen_x = -g*sin(20) => raw_y = -g*sin(20)
        let bank20 = 20.0_f32.to_radians();
        let right_down = map_bmi270_to_screen([0.0, -9.81 * bank20.sin(), 9.81 * bank20.cos()]);
        let (_, r_right) = screen_accel_to_angles(right_down);
        assert!((r_right.to_degrees() - 20.0).abs() < 0.2, "got {}", r_right.to_degrees());
        let roll_cmd = shape_tilt_axis(r_right - r0);
        assert!(roll_cmd > 0.50 && roll_cmd < 0.85, "right bank roll_cmd={roll_cmd}");

        // Tilt left edge DOWN by 20 deg: raw_y = +g*sin(20) -> negative roll stick!
        let left_down = map_bmi270_to_screen([0.0, 9.81 * bank20.sin(), 9.81 * bank20.cos()]);
        let (_, r_left) = screen_accel_to_angles(left_down);
        let roll_left_cmd = shape_tilt_axis(r_left - r0);
        assert!(roll_left_cmd < -0.50 && roll_left_cmd > -0.85, "left bank roll_cmd={roll_left_cmd}");

        // Inside deadzone (<= 3 deg) produces exact 0.0
        assert_eq!(shape_tilt_axis(2.5_f32.to_radians()), 0.0);
        assert_eq!(shape_tilt_axis(-2.5_f32.to_radians()), 0.0);
    }

    #[test]
    fn test_imu_probe_discovers_bmi270_and_reads_live_tilt() {
        let root = make_temp_iio_dir("bmi270_ok");
        // Neutral pose: held 30 deg toward user (raw_x = -4905, raw_y = 0, raw_z = 8496)
        let dev = write_fake_bmi270(&root, -4905, 0, 8496);

        let mut imu = ImuSensor::probe_from_root(&root, false);
        assert!(imu.is_available(), "BMI270 should be detected");
        assert_eq!(imu.device_name, "bmi270");

        let (r0, p0) = imu.tilt_sticks();
        assert_eq!(r0, 0.0);
        assert_eq!(p0, 0.0);

        // Now simulate tilting forward (to flat: raw_x=0, raw_z=9810) and banking right (raw_y=-3355, ~20 deg)
        fs::write(dev.join("in_accel_x_raw"), "0\n").unwrap();
        fs::write(dev.join("in_accel_y_raw"), "-3355\n").unwrap();
        fs::write(dev.join("in_accel_z_raw"), "9218\n").unwrap();

        // Give background thread enough ticks to converge through LPF
        for _ in 0..25 {
            thread::sleep(Duration::from_millis(15));
            imu.poll();
        }
        let (roll, pitch) = imu.tilt_sticks();
        assert!(pitch > 0.70, "tilting forward 30 deg should command strong positive pitch, got {pitch}");
        assert!(roll > 0.45, "banking right 20 deg should command positive roll, got {roll}");

        // Re-centering makes current pose level again
        imu.recenter();
        let (r_after, p_after) = imu.tilt_sticks();
        assert_eq!(r_after, 0.0);
        assert_eq!(p_after, 0.0);

        drop(imu);
        let _ = fs::remove_dir_all(&root);
    }

    #[test]
    fn test_imu_unavailable_on_empty_iio_or_lite() {
        let empty = make_temp_iio_dir("empty");
        let imu_none = ImuSensor::probe_from_root(&empty, false);
        assert!(!imu_none.is_available());
        assert!(!imu_none.is_lite);
        let _ = fs::remove_dir_all(&empty);

        // Even if an IIO root exists, CardputerZero Lite disables onboard IMU mode
        let root = make_temp_iio_dir("lite");
        write_fake_bmi270(&root, 0, 0, 9810);
        let imu_lite = ImuSensor::probe_from_root(&root, true);
        assert!(!imu_lite.is_available());
        assert!(imu_lite.is_lite);
        let _ = fs::remove_dir_all(&root);
    }
}
