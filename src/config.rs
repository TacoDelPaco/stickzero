//! Configuration, quad presets, rates, themes, and persistent best-lap storage.

use std::fs;
use std::path::PathBuf;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum QuadClass {
    Racer5,
    Freestyle5,
    Toothpick3,
    Whoop65,
}

impl QuadClass {
    pub const ALL: [QuadClass; 4] = [
        QuadClass::Racer5,
        QuadClass::Freestyle5,
        QuadClass::Toothpick3,
        QuadClass::Whoop65,
    ];

    pub fn name(self) -> &'static str {
        match self {
            Self::Racer5 => "5\" Racer",
            Self::Freestyle5 => "5\" Freestyle",
            Self::Toothpick3 => "3\" Toothpick",
            Self::Whoop65 => "65mm Whoop",
        }
    }

    pub fn next(self) -> Self {
        match self {
            Self::Racer5 => Self::Freestyle5,
            Self::Freestyle5 => Self::Toothpick3,
            Self::Toothpick3 => Self::Whoop65,
            Self::Whoop65 => Self::Racer5,
        }
    }

    /// Returns (max_thrust_accel m/s^2, drag_coeff, motor_tau s, hover_throttle, cells)
    pub fn params(self) -> QuadParams {
        match self {
            Self::Racer5 => QuadParams {
                max_accel: 76.0,
                drag: 0.036,
                motor_tau: 0.014,
                idle_accel: 2.2,
                cells: 6,
                mah: 1300.0,
                full_amp: 95.0,
            },
            Self::Freestyle5 => QuadParams {
                max_accel: 62.0,
                drag: 0.040,
                motor_tau: 0.018,
                idle_accel: 2.0,
                cells: 6,
                mah: 1300.0,
                full_amp: 78.0,
            },
            Self::Toothpick3 => QuadParams {
                max_accel: 54.0,
                drag: 0.048,
                motor_tau: 0.015,
                idle_accel: 1.8,
                cells: 3,
                mah: 450.0,
                full_amp: 28.0,
            },
            Self::Whoop65 => QuadParams {
                max_accel: 34.0,
                drag: 0.075,
                motor_tau: 0.022,
                idle_accel: 1.5,
                cells: 1,
                mah: 300.0,
                full_amp: 12.0,
            },
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct QuadParams {
    pub max_accel: f32,
    pub drag: f32,
    pub motor_tau: f32,
    pub idle_accel: f32,
    pub cells: u8,
    pub mah: f32,
    pub full_amp: f32,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FlightMode {
    Acro,
    Angle,
}

impl FlightMode {
    pub fn name(self) -> &'static str {
        match self {
            Self::Acro => "ACRO",
            Self::Angle => "ANGLE",
        }
    }

    pub fn toggle(self) -> Self {
        match self {
            Self::Acro => Self::Angle,
            Self::Angle => Self::Acro,
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum RatesPreset {
    Soft,
    Normal,
    Fast,
}

impl RatesPreset {
    pub fn name(self) -> &'static str {
        match self {
            Self::Soft => "Soft",
            Self::Normal => "Normal",
            Self::Fast => "Fast",
        }
    }

    pub fn next(self) -> Self {
        match self {
            Self::Soft => Self::Normal,
            Self::Normal => Self::Fast,
            Self::Fast => Self::Soft,
        }
    }

    /// Betaflight Actual rates: (center_sens deg/s, max_rate deg/s, expo 0..1)
    pub fn actual_rates(self) -> (f32, f32, f32) {
        match self {
            Self::Soft => (120.0, 520.0, 0.50),
            Self::Normal => (180.0, 680.0, 0.55),
            Self::Fast => (220.0, 860.0, 0.60),
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum VisualTheme {
    Summer,
    Evening,
    Night,
    Wireframe,
}

impl VisualTheme {
    pub fn name(self) -> &'static str {
        match self {
            Self::Summer => "Summer",
            Self::Evening => "Evening",
            Self::Night => "Night",
            Self::Wireframe => "Wireframe",
        }
    }

    pub fn next(self) -> Self {
        match self {
            Self::Summer => Self::Evening,
            Self::Evening => Self::Night,
            Self::Night => Self::Wireframe,
            Self::Wireframe => Self::Summer,
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum GameMode {
    Race,
    Freestyle,
}

impl GameMode {
    pub fn name(self) -> &'static str {
        match self {
            Self::Race => "Time Trial",
            Self::Freestyle => "Freestyle",
        }
    }

    pub fn toggle(self) -> Self {
        match self {
            Self::Race => Self::Freestyle,
            Self::Freestyle => Self::Race,
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum StickMode {
    Mode2, // Left: Thr/Yaw, Right: Pitch/Roll
    Mode1, // Left: Pitch/Yaw, Right: Thr/Roll
}

impl StickMode {
    pub fn name(self) -> &'static str {
        match self {
            Self::Mode2 => "Mode 2",
            Self::Mode1 => "Mode 1",
        }
    }

    pub fn toggle(self) -> Self {
        match self {
            Self::Mode2 => Self::Mode1,
            Self::Mode1 => Self::Mode2,
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ThrottleMode {
    /// RC transmitter gimbal: bottom = 0%, top = 100%
    RcFull,
    /// Spring-centered gamepad stick: center = hover (~30%), up = 100%, down = 0%
    GamepadCenter,
}

impl ThrottleMode {
    pub fn name(self) -> &'static str {
        match self {
            Self::RcFull => "RC Full",
            Self::GamepadCenter => "Pad Center",
        }
    }

    pub fn toggle(self) -> Self {
        match self {
            Self::RcFull => Self::GamepadCenter,
            Self::GamepadCenter => Self::RcFull,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Config {
    pub quad: QuadClass,
    pub flight_mode: FlightMode,
    pub rates: RatesPreset,
    pub camera_tilt_deg: u8,
    pub theme: VisualTheme,
    pub game_mode: GameMode,
    pub track_index: usize,
    pub stick_mode: StickMode,
    pub throttle_mode: ThrottleMode,
    pub invert_pitch: bool,
    pub invert_throttle: bool,
    pub show_sticks: bool,
    pub battery_sim: bool,
    pub best_laps_ms: [u32; 4],
}

impl Default for Config {
    fn default() -> Self {
        Self {
            quad: QuadClass::Racer5,
            flight_mode: FlightMode::Acro,
            rates: RatesPreset::Normal,
            camera_tilt_deg: 25,
            theme: VisualTheme::Summer,
            game_mode: GameMode::Race,
            track_index: 0,
            stick_mode: StickMode::Mode2,
            throttle_mode: ThrottleMode::RcFull,
            invert_pitch: false,
            invert_throttle: false,
            show_sticks: true,
            battery_sim: true,
            best_laps_ms: [0; 4],
        }
    }
}

impl Config {
    fn config_path() -> PathBuf {
        let home = std::env::var("HOME").unwrap_or_else(|_| "/tmp".to_string());
        PathBuf::from(home).join(".config").join("stickzero").join("config.txt")
    }

    pub fn load() -> Self {
        let mut cfg = Self::default();
        let path = Self::config_path();
        let Ok(text) = fs::read_to_string(&path) else {
            return cfg;
        };
        for line in text.lines() {
            let Some((k, v)) = line.split_once('=') else { continue };
            let k = k.trim();
            let v = v.trim();
            match k {
                "quad" => cfg.quad = match v {
                    "freestyle5" => QuadClass::Freestyle5,
                    "toothpick3" => QuadClass::Toothpick3,
                    "whoop65" => QuadClass::Whoop65,
                    _ => QuadClass::Racer5,
                },
                "flight_mode" => cfg.flight_mode = if v == "angle" { FlightMode::Angle } else { FlightMode::Acro },
                "rates" => cfg.rates = match v {
                    "soft" => RatesPreset::Soft,
                    "fast" => RatesPreset::Fast,
                    _ => RatesPreset::Normal,
                },
                "camera_tilt" => if let Ok(d) = v.parse::<u8>() {
                    cfg.camera_tilt_deg = d.clamp(0, 55);
                },
                "theme" => cfg.theme = match v {
                    "evening" => VisualTheme::Evening,
                    "night" => VisualTheme::Night,
                    "wireframe" => VisualTheme::Wireframe,
                    _ => VisualTheme::Summer,
                },
                "game_mode" => cfg.game_mode = if v == "freestyle" { GameMode::Freestyle } else { GameMode::Race },
                "track" => if let Ok(i) = v.parse::<usize>() {
                    cfg.track_index = i.min(3);
                },
                "stick_mode" => cfg.stick_mode = if v == "mode1" { StickMode::Mode1 } else { StickMode::Mode2 },
                "throttle_mode" => cfg.throttle_mode = if v == "pad" { ThrottleMode::GamepadCenter } else { ThrottleMode::RcFull },
                "invert_pitch" => cfg.invert_pitch = v == "1",
                "invert_throttle" => cfg.invert_throttle = v == "1",
                "show_sticks" => cfg.show_sticks = v != "0",
                "battery_sim" => cfg.battery_sim = v != "0",
                "best0" => if let Ok(ms) = v.parse() { cfg.best_laps_ms[0] = ms; },
                "best1" => if let Ok(ms) = v.parse() { cfg.best_laps_ms[1] = ms; },
                "best2" => if let Ok(ms) = v.parse() { cfg.best_laps_ms[2] = ms; },
                "best3" => if let Ok(ms) = v.parse() { cfg.best_laps_ms[3] = ms; },
                _ => {}
            }
        }
        cfg
    }

    pub fn save(&self) {
        let path = Self::config_path();
        if let Some(parent) = path.parent() {
            let _ = fs::create_dir_all(parent);
        }
        let quad = match self.quad {
            QuadClass::Racer5 => "racer5",
            QuadClass::Freestyle5 => "freestyle5",
            QuadClass::Toothpick3 => "toothpick3",
            QuadClass::Whoop65 => "whoop65",
        };
        let fm = match self.flight_mode {
            FlightMode::Acro => "acro",
            FlightMode::Angle => "angle",
        };
        let rates = match self.rates {
            RatesPreset::Soft => "soft",
            RatesPreset::Normal => "normal",
            RatesPreset::Fast => "fast",
        };
        let theme = match self.theme {
            VisualTheme::Summer => "summer",
            VisualTheme::Evening => "evening",
            VisualTheme::Night => "night",
            VisualTheme::Wireframe => "wireframe",
        };
        let gm = match self.game_mode {
            GameMode::Race => "race",
            GameMode::Freestyle => "freestyle",
        };
        let sm = match self.stick_mode {
            StickMode::Mode2 => "mode2",
            StickMode::Mode1 => "mode1",
        };
        let tm = match self.throttle_mode {
            ThrottleMode::RcFull => "rc",
            ThrottleMode::GamepadCenter => "pad",
        };
        let out = format!(
            "quad={quad}\nflight_mode={fm}\nrates={rates}\ncamera_tilt={}\ntheme={theme}\ngame_mode={gm}\ntrack={}\nstick_mode={sm}\nthrottle_mode={tm}\ninvert_pitch={}\ninvert_throttle={}\nshow_sticks={}\nbattery_sim={}\nbest0={}\nbest1={}\nbest2={}\nbest3={}\n",
            self.camera_tilt_deg,
            self.track_index,
            self.invert_pitch as u8,
            self.invert_throttle as u8,
            self.show_sticks as u8,
            self.battery_sim as u8,
            self.best_laps_ms[0],
            self.best_laps_ms[1],
            self.best_laps_ms[2],
            self.best_laps_ms[3],
        );
        let _ = fs::write(path, out);
    }
}
