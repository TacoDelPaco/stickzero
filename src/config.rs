//! Configuration, quad presets (including BetaFPV Air75 II & Meteor85 Pro II),
//! custom Betaflight Actual rates, graphics settings, and persistent storage.

use std::fs;
use std::path::PathBuf;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum QuadClass {
    Air75II,
    Meteor85ProII,
    Whoop65,
    Toothpick3,
    Freestyle5,
    Racer5,
}

impl QuadClass {
    pub fn name(self) -> &'static str {
        match self {
            Self::Air75II => "Air75 II 75mm",
            Self::Meteor85ProII => "Meteor85 Pro II",
            Self::Whoop65 => "65mm Whoop",
            Self::Toothpick3 => "3\" Toothpick",
            Self::Freestyle5 => "5\" Freestyle",
            Self::Racer5 => "5\" Racer",
        }
    }

    pub fn next(self) -> Self {
        match self {
            Self::Air75II => Self::Meteor85ProII,
            Self::Meteor85ProII => Self::Whoop65,
            Self::Whoop65 => Self::Toothpick3,
            Self::Toothpick3 => Self::Freestyle5,
            Self::Freestyle5 => Self::Racer5,
            Self::Racer5 => Self::Air75II,
        }
    }

    pub fn params(self) -> QuadParams {
        match self {
            // BetaFPV Air75 II Freestyle 75mm (1S 450mAh, ultra-light nimble freestyle whoop)
            Self::Air75II => QuadParams {
                max_accel: 42.0,
                drag: 0.058,
                motor_tau: 0.015,
                idle_accel: 1.6,
                mah: 450.0,
                full_amp: 16.0,
            },
            // BetaFPV Meteor85 Pro II 85mm (2S 450mAh, punchy outdoor/freestyle whoop)
            Self::Meteor85ProII => QuadParams {
                max_accel: 50.0,
                drag: 0.050,
                motor_tau: 0.016,
                idle_accel: 1.8,
                mah: 450.0,
                full_amp: 22.0,
            },
            Self::Whoop65 => QuadParams {
                max_accel: 34.0,
                drag: 0.075,
                motor_tau: 0.022,
                idle_accel: 1.5,
                mah: 300.0,
                full_amp: 12.0,
            },
            Self::Toothpick3 => QuadParams {
                max_accel: 54.0,
                drag: 0.048,
                motor_tau: 0.015,
                idle_accel: 1.8,
                mah: 450.0,
                full_amp: 28.0,
            },
            Self::Freestyle5 => QuadParams {
                max_accel: 62.0,
                drag: 0.040,
                motor_tau: 0.018,
                idle_accel: 2.0,
                mah: 1300.0,
                full_amp: 78.0,
            },
            Self::Racer5 => QuadParams {
                max_accel: 76.0,
                drag: 0.036,
                motor_tau: 0.014,
                idle_accel: 2.2,
                mah: 1300.0,
                full_amp: 95.0,
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
    Custom,
}

impl RatesPreset {
    pub fn name(self) -> &'static str {
        match self {
            Self::Soft => "Soft",
            Self::Normal => "Normal",
            Self::Fast => "Fast",
            Self::Custom => "Custom",
        }
    }

    pub fn next_preset(self) -> Self {
        match self {
            Self::Soft => Self::Normal,
            Self::Normal => Self::Fast,
            Self::Fast | Self::Custom => Self::Soft,
        }
    }

    /// Betaflight Actual rate starting points: (center_sens deg/s, max_rate deg/s, expo %)
    pub fn defaults(self) -> (u16, u16, u8) {
        match self {
            Self::Soft => (120, 520, 50),
            Self::Normal => (180, 680, 55),
            Self::Fast => (220, 860, 60),
            Self::Custom => (180, 680, 55),
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
    Mode2,
    Mode1,
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
    RcFull,
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
    /// Betaflight Actual Center Sensitivity (deg/s, 50..=400)
    pub rate_center: u16,
    /// Betaflight Actual Max Rate (deg/s, 300..=1200)
    pub rate_max: u16,
    /// Betaflight Actual Expo (0..=90, representing 0.00..=0.90)
    pub rate_expo: u8,
    pub camera_tilt_deg: u8,
    pub theme: VisualTheme,
    /// Graphics settings
    pub detail_high: bool,
    pub sky_detail: bool,
    pub fov_deg: u8,
    pub game_mode: GameMode,
    pub track_index: usize,
    pub stick_mode: StickMode,
    pub throttle_mode: ThrottleMode,
    pub invert_pitch: bool,
    pub invert_throttle: bool,
    pub invert_yaw: bool,
    pub invert_roll: bool,
    pub axis_thr: Option<u16>,
    pub axis_yaw: Option<u16>,
    pub axis_pit: Option<u16>,
    pub axis_rol: Option<u16>,
    pub show_sticks: bool,
    pub battery_sim: bool,
    pub best_laps_ms: [u32; 4],
}

impl Default for Config {
    fn default() -> Self {
        Self {
            quad: QuadClass::Air75II,
            flight_mode: FlightMode::Acro,
            rates: RatesPreset::Normal,
            rate_center: 180,
            rate_max: 680,
            rate_expo: 55,
            camera_tilt_deg: 25,
            theme: VisualTheme::Summer,
            detail_high: true,
            sky_detail: true,
            fov_deg: 95,
            game_mode: GameMode::Freestyle,
            track_index: 0,
            stick_mode: StickMode::Mode2,
            throttle_mode: ThrottleMode::RcFull,
            invert_pitch: false,
            invert_throttle: false,
            invert_yaw: false,
            invert_roll: false,
            axis_thr: None,
            axis_yaw: None,
            axis_pit: None,
            axis_rol: None,
            show_sticks: true,
            battery_sim: true,
            best_laps_ms: [0; 4],
        }
    }
}

impl Config {
    pub fn apply_rate_preset(&mut self, preset: RatesPreset) {
        self.rates = preset;
        let (c, m, e) = preset.defaults();
        self.rate_center = c;
        self.rate_max = m;
        self.rate_expo = e;
    }

    pub fn actual_rates(&self) -> (f32, f32, f32) {
        (
            self.rate_center as f32,
            self.rate_max as f32,
            (self.rate_expo as f32) / 100.0,
        )
    }

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
                    "air75ii" => QuadClass::Air75II,
                    "meteor85proii" => QuadClass::Meteor85ProII,
                    "whoop65" => QuadClass::Whoop65,
                    "toothpick3" => QuadClass::Toothpick3,
                    "freestyle5" => QuadClass::Freestyle5,
                    "racer5" => QuadClass::Racer5,
                    _ => QuadClass::Air75II,
                },
                "flight_mode" => cfg.flight_mode = if v == "angle" { FlightMode::Angle } else { FlightMode::Acro },
                "rates" => cfg.rates = match v {
                    "soft" => RatesPreset::Soft,
                    "fast" => RatesPreset::Fast,
                    "custom" => RatesPreset::Custom,
                    _ => RatesPreset::Normal,
                },
                "rate_center" => if let Ok(n) = v.parse::<u16>() { cfg.rate_center = n.clamp(50, 400); },
                "rate_max" => if let Ok(n) = v.parse::<u16>() { cfg.rate_max = n.clamp(300, 1200); },
                "rate_expo" => if let Ok(n) = v.parse::<u8>() { cfg.rate_expo = n.clamp(0, 90); },
                "camera_tilt" => if let Ok(d) = v.parse::<u8>() { cfg.camera_tilt_deg = d.clamp(0, 55); },
                "theme" => cfg.theme = match v {
                    "evening" => VisualTheme::Evening,
                    "night" => VisualTheme::Night,
                    "wireframe" => VisualTheme::Wireframe,
                    _ => VisualTheme::Summer,
                },
                "detail_high" => cfg.detail_high = v != "0",
                "sky_detail" => cfg.sky_detail = v != "0",
                "fov_deg" => if let Ok(f) = v.parse::<u8>() { cfg.fov_deg = f.clamp(70, 115); },
                "game_mode" => cfg.game_mode = if v == "race" { GameMode::Race } else { GameMode::Freestyle },
                "track" => if let Ok(i) = v.parse::<usize>() { cfg.track_index = i.min(3); },
                "stick_mode" => cfg.stick_mode = if v == "mode1" { StickMode::Mode1 } else { StickMode::Mode2 },
                "throttle_mode" => cfg.throttle_mode = if v == "pad" { ThrottleMode::GamepadCenter } else { ThrottleMode::RcFull },
                "invert_pitch" => cfg.invert_pitch = v == "1",
                "invert_throttle" => cfg.invert_throttle = v == "1",
                "invert_yaw" => cfg.invert_yaw = v == "1",
                "invert_roll" => cfg.invert_roll = v == "1",
                "axis_thr" => cfg.axis_thr = v.parse::<u16>().ok(),
                "axis_yaw" => cfg.axis_yaw = v.parse::<u16>().ok(),
                "axis_pit" => cfg.axis_pit = v.parse::<u16>().ok(),
                "axis_rol" => cfg.axis_rol = v.parse::<u16>().ok(),
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
            QuadClass::Air75II => "air75ii",
            QuadClass::Meteor85ProII => "meteor85proii",
            QuadClass::Whoop65 => "whoop65",
            QuadClass::Toothpick3 => "toothpick3",
            QuadClass::Freestyle5 => "freestyle5",
            QuadClass::Racer5 => "racer5",
        };
        let fm = match self.flight_mode {
            FlightMode::Acro => "acro",
            FlightMode::Angle => "angle",
        };
        let rates = match self.rates {
            RatesPreset::Soft => "soft",
            RatesPreset::Normal => "normal",
            RatesPreset::Fast => "fast",
            RatesPreset::Custom => "custom",
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
        let fmt_opt = |o: Option<u16>| o.map(|x| x.to_string()).unwrap_or_else(|| "auto".to_string());
        let out = format!(
            "quad={quad}\nflight_mode={fm}\nrates={rates}\nrate_center={}\nrate_max={}\nrate_expo={}\ncamera_tilt={}\ntheme={theme}\ndetail_high={}\nsky_detail={}\nfov_deg={}\ngame_mode={gm}\ntrack={}\nstick_mode={sm}\nthrottle_mode={tm}\ninvert_pitch={}\ninvert_throttle={}\ninvert_yaw={}\ninvert_roll={}\naxis_thr={}\naxis_yaw={}\naxis_pit={}\naxis_rol={}\nshow_sticks={}\nbattery_sim={}\nbest0={}\nbest1={}\nbest2={}\nbest3={}\n",
            self.rate_center,
            self.rate_max,
            self.rate_expo,
            self.camera_tilt_deg,
            self.detail_high as u8,
            self.sky_detail as u8,
            self.fov_deg,
            self.track_index,
            self.invert_pitch as u8,
            self.invert_throttle as u8,
            self.invert_yaw as u8,
            self.invert_roll as u8,
            fmt_opt(self.axis_thr),
            fmt_opt(self.axis_yaw),
            fmt_opt(self.axis_pit),
            fmt_opt(self.axis_rol),
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
