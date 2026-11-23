//! Configuration, BetaFPV whoop & quad presets (Air75 II, Meteor75 Pro II),
//! Lava II 580mAh / 680mAh battery packs with real weight/TWR physics,
//! custom Betaflight Actual rates (default 100/480/0.20), prop wash,
//! radio AUX switch/button bindings (Arm, Flight Mode, Turtle Mode), and OSD/graphics settings.

use std::fs;
use std::path::PathBuf;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum QuadClass {
    Air75II,
    Meteor75ProII,
    Whoop65,
    Toothpick3,
    Freestyle5,
    Racer5,
}

impl QuadClass {
    pub fn name(self) -> &'static str {
        match self {
            Self::Air75II => "Air75 II 75mm",
            Self::Meteor75ProII => "Meteor75 Pro II",
            Self::Whoop65 => "65mm Whoop",
            Self::Toothpick3 => "3\" Toothpick",
            Self::Freestyle5 => "5\" Freestyle",
            Self::Racer5 => "5\" Racer",
        }
    }

    pub fn craft_name(self) -> &'static str {
        match self {
            Self::Air75II => "AIR75 II",
            Self::Meteor75ProII => "METEOR75",
            Self::Whoop65 => "WHOOP65",
            Self::Toothpick3 => "TPICK 3",
            Self::Freestyle5 => "FREE 5",
            Self::Racer5 => "RACER 5",
        }
    }

    pub fn next(self) -> Self {
        match self {
            Self::Air75II => Self::Meteor75ProII,
            Self::Meteor75ProII => Self::Whoop65,
            Self::Whoop65 => Self::Toothpick3,
            Self::Toothpick3 => Self::Freestyle5,
            Self::Freestyle5 => Self::Racer5,
            Self::Racer5 => Self::Air75II,
        }
    }

    pub fn params(self) -> QuadParams {
        match self {
            // BetaFPV Air75 II Freestyle 75mm (ultra-light ~21g dry, 40mm props, realistic 1S ~3.0:1 TWR)
            Self::Air75II => QuadParams {
                max_accel: 30.0,
                drag: 0.030,
                motor_tau: 0.015,
                idle_accel: 0.7,
                dry_mass_g: 21.0,
                ref_batt_g: 13.5,
                mah: 450.0,
                full_amp: 15.5,
            },
            // BetaFPV Meteor75 Pro II 75mm (1S, 45mm props, ~31g dry, heavier carry & momentum)
            Self::Meteor75ProII => QuadParams {
                max_accel: 27.5,
                drag: 0.027,
                motor_tau: 0.017,
                idle_accel: 0.7,
                dry_mass_g: 31.0,
                ref_batt_g: 13.5,
                mah: 550.0,
                full_amp: 17.5,
            },
            Self::Whoop65 => QuadParams {
                max_accel: 24.5,
                drag: 0.040,
                motor_tau: 0.022,
                idle_accel: 0.6,
                dry_mass_g: 18.0,
                ref_batt_g: 8.5,
                mah: 300.0,
                full_amp: 12.0,
            },
            Self::Toothpick3 => QuadParams {
                max_accel: 50.0,
                drag: 0.042,
                motor_tau: 0.015,
                idle_accel: 1.4,
                dry_mass_g: 68.0,
                ref_batt_g: 36.0,
                mah: 450.0,
                full_amp: 28.0,
            },
            Self::Freestyle5 => QuadParams {
                max_accel: 60.0,
                drag: 0.036,
                motor_tau: 0.018,
                idle_accel: 1.8,
                dry_mass_g: 410.0,
                ref_batt_g: 220.0,
                mah: 1300.0,
                full_amp: 78.0,
            },
            Self::Racer5 => QuadParams {
                max_accel: 74.0,
                drag: 0.032,
                motor_tau: 0.014,
                idle_accel: 2.0,
                dry_mass_g: 320.0,
                ref_batt_g: 210.0,
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
    pub dry_mass_g: f32,
    pub ref_batt_g: f32,
    pub mah: f32,
    pub full_amp: f32,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum BatteryPack {
    LavaII580,
    LavaII680,
    Stock,
    Off,
}

impl BatteryPack {
    pub fn name(self) -> &'static str {
        match self {
            Self::LavaII580 => "Lava II 580mAh",
            Self::LavaII680 => "Lava II 680mAh",
            Self::Stock => "Stock Pack",
            Self::Off => "Off (Infinite)",
        }
    }

    pub fn next(self) -> Self {
        match self {
            Self::LavaII580 => Self::LavaII680,
            Self::LavaII680 => Self::Stock,
            Self::Stock => Self::Off,
            Self::Off => Self::LavaII580,
        }
    }

    /// Returns (capacity_mah, mass_g, sag_factor) where lower sag_factor = higher C-rating.
    pub fn specs(self, quad: &QuadParams) -> (f32, f32, f32) {
        match self {
            // BetaFPV LAVA II 1S 580mAh folded cell (~13.5g, high C-rating, low sag)
            Self::LavaII580 => (580.0, 13.5, 0.24),
            // BetaFPV LAVA II 1S 680mAh folded cell (~16.2g, longer flight, slightly heavier)
            Self::LavaII680 => (680.0, 16.2, 0.22),
            Self::Stock => (quad.mah, quad.ref_batt_g, 0.34),
            Self::Off => (quad.mah, quad.ref_batt_g, 0.0),
        }
    }
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

    /// Betaflight Actual rate presets: (center_sens deg/s, max_rate deg/s, expo %)
    pub fn defaults(self) -> (u16, u16, u8) {
        match self {
            Self::Soft => (100, 480, 20),
            Self::Normal => (160, 640, 45),
            Self::Fast => (220, 860, 55),
            Self::Custom => (100, 480, 20),
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

/// Binding for a radio controller AUX switch (EV_ABS axis) or button (EV_KEY code).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum AuxBinding {
    None,
    /// Switch on an EV_ABS axis (e.g., EdgeTX CH5..CH8). Active when normalized value > 0.35 (or < -0.35 if inverted).
    Axis { code: u16, invert: bool },
    /// Momentary/latching button on an EV_KEY code (EdgeTX HID button or gamepad button).
    Button { code: u16 },
}

impl AuxBinding {
    pub fn short_label(self) -> String {
        match self {
            Self::None => "-".to_string(),
            Self::Axis { code, invert } => {
                if invert { format!("A{code}-") } else { format!("A{code}+") }
            }
            Self::Button { code } => format!("B{code}"),
        }
    }

    pub fn serialize(self) -> String {
        match self {
            Self::None => "none".to_string(),
            Self::Axis { code, invert } => format!("axis:{code}:{}", invert as u8),
            Self::Button { code } => format!("btn:{code}"),
        }
    }

    pub fn parse(s: &str) -> Self {
        let mut parts = s.split(':');
        match parts.next() {
            Some("axis") => {
                if let Some(Ok(code)) = parts.next().map(|x| x.parse::<u16>()) {
                    let invert = parts.next() == Some("1");
                    return Self::Axis { code, invert };
                }
            }
            Some("btn") => {
                if let Some(Ok(code)) = parts.next().map(|x| x.parse::<u16>()) {
                    return Self::Button { code };
                }
            }
            _ => {}
        }
        Self::None
    }
}

#[derive(Clone, Debug)]
pub struct Config {
    pub quad: QuadClass,
    pub battery_pack: BatteryPack,
    pub prop_wash: bool,
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
    /// Graphics & OSD settings
    pub detail_high: bool,
    pub sky_detail: bool,
    pub analog_filter: bool,
    pub sound_on: bool,
    pub debug_osd: bool,
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
    /// Radio AUX switch / button bindings
    pub aux_arm: AuxBinding,
    pub aux_mode: AuxBinding,
    pub aux_turtle: AuxBinding,
    pub show_sticks: bool,
    pub best_laps_ms: [u32; 4],
}

impl Default for Config {
    fn default() -> Self {
        Self {
            quad: QuadClass::Air75II,
            battery_pack: BatteryPack::LavaII580,
            prop_wash: true,
            flight_mode: FlightMode::Acro,
            rates: RatesPreset::Soft,
            rate_center: 100,
            rate_max: 480,
            rate_expo: 20,
            camera_tilt_deg: 25,
            theme: VisualTheme::Summer,
            detail_high: true,
            sky_detail: true,
            analog_filter: true,
            sound_on: true,
            debug_osd: false,
            fov_deg: 90,
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
            aux_arm: AuxBinding::None,
            aux_mode: AuxBinding::None,
            aux_turtle: AuxBinding::None,
            show_sticks: true,
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
                    "meteor75proii" | "meteor85proii" => QuadClass::Meteor75ProII,
                    "whoop65" => QuadClass::Whoop65,
                    "toothpick3" => QuadClass::Toothpick3,
                    "freestyle5" => QuadClass::Freestyle5,
                    "racer5" => QuadClass::Racer5,
                    _ => QuadClass::Air75II,
                },
                "battery_pack" => cfg.battery_pack = match v {
                    "lava580" => BatteryPack::LavaII580,
                    "lava680" => BatteryPack::LavaII680,
                    "stock" => BatteryPack::Stock,
                    "off" => BatteryPack::Off,
                    _ => BatteryPack::LavaII580,
                },
                "prop_wash" => cfg.prop_wash = v != "0",
                "flight_mode" => cfg.flight_mode = if v == "angle" { FlightMode::Angle } else { FlightMode::Acro },
                "rates" => cfg.rates = match v {
                    "soft" => RatesPreset::Soft,
                    "normal" => RatesPreset::Normal,
                    "fast" => RatesPreset::Fast,
                    "custom" => RatesPreset::Custom,
                    _ => RatesPreset::Soft,
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
                "analog_filter" => cfg.analog_filter = v != "0",
                "sound_on" => cfg.sound_on = v != "0",
                "debug_osd" => cfg.debug_osd = v == "1",
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
                "aux_arm" => cfg.aux_arm = AuxBinding::parse(v),
                "aux_mode" => cfg.aux_mode = AuxBinding::parse(v),
                "aux_turtle" => cfg.aux_turtle = AuxBinding::parse(v),
                "show_sticks" => cfg.show_sticks = v != "0",
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
            QuadClass::Meteor75ProII => "meteor75proii",
            QuadClass::Whoop65 => "whoop65",
            QuadClass::Toothpick3 => "toothpick3",
            QuadClass::Freestyle5 => "freestyle5",
            QuadClass::Racer5 => "racer5",
        };
        let batt = match self.battery_pack {
            BatteryPack::LavaII580 => "lava580",
            BatteryPack::LavaII680 => "lava680",
            BatteryPack::Stock => "stock",
            BatteryPack::Off => "off",
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
            "quad={quad}\nbattery_pack={batt}\nprop_wash={}\nflight_mode={fm}\nrates={rates}\nrate_center={}\nrate_max={}\nrate_expo={}\ncamera_tilt={}\ntheme={theme}\ndetail_high={}\nsky_detail={}\nanalog_filter={}\nsound_on={}\ndebug_osd={}\nfov_deg={}\ngame_mode={gm}\ntrack={}\nstick_mode={sm}\nthrottle_mode={tm}\ninvert_pitch={}\ninvert_throttle={}\ninvert_yaw={}\ninvert_roll={}\naxis_thr={}\naxis_yaw={}\naxis_pit={}\naxis_rol={}\naux_arm={}\naux_mode={}\naux_turtle={}\nshow_sticks={}\nbest0={}\nbest1={}\nbest2={}\nbest3={}\n",
            self.prop_wash as u8,
            self.rate_center,
            self.rate_max,
            self.rate_expo,
            self.camera_tilt_deg,
            self.detail_high as u8,
            self.sky_detail as u8,
            self.analog_filter as u8,
            self.sound_on as u8,
            self.debug_osd as u8,
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
            self.aux_arm.serialize(),
            self.aux_mode.serialize(),
            self.aux_turtle.serialize(),
            self.show_sticks as u8,
            self.best_laps_ms[0],
            self.best_laps_ms[1],
            self.best_laps_ms[2],
            self.best_laps_ms[3],
        );
        let _ = fs::write(path, out);
    }
}
