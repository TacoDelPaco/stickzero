//! Configuration, BetaFPV whoop & quad presets (Air75 II, Meteor75 Pro II),
//! optional "The Zone"-style Drone Tuning (Thrust, Weight, Air Drag, Rotor Grip, Motor Resp, Thr Expo),
//! Lava II 580mAh / 680mAh battery packs, Prop Wash (Off/Stock/Max),
//! Acro / Horizon / Angle flight modes, custom Betaflight Actual rates, UI Theme (Classic/Monokai),
//! and radio AUX switch/button bindings.

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

    pub fn prev(self) -> Self {
        match self {
            Self::Air75II => Self::Racer5,
            Self::Meteor75ProII => Self::Air75II,
            Self::Whoop65 => Self::Meteor75ProII,
            Self::Toothpick3 => Self::Whoop65,
            Self::Freestyle5 => Self::Toothpick3,
            Self::Racer5 => Self::Freestyle5,
        }
    }

    pub fn params(self) -> QuadParams {
        match self {
            // BetaFPV Air75 II Freestyle 75mm (~21g dry, 40mm props, nimble 1S ~3.4:1 TWR)
            Self::Air75II => QuadParams {
                max_accel: 35.5,
                drag: 0.029,
                grip: 0.34,
                motor_tau: 0.015,
                idle_accel: 0.75,
                dry_mass_g: 21.0,
                ref_batt_g: 13.5,
                mah: 450.0,
                full_amp: 15.5,
            },
            // BetaFPV Meteor75 Pro II 75mm (1S, 45mm props, ~31g dry, planted carry & stability)
            Self::Meteor75ProII => QuadParams {
                max_accel: 33.0,
                drag: 0.026,
                grip: 0.38,
                motor_tau: 0.017,
                idle_accel: 0.75,
                dry_mass_g: 31.0,
                ref_batt_g: 13.5,
                mah: 550.0,
                full_amp: 17.5,
            },
            Self::Whoop65 => QuadParams {
                max_accel: 28.0,
                drag: 0.038,
                grip: 0.42,
                motor_tau: 0.021,
                idle_accel: 0.65,
                dry_mass_g: 18.0,
                ref_batt_g: 8.5,
                mah: 300.0,
                full_amp: 12.0,
            },
            Self::Toothpick3 => QuadParams {
                max_accel: 54.0,
                drag: 0.036,
                grip: 0.28,
                motor_tau: 0.014,
                idle_accel: 1.4,
                dry_mass_g: 68.0,
                ref_batt_g: 36.0,
                mah: 450.0,
                full_amp: 28.0,
            },
            Self::Freestyle5 => QuadParams {
                max_accel: 68.0,
                drag: 0.032,
                grip: 0.26,
                motor_tau: 0.016,
                idle_accel: 1.8,
                dry_mass_g: 410.0,
                ref_batt_g: 220.0,
                mah: 1300.0,
                full_amp: 78.0,
            },
            Self::Racer5 => QuadParams {
                max_accel: 82.0,
                drag: 0.028,
                grip: 0.24,
                motor_tau: 0.012,
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
    /// In-plane rotor H-force grip coefficient
    pub grip: f32,
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

    pub fn short_name(self) -> &'static str {
        match self {
            Self::LavaII580 => "Lava580",
            Self::LavaII680 => "Lava680",
            Self::Stock => "Stock",
            Self::Off => "Inf",
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

    pub fn prev(self) -> Self {
        match self {
            Self::LavaII580 => Self::Off,
            Self::LavaII680 => Self::LavaII580,
            Self::Stock => Self::LavaII680,
            Self::Off => Self::Stock,
        }
    }

    /// Returns (capacity_mah, mass_g, sag_factor) where lower sag_factor = higher C-rating.
    pub fn specs(self, quad: &QuadParams) -> (f32, f32, f32) {
        match self {
            Self::LavaII580 => (580.0, 13.5, 0.24),
            Self::LavaII680 => (680.0, 16.2, 0.22),
            Self::Stock => (quad.mah, quad.ref_batt_g, 0.34),
            Self::Off => (quad.mah, quad.ref_batt_g, 0.0),
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum PropWashLevel {
    Off,
    Stock,
    Max,
}

impl PropWashLevel {
    pub fn name(self) -> &'static str {
        match self {
            Self::Off => "Off",
            Self::Stock => "Stock",
            Self::Max => "Max",
        }
    }

    pub fn next(self) -> Self {
        match self {
            Self::Off => Self::Stock,
            Self::Stock => Self::Max,
            Self::Max => Self::Off,
        }
    }

    pub fn prev(self) -> Self {
        match self {
            Self::Off => Self::Max,
            Self::Stock => Self::Off,
            Self::Max => Self::Stock,
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FlightMode {
    Acro,
    Horizon,
    Angle,
}

impl FlightMode {
    pub fn name(self) -> &'static str {
        match self {
            Self::Acro => "ACRO",
            Self::Horizon => "HORIZON",
            Self::Angle => "ANGLE",
        }
    }

    pub fn next(self) -> Self {
        match self {
            Self::Acro => Self::Horizon,
            Self::Horizon => Self::Angle,
            Self::Angle => Self::Acro,
        }
    }

    pub fn prev(self) -> Self {
        match self {
            Self::Acro => Self::Angle,
            Self::Horizon => Self::Acro,
            Self::Angle => Self::Horizon,
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
pub enum UiTheme {
    Monokai,
    StickTime,
}

impl UiTheme {
    pub fn name(self) -> &'static str {
        match self {
            Self::Monokai => "Monokai",
            Self::StickTime => "StickTime",
        }
    }

    pub fn toggle(self) -> Self {
        match self {
            Self::Monokai => Self::StickTime,
            Self::StickTime => Self::Monokai,
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum StickOverlay {
    Off,
    Dot,
    Trails,
}

impl StickOverlay {
    pub fn name(self) -> &'static str {
        match self {
            Self::Off => "Off",
            Self::Dot => "On (Dot)",
            Self::Trails => "Trails",
        }
    }

    pub fn is_visible(self) -> bool {
        self != Self::Off
    }

    pub fn next(self) -> Self {
        match self {
            Self::Off => Self::Dot,
            Self::Dot => Self::Trails,
            Self::Trails => Self::Off,
        }
    }

    pub fn prev(self) -> Self {
        match self {
            Self::Off => Self::Trails,
            Self::Dot => Self::Off,
            Self::Trails => Self::Dot,
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
    Axis { code: u16, invert: bool },
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
    pub prop_wash: PropWashLevel,
    // Optional "The Zone"-style Drone Tuning multipliers
    pub thrust_pct: u16, // 60..=150 %
    pub weight_pct: u16, // 70..=150 %
    pub drag_pct: u16,   // 50..=160 %
    pub grip_pct: u16,   // 0..=160 %
    pub motor_pct: u16,  // 60..=150 %
    pub thr_expo: u8,    // 0..=60 (0.00..=0.60 Betaflight throttle expo around hover)

    pub flight_mode: FlightMode,
    pub rates: RatesPreset,
    pub rate_center: u16,
    pub rate_max: u16,
    pub rate_expo: u8,
    pub camera_tilt_deg: u8,
    pub theme: VisualTheme,
    pub ui_theme: UiTheme,
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
    pub aux_arm: AuxBinding,
    pub aux_mode: AuxBinding,
    pub aux_turtle: AuxBinding,
    pub stick_overlay: StickOverlay,
    pub best_laps_ms: [u32; 4],
}

impl Default for Config {
    fn default() -> Self {
        Self {
            quad: QuadClass::Air75II,
            battery_pack: BatteryPack::LavaII580,
            prop_wash: PropWashLevel::Stock,
            thrust_pct: 100,
            weight_pct: 100,
            drag_pct: 100,
            grip_pct: 100,
            motor_pct: 100,
            thr_expo: 25,
            flight_mode: FlightMode::Acro,
            rates: RatesPreset::Soft,
            rate_center: 100,
            rate_max: 480,
            rate_expo: 20,
            camera_tilt_deg: 25,
            theme: VisualTheme::Summer,
            ui_theme: UiTheme::Monokai,
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
            stick_overlay: StickOverlay::Trails,
            best_laps_ms: [0; 4],
        }
    }
}

impl Config {
    pub fn reset_drone_tuning(&mut self) {
        self.thrust_pct = 100;
        self.weight_pct = 100;
        self.drag_pct = 100;
        self.grip_pct = 100;
        self.motor_pct = 100;
        self.thr_expo = 25;
    }

    pub fn effective_auw_grams(&self) -> f32 {
        let p = self.quad.params();
        let (_, batt_g, _) = self.battery_pack.specs(&p);
        p.dry_mass_g * (self.weight_pct as f32 / 100.0) + batt_g
    }

    pub fn effective_twr(&self) -> f32 {
        let p = self.quad.params();
        let (_, batt_g, _) = self.battery_pack.specs(&p);
        let mass_ratio = (p.dry_mass_g * (self.weight_pct as f32 / 100.0) + batt_g)
            / (p.dry_mass_g + p.ref_batt_g);
        let max_a = p.max_accel * (self.thrust_pct as f32 / 100.0) / mass_ratio;
        max_a / 10.6
    }

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
                "prop_wash" => cfg.prop_wash = match v {
                    "0" | "off" => PropWashLevel::Off,
                    "2" | "max" => PropWashLevel::Max,
                    _ => PropWashLevel::Stock,
                },
                "thrust_pct" => if let Ok(n) = v.parse::<u16>() { cfg.thrust_pct = n.clamp(60, 150); },
                "weight_pct" => if let Ok(n) = v.parse::<u16>() { cfg.weight_pct = n.clamp(70, 150); },
                "drag_pct" => if let Ok(n) = v.parse::<u16>() { cfg.drag_pct = n.clamp(50, 160); },
                "grip_pct" => if let Ok(n) = v.parse::<u16>() { cfg.grip_pct = n.clamp(0, 160); },
                "motor_pct" => if let Ok(n) = v.parse::<u16>() { cfg.motor_pct = n.clamp(60, 150); },
                "thr_expo" => if let Ok(n) = v.parse::<u8>() { cfg.thr_expo = n.clamp(0, 60); },
                "flight_mode" => cfg.flight_mode = match v {
                    "horizon" => FlightMode::Horizon,
                    "angle" => FlightMode::Angle,
                    _ => FlightMode::Acro,
                },
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
                "ui_theme" => cfg.ui_theme = match v {
                    "sticktime" | "classic" => UiTheme::StickTime,
                    _ => UiTheme::Monokai,
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
                "stick_overlay" => cfg.stick_overlay = match v {
                    "off" => StickOverlay::Off,
                    "dot" => StickOverlay::Dot,
                    _ => StickOverlay::Trails,
                },
                "show_sticks" => cfg.stick_overlay = if v == "0" { StickOverlay::Off } else { StickOverlay::Trails },
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
        let pw = match self.prop_wash {
            PropWashLevel::Off => "off",
            PropWashLevel::Stock => "stock",
            PropWashLevel::Max => "max",
        };
        let fm = match self.flight_mode {
            FlightMode::Acro => "acro",
            FlightMode::Horizon => "horizon",
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
        let ui = match self.ui_theme {
            UiTheme::Monokai => "monokai",
            UiTheme::StickTime => "sticktime",
        };
        let so = match self.stick_overlay {
            StickOverlay::Off => "off",
            StickOverlay::Dot => "dot",
            StickOverlay::Trails => "trails",
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
            "quad={quad}\nbattery_pack={batt}\nprop_wash={pw}\nthrust_pct={}\nweight_pct={}\ndrag_pct={}\ngrip_pct={}\nmotor_pct={}\nthr_expo={}\nflight_mode={fm}\nrates={rates}\nrate_center={}\nrate_max={}\nrate_expo={}\ncamera_tilt={}\ntheme={theme}\nui_theme={ui}\ndetail_high={}\nsky_detail={}\nanalog_filter={}\nsound_on={}\ndebug_osd={}\nfov_deg={}\ngame_mode={gm}\ntrack={}\nstick_mode={sm}\nthrottle_mode={tm}\ninvert_pitch={}\ninvert_throttle={}\ninvert_yaw={}\ninvert_roll={}\naxis_thr={}\naxis_yaw={}\naxis_pit={}\naxis_rol={}\naux_arm={}\naux_mode={}\naux_turtle={}\nstick_overlay={so}\nbest0={}\nbest1={}\nbest2={}\nbest3={}\n",
            self.thrust_pct,
            self.weight_pct,
            self.drag_pct,
            self.grip_pct,
            self.motor_pct,
            self.thr_expo,
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
            self.best_laps_ms[0],
            self.best_laps_ms[1],
            self.best_laps_ms[2],
            self.best_laps_ms[3],
        );
        let _ = fs::write(path, out);
    }
}
