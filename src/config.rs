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
            // BetaFPV Air75 II Freestyle 75mm (~21g dry + 13.5g Lava580 = 35g AUW, or 37g w/ Lava680)
            Self::Air75II => QuadParams {
                thrust_g: 125,
                air_drag: 29,     // 0.29
                rotor_grip: 34,   // 0.34
                motor_lag_ms: 15, // 15 ms
                thr_mid: 38,      // 0.38
                idle_accel: 0.22,
                dry_mass_g: 21.0,
                ref_batt_g: 13.5,
                mah: 450.0,
                full_amp: 15.5,
            },
            // BetaFPV Meteor75 Pro II 75mm (~31g dry + 13.5g Lava580 = 45g AUW, 45mm props)
            Self::Meteor75ProII => QuadParams {
                thrust_g: 155,
                air_drag: 26,     // 0.26
                rotor_grip: 38,   // 0.38
                motor_lag_ms: 17, // 17 ms
                thr_mid: 38,      // 0.38
                idle_accel: 0.22,
                dry_mass_g: 31.0,
                ref_batt_g: 13.5,
                mah: 550.0,
                full_amp: 17.5,
            },
            // 65mm Indoor Whoop (~18g dry + 8.5g 300mAh = 27g AUW)
            Self::Whoop65 => QuadParams {
                thrust_g: 90,
                air_drag: 38,     // 0.38
                rotor_grip: 42,   // 0.42
                motor_lag_ms: 21, // 21 ms
                thr_mid: 40,      // 0.40
                idle_accel: 0.18,
                dry_mass_g: 18.0,
                ref_batt_g: 8.5,
                mah: 300.0,
                full_amp: 12.0,
            },
            // 3" Sub-120g Toothpick (~68g dry + 36g 2S/3S = 104g AUW)
            Self::Toothpick3 => QuadParams {
                thrust_g: 540,
                air_drag: 32,     // 0.32
                rotor_grip: 28,   // 0.28
                motor_lag_ms: 14, // 14 ms
                thr_mid: 34,      // 0.34
                idle_accel: 0.26,
                dry_mass_g: 68.0,
                ref_batt_g: 36.0,
                mah: 450.0,
                full_amp: 28.0,
            },
            // 5" 6S Freestyle (~410g dry + 220g 1300mAh = 630g AUW)
            Self::Freestyle5 => QuadParams {
                thrust_g: 4100,
                air_drag: 32,     // 0.32
                rotor_grip: 26,   // 0.26
                motor_lag_ms: 16, // 16 ms
                thr_mid: 30,      // 0.30
                idle_accel: 0.32,
                dry_mass_g: 410.0,
                ref_batt_g: 220.0,
                mah: 1300.0,
                full_amp: 78.0,
            },
            // 5" 6S Racer (~320g dry + 210g 1300mAh = 530g AUW)
            Self::Racer5 => QuadParams {
                thrust_g: 4350,
                air_drag: 28,     // 0.28
                rotor_grip: 24,   // 0.24
                motor_lag_ms: 12, // 12 ms
                thr_mid: 26,      // 0.26
                idle_accel: 0.35,
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
    /// Total static max thrust in grams
    pub thrust_g: u16,
    /// Aerodynamic drag coefficient * 100 (e.g. 29 -> 0.29)
    pub air_drag: u8,
    /// In-plane rotor H-force grip * 100 (e.g. 34 -> 0.34)
    pub rotor_grip: u8,
    /// Motor/ESC spin-up time constant in milliseconds
    pub motor_lag_ms: u8,
    /// Betaflight throttle mid * 100 (e.g. 38 -> 0.38)
    pub thr_mid: u8,
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

    pub fn prev_preset(self) -> Self {
        match self {
            Self::Soft | Self::Custom => Self::Fast,
            Self::Normal => Self::Soft,
            Self::Fast => Self::Normal,
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

    pub fn prev(self) -> Self {
        match self {
            Self::Summer => Self::Wireframe,
            Self::Evening => Self::Summer,
            Self::Night => Self::Evening,
            Self::Wireframe => Self::Night,
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

/// Output routing between CardputerZero's 1.9" ST7789v3 LCD and Mini-HDMI external display.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum DisplayOutput {
    /// Mirror to HDMI automatically whenever an HDMI display is connected (hotplug supported).
    Auto,
    /// Render exclusively to HDMI (shows static status card on LCD; falls back to LCD if unplugged).
    HdmiOnly,
    /// Render strictly to the built-in 1.9" LCD and ignore HDMI.
    LcdOnly,
}

impl DisplayOutput {
    #[allow(dead_code)]
    pub fn name(self) -> &'static str {
        match self {
            Self::Auto => "Auto (Mirror)",
            Self::HdmiOnly => "HDMI Only",
            Self::LcdOnly => "LCD Only",
        }
    }

    pub fn next(self) -> Self {
        match self {
            Self::Auto => Self::HdmiOnly,
            Self::HdmiOnly => Self::LcdOnly,
            Self::LcdOnly => Self::Auto,
        }
    }

    pub fn prev(self) -> Self {
        match self {
            Self::Auto => Self::LcdOnly,
            Self::HdmiOnly => Self::Auto,
            Self::LcdOnly => Self::HdmiOnly,
        }
    }

    pub fn serialize(self) -> &'static str {
        match self {
            Self::Auto => "auto",
            Self::HdmiOnly => "hdmi",
            Self::LcdOnly => "lcd",
        }
    }

    pub fn parse(s: &str) -> Self {
        match s {
            "hdmi" | "hdmi_only" => Self::HdmiOnly,
            "lcd" | "lcd_only" => Self::LcdOnly,
            _ => Self::Auto,
        }
    }
}

/// Internal software render resolution when Mini-HDMI output is active.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum HdmiResolution {
    /// Native built-in LCD resolution (320x170, lowest CPU & power).
    Native320,
    /// True 16:9 HD render (640x360: exact 2x to 720p, exact 3x to 1080p, no letterbox bars).
    Hd640,
    /// Extra-sharp 16:9 qHD render (960x540: exact 2x to 1080p).
    Qhd960,
}

impl HdmiResolution {
    pub fn name(self) -> &'static str {
        match self {
            Self::Native320 => "320x170 (1x)",
            Self::Hd640 => "640x360 (16:9)",
            Self::Qhd960 => "960x540 (qHD)",
        }
    }

    pub const fn dimensions(self) -> (usize, usize) {
        match self {
            Self::Native320 => (320, 170),
            Self::Hd640 => (640, 360),
            Self::Qhd960 => (960, 540),
        }
    }

    pub fn next(self) -> Self {
        match self {
            Self::Native320 => Self::Hd640,
            Self::Hd640 => Self::Qhd960,
            Self::Qhd960 => Self::Native320,
        }
    }

    pub fn prev(self) -> Self {
        match self {
            Self::Native320 => Self::Qhd960,
            Self::Hd640 => Self::Native320,
            Self::Qhd960 => Self::Hd640,
        }
    }

    pub fn serialize(self) -> &'static str {
        match self {
            Self::Native320 => "320",
            Self::Hd640 => "640",
            Self::Qhd960 => "960",
        }
    }

    pub fn parse(s: &str) -> Self {
        match s {
            "320" | "320x170" | "1x" => Self::Native320,
            "960" | "960x540" | "qhd" => Self::Qhd960,
            _ => Self::Hd640,
        }
    }
}

/// Built-in 1.9" LCD backlight policy while Mini-HDMI output is active.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum HdmiLcdBacklight {
    /// Dim the built-in LCD backlight (~12%) while HDMI is active to save battery.
    Dim,
    /// Turn the built-in LCD backlight completely off (0%) and pause SPI blits while HDMI is active.
    Off,
    /// Leave the built-in LCD backlight untouched at full user brightness.
    KeepOn,
}

impl HdmiLcdBacklight {
    pub fn name(self) -> &'static str {
        match self {
            Self::Dim => "Dim (12%)",
            Self::Off => "Off (0%)",
            Self::KeepOn => "Keep On",
        }
    }

    pub fn next(self) -> Self {
        match self {
            Self::Dim => Self::Off,
            Self::Off => Self::KeepOn,
            Self::KeepOn => Self::Dim,
        }
    }

    pub fn prev(self) -> Self {
        match self {
            Self::Dim => Self::KeepOn,
            Self::Off => Self::Dim,
            Self::KeepOn => Self::Off,
        }
    }

    pub fn serialize(self) -> &'static str {
        match self {
            Self::Dim => "dim",
            Self::Off => "off",
            Self::KeepOn => "on",
        }
    }

    pub fn parse(s: &str) -> Self {
        match s {
            "off" | "0" => Self::Off,
            "on" | "keep" | "keep_on" => Self::KeepOn,
            _ => Self::Dim,
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
    // "The Zone"-style real physical Drone Tuning parameters (no abstract percentages!)
    pub weight_g: u16,     // Total AUW in grams (15..=1500 g)
    pub thrust_g: u16,     // Max static thrust in grams (40..=8000 g)
    pub air_drag: u8,      // Aerodynamic drag * 100 (5..=100 -> 0.05..=1.00)
    pub rotor_grip: u8,    // In-plane rotor grip * 100 (0..=100 -> 0.00..=1.00)
    pub motor_lag_ms: u8,  // Motor/ESC spool lag in ms (5..=45 ms)
    pub thr_mid: u8,       // Betaflight throttle mid * 100 (20..=70 -> 0.20..=0.70)
    pub thr_expo: u8,      // Betaflight throttle expo * 100 (0..=80 -> 0.00..=0.80)

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
    pub display_output: DisplayOutput,
    pub hdmi_resolution: HdmiResolution,
    pub hdmi_lcd_backlight: HdmiLcdBacklight,
    pub best_laps_ms: [u32; 4],
}

impl Default for Config {
    fn default() -> Self {
        Self {
            quad: QuadClass::Air75II,
            battery_pack: BatteryPack::LavaII580,
            prop_wash: PropWashLevel::Stock,
            weight_g: 35,
            thrust_g: 125,
            air_drag: 29,
            rotor_grip: 34,
            motor_lag_ms: 15,
            thr_mid: 38,
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
            display_output: DisplayOutput::Auto,
            hdmi_resolution: HdmiResolution::Hd640,
            hdmi_lcd_backlight: HdmiLcdBacklight::Dim,
            best_laps_ms: [0; 4],
        }
    }
}

impl Config {
    /// Switch quad preset and load its real-world physical stock specs (grams, drag, grip, motor lag).
    pub fn set_quad(&mut self, new_quad: QuadClass) {
        self.quad = new_quad;
        if !matches!(new_quad, QuadClass::Air75II | QuadClass::Meteor75ProII)
            && matches!(self.battery_pack, BatteryPack::LavaII580 | BatteryPack::LavaII680)
        {
            self.battery_pack = BatteryPack::Stock;
        }
        self.reset_drone_tuning();
    }

    /// Switch battery pack and adjust total `weight_g` by the real battery mass delta.
    pub fn set_battery_pack(&mut self, new_pack: BatteryPack) {
        let p = self.quad.params();
        let (_, old_g, _) = self.battery_pack.specs(&p);
        let (_, new_g, _) = new_pack.specs(&p);
        let delta = (new_g - old_g).round() as i32;
        self.battery_pack = new_pack;
        self.weight_g = (self.weight_g as i32 + delta).clamp(15, 1500) as u16;
    }

    /// Reset all numeric drone tuning values back to the current Quad Preset + Battery Pack stock specs.
    pub fn reset_drone_tuning(&mut self) {
        let p = self.quad.params();
        let (_, batt_g, _) = self.battery_pack.specs(&p);
        self.weight_g = (p.dry_mass_g + batt_g).round().clamp(15.0, 1500.0) as u16;
        self.thrust_g = p.thrust_g;
        self.air_drag = p.air_drag;
        self.rotor_grip = p.rotor_grip;
        self.motor_lag_ms = p.motor_lag_ms;
        self.thr_mid = p.thr_mid;
        self.thr_expo = 25;
    }

    /// Thrust-to-weight ratio computed directly from `thrust_g / weight_g`.
    pub fn effective_twr(&self) -> f32 {
        (self.thrust_g as f32) / (self.weight_g as f32).max(15.0)
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
        let mut custom_weight: Option<u16> = None;
        let mut custom_thrust: Option<u16> = None;
        let mut custom_drag: Option<u8> = None;
        let mut custom_grip: Option<u8> = None;
        let mut custom_lag: Option<u8> = None;
        let mut custom_mid: Option<u8> = None;

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
                "weight_g" => if let Ok(n) = v.parse::<u16>() { custom_weight = Some(n.clamp(15, 1500)); },
                "thrust_g" => if let Ok(n) = v.parse::<u16>() { custom_thrust = Some(n.clamp(40, 8000)); },
                "air_drag" => if let Ok(n) = v.parse::<u8>() { custom_drag = Some(n.clamp(5, 100)); },
                "rotor_grip" => if let Ok(n) = v.parse::<u8>() { custom_grip = Some(n.clamp(0, 100)); },
                "motor_lag_ms" => if let Ok(n) = v.parse::<u8>() { custom_lag = Some(n.clamp(5, 45)); },
                "thr_mid" => if let Ok(n) = v.parse::<u8>() { custom_mid = Some(n.clamp(20, 70)); },
                "thr_expo" => if let Ok(n) = v.parse::<u8>() { cfg.thr_expo = n.clamp(0, 80); },
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
                "display_output" => cfg.display_output = DisplayOutput::parse(v),
                "hdmi_resolution" => cfg.hdmi_resolution = HdmiResolution::parse(v),
                "hdmi_lcd_backlight" => cfg.hdmi_lcd_backlight = HdmiLcdBacklight::parse(v),
                "best0" => if let Ok(ms) = v.parse() { cfg.best_laps_ms[0] = ms; },
                "best1" => if let Ok(ms) = v.parse() { cfg.best_laps_ms[1] = ms; },
                "best2" => if let Ok(ms) = v.parse() { cfg.best_laps_ms[2] = ms; },
                "best3" => if let Ok(ms) = v.parse() { cfg.best_laps_ms[3] = ms; },
                _ => {}
            }
        }

        // Initialize baseline real-unit specs from the loaded quad & battery pack, then apply any saved custom values
        let saved_expo = cfg.thr_expo;
        cfg.reset_drone_tuning();
        cfg.thr_expo = saved_expo;
        if let Some(w) = custom_weight { cfg.weight_g = w; }
        if let Some(t) = custom_thrust { cfg.thrust_g = t; }
        if let Some(d) = custom_drag { cfg.air_drag = d; }
        if let Some(g) = custom_grip { cfg.rotor_grip = g; }
        if let Some(l) = custom_lag { cfg.motor_lag_ms = l; }
        if let Some(m) = custom_mid { cfg.thr_mid = m; }
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
        let disp = self.display_output.serialize();
        let hdmi_res = self.hdmi_resolution.serialize();
        let hdmi_bl = self.hdmi_lcd_backlight.serialize();
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
            "quad={quad}\nbattery_pack={batt}\nprop_wash={pw}\nweight_g={}\nthrust_g={}\nair_drag={}\nrotor_grip={}\nmotor_lag_ms={}\nthr_mid={}\nthr_expo={}\nflight_mode={fm}\nrates={rates}\nrate_center={}\nrate_max={}\nrate_expo={}\ncamera_tilt={}\ntheme={theme}\nui_theme={ui}\ndetail_high={}\nsky_detail={}\nanalog_filter={}\nsound_on={}\ndebug_osd={}\nfov_deg={}\ngame_mode={gm}\ntrack={}\nstick_mode={sm}\nthrottle_mode={tm}\ninvert_pitch={}\ninvert_throttle={}\ninvert_yaw={}\ninvert_roll={}\naxis_thr={}\naxis_yaw={}\naxis_pit={}\naxis_rol={}\naux_arm={}\naux_mode={}\naux_turtle={}\nstick_overlay={so}\ndisplay_output={disp}\nhdmi_resolution={hdmi_res}\nhdmi_lcd_backlight={hdmi_bl}\nbest0={}\nbest1={}\nbest2={}\nbest3={}\n",
            self.weight_g,
            self.thrust_g,
            self.air_drag,
            self.rotor_grip,
            self.motor_lag_ms,
            self.thr_mid,
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_display_output_cycle_and_roundtrip() {
        let mut d = DisplayOutput::Auto;
        assert_eq!(d.name(), "Auto (Mirror)");
        assert_eq!(DisplayOutput::parse(d.serialize()), DisplayOutput::Auto);
        d = d.next();
        assert_eq!(d, DisplayOutput::HdmiOnly);
        assert_eq!(d.name(), "HDMI Only");
        assert_eq!(DisplayOutput::parse(d.serialize()), DisplayOutput::HdmiOnly);
        d = d.next();
        assert_eq!(d, DisplayOutput::LcdOnly);
        assert_eq!(d.name(), "LCD Only");
        assert_eq!(DisplayOutput::parse(d.serialize()), DisplayOutput::LcdOnly);
        assert_eq!(d.next(), DisplayOutput::Auto);
        assert_eq!(d.prev(), DisplayOutput::HdmiOnly);
    }

    #[test]
    fn test_hdmi_resolution_and_backlight_roundtrip() {
        let mut r = HdmiResolution::Hd640;
        assert_eq!(r.dimensions(), (640, 360));
        assert_eq!(HdmiResolution::parse(r.serialize()), HdmiResolution::Hd640);
        r = r.next();
        assert_eq!(r, HdmiResolution::Qhd960);
        assert_eq!(r.dimensions(), (960, 540));
        assert_eq!(HdmiResolution::parse(r.serialize()), HdmiResolution::Qhd960);
        r = r.next();
        assert_eq!(r, HdmiResolution::Native320);
        assert_eq!(r.dimensions(), (320, 170));
        assert_eq!(HdmiResolution::parse(r.serialize()), HdmiResolution::Native320);
        assert_eq!(r.next(), HdmiResolution::Hd640);
        assert_eq!(r.prev(), HdmiResolution::Qhd960);

        let mut b = HdmiLcdBacklight::Dim;
        assert_eq!(HdmiLcdBacklight::parse(b.serialize()), HdmiLcdBacklight::Dim);
        b = b.next();
        assert_eq!(b, HdmiLcdBacklight::Off);
        assert_eq!(HdmiLcdBacklight::parse(b.serialize()), HdmiLcdBacklight::Off);
        b = b.next();
        assert_eq!(b, HdmiLcdBacklight::KeepOn);
        assert_eq!(HdmiLcdBacklight::parse(b.serialize()), HdmiLcdBacklight::KeepOn);
        assert_eq!(b.next(), HdmiLcdBacklight::Dim);
        assert_eq!(b.prev(), HdmiLcdBacklight::Off);
    }
}
