//! Ultra-fast zero-allocation 320x170 RGB565 3D software rasterizer with a
//! per-pixel 16-bit perspective-correct Z-buffer, solid 3D racing gates,
//! authentic Betaflight OSD + post-flight statistics, and CardputerZero 1.9" menus.

use crate::config::{
    AuxBinding, BatteryPack, Config, DisplayOutput, GameMode, HdmiLcdBacklight, StickOverlay,
    UiTheme, VisualTheme,
};
use crate::display::{SCREEN_H, SCREEN_W};
use crate::font::{glyph, GLYPH_H, GLYPH_W};
use crate::input::{FlightControlSource, InputManager};
use crate::math::{clampf, Quat, Vec3};
use crate::physics::{QuadState, StickInput};
use crate::tracks::{GateKind, StructureColor, Track, TRACKS};

const UI_W: i32 = SCREEN_W as i32;
const UI_H: i32 = SCREEN_H as i32;
const NEAR_Z: f32 = 0.16;
/// Scale factor mapping 1/z (0 ..= 6.25) into u16 Z-buffer (0 ..= 65000). Larger = closer.
const IZ_SCALE: f32 = 10400.0;
const STICK_TRAIL_LEN: usize = 18;

#[inline(always)]
pub const fn rgb565(r: u8, g: u8, b: u8) -> u16 {
    (((r as u16) & 0xF8) << 8) | (((g as u16) & 0xFC) << 3) | ((b as u16) >> 3)
}

#[inline(always)]
fn lerp_rgb565(a: u16, b: u16, t: f32) -> u16 {
    let t = clampf(t, 0.0, 1.0);
    let ar = ((a >> 11) & 0x1F) as f32;
    let ag = ((a >> 5) & 0x3F) as f32;
    let ab = (a & 0x1F) as f32;
    let br = ((b >> 11) & 0x1F) as f32;
    let bg = ((b >> 5) & 0x3F) as f32;
    let bb = (b & 0x1F) as f32;
    let r = (ar + (br - ar) * t) as u16;
    let g = (ag + (bg - ag) * t) as u16;
    let bl = (ab + (bb - ab) * t) as u16;
    ((r & 0x1F) << 11) | ((g & 0x3F) << 5) | (bl & 0x1F)
}

/// Fast zero-float RGB565 alpha blend (`alpha_8` in `0..=8`, where 8 = 100% `src`).
#[inline(always)]
fn blend_rgb565(dst: u16, src: u16, alpha_8: u16) -> u16 {
    if alpha_8 >= 8 {
        return src;
    }
    if alpha_8 == 0 {
        return dst;
    }
    let inv = 8 - alpha_8;
    let r = ((((dst >> 11) & 0x1F) * inv + ((src >> 11) & 0x1F) * alpha_8) >> 3) & 0x1F;
    let g = ((((dst >> 5) & 0x3F) * inv + ((src >> 5) & 0x3F) * alpha_8) >> 3) & 0x3F;
    let b = (((dst & 0x1F) * inv + (src & 0x1F) * alpha_8) >> 3) & 0x1F;
    (r << 11) | (g << 5) | b
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SettingsTab {
    Flight,
    Rates,
    Camera,
    System,
}

impl SettingsTab {
    pub const ALL: [SettingsTab; 4] = [
        SettingsTab::Flight,
        SettingsTab::Rates,
        SettingsTab::Camera,
        SettingsTab::System,
    ];

    pub fn index(self) -> usize {
        match self {
            Self::Flight => 0,
            Self::Rates => 1,
            Self::Camera => 2,
            Self::System => 3,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::Flight => "FLIGHT",
            Self::Rates => "RATES",
            Self::Camera => "CAMERA",
            Self::System => "SYSTEM",
        }
    }

    pub fn next(self) -> Self {
        Self::ALL[(self.index() + 1) % Self::ALL.len()]
    }

    pub fn prev(self) -> Self {
        Self::ALL[(self.index() + Self::ALL.len() - 1) % Self::ALL.len()]
    }

    pub fn row_count(self) -> usize {
        match self {
            Self::Flight => 4,
            Self::Rates => 4,
            Self::Camera => 5,
            Self::System => 7,
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ControllerAction {
    CalibrateSticks,
    BindAux,
    ToggleThrottleMode,
    ToggleRawDebug,
    NextDevice,
    ResetMapping,
    ZeroImu,
    ScanDevices,
    Back,
}

impl ControllerAction {
    pub fn label(self) -> &'static str {
        match self {
            Self::CalibrateSticks => "CALIBRATE",
            Self::BindAux => "BIND AUX",
            Self::ToggleThrottleMode => "THROTTLE",
            Self::ToggleRawDebug => "DEBUG",
            Self::NextDevice => "NEXT DEV",
            Self::ResetMapping => "RESET MAP",
            Self::ZeroImu => "ZERO IMU",
            Self::ScanDevices => "SCAN DEV",
            Self::Back => "BACK",
        }
    }
}

/// Returns the contextual action list shown on the Controller Setup screen
/// so only options relevant to the connected hardware (USB/BT, IMU, or None) appear.
pub fn controller_actions(ctrl_src: FlightControlSource, multi_dev: bool) -> &'static [ControllerAction] {
    match ctrl_src {
        FlightControlSource::Controller if multi_dev => &[
            ControllerAction::CalibrateSticks,
            ControllerAction::BindAux,
            ControllerAction::ToggleThrottleMode,
            ControllerAction::ToggleRawDebug,
            ControllerAction::NextDevice,
            ControllerAction::ResetMapping,
        ],
        FlightControlSource::Controller => &[
            ControllerAction::CalibrateSticks,
            ControllerAction::BindAux,
            ControllerAction::ToggleThrottleMode,
            ControllerAction::ToggleRawDebug,
            ControllerAction::ResetMapping,
        ],
        FlightControlSource::ImuSimplified => &[
            ControllerAction::ZeroImu,
            ControllerAction::ScanDevices,
            ControllerAction::ToggleRawDebug,
            ControllerAction::Back,
        ],
        FlightControlSource::None => &[
            ControllerAction::ScanDevices,
            ControllerAction::ResetMapping,
            ControllerAction::Back,
        ],
    }
}

#[derive(Copy, Clone)]
struct UiColors {
    bg: u16,
    header_bg: u16,
    header_line: u16,
    title: u16,
    brand_zero: u16,
    tag: u16,
    sel_bg: u16,
    sel_border: u16,
    sel_label: u16,
    sel_val: u16,
    label: u16,
    val: u16,
    muted: u16,
    ok: u16,
    warn: u16,
    accent: u16,
}

fn ui_colors(theme: UiTheme) -> UiColors {
    match theme {
        // Authentic Monokai palette tuned for crisp 1.9" 320x170 RGB565 readability (Default)
        UiTheme::Monokai => UiColors {
            bg: rgb565(39, 40, 34),           // #272822 Monokai background
            header_bg: rgb565(29, 30, 26),    // #1D1E1A deeper header bar
            header_line: rgb565(249, 38, 114),// #F92672 Monokai magenta/pink
            title: rgb565(248, 248, 242),     // #F8F8F2 off-white
            brand_zero: rgb565(249, 38, 114), // #F92672 Monokai pink for "ZERO"
            tag: rgb565(102, 217, 239),       // #66D9EF Monokai cyan
            sel_bg: rgb565(62, 61, 50),       // #3E3D32 Monokai selection
            sel_border: rgb565(230, 219, 116),// #E6DB74 Monokai yellow border
            sel_label: rgb565(166, 226, 46),  // #A6E22E Monokai lime green
            sel_val: rgb565(248, 248, 242),   // #F8F8F2 crisp white value
            label: rgb565(230, 219, 116),     // #E6DB74 Monokai soft yellow
            val: rgb565(253, 151, 31),        // #FD971F Monokai orange
            muted: rgb565(152, 148, 128),     // Monokai comment (lifted for 1.9" LCD)
            ok: rgb565(166, 226, 46),         // #A6E22E green
            warn: rgb565(253, 151, 31),       // #FD971F orange
            accent: rgb565(174, 129, 255),    // #AE81FF Monokai purple
        },
        // Classic StickTime blue-slate palette
        UiTheme::StickTime => UiColors {
            bg: rgb565(12, 16, 26),
            header_bg: rgb565(22, 32, 52),
            header_line: rgb565(65, 140, 240),
            title: rgb565(255, 255, 255),
            brand_zero: rgb565(120, 220, 255),
            tag: rgb565(120, 220, 255),
            sel_bg: rgb565(36, 66, 118),
            sel_border: rgb565(90, 190, 255),
            sel_label: rgb565(255, 230, 80),
            sel_val: rgb565(255, 255, 255),
            label: rgb565(180, 195, 215),
            val: rgb565(135, 200, 235),
            muted: rgb565(145, 160, 180),
            ok: rgb565(70, 240, 110),
            warn: rgb565(240, 160, 50),
            accent: rgb565(90, 220, 255),
        },
    }
}

#[derive(Copy, Clone)]
struct ThemePalette {
    sky_top: u16,
    sky_horizon: u16,
    ground_near: u16,
    ground_far: u16,
    horizon_line: Option<u16>,
    grid_line: u16,
    active_gate: u16,
    active_gate_hi: u16,
    idle_gate: u16,
    wireframe: bool,
}

fn palette(theme: VisualTheme) -> ThemePalette {
    match theme {
        VisualTheme::Summer => ThemePalette {
            sky_top: rgb565(30, 108, 212),
            sky_horizon: rgb565(155, 206, 240),
            ground_near: rgb565(46, 108, 46),
            ground_far: rgb565(100, 146, 104),
            horizon_line: None,
            grid_line: rgb565(118, 200, 108),
            active_gate: rgb565(255, 110, 20),
            active_gate_hi: rgb565(255, 224, 65),
            idle_gate: rgb565(180, 192, 208),
            wireframe: false,
        },
        VisualTheme::Evening => ThemePalette {
            sky_top: rgb565(40, 32, 92),
            sky_horizon: rgb565(238, 132, 72),
            ground_near: rgb565(46, 54, 38),
            ground_far: rgb565(104, 82, 64),
            horizon_line: Some(rgb565(245, 155, 88)),
            grid_line: rgb565(140, 150, 96),
            active_gate: rgb565(255, 210, 40),
            active_gate_hi: rgb565(255, 255, 185),
            idle_gate: rgb565(160, 145, 168),
            wireframe: false,
        },
        VisualTheme::Night => ThemePalette {
            sky_top: rgb565(8, 16, 48),
            sky_horizon: rgb565(30, 46, 104),
            ground_near: rgb565(18, 30, 24),
            ground_far: rgb565(24, 40, 34),
            horizon_line: Some(rgb565(80, 128, 225)),
            grid_line: rgb565(52, 168, 140),
            active_gate: rgb565(0, 255, 180),
            active_gate_hi: rgb565(170, 255, 245),
            idle_gate: rgb565(90, 125, 225),
            wireframe: false,
        },
        // Redesigned Vector Wireframe:
        // Black sky | Emerald green grid ground | Dark-occluded Neon Cyan/Magenta walls | Hot Orange/Yellow gates!
        VisualTheme::Wireframe => ThemePalette {
            sky_top: rgb565(0, 0, 0),
            sky_horizon: rgb565(4, 12, 24),
            ground_near: rgb565(8, 34, 18),
            ground_far: rgb565(12, 48, 24),
            horizon_line: Some(rgb565(95, 255, 185)),
            grid_line: rgb565(48, 215, 105),
            active_gate: rgb565(255, 115, 20),
            active_gate_hi: rgb565(255, 242, 75),
            idle_gate: rgb565(235, 165, 45),
            wireframe: true,
        },
    }
}

#[derive(Copy, Clone, Debug)]
struct ScreenVert {
    x: f32,
    y: f32,
    iz: f32, // Scaled 1/z (larger = closer)
}

pub struct Renderer {
    pub width: usize,
    pub height: usize,
    pub fb: Vec<u16>,
    zbuf: Vec<u16>,
    focal: f32,
    rng_state: u32,
    /// Zero-allocation ring buffer storing recent (lx, ly, rx, ry) stick positions for OSD trails.
    stick_trail: [(f32, f32, f32, f32); STICK_TRAIL_LEN],
    trail_head: usize,
    trail_len: usize,
}

impl Renderer {
    pub fn new() -> Self {
        Self::with_size(SCREEN_W, SCREEN_H)
    }

    pub fn with_size(width: usize, height: usize) -> Self {
        let w = width.max(SCREEN_W);
        let h = height.max(SCREEN_H);
        Self {
            width: w,
            height: h,
            fb: vec![0u16; w * h],
            zbuf: vec![0u16; w * h],
            focal: (w as f32) * 0.5,
            rng_state: 0x51A7_1C09,
            stick_trail: [(0.0, 0.0, 0.0, 0.0); STICK_TRAIL_LEN],
            trail_head: 0,
            trail_len: 0,
        }
    }

    /// Dynamically switch internal render resolution (e.g. 320x170 <-> 640x360 <-> 960x540).
    /// Zero allocations when `(width, height)` is unchanged.
    pub fn set_size(&mut self, width: usize, height: usize) {
        let w = width.max(SCREEN_W);
        let h = height.max(SCREEN_H);
        if self.width == w && self.height == h {
            return;
        }
        self.width = w;
        self.height = h;
        let len = w * h;
        self.fb.resize(len, 0);
        self.zbuf.resize(len, 0);
        self.focal = (w as f32) * 0.5;
    }

    #[inline(always)]
    fn w(&self) -> i32 {
        self.width as i32
    }

    #[inline(always)]
    fn h(&self) -> i32 {
        self.height as i32
    }

    #[inline(always)]
    fn half_w(&self) -> f32 {
        (self.width as f32) * 0.5
    }

    #[inline(always)]
    fn half_h(&self) -> f32 {
        (self.height as f32) * 0.5
    }

    #[inline(always)]
    fn ui_scale(&self) -> i32 {
        (self.width / SCREEN_W).min(self.height / SCREEN_H).max(1) as i32
    }

    #[inline(always)]
    fn ui_origin(&self) -> (i32, i32) {
        let s = self.ui_scale();
        ((self.w() - UI_W * s) / 2, (self.h() - UI_H * s) / 2)
    }

    fn ui_fill_rect(&mut self, vx: i32, vy: i32, vw: i32, vh: i32, color: u16) {
        let s = self.ui_scale();
        let (ox, oy) = self.ui_origin();
        self.fill_rect(ox + vx * s, oy + vy * s, vw * s, vh * s, color);
    }

    fn ui_draw_rect(&mut self, vx: i32, vy: i32, vw: i32, vh: i32, color: u16) {
        let s = self.ui_scale();
        let (ox, oy) = self.ui_origin();
        self.draw_rect_thick(ox + vx * s, oy + vy * s, vw * s, vh * s, s, color);
    }

    fn ui_draw_hline(&mut self, vx0: i32, vx1: i32, vy: i32, color: u16) {
        let s = self.ui_scale();
        let (ox, oy) = self.ui_origin();
        let x0 = vx0.min(vx1);
        let len = (vx1 - vx0).abs() + 1;
        self.fill_rect(ox + x0 * s, oy + vy * s, len * s, s, color);
    }

    fn ui_draw_vline(&mut self, vx: i32, vy0: i32, vy1: i32, color: u16) {
        let s = self.ui_scale();
        let (ox, oy) = self.ui_origin();
        let y0 = vy0.min(vy1);
        let len = (vy1 - vy0).abs() + 1;
        self.fill_rect(ox + vx * s, oy + y0 * s, s, len * s, color);
    }

    fn ui_draw_text(&mut self, vx: i32, vy: i32, text: &str, color: u16, scale: i32, outline: bool) {
        let s = self.ui_scale();
        let (ox, oy) = self.ui_origin();
        self.draw_text(ox + vx * s, oy + vy * s, text, color, scale * s, outline);
    }

    /// Translucent rectangle fill blended over the existing framebuffer (`alpha_8` in `0..=8`).
    pub fn fill_rect_alpha(&mut self, x: i32, y: i32, w: i32, h: i32, color: u16, alpha_8: u16) {
        if alpha_8 >= 8 {
            self.fill_rect(x, y, w, h, color);
            return;
        }
        if alpha_8 == 0 {
            return;
        }
        let x0 = x.max(0);
        let y0 = y.max(0);
        let x1 = (x + w).min(self.w());
        let y1 = (y + h).min(self.h());
        if x0 >= x1 || y0 >= y1 {
            return;
        }
        let stride = self.width;
        for py in y0..y1 {
            let row = (py as usize) * stride;
            let slice = &mut self.fb[row + (x0 as usize)..row + (x1 as usize)];
            for px in slice.iter_mut() {
                *px = blend_rgb565(*px, color, alpha_8);
            }
        }
    }

    fn ui_fill_rect_alpha(&mut self, vx: i32, vy: i32, vw: i32, vh: i32, color: u16, alpha_8: u16) {
        let s = self.ui_scale();
        let (ox, oy) = self.ui_origin();
        self.fill_rect_alpha(ox + vx * s, oy + vy * s, vw * s, vh * s, color, alpha_8);
    }

    fn ui_draw_panel(&mut self, vx: i32, vy: i32, vw: i32, vh: i32, bg: u16, border: u16, alpha_8: u16) {
        self.ui_fill_rect_alpha(vx, vy, vw, vh, bg, alpha_8);
        self.ui_draw_rect(vx, vy, vw, vh, border);
    }

    fn ui_draw_line(&mut self, vx0: i32, vy0: i32, vx1: i32, vy1: i32, color: u16) {
        let s = self.ui_scale();
        let (ox, oy) = self.ui_origin();
        let x0 = ox + vx0 * s + s / 2;
        let y0 = oy + vy0 * s + s / 2;
        let x1 = ox + vx1 * s + s / 2;
        let y1 = oy + vy1 * s + s / 2;
        self.draw_line(x0, y0, x1, y1, color);
        if s >= 2 {
            self.draw_line(x0, y0 + 1, x1, y1 + 1, color);
        }
    }

    pub fn reset_stick_trail(&mut self) {
        self.trail_head = 0;
        self.trail_len = 0;
    }

    pub fn push_stick_trail(&mut self, lx: f32, ly: f32, rx: f32, ry: f32) {
        self.stick_trail[self.trail_head] = (lx, ly, rx, ry);
        self.trail_head = (self.trail_head + 1) % STICK_TRAIL_LEN;
        if self.trail_len < STICK_TRAIL_LEN {
            self.trail_len += 1;
        }
    }

    #[inline(always)]
    pub fn put_pixel(&mut self, x: i32, y: i32, color: u16) {
        if x >= 0 && x < self.w() && y >= 0 && y < self.h() {
            self.fb[(y as usize) * self.width + (x as usize)] = color;
        }
    }

    pub fn clear(&mut self, color: u16) {
        self.fb.fill(color);
    }

    pub fn fill_rect(&mut self, x: i32, y: i32, w: i32, h: i32, color: u16) {
        let x0 = x.max(0);
        let y0 = y.max(0);
        let x1 = (x + w).min(self.w());
        let y1 = (y + h).min(self.h());
        if x0 >= x1 || y0 >= y1 {
            return;
        }
        let stride = self.width;
        for py in y0..y1 {
            let row = (py as usize) * stride;
            self.fb[row + (x0 as usize)..row + (x1 as usize)].fill(color);
        }
    }

    pub fn draw_rect(&mut self, x: i32, y: i32, w: i32, h: i32, color: u16) {
        if w <= 0 || h <= 0 {
            return;
        }
        self.draw_hline(x, x + w - 1, y, color);
        self.draw_hline(x, x + w - 1, y + h - 1, color);
        self.draw_vline(x, y, y + h - 1, color);
        self.draw_vline(x + w - 1, y, y + h - 1, color);
    }

    fn draw_rect_thick(&mut self, x: i32, y: i32, w: i32, h: i32, thick: i32, color: u16) {
        if w <= 0 || h <= 0 || thick <= 0 {
            return;
        }
        if thick == 1 {
            self.draw_rect(x, y, w, h, color);
            return;
        }
        self.fill_rect(x, y, w, thick, color);
        self.fill_rect(x, y + h - thick, w, thick, color);
        self.fill_rect(x, y, thick, h, color);
        self.fill_rect(x + w - thick, y, thick, h, color);
    }

    #[inline]
    fn draw_hline(&mut self, x0: i32, x1: i32, y: i32, color: u16) {
        if y < 0 || y >= self.h() {
            return;
        }
        let lx = x0.min(x1).max(0);
        let rx = x0.max(x1).min(self.w() - 1);
        if lx <= rx {
            let row = (y as usize) * self.width;
            self.fb[row + (lx as usize)..=row + (rx as usize)].fill(color);
        }
    }

    #[inline]
    fn draw_vline(&mut self, x: i32, y0: i32, y1: i32, color: u16) {
        if x < 0 || x >= self.w() {
            return;
        }
        let ty = y0.min(y1).max(0);
        let by = y0.max(y1).min(self.h() - 1);
        let stride = self.width;
        for y in ty..=by {
            self.fb[(y as usize) * stride + (x as usize)] = color;
        }
    }

    pub fn draw_line(&mut self, mut x0: i32, mut y0: i32, x1: i32, y1: i32, color: u16) {
        let dx = (x1 - x0).abs();
        let sx = if x0 < x1 { 1 } else { -1 };
        let dy = -(y1 - y0).abs();
        let sy = if y0 < y1 { 1 } else { -1 };
        let mut err = dx + dy;
        loop {
            self.put_pixel(x0, y0, color);
            if x0 == x1 && y0 == y1 {
                break;
            }
            let e2 = 2 * err;
            if e2 >= dy {
                err += dy;
                x0 += sx;
            }
            if e2 <= dx {
                err += dx;
                y0 += sy;
            }
        }
    }

    /// Z-buffered Bresenham line with linear `1/z` interpolation and proportional Z-bias
    /// so foreground outlines never Z-fight their own face and distant back edges NEVER bleed through nearer walls.
    fn draw_line_z(
        &mut self,
        mut x0: i32,
        mut y0: i32,
        iz0: f32,
        x1: i32,
        y1: i32,
        iz1: f32,
        color: u16,
        write_z: bool,
    ) {
        let w = self.w();
        let h = self.h();
        let stride = self.width;
        let dx = (x1 - x0).abs();
        let dy_abs = (y1 - y0).abs();
        let steps = dx.max(dy_abs);
        let sx = if x0 < x1 { 1 } else { -1 };
        let dy = -dy_abs;
        let sy = if y0 < y1 { 1 } else { -1 };
        let mut err = dx + dy;
        let diz = if steps > 0 { (iz1 - iz0) / (steps as f32) } else { 0.0 };
        let mut iz = iz0;

        for _ in 0..=steps {
            if x0 >= 0 && x0 < w && y0 >= 0 && y0 < h {
                let idx = (y0 as usize) * stride + (x0 as usize);
                // Proportional ~0.5% depth tolerance (at least 2 u16 units, max 6) beats own-face quantization
                // while rejecting any back wall behind the foreground surface!
                let bias = (iz * 0.005).clamp(2.0, 6.0);
                let z_test = ((iz + bias) as i32).clamp(1, 65535) as u16;
                if z_test >= self.zbuf[idx] {
                    if write_z {
                        let z_store = (iz as i32).clamp(1, 65535) as u16;
                        if z_store > self.zbuf[idx] {
                            self.zbuf[idx] = z_store;
                        }
                    }
                    self.fb[idx] = color;
                }
            }
            if x0 == x1 && y0 == y1 {
                break;
            }
            let e2 = 2 * err;
            if e2 >= dy {
                err += dy;
                x0 += sx;
            }
            if e2 <= dx {
                err += dx;
                y0 += sy;
            }
            iz += diz;
        }
    }

    /// Per-pixel perspective-correct Z-buffered triangle rasterizer!
    fn fill_tri_z(&mut self, v0: ScreenVert, v1: ScreenVert, v2: ScreenVert, color: u16) {
        let mut v = [v0, v1, v2];
        if v[0].y > v[1].y { v.swap(0, 1); }
        if v[1].y > v[2].y { v.swap(1, 2); }
        if v[0].y > v[1].y { v.swap(0, 1); }

        let total_h = v[2].y - v[0].y;
        if total_h < 0.25 {
            return;
        }

        let w = self.w();
        let h = self.h();
        let stride = self.width;
        let y_start = (v[0].y.ceil() as i32).max(0);
        let y_end = (v[2].y.floor() as i32).min(h - 1);
        if y_start > y_end {
            return;
        }

        let inv_total_h = 1.0 / total_h;
        let h_top = v[1].y - v[0].y;
        let h_bot = v[2].y - v[1].y;
        let inv_h_top = if h_top > 1e-4 { 1.0 / h_top } else { 0.0 };
        let inv_h_bot = if h_bot > 1e-4 { 1.0 / h_bot } else { 0.0 };

        for y in y_start..=y_end {
            let fy = y as f32;
            let alpha = (fy - v[0].y) * inv_total_h;
            let mut xa = v[0].x + (v[2].x - v[0].x) * alpha;
            let mut iza = v[0].iz + (v[2].iz - v[0].iz) * alpha;

            let (mut xb, mut izb) = if fy < v[1].y && h_top > 1e-4 {
                let beta = (fy - v[0].y) * inv_h_top;
                (
                    v[0].x + (v[1].x - v[0].x) * beta,
                    v[0].iz + (v[1].iz - v[0].iz) * beta,
                )
            } else if h_bot > 1e-4 {
                let beta = (fy - v[1].y) * inv_h_bot;
                (
                    v[1].x + (v[2].x - v[1].x) * beta,
                    v[1].iz + (v[2].iz - v[1].iz) * beta,
                )
            } else {
                continue;
            };

            if xa > xb {
                std::mem::swap(&mut xa, &mut xb);
                std::mem::swap(&mut iza, &mut izb);
            }

            let x_start = (xa.ceil() as i32).max(0);
            let x_end = (xb.floor() as i32).min(w - 1);
            if x_start > x_end {
                continue;
            }

            let dx = xb - xa;
            let diz_dx = if dx > 1e-4 { (izb - iza) / dx } else { 0.0 };
            let mut iz = iza + diz_dx * (x_start as f32 - xa);
            let row = (y as usize) * stride;

            for x in x_start..=x_end {
                let idx = row + (x as usize);
                let zu = (iz as i32).clamp(1, 65535) as u16;
                if zu > self.zbuf[idx] {
                    self.zbuf[idx] = zu;
                    self.fb[idx] = color;
                }
                iz += diz_dx;
            }
        }
    }

    pub fn draw_text(&mut self, x: i32, y: i32, text: &str, color: u16, scale: i32, outline: bool) {
        let mut cx = x;
        let adv = (GLYPH_W + 1) * scale;
        let black = rgb565(0, 0, 0);
        let ol = (scale / 2).max(1);
        for &b in text.as_bytes() {
            let g = glyph(b);
            if outline {
                for (col_idx, &col) in g.iter().enumerate() {
                    for row in 0..GLYPH_H {
                        if (col & (1 << row)) != 0 {
                            let px = cx + (col_idx as i32) * scale;
                            let py = y + row * scale;
                            self.fill_rect(px - ol, py, scale + ol * 2, scale, black);
                            self.fill_rect(px, py - ol, scale, scale + ol * 2, black);
                        }
                    }
                }
            }
            for (col_idx, &col) in g.iter().enumerate() {
                for row in 0..GLYPH_H {
                    if (col & (1 << row)) != 0 {
                        let px = cx + (col_idx as i32) * scale;
                        let py = y + row * scale;
                        if scale == 1 {
                            self.put_pixel(px, py, color);
                        } else {
                            self.fill_rect(px, py, scale, scale, color);
                        }
                    }
                }
            }
            cx += adv;
        }
    }

    #[inline(always)]
    fn world_to_cam(p: Vec3, cam_pos: Vec3, cam_rot_inv: Quat) -> Vec3 {
        cam_rot_inv.rotate(p.sub(cam_pos))
    }

    #[inline(always)]
    fn project_sv(&self, c: Vec3) -> ScreenVert {
        let inv_z = 1.0 / c.z;
        ScreenVert {
            x: self.half_w() + c.x * self.focal * inv_z,
            y: self.half_h() - c.y * self.focal * inv_z,
            iz: (inv_z * IZ_SCALE).clamp(1.0, 65000.0),
        }
    }

    /// Exact zero-safe Cohen-Sutherland 2D line clipper that also linearly clips `iz`.
    fn clip_line_2d_z(
        &self,
        mut x0: f32,
        mut y0: f32,
        mut iz0: f32,
        mut x1: f32,
        mut y1: f32,
        mut iz1: f32,
    ) -> Option<(i32, i32, f32, i32, i32, f32)> {
        const XMIN: f32 = 0.0;
        const YMIN: f32 = 0.0;
        let xmax: f32 = (self.w() - 1) as f32;
        let ymax: f32 = (self.h() - 1) as f32;

        let outcode = |x: f32, y: f32| -> u8 {
            let mut c = 0u8;
            if x < XMIN { c |= 1; } else if x > xmax { c |= 2; }
            if y < YMIN { c |= 4; } else if y > ymax { c |= 8; }
            c
        };

        let mut c0 = outcode(x0, y0);
        let mut c1 = outcode(x1, y1);
        for _ in 0..8 {
            if (c0 | c1) == 0 {
                return Some((x0.round() as i32, y0.round() as i32, iz0, x1.round() as i32, y1.round() as i32, iz1));
            }
            if (c0 & c1) != 0 {
                return None;
            }
            let out = if c0 != 0 { c0 } else { c1 };
            let (nx, ny, t) = if (out & 8) != 0 {
                let dy = y1 - y0;
                if dy.abs() < 1e-6 { return None; }
                let t = (ymax - y0) / dy;
                (x0 + (x1 - x0) * t, ymax, t)
            } else if (out & 4) != 0 {
                let dy = y1 - y0;
                if dy.abs() < 1e-6 { return None; }
                let t = (YMIN - y0) / dy;
                (x0 + (x1 - x0) * t, YMIN, t)
            } else if (out & 2) != 0 {
                let dx = x1 - x0;
                if dx.abs() < 1e-6 { return None; }
                let t = (xmax - x0) / dx;
                (xmax, y0 + (y1 - y0) * t, t)
            } else {
                let dx = x1 - x0;
                if dx.abs() < 1e-6 { return None; }
                let t = (XMIN - x0) / dx;
                (XMIN, y0 + (y1 - y0) * t, t)
            };
            let niz = iz0 + (iz1 - iz0) * t;
            if out == c0 {
                x0 = nx;
                y0 = ny;
                iz0 = niz;
                c0 = outcode(x0, y0);
            } else {
                x1 = nx;
                y1 = ny;
                iz1 = niz;
                c1 = outcode(x1, y1);
            }
        }
        None
    }

    /// Draw a 3D ground grid line immediately (no Z-write so 3D geometry always renders cleanly on top).
    fn draw_ground_line(&mut self, a: Vec3, b: Vec3, cam_pos: Vec3, cam_inv: Quat, color: u16) {
        let mut ca = Self::world_to_cam(a, cam_pos, cam_inv);
        let mut cb = Self::world_to_cam(b, cam_pos, cam_inv);
        if ca.z < NEAR_Z && cb.z < NEAR_Z {
            return;
        }
        if ca.z < NEAR_Z {
            let t = (NEAR_Z - ca.z) / (cb.z - ca.z);
            ca = ca.add(cb.sub(ca).scale(t));
        } else if cb.z < NEAR_Z {
            let t = (NEAR_Z - cb.z) / (ca.z - cb.z);
            cb = cb.add(ca.sub(cb).scale(t));
        }
        let sa = self.project_sv(ca);
        let sb = self.project_sv(cb);
        if let Some((x0, y0, _, x1, y1, _)) = self.clip_line_2d_z(sa.x, sa.y, sa.iz, sb.x, sb.y, sb.iz) {
            self.draw_line(x0, y0, x1, y1, color);
        }
    }

    /// Draw a Z-buffered 3D line (optional 2px thickness for flags/stilts).
    fn draw_3d_line_z(&mut self, a: Vec3, b: Vec3, cam_pos: Vec3, cam_inv: Quat, color: u16, thick: bool) {
        let mut ca = Self::world_to_cam(a, cam_pos, cam_inv);
        let mut cb = Self::world_to_cam(b, cam_pos, cam_inv);
        if ca.z < NEAR_Z && cb.z < NEAR_Z {
            return;
        }
        if ca.z < NEAR_Z {
            let t = (NEAR_Z - ca.z) / (cb.z - ca.z);
            ca = ca.add(cb.sub(ca).scale(t));
        } else if cb.z < NEAR_Z {
            let t = (NEAR_Z - cb.z) / (ca.z - cb.z);
            cb = cb.add(ca.sub(cb).scale(t));
        }
        let sa = self.project_sv(ca);
        let sb = self.project_sv(cb);
        if let Some((x0, y0, iz0, x1, y1, iz1)) = self.clip_line_2d_z(sa.x, sa.y, sa.iz, sb.x, sb.y, sb.iz) {
            self.draw_line_z(x0, y0, iz0, x1, y1, iz1, color, true);
            if thick {
                self.draw_line_z(x0 + 1, y0, iz0, x1 + 1, y1, iz1, color, false);
                self.draw_line_z(x0, y0 + 1, iz0, x1, y1 + 1, iz1, color, false);
            }
        }
    }

    /// Sutherland-Hodgman clip a 3D quad against `z >= NEAR_Z` and project to screen vertices.
    fn clip_project_quad(
        &self,
        verts: [Vec3; 4],
        cam_pos: Vec3,
        cam_inv: Quat,
    ) -> Option<([Vec3; 5], [ScreenVert; 5], usize)> {
        let mut c = [Vec3::ZERO; 4];
        for i in 0..4 {
            c[i] = Self::world_to_cam(verts[i], cam_pos, cam_inv);
        }

        let mut clipped = [Vec3::ZERO; 5];
        let mut count = 0usize;
        for i in 0..4 {
            let cur = c[i];
            let nxt = c[(i + 1) & 3];
            let cur_in = cur.z >= NEAR_Z;
            let nxt_in = nxt.z >= NEAR_Z;
            if cur_in {
                if count < 5 {
                    clipped[count] = cur;
                    count += 1;
                }
                if !nxt_in && count < 5 {
                    let t = (NEAR_Z - cur.z) / (nxt.z - cur.z);
                    clipped[count] = cur.add(nxt.sub(cur).scale(t));
                    count += 1;
                }
            } else if nxt_in && count < 5 {
                let t = (NEAR_Z - cur.z) / (nxt.z - cur.z);
                clipped[count] = cur.add(nxt.sub(cur).scale(t));
                count += 1;
            }
        }
        if count < 3 {
            return None;
        }

        let mut sv = [ScreenVert { x: 0.0, y: 0.0, iz: 0.0 }; 5];
        let mut min_x = f32::MAX;
        let mut max_x = f32::MIN;
        let mut min_y = f32::MAX;
        let mut max_y = f32::MIN;
        for i in 0..count {
            let p = self.project_sv(clipped[i]);
            sv[i] = p;
            min_x = min_x.min(p.x);
            max_x = max_x.max(p.x);
            min_y = min_y.min(p.y);
            max_y = max_y.max(p.y);
        }
        if max_x < 0.0 || min_x >= self.w() as f32 || max_y < 0.0 || min_y >= self.h() as f32 {
            return None;
        }
        Some((clipped, sv, count))
    }

    /// Pass 1: Fill a 3D quad into framebuffer + 16-bit Z-buffer.
    fn draw_3d_quad_fill_z(&mut self, verts: [Vec3; 4], cam_pos: Vec3, cam_inv: Quat, fill: u16) {
        if let Some((_, sv, count)) = self.clip_project_quad(verts, cam_pos, cam_inv) {
            for k in 1..(count - 1) {
                self.fill_tri_z(sv[0], sv[k], sv[k + 1], fill);
            }
        }
    }

    /// Pass 2: Draw strictly hidden-line occluded 3D quad edges against the completed Z-buffer.
    fn draw_3d_quad_edges_z(&mut self, verts: [Vec3; 4], cam_pos: Vec3, cam_inv: Quat, edge: u16) {
        if let Some((clipped, sv, count)) = self.clip_project_quad(verts, cam_pos, cam_inv) {
            for k in 0..count {
                let n = (k + 1) % count;
                // Skip artificial edge generated along the camera near clip plane
                if (clipped[k].z - NEAR_Z).abs() < 1e-3 && (clipped[n].z - NEAR_Z).abs() < 1e-3 {
                    continue;
                }
                if let Some((x0, y0, iz0, x1, y1, iz1)) =
                    self.clip_line_2d_z(sv[k].x, sv[k].y, sv[k].iz, sv[n].x, sv[n].y, sv[n].iz)
                {
                    self.draw_line_z(x0, y0, iz0, x1, y1, iz1, edge, false);
                }
            }
        }
    }

    /// Clip and rasterize both fill + outline of a 3D quad (used for gate beams & banners).
    fn draw_3d_quad_z(&mut self, verts: [Vec3; 4], cam_pos: Vec3, cam_inv: Quat, fill: u16, edge: u16, _wireframe: bool) {
        if let Some((clipped, sv, count)) = self.clip_project_quad(verts, cam_pos, cam_inv) {
            for k in 1..(count - 1) {
                self.fill_tri_z(sv[0], sv[k], sv[k + 1], fill);
            }
            for k in 0..count {
                let n = (k + 1) % count;
                if (clipped[k].z - NEAR_Z).abs() < 1e-3 && (clipped[n].z - NEAR_Z).abs() < 1e-3 {
                    continue;
                }
                if let Some((x0, y0, iz0, x1, y1, iz1)) =
                    self.clip_line_2d_z(sv[k].x, sv[k].y, sv[k].iz, sv[n].x, sv[n].y, sv[n].iz)
                {
                    self.draw_line_z(x0, y0, iz0, x1, y1, iz1, edge, false);
                }
            }
        }
    }

    /// Draw a solid 3D gate beam/post (two crossed orthogonal ribbons) so gates have true volume,
    /// write to the Z-buffer, and NEVER vanish from any viewing angle!
    fn draw_gate_beam(
        &mut self,
        a: Vec3,
        b: Vec3,
        axis_u: Vec3,
        axis_v: Vec3,
        half_w: f32,
        cam_pos: Vec3,
        cam_inv: Quat,
        fill: u16,
        edge: u16,
        wireframe: bool,
    ) {
        let u = axis_u.scale(half_w);
        let v = axis_v.scale(half_w);
        let _ = wireframe;
        self.draw_3d_quad_z([a.sub(u), a.add(u), b.add(u), b.sub(u)], cam_pos, cam_inv, fill, edge, false);
        self.draw_3d_quad_z([a.sub(v), a.add(v), b.add(v), b.sub(v)], cam_pos, cam_inv, fill, edge, false);
    }

    /// Ultra-fast zero-per-pixel-float smooth sky & ground gradient renderer at EVERY roll angle (0-360 deg),
    /// using a 128-entry precomputed RGB565 LUT + 16.16 fixed-point scanlines, plus an unbroken horizon line!
    fn draw_sky_and_ground(&mut self, cam_rot: Quat, pal: ThemePalette, detail_high: bool) {
        let sw = self.width;
        let sh = self.height;
        let w = self.w();
        let half_w = self.half_w();
        let half_h = self.half_h();
        let (cam_right, cam_up, cam_fwd) = cam_rot.basis();
        let ux = cam_right.y;
        let uy = cam_up.y;
        let uz = cam_fwd.y * self.focal;
        let inv_f = 1.0 / self.focal;

        if !detail_high {
            // Fast 2-tone flat mode when Gfx Detail = Low
            if ux.abs() < 0.002 {
                let mut prev_sky: Option<bool> = None;
                for y in 0..sh {
                    let ry = half_h - (y as f32);
                    let is_sky = (uy * ry + uz) >= 0.0;
                    let row = y * sw;
                    self.fb[row..row + sw].fill(if is_sky { pal.sky_top } else { pal.ground_near });
                    if let (Some(was_sky), Some(hcol)) = (prev_sky, pal.horizon_line) {
                        if was_sky != is_sky {
                            self.fb[row..row + sw].fill(hcol);
                        }
                    }
                    prev_sky = Some(is_sky);
                }
            } else {
                let inv_ux = 1.0 / ux;
                let sky_on_right = ux > 0.0;
                for y in 0..sh {
                    let ry = half_h - (y as f32);
                    let x_hor = half_w - (uy * ry + uz) * inv_ux;
                    let x_split = (x_hor.round() as i32).clamp(0, w) as usize;
                    let row = y * sw;
                    let (left_col, right_col) = if sky_on_right {
                        (pal.ground_near, pal.sky_top)
                    } else {
                        (pal.sky_top, pal.ground_near)
                    };
                    if x_split > 0 {
                        self.fb[row..row + x_split].fill(left_col);
                    }
                    if x_split < sw {
                        self.fb[row + x_split..row + sw].fill(right_col);
                    }
                    if let Some(hcol) = pal.horizon_line {
                        let x_hor_next = half_w - (uy * (ry - 1.0) + uz) * inv_ux;
                        let lx = x_hor.min(x_hor_next).round() as i32;
                        let rx = x_hor.max(x_hor_next).round() as i32;
                        if rx >= 0 && lx < w {
                            self.draw_hline(lx.max(0), rx.min(w - 1), y as i32, hcol);
                        }
                    }
                }
            }
            return;
        }

        // Build 128-entry elevation -> RGB565 LUT once per frame (256 bytes on stack).
        // Index 64 corresponds to elev == 0.0 (horizon): [0..64) = ground gradient, [64..128) = sky gradient.
        const LUT_SCALE: f32 = 112.0;
        let mut lut = [0u16; 128];
        for i in 0..64 {
            let elev = (i as f32 - 64.0) / LUT_SCALE; // negative (below horizon)
            let t = (-elev * 2.2).min(1.0);
            lut[i] = lerp_rgb565(pal.ground_far, pal.ground_near, t);
        }
        for i in 64..128 {
            let elev = (i as f32 - 64.0) / LUT_SCALE; // >= 0 (above horizon)
            let t = (elev * 1.8).min(1.0);
            lut[i] = lerp_rgb565(pal.sky_horizon, pal.sky_top, t);
        }

        // 16.16 fixed-point horizontal elevation step across each scanline
        let step_fp: i32 = (ux * inv_f * LUT_SCALE * 65536.0).round() as i32;
        let step_total = step_fp * ((sw - 1) as i32);
        let rolled = ux.abs() >= 0.002;
        let inv_ux = if rolled { 1.0 / ux } else { 0.0 };
        let mut prev_sky: Option<bool> = None;

        for y in 0..sh {
            let ry = half_h - (y as f32);
            let base = uy * ry + uz;
            let row = y * sw;
            let e_left = (ux * (-half_w) + base) * inv_f;
            let mut acc: i32 = ((64.0 + e_left * LUT_SCALE) * 65536.0) as i32;
            let acc_right = acc + step_total;

            // Fast-path uniform scanlines (flat/upside-down or fully clamped sky/ground)
            if step_fp == 0 {
                let idx = (acc >> 16).clamp(0, 127) as usize;
                self.fb[row..row + sw].fill(lut[idx]);
            } else if acc <= 0 && acc_right <= 0 {
                self.fb[row..row + sw].fill(lut[0]);
            } else if acc >= (127 << 16) && acc_right >= (127 << 16) {
                self.fb[row..row + sw].fill(lut[127]);
            } else {
                // Smooth per-pixel rolled gradient using pure integer fixed-point LUT lookup (zero floats!)
                for px in &mut self.fb[row..row + sw] {
                    let idx = (acc >> 16).clamp(0, 127) as usize;
                    *px = lut[idx];
                    acc += step_fp;
                }
            }

            // Crisp unbroken horizon glow line in Evening / Night / Wireframe themes
            if let Some(hcol) = pal.horizon_line {
                if !rolled {
                    let is_sky = base >= 0.0;
                    if let Some(was_sky) = prev_sky {
                        if was_sky != is_sky {
                            self.fb[row..row + sw].fill(hcol);
                        }
                    }
                    prev_sky = Some(is_sky);
                } else {
                    let x0 = half_w - base * inv_ux;
                    let x1 = half_w - (uy * (ry - 1.0) + uz) * inv_ux;
                    let lx = x0.min(x1).round() as i32;
                    let rx = x0.max(x1).round() as i32;
                    if rx >= 0 && lx < w {
                        self.draw_hline(lx.max(0), rx.min(w - 1), y as i32, hcol);
                    }
                }
            }
        }
    }

    fn draw_sky_detail(&mut self, cam_inv: Quat, theme: VisualTheme) {
        let s = self.ui_scale();
        // Sun / Moon disc
        let sun_spec = match theme {
            VisualTheme::Summer => Some((
                Vec3::new(0.45, 0.48, 0.75),
                rgb565(255, 252, 200),
                rgb565(255, 235, 140),
            )),
            VisualTheme::Evening => Some((
                Vec3::new(0.35, 0.16, 0.92),
                rgb565(255, 210, 95),
                rgb565(245, 130, 60),
            )),
            VisualTheme::Night => Some((
                Vec3::new(-0.40, 0.52, 0.75),
                rgb565(240, 246, 255),
                rgb565(130, 155, 215),
            )),
            VisualTheme::Wireframe => None,
        };
        if let Some((sun_dir, sun_col, halo_col)) = sun_spec {
            let sc = cam_inv.rotate(sun_dir);
            if sc.z > 0.15 {
                let p = self.project_sv(sc);
                let sx = p.x as i32;
                let sy = p.y as i32;
                self.fill_rect(sx - 5 * s, sy - 5 * s, 11 * s, 11 * s, halo_col);
                self.fill_rect(sx - 3 * s, sy - 3 * s, 7 * s, 7 * s, sun_col);
            }
        }
        // Rich sky starfield in Night & Wireframe modes so sky is unmistakably distinct from ground
        if theme == VisualTheme::Night || theme == VisualTheme::Wireframe {
            const STARS: [Vec3; 20] = [
                Vec3::new(0.60, 0.50, 0.60),
                Vec3::new(-0.70, 0.40, 0.55),
                Vec3::new(0.20, 0.75, 0.62),
                Vec3::new(-0.25, 0.65, -0.70),
                Vec3::new(0.75, 0.35, -0.55),
                Vec3::new(-0.65, 0.55, -0.50),
                Vec3::new(0.10, 0.45, -0.88),
                Vec3::new(-0.85, 0.30, 0.42),
                Vec3::new(0.42, 0.68, -0.60),
                Vec3::new(-0.15, 0.32, 0.93),
                Vec3::new(0.05, 0.95, 0.15),
                Vec3::new(-0.45, 0.80, 0.35),
                Vec3::new(0.50, 0.78, -0.30),
                Vec3::new(-0.88, 0.25, -0.38),
                Vec3::new(0.88, 0.28, 0.32),
                Vec3::new(0.28, 0.25, 0.92),
                Vec3::new(-0.32, 0.22, -0.90),
                Vec3::new(0.62, 0.22, -0.74),
                Vec3::new(-0.55, 0.62, 0.55),
                Vec3::new(0.35, 0.58, 0.73),
            ];
            let star_col = if theme == VisualTheme::Night {
                rgb565(225, 238, 255)
            } else {
                rgb565(75, 195, 230)
            };
            for (idx, &st) in STARS.iter().enumerate() {
                let c = cam_inv.rotate(st);
                if c.z > 0.2 {
                    let p = self.project_sv(c);
                    let px = p.x as i32;
                    let py = p.y as i32;
                    if s == 1 {
                        self.put_pixel(px, py, star_col);
                        if idx % 3 == 0 {
                            self.put_pixel(px + 1, py, star_col);
                        }
                    } else {
                        let w_star = if idx % 3 == 0 { s + 1 } else { s };
                        self.fill_rect(px, py, w_star, s, star_col);
                    }
                }
            }
        }
    }

    /// Render the 3D world (sky, ground grid, Z-buffered structures & gates) from an arbitrary camera.
    fn render_world_scene(
        &mut self,
        cam_pos: Vec3,
        cam_rot: Quat,
        cfg: &Config,
        track: &Track,
        active_gate: Option<usize>,
    ) {
        let pal = palette(cfg.theme);
        let half_fov_rad = ((cfg.fov_deg as f32).clamp(70.0, 115.0) * 0.5).to_radians();
        self.focal = self.half_w() / half_fov_rad.tan();

        // Clear 16-bit per-pixel Z-buffer (0 = farthest)
        self.zbuf.fill(0);

        let cam_inv = cam_rot.conjugate();

        // 1. Sky & Ground + Horizon Line + Sun/Moon/Stars
        self.draw_sky_and_ground(cam_rot, pal, cfg.detail_high);
        if cfg.sky_detail || cfg.theme == VisualTheme::Night || cfg.theme == VisualTheme::Wireframe {
            self.draw_sky_detail(cam_inv, cfg.theme);
        }

        // 2. Crisp 8m ground orientation grid (drawn before 3D geometry with Z=0)
        let step = if cfg.detail_high { 8 } else { 16 };
        let half_cnt = if cfg.detail_high { 5 } else { 3 };
        let span = (half_cnt * step) as f32;
        let gx0 = (cam_pos.x / step as f32).floor() as i32 * step;
        let gz0 = (cam_pos.z / step as f32).floor() as i32 * step;
        for i in -half_cnt..=half_cnt {
            let x = (gx0 + i * step) as f32;
            self.draw_ground_line(
                Vec3::new(x, 0.0, gz0 as f32 - span),
                Vec3::new(x, 0.0, gz0 as f32 + span),
                cam_pos,
                cam_inv,
                pal.grid_line,
            );
            let z = (gz0 + i * step) as f32;
            self.draw_ground_line(
                Vec3::new(gx0 as f32 - span, 0.0, z),
                Vec3::new(gx0 as f32 + span, 0.0, z),
                cam_pos,
                cam_inv,
                pal.grid_line,
            );
        }

        // 3. Structures — Two-pass Z-buffered hidden-line rendering:
        //    Pass 3A fills all solid faces into the Z-buffer first;
        //    Pass 3B draws all visible outlines against the completed Z-buffer so NO back edges ever overlap!
        let struct_colors = |color: StructureColor, wf: bool| -> (u16, u16, u16, u16) {
            if wf {
                match color {
                    StructureColor::Concrete => (
                        rgb565(18, 46, 70),
                        rgb565(12, 32, 52),
                        rgb565(8, 22, 38),
                        rgb565(65, 235, 255), // Electric Cyan walls
                    ),
                    StructureColor::Steel => (
                        rgb565(30, 24, 66),
                        rgb565(20, 16, 48),
                        rgb565(14, 11, 34),
                        rgb565(160, 148, 255), // Electric Indigo/Violet pillars
                    ),
                    StructureColor::RedWhite => (
                        rgb565(52, 14, 38),
                        rgb565(36, 10, 28),
                        rgb565(24, 7, 18),
                        rgb565(255, 82, 188), // Neon Magenta towers
                    ),
                    StructureColor::Wood => (
                        rgb565(48, 30, 10),
                        rgb565(34, 22, 8),
                        rgb565(24, 14, 6),
                        rgb565(255, 188, 68), // Neon Amber barns/crates
                    ),
                    StructureColor::DarkWood => (
                        rgb565(38, 22, 12),
                        rgb565(28, 16, 8),
                        rgb565(20, 11, 6),
                        rgb565(225, 138, 72), // Warm copper trunks/fences
                    ),
                    StructureColor::Foliage => (
                        rgb565(14, 52, 26),
                        rgb565(10, 38, 18),
                        rgb565(6, 26, 12),
                        rgb565(90, 255, 130), // Neon Emerald tree canopies
                    ),
                    StructureColor::Hay => (
                        rgb565(56, 44, 12),
                        rgb565(42, 32, 8),
                        rgb565(30, 22, 6),
                        rgb565(255, 228, 88), // Neon Gold hay bales
                    ),
                }
            } else {
                match color {
                    StructureColor::Concrete => (
                        rgb565(175, 178, 182),
                        rgb565(138, 142, 148),
                        rgb565(106, 110, 118),
                        rgb565(72, 76, 82),
                    ),
                    StructureColor::Steel => (
                        rgb565(115, 132, 152),
                        rgb565(82, 98, 118),
                        rgb565(58, 72, 92),
                        rgb565(40, 52, 68),
                    ),
                    StructureColor::RedWhite => (
                        rgb565(235, 235, 235),
                        rgb565(215, 58, 52),
                        rgb565(170, 42, 40),
                        rgb565(90, 22, 22),
                    ),
                    StructureColor::Wood => (
                        rgb565(175, 134, 86),
                        rgb565(145, 106, 62),
                        rgb565(112, 80, 46),
                        rgb565(72, 50, 28),
                    ),
                    StructureColor::DarkWood => (
                        rgb565(126, 88, 54),
                        rgb565(98, 66, 38),
                        rgb565(76, 50, 28),
                        rgb565(48, 32, 18),
                    ),
                    StructureColor::Foliage => (
                        rgb565(72, 156, 68),
                        rgb565(48, 122, 46),
                        rgb565(34, 92, 34),
                        rgb565(22, 60, 22),
                    ),
                    StructureColor::Hay => (
                        rgb565(228, 196, 86),
                        rgb565(202, 166, 62),
                        rgb565(168, 134, 46),
                        rgb565(112, 86, 26),
                    ),
                }
            }
        };

        let struct_in_frustum = |s: &crate::tracks::Structure| -> bool {
            let c = s.center();
            let hx = (s.max.x - s.min.x) * 0.5;
            let hy = (s.max.y - s.min.y) * 0.5;
            let hz = (s.max.z - s.min.z) * 0.5;
            let rad = (hx * hx + hy * hy + hz * hz).sqrt();
            let rel = cam_inv.rotate(c.sub(cam_pos));
            if rel.z + rad < 0.05 || rel.z - rad > 185.0 {
                return false;
            }
            if rel.z > 0.0 && rel.x.abs() - rad > rel.z * 1.55 {
                return false;
            }
            true
        };

        // Pass 3A: Rasterize all visible structure faces into framebuffer + Z-buffer
        for s in track.structures {
            if !struct_in_frustum(s) {
                continue;
            }
            let (c_top, c_side_a, c_side_b, _) = struct_colors(s.color, pal.wireframe);
            let (x0, y0, z0) = (s.min.x, s.min.y, s.min.z);
            let (x1, y1, z1) = (s.max.x, s.max.y, s.max.z);
            if cam_pos.y > y1 {
                self.draw_3d_quad_fill_z(
                    [Vec3::new(x0, y1, z0), Vec3::new(x1, y1, z0), Vec3::new(x1, y1, z1), Vec3::new(x0, y1, z1)],
                    cam_pos, cam_inv, c_top,
                );
            }
            if cam_pos.y < y0 {
                self.draw_3d_quad_fill_z(
                    [Vec3::new(x0, y0, z0), Vec3::new(x1, y0, z0), Vec3::new(x1, y0, z1), Vec3::new(x0, y0, z1)],
                    cam_pos, cam_inv, c_side_b,
                );
            }
            if cam_pos.z < z0 {
                self.draw_3d_quad_fill_z(
                    [Vec3::new(x0, y0, z0), Vec3::new(x1, y0, z0), Vec3::new(x1, y1, z0), Vec3::new(x0, y1, z0)],
                    cam_pos, cam_inv, c_side_a,
                );
            }
            if cam_pos.z > z1 {
                self.draw_3d_quad_fill_z(
                    [Vec3::new(x0, y0, z1), Vec3::new(x1, y0, z1), Vec3::new(x1, y1, z1), Vec3::new(x0, y1, z1)],
                    cam_pos, cam_inv, c_side_a,
                );
            }
            if cam_pos.x < x0 {
                self.draw_3d_quad_fill_z(
                    [Vec3::new(x0, y0, z0), Vec3::new(x0, y0, z1), Vec3::new(x0, y1, z1), Vec3::new(x0, y1, z0)],
                    cam_pos, cam_inv, c_side_b,
                );
            }
            if cam_pos.x > x1 {
                self.draw_3d_quad_fill_z(
                    [Vec3::new(x1, y0, z0), Vec3::new(x1, y0, z1), Vec3::new(x1, y1, z1), Vec3::new(x1, y1, z0)],
                    cam_pos, cam_inv, c_side_b,
                );
            }
        }

        // Pass 3B: Draw all visible structure edges strictly occluded against the completed Z-buffer
        for s in track.structures {
            if !struct_in_frustum(s) {
                continue;
            }
            let (_, _, _, c_edge) = struct_colors(s.color, pal.wireframe);
            let (x0, y0, z0) = (s.min.x, s.min.y, s.min.z);
            let (x1, y1, z1) = (s.max.x, s.max.y, s.max.z);
            if cam_pos.y > y1 {
                self.draw_3d_quad_edges_z(
                    [Vec3::new(x0, y1, z0), Vec3::new(x1, y1, z0), Vec3::new(x1, y1, z1), Vec3::new(x0, y1, z1)],
                    cam_pos, cam_inv, c_edge,
                );
            }
            if cam_pos.y < y0 {
                self.draw_3d_quad_edges_z(
                    [Vec3::new(x0, y0, z0), Vec3::new(x1, y0, z0), Vec3::new(x1, y0, z1), Vec3::new(x0, y0, z1)],
                    cam_pos, cam_inv, c_edge,
                );
            }
            if cam_pos.z < z0 {
                self.draw_3d_quad_edges_z(
                    [Vec3::new(x0, y0, z0), Vec3::new(x1, y0, z0), Vec3::new(x1, y1, z0), Vec3::new(x0, y1, z0)],
                    cam_pos, cam_inv, c_edge,
                );
            }
            if cam_pos.z > z1 {
                self.draw_3d_quad_edges_z(
                    [Vec3::new(x0, y0, z1), Vec3::new(x1, y0, z1), Vec3::new(x1, y1, z1), Vec3::new(x0, y1, z1)],
                    cam_pos, cam_inv, c_edge,
                );
            }
            if cam_pos.x < x0 {
                self.draw_3d_quad_edges_z(
                    [Vec3::new(x0, y0, z0), Vec3::new(x0, y0, z1), Vec3::new(x0, y1, z1), Vec3::new(x0, y1, z0)],
                    cam_pos, cam_inv, c_edge,
                );
            }
            if cam_pos.x > x1 {
                self.draw_3d_quad_edges_z(
                    [Vec3::new(x1, y0, z0), Vec3::new(x1, y0, z1), Vec3::new(x1, y1, z1), Vec3::new(x1, y1, z0)],
                    cam_pos, cam_inv, c_edge,
                );
            }
        }

        // 4. Solid 3D Racing Gates (Z-buffered volume posts & beams so they never disappear!)
        for (idx, g) in track.gates.iter().enumerate() {
            let is_next = active_gate.map_or(true, |ag| idx == ag);
            let main_col = if is_next { pal.active_gate } else { pal.idle_gate };
            let hi_col = if is_next {
                pal.active_gate_hi
            } else if pal.wireframe {
                rgb565(255, 200, 80)
            } else {
                rgb565(225, 230, 235)
            };
            let wf = pal.wireframe;

            match g.kind {
                GateKind::Box => {
                    let right = g.right();
                    let fwd = g.forward();
                    let r = right.scale(g.width * 0.5);
                    let p_bl = g.pos.sub(r);
                    let p_br = g.pos.add(r);
                    let p_tl = Vec3::new(p_bl.x, g.pos.y + g.height, p_bl.z);
                    let p_tr = Vec3::new(p_br.x, g.pos.y + g.height, p_br.z);

                    // Left & Right vertical 3D posts
                    self.draw_gate_beam(p_bl, p_tl, right, fwd, 0.14, cam_pos, cam_inv, main_col, hi_col, wf);
                    self.draw_gate_beam(p_br, p_tr, right, fwd, 0.14, cam_pos, cam_inv, main_col, hi_col, wf);
                    // Top horizontal 3D crossbar + banner
                    self.draw_gate_beam(p_tl, p_tr, Vec3::UP, fwd, 0.18, cam_pos, cam_inv, hi_col, main_col, wf);
                    if is_next && !wf {
                        let b_tl = Vec3::new(p_tl.x, p_tl.y + 0.52, p_tl.z);
                        let b_tr = Vec3::new(p_tr.x, p_tr.y + 0.52, p_tr.z);
                        self.draw_3d_quad_z([p_tl, p_tr, b_tr, b_tl], cam_pos, cam_inv, main_col, hi_col, false);
                    }
                    // Base line + stilts
                    self.draw_3d_line_z(p_bl, p_br, cam_pos, cam_inv, main_col, false);
                    if g.pos.y > 0.2 {
                        let g_bl = Vec3::new(p_bl.x, 0.0, p_bl.z);
                        let g_br = Vec3::new(p_br.x, 0.0, p_br.z);
                        self.draw_3d_line_z(g_bl, p_bl, cam_pos, cam_inv, rgb565(100, 108, 120), true);
                        self.draw_3d_line_z(g_br, p_br, cam_pos, cam_inv, rgb565(100, 108, 120), true);
                    }
                }
                GateKind::Dive => {
                    let hw = g.width * 0.5;
                    let c0 = Vec3::new(g.pos.x - hw, g.pos.y, g.pos.z - hw);
                    let c1 = Vec3::new(g.pos.x + hw, g.pos.y, g.pos.z - hw);
                    let c2 = Vec3::new(g.pos.x + hw, g.pos.y, g.pos.z + hw);
                    let c3 = Vec3::new(g.pos.x - hw, g.pos.y, g.pos.z + hw);
                    self.draw_gate_beam(c0, c1, Vec3::UP, Vec3::FORWARD, 0.16, cam_pos, cam_inv, hi_col, main_col, wf);
                    self.draw_gate_beam(c1, c2, Vec3::UP, Vec3::RIGHT, 0.16, cam_pos, cam_inv, main_col, hi_col, wf);
                    self.draw_gate_beam(c2, c3, Vec3::UP, Vec3::FORWARD, 0.16, cam_pos, cam_inv, hi_col, main_col, wf);
                    self.draw_gate_beam(c3, c0, Vec3::UP, Vec3::RIGHT, 0.16, cam_pos, cam_inv, main_col, hi_col, wf);
                }
                GateKind::Flag => {
                    let top = Vec3::new(g.pos.x, g.height, g.pos.z);
                    let tip = Vec3::new(g.pos.x + 1.2, g.height - 0.8, g.pos.z);
                    self.draw_3d_line_z(g.pos, top, cam_pos, cam_inv, hi_col, true);
                    self.draw_3d_line_z(top, tip, cam_pos, cam_inv, main_col, true);
                }
            }
        }
    }

    /// Render the full 3D FPV scene with per-pixel Z-buffer + Betaflight OSD!
    pub fn render_flight(
        &mut self,
        quad: &QuadState,
        cfg: &Config,
        track: &Track,
        sticks: StickInput,
        fps: u32,
        frame_ms: f32,
    ) {
        let tilt = -(cfg.camera_tilt_deg as f32).to_radians();
        let cam_rot = quad.rot.mul(Quat::from_axis_angle(Vec3::RIGHT, tilt)).normalized();
        let cam_inv = cam_rot.conjugate();
        let cam_pos = quad.pos;
        let active_gate = if cfg.game_mode == GameMode::Freestyle {
            None
        } else {
            Some(quad.next_gate)
        };
        self.render_world_scene(cam_pos, cam_rot, cfg, track, active_gate);

        // 5. Hit flash border
        if quad.hit_flash > 0.0 {
            let red = rgb565(255, 45, 45);
            let s = self.ui_scale();
            self.draw_rect_thick(0, 0, self.w(), self.h(), 2 * s, red);
        }

        // 6. Next-gate screen-space beacon / off-screen edge pointer (Race mode)
        if cfg.game_mode == GameMode::Race && !track.gates.is_empty() {
            let target = track.gates[quad.next_gate].center();
            let tc = Self::world_to_cam(target, cam_pos, cam_inv);
            let marker_col = rgb565(255, 230, 40);
            let s = self.ui_scale();
            let w = self.w();
            let h = self.h();
            if tc.z > 0.5 {
                let p = self.project_sv(tc);
                let sx = p.x as i32;
                let sy = p.y as i32;
                if sx >= 8 * s && sx < w - 8 * s && sy >= 14 * s && sy < h - 14 * s {
                    for t in 0..s {
                        self.draw_line(sx - 4 * s, sy - 9 * s + t, sx, sy - 5 * s + t, marker_col);
                        self.draw_line(sx + 4 * s, sy - 9 * s + t, sx, sy - 5 * s + t, marker_col);
                    }
                } else {
                    self.draw_edge_arrow(tc, marker_col);
                }
            } else {
                self.draw_edge_arrow(tc, marker_col);
            }
        }

        // 7. Optional fast Analog 5.8GHz FPV video filter (NTSC scanlines + vignette + RF grain)
        if cfg.analog_filter {
            self.apply_analog_filter(quad.hit_flash > 0.0);
        }

        // 8. Authentic Betaflight OSD (drawn crisp over analog feed)
        self.draw_osd(quad, cfg, track, sticks, fps, frame_ms);
    }

    fn apply_analog_filter(&mut self, static_burst: bool) {
        let mut s = self.rng_state;
        let grain_mask: u32 = if static_burst { 0x07 } else { 0x3F };
        let sw = self.width;
        let sh = self.height;
        let y_edge_px = (sh / 14).max(12);
        let x_edge_px = (sw / 18).max(18);
        for y in 0..sh {
            let row = y * sw;
            let scanline = (y & 1) == 1;
            let y_edge = y < y_edge_px || y >= sh - y_edge_px;
            for x in 0..sw {
                let mut c = self.fb[row + x];
                if scanline {
                    c = ((c >> 1) & 0x7BEF) + ((c >> 2) & 0x39E7) + ((c >> 3) & 0x18E3);
                }
                if y_edge && (x < x_edge_px || x >= sw - x_edge_px) {
                    c = ((c >> 1) & 0x7BEF) + ((c >> 2) & 0x39E7);
                }
                s ^= s << 13;
                s ^= s >> 17;
                s ^= s << 5;
                if (s & grain_mask) == 0 {
                    let luma = if (s & 0x100) != 0 {
                        rgb565(200, 200, 200)
                    } else {
                        rgb565(40, 40, 40)
                    };
                    c = ((c >> 1) & 0x7BEF) + ((luma >> 1) & 0x7BEF);
                }
                self.fb[row + x] = c;
            }
        }
        self.rng_state = s;
    }

    fn draw_edge_arrow(&mut self, tc: Vec3, color: u16) {
        let s = self.ui_scale();
        let sf = s as f32;
        let half_w = self.half_w();
        let half_h = self.half_h();
        let dir_x = tc.x;
        let dir_y = -tc.y;
        let len = (dir_x * dir_x + dir_y * dir_y).sqrt().max(0.001);
        let nx = dir_x / len;
        let ny = dir_y / len;
        let ax = clampf(half_w + nx * (half_w - 14.0 * sf), 10.0 * sf, (self.w() - 10 * s) as f32) as i32;
        let ay = clampf(half_h + ny * (half_h - 16.0 * sf), 16.0 * sf, (self.h() - 16 * s) as f32) as i32;
        self.fill_rect(ax - 2 * s, ay - 2 * s, 5 * s, 5 * s, rgb565(0, 0, 0));
        self.fill_rect(ax - s, ay - s, 3 * s, 3 * s, color);
    }

    fn draw_osd(
        &mut self,
        quad: &QuadState,
        cfg: &Config,
        track: &Track,
        sticks: StickInput,
        fps: u32,
        frame_ms: f32,
    ) {
        let white = rgb565(255, 255, 255);
        let cyan = rgb565(120, 240, 255);
        let yellow = rgb565(255, 225, 65);
        let green = rgb565(85, 255, 120);
        let red = rgb565(255, 70, 70);
        let batt_on = cfg.battery_pack != BatteryPack::Off;

        let w = self.w();
        let h = self.h();
        let s = self.ui_scale();
        let cx = w / 2;
        let cy = h / 2;

        // Post-Flight Betaflight Stats Card when Disarmed after flying or when Battery is Empty!
        if (!quad.armed && (quad.has_flown || quad.battery_empty)) || quad.battery_empty {
            self.draw_post_flight_stats(quad, cfg);
            return;
        }

        // Betaflight crosshair
        self.fill_rect(cx - 5 * s, cy, 4 * s, s, white);
        self.fill_rect(cx + 2 * s, cy, 4 * s, s, white);
        self.fill_rect(cx, cy - 3 * s, s, 2 * s, white);

        // Top-left: Mode (IMU EASY / ACRO / HORIZON / ANGLE / TURTLE) + Gate / Points
        let imu_mode = quad.control_source == FlightControlSource::ImuSimplified;
        let (mode_str, mode_col) = if quad.turtle_mode {
            ("TURTLE", yellow)
        } else if imu_mode {
            ("IMU EASY", cyan)
        } else {
            (cfg.flight_mode.name(), white)
        };
        self.draw_text(4 * s, 3 * s, mode_str, mode_col, s, true);
        if cfg.game_mode == GameMode::Race {
            let g_str = format!("G {}/{}", quad.next_gate + 1, track.gates.len());
            self.draw_text(4 * s, 13 * s, &g_str, yellow, s, true);
            let s_str = format!("PTS {}", quad.score);
            self.draw_text(4 * s, 23 * s, &s_str, green, s, true);
        } else {
            let s_str = format!("PTS {}", quad.score);
            self.draw_text(4 * s, 13 * s, &s_str, yellow, s, true);
        }

        // Top-center: Betaflight Flight Timer (Freestyle) or Lap Timer (Time Trial)
        if cfg.game_mode == GameMode::Race {
            let secs = quad.lap_time;
            let mins = (secs / 60.0) as u32;
            let rem = secs - (mins as f32) * 60.0;
            let t_str = format!("{mins}:{rem:05.2}");
            self.draw_text(cx - (t_str.len() as i32) * 3 * s, 3 * s, &t_str, white, s, true);
            let best = cfg.best_laps_ms[cfg.track_index.min(3)];
            if best > 0 {
                let bs = best as f32 / 1000.0;
                let bm = (bs / 60.0) as u32;
                let br = bs - (bm as f32) * 60.0;
                let b_str = format!("B {bm}:{br:05.2}");
                self.draw_text(cx - (b_str.len() as i32) * 3 * s, 13 * s, &b_str, green, s, true);
            }
        } else {
            let ft = quad.flight_time as u32;
            let mins = ft / 60;
            let secs = ft % 60;
            let t_str = format!("{mins:02}:{secs:02}");
            self.draw_text(cx - 15 * s, 3 * s, &t_str, white, s, true);
        }

        // Top-right: Betaflight Throttle % + mAh used (+ optional Debug FPS/ms)
        let thr_str = format!("THR {:3}%", quad.throttle_pct);
        self.draw_text(w - 52 * s, 3 * s, &thr_str, white, s, true);
        if batt_on {
            let mah_str = format!("{:3.0}mAh", quad.mah_used);
            self.draw_text(w - 40 * s, 13 * s, &mah_str, cyan, s, true);
        }
        if cfg.debug_osd {
            let perf_str = format!("{}F {:.1}ms", fps, frame_ms);
            let pw = (perf_str.len() as i32) * 6 * s;
            self.draw_text(w - pw - 4 * s, 23 * s, &perf_str, rgb565(180, 195, 210), s, true);
        }

        // Bottom-left: Betaflight Cell Voltage
        let batt_y = if cfg.stick_overlay.is_visible() { h - 41 * s } else { h - 11 * s };
        if batt_on {
            let v_col = if quad.cell_voltage < 3.38 {
                red
            } else if quad.cell_voltage < 3.55 {
                yellow
            } else {
                green
            };
            let v_str = format!("{:.2}V", quad.cell_voltage);
            self.draw_text(4 * s, batt_y, &v_str, v_col, s, true);
        }

        // Bottom-center: Betaflight Craft Name + Speed / Altitude
        let craft = cfg.quad.craft_name();
        self.draw_text(cx - (craft.len() as i32) * 3 * s, h - 20 * s, craft, rgb565(215, 225, 235), s, true);
        let spd = format!("{:3.0}KPH {:2.0}M", quad.speed_kmh(), (quad.pos.y - 0.22).max(0.0));
        self.draw_text(cx - (spd.len() as i32) * 3 * s, h - 10 * s, &spd, white, s, true);

        // Center Betaflight Warnings & Score Popups
        if quad.arm_blocked_throttle {
            self.draw_text(cx - 42 * s, cy - 22 * s, "THROTTLE HIGH", red, s, true);
        } else if quad.arm_aux_connected && !quad.armed {
            self.draw_text(cx - 24 * s, cy - 22 * s, "DISARMED", yellow, s, true);
            let hint = match cfg.aux_arm {
                AuxBinding::Button { .. } => "PRESS ARM BUTTON",
                _ => "FLIP ARM SWITCH",
            };
            self.draw_text(cx - (hint.len() as i32) * 3 * s, cy - 11 * s, hint, white, s, true);
        } else if !quad.arm_aux_connected
            && quad.on_ground
            && !quad.has_flown
            && !quad.inverted
            && !quad.battery_empty
        {
            match quad.control_source {
                FlightControlSource::ImuSimplified => {
                    self.draw_text(cx - 15 * s, cy - 22 * s, "READY", green, s, true);
                    let hint = "[F/UP] FLY  TILT STEER  [SPC] ZERO";
                    self.draw_text(cx - (hint.len() as i32) * 3 * s, cy - 11 * s, hint, white, s, true);
                }
                FlightControlSource::None => {
                    let title = "CONTROLLER REQUIRED";
                    self.draw_text(cx - (title.len() as i32) * 3 * s, cy - 22 * s, title, red, s, true);
                    let hint = "CONNECT USB/BT CONTROLLER";
                    self.draw_text(cx - (hint.len() as i32) * 3 * s, cy - 11 * s, hint, white, s, true);
                }
                FlightControlSource::Controller => {
                    self.draw_text(cx - 15 * s, cy - 22 * s, "READY", green, s, true);
                }
            }
        } else if quad.turtle_active && quad.turtle_mode {
            self.draw_text(cx - 30 * s, cy - 22 * s, "CRASH FLIP", yellow, s, true);
            let sub = if quad.inverted {
                "PUSH ROLL / PITCH"
            } else if quad.turtle_aux_connected {
                "RELEASE CRASH FLIP"
            } else {
                "PRESS [T] TO EXIT"
            };
            self.draw_text(cx - (sub.len() as i32) * 3 * s, cy - 11 * s, sub, if quad.inverted { white } else { green }, s, true);
        } else if quad.combo_timer > 0.0 && !quad.combo_text.is_empty() {
            let tw = (quad.combo_text.len() as i32) * 6 * s;
            self.draw_text(cx - tw / 2, 32 * s, quad.combo_text, yellow, s, true);
        } else if quad.on_ground && quad.inverted {
            let prompt = if quad.turtle_aux_connected {
                "TURTLE: PRESS CRASH FLIP"
            } else {
                "TURTLE: ROLL TO FLIP"
            };
            self.draw_text(cx - (prompt.len() as i32) * 3 * s, 32 * s, prompt, red, s, true);
        } else if batt_on && quad.cell_voltage < 3.35 {
            self.draw_text(cx - 24 * s, 32 * s, "LAND NOW", red, s, true);
        } else if batt_on && quad.cell_voltage < 3.48 {
            self.draw_text(cx - 33 * s, 32 * s, "LOW BATTERY", yellow, s, true);
        }

        // Compact live stick / IMU control overlay in bottom corners (Off / Dot / Trails)
        if cfg.stick_overlay.is_visible() {
            if imu_mode {
                let show_trails = cfg.stick_overlay == StickOverlay::Trails;
                if show_trails {
                    self.push_stick_trail(0.0, quad.throttle_pct as f32 / 50.0 - 1.0, sticks.roll, sticks.pitch);
                }
                self.draw_imu_throttle_bar(4 * s, h - 30 * s, 14 * s, 26 * s, quad.throttle_pct, quad.imu_climb_cmd, quad.on_ground, s);
                self.draw_text(w - 26 * s, h - 39 * s, "IMU", cyan, s, true);
                self.draw_imu_tilt_box(w - 30 * s, h - 30 * s, 26 * s, sticks.roll, sticks.pitch, show_trails, s);
            } else {
                let lx = sticks.yaw;
                let ly = sticks.throttle * 2.0 - 1.0;
                let rx = sticks.roll;
                let ry = sticks.pitch;
                if cfg.stick_overlay == StickOverlay::Trails {
                    self.push_stick_trail(lx, ly, rx, ry);
                    self.draw_stick_box_trails(4 * s, h - 30 * s, 26 * s, lx, ly, true, s);
                    self.draw_stick_box_trails(w - 30 * s, h - 30 * s, 26 * s, rx, ry, false, s);
                } else {
                    self.draw_stick_box(4 * s, h - 30 * s, 26 * s, lx, ly, s);
                    self.draw_stick_box(w - 30 * s, h - 30 * s, 26 * s, rx, ry, s);
                }
            }
        }
    }

    fn draw_post_flight_stats(&mut self, quad: &QuadState, cfg: &Config) {
        let ui = ui_colors(cfg.ui_theme);
        let mx = 46;
        let my = 20;
        let mw = UI_W - 92;
        let mh = UI_H - 40;
        self.ui_fill_rect(mx, my, mw, mh, ui.header_bg);
        self.ui_draw_rect(mx, my, mw, mh, ui.sel_border);

        let title = if quad.battery_empty {
            "BATTERY EMPTY"
        } else {
            "DISARMED - STATS"
        };
        let tcol = if quad.battery_empty { rgb565(255, 80, 80) } else { ui.sel_label };
        self.ui_draw_text(UI_W / 2 - (title.len() as i32) * 3, my + 8, title, tcol, 1, false);
        self.ui_draw_hline(mx + 8, mx + mw - 9, my + 19, ui.header_line);

        let ft = quad.flight_time as u32;
        let mins = ft / 60;
        let secs = ft % 60;
        let lines = [
            ("FLIGHT TIME", format!("{mins:02}:{secs:02}")),
            ("USED MAH", format!("{:.0} mAh", quad.mah_used)),
            ("MIN VOLTAGE", format!("{:.2} V", quad.min_cell_voltage)),
            ("MAX SPEED", format!("{:.0} KPH", quad.max_speed_kmh)),
            ("SCORE", format!("{} PTS", quad.score)),
        ];

        for (i, (k, v)) in lines.iter().enumerate() {
            let y = my + 26 + (i as i32) * 15;
            self.ui_draw_text(mx + 14, y, k, ui.label, 1, false);
            self.ui_draw_text(mx + 124, y, v, ui.sel_val, 1, false);
        }

        let footer = if quad.battery_empty {
            "[R] FRESH PACK | [ESC] MENU"
        } else if quad.arm_aux_connected {
            "[ARM] FLY | [R] PACK | [ESC] MENU"
        } else {
            "[R] FRESH PACK | [ESC] MENU"
        };
        self.ui_draw_text(UI_W / 2 - (footer.len() as i32) * 3, my + mh - 14, footer, ui.tag, 1, false);
    }

    fn draw_stick_box(&mut self, x: i32, y: i32, size: i32, sx: f32, sy: f32, s: i32) {
        let border = rgb565(150, 165, 185);
        let bg = rgb565(18, 22, 30);
        let dot = rgb565(255, 190, 40);
        self.fill_rect(x, y, size, size, bg);
        self.draw_rect_thick(x, y, size, size, s, border);
        let cx = x + size / 2;
        let cy = y + size / 2;
        self.fill_rect(cx, cy, s, s, rgb565(80, 90, 105));
        let half = (size / 2 - 2 * s).max(1) as f32;
        let dx = cx + (clampf(sx, -1.0, 1.0) * half) as i32;
        let dy = cy - (clampf(sy, -1.0, 1.0) * half) as i32;
        self.fill_rect(dx - s, dy - s, 3 * s, 3 * s, dot);
    }

    /// Zero-allocation fading stick-trail gimbal box (~0.002 ms per frame).
    fn draw_stick_box_trails(&mut self, x: i32, y: i32, size: i32, sx: f32, sy: f32, left_stick: bool, s: i32) {
        let border = rgb565(150, 165, 185);
        let bg = rgb565(18, 22, 30);
        let dot = rgb565(255, 232, 90);
        let (trail_old, trail_new) = if left_stick {
            (rgb565(28, 78, 108), rgb565(102, 232, 255)) // Deep -> bright cyan trail for throttle/yaw
        } else {
            (rgb565(115, 52, 18), rgb565(255, 165, 50))  // Deep -> bright orange trail for pitch/roll
        };
        self.fill_rect(x, y, size, size, bg);
        self.draw_rect_thick(x, y, size, size, s, border);
        let cx = x + size / 2;
        let cy = y + size / 2;
        self.fill_rect(cx, cy, s, s, rgb565(75, 85, 100));
        let half = (size / 2 - 2 * s).max(1) as f32;

        let n = self.trail_len;
        if n >= 2 {
            let start = (self.trail_head + STICK_TRAIL_LEN - n) % STICK_TRAIL_LEN;
            let mut prev: Option<(i32, i32)> = None;
            for i in 0..n {
                let sample = self.stick_trail[(start + i) % STICK_TRAIL_LEN];
                let (px_f, py_f) = if left_stick { (sample.0, sample.1) } else { (sample.2, sample.3) };
                let px = cx + (clampf(px_f, -1.0, 1.0) * half) as i32;
                let py = cy - (clampf(py_f, -1.0, 1.0) * half) as i32;
                let age_t = (i as f32) / ((n - 1) as f32);
                let col = lerp_rgb565(trail_old, trail_new, age_t);
                if let Some((ox, oy)) = prev {
                    self.draw_line(ox, oy, px, py, col);
                    if s > 1 {
                        self.draw_line(ox + 1, oy, px + 1, py, col);
                        self.draw_line(ox, oy + 1, px, py + 1, col);
                    }
                } else {
                    self.fill_rect(px, py, s, s, col);
                }
                prev = Some((px, py));
            }
        }

        let dx = cx + (clampf(sx, -1.0, 1.0) * half) as i32;
        let dy = cy - (clampf(sy, -1.0, 1.0) * half) as i32;
        self.fill_rect(dx - s, dy - s, 3 * s, 3 * s, dot);
    }

    /// Simplified vertical throttle/altitude gauge for CardputerZero IMU flight (replaces left 2D gimbal).
    fn draw_imu_throttle_bar(
        &mut self,
        x: i32,
        y: i32,
        w: i32,
        h: i32,
        throttle_pct: u8,
        climb_cmd: f32,
        on_ground: bool,
        s: i32,
    ) {
        let bg = rgb565(18, 22, 30);
        let border = rgb565(102, 217, 239);
        let hover_tick = rgb565(85, 110, 130);
        let (fill_col, cue, cue_col) = if on_ground && climb_cmd <= 0.05 {
            (rgb565(90, 115, 135), "F/X", rgb565(180, 195, 210))
        } else if climb_cmd > 0.1 {
            (rgb565(85, 235, 125), "+CLB", rgb565(85, 255, 120))
        } else if climb_cmd < -0.1 {
            (rgb565(253, 151, 31), "-DSC", rgb565(255, 175, 65))
        } else {
            (rgb565(75, 210, 245), "=HOV", rgb565(120, 240, 255))
        };

        self.fill_rect(x, y, w, h, bg);
        self.draw_rect_thick(x, y, w, h, s, border);

        // Center auto-hover tick reference
        let mid_y = y + h / 2;
        self.fill_rect(x + 2 * s, mid_y, (w - 4 * s).max(1), s, hover_tick);

        // Live vertical throttle fill
        let inner_h = (h - 4 * s).max(1);
        let frac = (throttle_pct as f32 / 100.0).clamp(0.0, 1.0);
        let fill_h = (frac * inner_h as f32).round() as i32;
        if fill_h > 0 {
            let fy = y + h - 2 * s - fill_h;
            self.fill_rect(x + 2 * s, fy, (w - 4 * s).max(1), fill_h, fill_col);
            self.fill_rect(x + 2 * s, fy, (w - 4 * s).max(1), s, rgb565(255, 245, 150));
        }

        // Compact climb / hover / descend indicator right next to gauge
        self.draw_text(x + w + 3 * s, y + h / 2 - 3 * s, cue, cue_col, s, true);
    }

    /// Simplified 2D IMU tilt reticle box (replaces right gimbal with an attitude/tilt horizon pad).
    fn draw_imu_tilt_box(&mut self, x: i32, y: i32, size: i32, roll: f32, pitch: f32, trails: bool, s: i32) {
        let border = rgb565(102, 217, 239);
        let bg = rgb565(18, 22, 30);
        let guide = rgb565(46, 62, 78);
        let dz_col = rgb565(72, 108, 128);
        let dot = rgb565(255, 232, 90);
        let trail_old = rgb565(35, 92, 118);
        let trail_new = rgb565(255, 165, 50);

        self.fill_rect(x, y, size, size, bg);
        self.draw_rect_thick(x, y, size, size, s, border);

        let cx = x + size / 2;
        let cy = y + size / 2;
        // Crosshair axes + small center deadzone box so it visually reads as an IMU tilt pad
        self.fill_rect(x + 2 * s, cy, (size - 4 * s).max(1), s, guide);
        self.fill_rect(cx, y + 2 * s, s, (size - 4 * s).max(1), guide);
        self.draw_rect_thick(cx - 2 * s, cy - 2 * s, 5 * s, 5 * s, s, dz_col);

        let half = (size / 2 - 2 * s).max(1) as f32;
        if trails && self.trail_len >= 2 {
            let n = self.trail_len;
            let start = (self.trail_head + STICK_TRAIL_LEN - n) % STICK_TRAIL_LEN;
            let mut prev: Option<(i32, i32)> = None;
            for i in 0..n {
                let sample = self.stick_trail[(start + i) % STICK_TRAIL_LEN];
                let px = cx + (clampf(sample.2, -1.0, 1.0) * half) as i32;
                let py = cy - (clampf(sample.3, -1.0, 1.0) * half) as i32;
                let age_t = (i as f32) / ((n - 1) as f32);
                let col = lerp_rgb565(trail_old, trail_new, age_t);
                if let Some((ox, oy)) = prev {
                    self.draw_line(ox, oy, px, py, col);
                    if s > 1 {
                        self.draw_line(ox + 1, oy, px + 1, py, col);
                        self.draw_line(ox, oy + 1, px, py + 1, col);
                    }
                } else {
                    self.fill_rect(px, py, s, s, col);
                }
                prev = Some((px, py));
            }
        }

        let dx = cx + (clampf(roll, -1.0, 1.0) * half) as i32;
        let dy = cy - (clampf(pitch, -1.0, 1.0) * half) as i32;
        self.fill_rect(dx - s, dy - s, 3 * s, 3 * s, dot);
    }

    // -----------------------------------------------------------------------
    // Steam-Sim Menu UIs (320 x 170 virtual layout, scaled cleanly to 640x360 / 960x540 on HDMI)
    // -----------------------------------------------------------------------

    /// Live orbiting 3D camera over the currently selected track, dimmed behind floating UI panels.
    fn render_menu_backdrop(&mut self, cfg: &Config, menu_time: f32, dim_alpha: u16) {
        let track = &TRACKS[cfg.track_index.min(TRACKS.len() - 1)];
        let (mut cx, mut cz) = (track.start_pos.x, track.start_pos.z + 24.0);
        if !track.gates.is_empty() {
            let mut sx = 0.0_f32;
            let mut sz = 0.0_f32;
            for g in track.gates {
                sx += g.pos.x;
                sz += g.pos.z;
            }
            let n = track.gates.len() as f32;
            cx = sx / n;
            cz = sz / n;
        }

        let angle = menu_time * 0.16 + 0.55;
        let radius = 34.0;
        let cam_pos = Vec3::new(
            cx + angle.cos() * radius,
            11.0 + (menu_time * 0.35).sin() * 1.8,
            cz + angle.sin() * radius,
        );
        let dx = cx - cam_pos.x;
        let dz = cz - cam_pos.z;
        let yaw = dx.atan2(dz);
        let pitch = 0.22;
        let cam_rot = Quat::from_axis_angle(Vec3::UP, yaw)
            .mul(Quat::from_axis_angle(Vec3::RIGHT, pitch))
            .normalized();

        self.render_world_scene(cam_pos, cam_rot, cfg, track, Some(0));

        let ui = ui_colors(cfg.ui_theme);
        self.fill_rect_alpha(0, 0, self.w(), self.h(), ui.bg, dim_alpha);
    }

    fn draw_topbar(&mut self, title: &str, right_badge: &str, dot_col: Option<u16>, ui: UiColors) {
        let s = self.ui_scale();
        let (_, oy) = self.ui_origin();
        let bar_h = oy + 20 * s;
        self.fill_rect_alpha(0, 0, self.w(), bar_h, ui.header_bg, 7);
        self.fill_rect(0, 0, 3 * s, bar_h, ui.header_line);
        self.fill_rect(0, bar_h, self.w(), s, ui.header_line);

        if title == "STICKZERO" {
            let shadow = rgb565(12, 12, 10);
            self.ui_draw_text(9, 4, "STICK", shadow, 2, false);
            self.ui_draw_text(69, 4, "ZERO", shadow, 2, false);
            self.ui_draw_text(8, 3, "STICK", ui.title, 2, false);
            self.ui_draw_text(68, 3, "ZERO", ui.brand_zero, 2, false);
        } else {
            self.ui_draw_text(8, 3, title, ui.title, 2, false);
        }

        let rw = (right_badge.len() as i32) * 6;
        let pill_w = rw + if dot_col.is_some() { 16 } else { 8 };
        let px = UI_W - pill_w - 5;
        self.ui_fill_rect(px, 4, pill_w, 12, ui.bg);
        self.ui_draw_rect(px, 4, pill_w, 12, ui.muted);
        let tx = if let Some(dc) = dot_col {
            self.ui_fill_rect(px + 4, 8, 4, 4, dc);
            px + 11
        } else {
            px + 4
        };
        self.ui_draw_text(tx, 7, right_badge, ui.tag, 1, false);
    }

    fn draw_footer(&mut self, hint: &str, dot_col: Option<u16>, ui: UiColors) {
        let s = self.ui_scale();
        let (_, oy) = self.ui_origin();
        let fy = oy + (UI_H - 13) * s;
        self.fill_rect_alpha(0, fy, self.w(), self.h() - fy, ui.header_bg, 7);
        self.fill_rect(0, fy, self.w(), s, ui.muted);
        let tx = if let Some(dc) = dot_col {
            self.ui_fill_rect(6, UI_H - 9, 5, 5, dc);
            15
        } else {
            6
        };
        self.ui_draw_text(tx, UI_H - 10, hint, ui.muted, 1, false);
    }

    fn draw_slider_bar(
        &mut self,
        x: i32,
        y: i32,
        w: i32,
        h: i32,
        frac: f32,
        fill_col: u16,
        ui: UiColors,
        selected: bool,
    ) {
        let f = clampf(frac, 0.0, 1.0);
        self.ui_fill_rect(x, y, w, h, ui.header_bg);
        let border = if selected { ui.sel_border } else { ui.muted };
        self.ui_draw_rect(x, y, w, h, border);
        let inner_w = (w - 2).max(1);
        let fill_w = ((inner_w as f32) * f).round() as i32;
        if fill_w > 0 {
            self.ui_fill_rect(x + 1, y + 1, fill_w, (h - 2).max(1), fill_col);
        }
        if selected {
            let knob_x = (x + 1 + fill_w - 1).clamp(x + 1, x + w - 3);
            self.ui_fill_rect(knob_x, y - 1, 3, h + 2, ui.title);
        }
    }

    fn draw_toggle_pill(&mut self, x: i32, y: i32, on: bool, ui: UiColors, selected: bool) {
        let w = 34;
        let h = 11;
        let bg = if on {
            blend_rgb565(ui.header_bg, ui.ok, 3)
        } else {
            ui.header_bg
        };
        let border = if selected {
            ui.sel_border
        } else if on {
            ui.ok
        } else {
            ui.muted
        };
        self.ui_fill_rect(x, y, w, h, bg);
        self.ui_draw_rect(x, y, w, h, border);
        if on {
            self.ui_fill_rect(x + w - 9, y + 2, 7, h - 4, ui.ok);
            self.ui_draw_text(x + 4, y + 2, "ON", ui.title, 1, false);
        } else {
            self.ui_fill_rect(x + 2, y + 2, 7, h - 4, ui.muted);
            self.ui_draw_text(x + 12, y + 2, "OFF", ui.muted, 1, false);
        }
    }

    /// Top-down 2D tactical track minimap showing structures, race path, and gates.
    fn draw_track_minimap(&mut self, x: i32, y: i32, w: i32, h: i32, track: &Track, ui: UiColors) {
        self.ui_fill_rect(x, y, w, h, ui.header_bg);
        self.ui_draw_rect(x, y, w, h, ui.muted);

        let mut min_x = track.start_pos.x;
        let mut max_x = track.start_pos.x;
        let mut min_z = track.start_pos.z;
        let mut max_z = track.start_pos.z;
        for g in track.gates {
            min_x = min_x.min(g.pos.x);
            max_x = max_x.max(g.pos.x);
            min_z = min_z.min(g.pos.z);
            max_z = max_z.max(g.pos.z);
        }
        for s in track.structures {
            min_x = min_x.min(s.min.x);
            max_x = max_x.max(s.max.x);
            min_z = min_z.min(s.min.z);
            max_z = max_z.max(s.max.z);
        }
        let span_x = (max_x - min_x).max(20.0);
        let span_z = (max_z - min_z).max(20.0);
        let pad = 7.0;
        let avail_w = (w as f32 - pad * 2.0).max(8.0);
        let avail_h = (h as f32 - pad * 2.0).max(8.0);
        let scale = (avail_w / span_x).min(avail_h / span_z);
        let cx = (min_x + max_x) * 0.5;
        let cz = (min_z + max_z) * 0.5;
        let map_pt = |wx: f32, wz: f32| -> (i32, i32) {
            let px = x + w / 2 + ((wx - cx) * scale) as i32;
            let py = y + h / 2 - ((wz - cz) * scale) as i32;
            (px.clamp(x + 2, x + w - 3), py.clamp(y + 2, y + h - 3))
        };

        self.ui_draw_hline(x + 2, x + w - 3, y + h / 2, blend_rgb565(ui.header_bg, ui.muted, 2));
        self.ui_draw_vline(x + w / 2, y + 2, y + h - 3, blend_rgb565(ui.header_bg, ui.muted, 2));

        let struct_col = blend_rgb565(ui.header_bg, ui.muted, 4);
        let tree_col = blend_rgb565(ui.header_bg, rgb565(70, 165, 70), 5);
        let hay_col = blend_rgb565(ui.header_bg, rgb565(220, 185, 70), 5);
        let fence_col = blend_rgb565(ui.header_bg, rgb565(150, 105, 65), 4);
        for s in track.structures {
            let (sx0, sy1) = map_pt(s.min.x, s.min.z);
            let (sx1, sy0) = map_pt(s.max.x, s.max.z);
            let rx = sx0.min(sx1);
            let ry = sy0.min(sy1);
            let rw = (sx1 - sx0).abs().max(2);
            let rh = (sy1 - sy0).abs().max(2);
            let col = match s.color {
                StructureColor::Foliage => tree_col,
                StructureColor::Hay => hay_col,
                StructureColor::Wood | StructureColor::DarkWood => fence_col,
                _ => struct_col,
            };
            self.ui_fill_rect(rx, ry, rw, rh, col);
        }

        let path_col = blend_rgb565(ui.header_bg, ui.tag, 4);
        if track.gates.len() >= 2 {
            for i in 0..track.gates.len() {
                let g0 = &track.gates[i];
                let g1 = &track.gates[(i + 1) % track.gates.len()];
                let (x0, y0) = map_pt(g0.pos.x, g0.pos.z);
                let (x1, y1) = map_pt(g1.pos.x, g1.pos.z);
                self.ui_draw_line(x0, y0, x1, y1, path_col);
            }
        }

        for (i, g) in track.gates.iter().enumerate() {
            let (gx, gy) = map_pt(g.pos.x, g.pos.z);
            let col = if i == 0 { ui.ok } else { ui.val };
            self.ui_fill_rect(gx - 2, gy - 2, 5, 5, col);
        }

        let (sp_x, sp_y) = map_pt(track.start_pos.x, track.start_pos.z);
        self.ui_fill_rect(sp_x - 2, sp_y - 2, 4, 4, ui.title);
        self.ui_draw_rect(sp_x - 3, sp_y - 3, 6, 6, ui.tag);
    }

    /// Live Betaflight Actual Rates response curve (`0..100%` stick -> `deg/s`).
    fn draw_rates_curve(&mut self, x: i32, y: i32, w: i32, h: i32, cfg: &Config, ui: UiColors) {
        self.ui_draw_panel(x, y, w, h, ui.header_bg, ui.muted, 7);
        self.ui_draw_text(x + 5, y + 4, "ACTUAL CURVE", ui.tag, 1, false);

        let gx = x + 6;
        let gy = y + 16;
        let gw = w - 12;
        let gh = h - 38;
        self.ui_fill_rect(gx, gy, gw, gh, ui.bg);
        self.ui_draw_rect(gx, gy, gw, gh, ui.muted);

        let grid_col = blend_rgb565(ui.bg, ui.muted, 2);
        for q in 1..4 {
            let lx = gx + (gw * q) / 4;
            let ly = gy + (gh * q) / 4;
            self.ui_draw_vline(lx, gy + 1, gy + gh - 2, grid_col);
            self.ui_draw_hline(gx + 1, gx + gw - 2, ly, grid_col);
        }

        let (center_sens, max_rate, expo) = cfg.actual_rates();
        let y_ceil = max_rate.max(600.0);

        let mut prev_px = gx + 1;
        let mut prev_py = gy + gh - 2;
        for step in 1..=24 {
            let s = (step as f32) / 24.0;
            let deg_s = QuadState::betaflight_actual(s, center_sens, max_rate, expo).to_degrees();
            let px = gx + 1 + ((s * (gw - 3) as f32).round() as i32);
            let py = gy + gh - 2 - (((deg_s / y_ceil).clamp(0.0, 1.0) * (gh - 3) as f32).round() as i32);
            self.ui_draw_line(prev_px, prev_py, px, py, ui.ok);
            prev_px = px;
            prev_py = py;
        }

        let mid_deg = QuadState::betaflight_actual(0.5, center_sens, max_rate, expo).to_degrees();
        let mx = gx + gw / 2;
        let my = gy + gh - 2 - (((mid_deg / y_ceil).clamp(0.0, 1.0) * (gh - 3) as f32).round() as i32);
        self.ui_fill_rect(mx - 2, my - 2, 5, 5, ui.sel_border);

        let mid_txt = format!("50%:{:.0}d/s", mid_deg);
        let max_txt = format!("MAX:{}d/s", cfg.rate_max);
        self.ui_draw_text(x + 6, y + h - 18, &mid_txt, ui.sel_border, 1, false);
        self.ui_draw_text(x + 6, y + h - 9, &max_txt, ui.title, 1, false);
    }

    /// Live Betaflight Throttle Mid/Expo curve (`0..100%` stick -> motor thrust).
    fn draw_throttle_curve(&mut self, x: i32, y: i32, w: i32, h: i32, cfg: &Config, ui: UiColors) {
        self.ui_fill_rect(x, y, w, h, ui.bg);
        self.ui_draw_rect(x, y, w, h, ui.muted);

        let grid_col = blend_rgb565(ui.bg, ui.muted, 2);
        self.ui_draw_vline(x + w / 2, y + 1, y + h - 2, grid_col);
        self.ui_draw_hline(x + 1, x + w - 2, y + h / 2, grid_col);
        self.ui_draw_line(x + 1, y + h - 2, x + w - 2, y + 1, grid_col);

        let mut prev_px = x + 1;
        let mut prev_py = y + h - 2;
        for step in 1..=20 {
            let s = (step as f32) / 20.0;
            let out = QuadState::apply_throttle_expo(s, cfg.thr_mid, cfg.thr_expo);
            let px = x + 1 + ((s * (w - 3) as f32).round() as i32);
            let py = y + h - 2 - ((out.clamp(0.0, 1.0) * (h - 3) as f32).round() as i32);
            self.ui_draw_line(prev_px, prev_py, px, py, ui.tag);
            prev_px = px;
            prev_py = py;
        }

        let m = (cfg.thr_mid as f32 / 100.0).clamp(0.20, 0.70);
        let mx = x + 1 + ((m * (w - 3) as f32).round() as i32);
        let my = y + h - 2 - ((m * (h - 3) as f32).round() as i32);
        self.ui_fill_rect(mx - 2, my - 2, 5, 5, ui.sel_border);
    }

    /// Live side-view quad camera tilt & FOV cone diagram.
    fn draw_camera_preview(&mut self, x: i32, y: i32, w: i32, h: i32, cfg: &Config, ui: UiColors) {
        self.ui_draw_panel(x, y, w, h, ui.header_bg, ui.muted, 7);
        self.ui_draw_text(x + 5, y + 4, "CAMERA CONE", ui.tag, 1, false);

        let bx = x + 6;
        let by = y + 16;
        let bw = w - 12;
        let bh = h - 38;
        self.ui_fill_rect(bx, by, bw, bh, ui.bg);
        self.ui_draw_rect(bx, by, bw, bh, ui.muted);

        let qx = bx + 18;
        let qy = by + (bh * 2) / 3;
        let horiz_col = blend_rgb565(ui.bg, ui.muted, 3);
        self.ui_draw_hline(bx + 2, bx + bw - 3, qy, horiz_col);

        self.ui_fill_rect(qx - 9, qy - 1, 18, 3, ui.title);
        self.ui_draw_hline(qx - 12, qx - 4, qy - 3, ui.muted);
        self.ui_draw_hline(qx + 4, qx + 12, qy - 3, ui.muted);

        let tilt_rad = (cfg.camera_tilt_deg as f32).to_radians();
        let half_fov = (cfg.fov_deg as f32 * 0.32).to_radians();
        let ray_len = (bw - 24) as f32;
        let end_pt = |ang: f32| -> (i32, i32) {
            let ex = qx + (ang.cos() * ray_len) as i32;
            let ey = qy - (ang.sin() * ray_len) as i32;
            (ex.clamp(bx + 2, bx + bw - 3), ey.clamp(by + 2, by + bh - 3))
        };
        let (cx, cy) = end_pt(tilt_rad);
        let (tx, ty) = end_pt(tilt_rad + half_fov);
        let (lx, ly) = end_pt(tilt_rad - half_fov);
        self.ui_draw_line(qx + 2, qy - 1, tx, ty, ui.tag);
        self.ui_draw_line(qx + 2, qy - 1, lx, ly, ui.tag);
        self.ui_draw_line(qx + 2, qy - 1, cx, cy, ui.sel_border);
        self.ui_fill_rect(qx, qy - 3, 4, 4, ui.ok);

        let l1 = format!("TILT: {} deg", cfg.camera_tilt_deg);
        let l2 = format!("FOV : {} deg", cfg.fov_deg);
        self.ui_draw_text(x + 6, y + h - 18, &l1, ui.sel_border, 1, false);
        self.ui_draw_text(x + 6, y + h - 9, &l2, ui.title, 1, false);
    }

    pub fn render_main_menu(
        &mut self,
        cfg: &Config,
        selected: usize,
        input: &InputManager,
        hdmi_tag: Option<&str>,
        menu_time: f32,
    ) {
        let ui = ui_colors(cfg.ui_theme);
        let dim = if selected == 3 { 2 } else { 4 };
        self.render_menu_backdrop(cfg, menu_time, dim);

        let (ctrl_dot, ctrl_short) = match input.control_source() {
            FlightControlSource::Controller => (ui.ok, "CTRL OK"),
            FlightControlSource::ImuSimplified => (ui.tag, "IMU EASY"),
            FlightControlSource::None => (ui.warn, "NO CTRL"),
        };
        let right_badge = match hdmi_tag {
            Some(tag) => format!("HDMI {tag}"),
            None => format!("{} | {}", cfg.quad.craft_name(), ctrl_short),
        };
        self.draw_topbar("STICKZERO", &right_badge, Some(ctrl_dot), ui);

        let track = &TRACKS[cfg.track_index.min(TRACKS.len() - 1)];

        // --- Left Navigation Rail ---
        let nav_x = 6;
        let nav_w = 108;

        // Primary CTA: FLY NOW
        {
            let y = 25;
            let h = 22;
            let is_sel = selected == 0;
            let bg = if is_sel {
                blend_rgb565(ui.sel_bg, ui.ok, 2)
            } else {
                ui.header_bg
            };
            let border = if is_sel { ui.ok } else { ui.muted };
            self.ui_draw_panel(nav_x, y, nav_w, h, bg, border, 7);
            if is_sel {
                self.ui_fill_rect(nav_x + 1, y + 1, 3, h - 2, ui.ok);
            }
            let lbl = if is_sel { "> FLY NOW" } else { "  FLY NOW" };
            self.ui_draw_text(nav_x + 7, y + 7, lbl, if is_sel { ui.title } else { ui.ok }, 1, false);
            let mode_tag = if cfg.game_mode == GameMode::Race { "RACE" } else { "FREE" };
            self.ui_draw_text(nav_x + nav_w - 28, y + 7, mode_tag, ui.sel_border, 1, false);
        }

        let nav_items: [&str; 5] = ["TRACK", "WORKBENCH", "THEME", "SETTINGS", "CONTROLLER"];
        for (idx, label) in nav_items.iter().enumerate() {
            let item_i = idx + 1;
            let y = 51 + (idx as i32) * 20;
            let h = 17;
            let is_sel = selected == item_i;
            let bg = if is_sel { ui.sel_bg } else { ui.header_bg };
            let border = if is_sel { ui.sel_border } else { blend_rgb565(ui.bg, ui.muted, 4) };
            self.ui_draw_panel(nav_x, y, nav_w, h, bg, border, if is_sel { 7 } else { 6 });
            if is_sel {
                self.ui_fill_rect(nav_x + 1, y + 1, 3, h - 2, ui.header_line);
            }
            let col = if is_sel { ui.sel_label } else { ui.label };
            let prefix = if is_sel { "> " } else { "  " };
            self.ui_draw_text(nav_x + 6, y + 5, prefix, ui.header_line, 1, false);
            self.ui_draw_text(nav_x + 18, y + 5, label, col, 1, false);
        }

        // --- Right Context Preview Card ---
        let cx = 120;
        let cy = 25;
        let cw = 194;
        let ch = 124;
        let card_alpha = if selected == 3 { 5 } else { 7 };
        self.ui_draw_panel(cx, cy, cw, ch, ui.bg, ui.muted, card_alpha);
        self.ui_fill_rect_alpha(cx + 1, cy + 1, cw - 2, 15, ui.header_bg, 7);
        self.ui_draw_hline(cx + 1, cx + cw - 2, cy + 16, ui.muted);

        match selected {
            0 => {
                self.ui_draw_text(cx + 6, cy + 5, "FLIGHT BRIEFING", ui.tag, 1, false);
                let g_tag = format!("{} GATES", track.gates.len());
                self.ui_draw_text(cx + cw - (g_tag.len() as i32) * 6 - 5, cy + 5, &g_tag, ui.sel_border, 1, false);

                self.ui_draw_text(cx + 6, cy + 21, track.name, ui.title, 1, false);
                self.draw_track_minimap(cx + 6, cy + 32, 86, 68, track, ui);

                let sx = cx + 98;
                self.ui_draw_text(sx, cy + 33, cfg.quad.craft_name(), ui.sel_label, 1, false);
                let wt = format!("{}g {:.1}:1", cfg.weight_g, cfg.effective_twr());
                self.ui_draw_text(sx, cy + 45, &wt, ui.title, 1, false);
                self.ui_draw_text(sx, cy + 57, cfg.battery_pack.short_name(), ui.val, 1, false);
                let fm = if input.control_source() == FlightControlSource::ImuSimplified {
                    "IMU EASY"
                } else {
                    cfg.flight_mode.name()
                };
                self.ui_draw_text(sx, cy + 69, fm, ui.tag, 1, false);
                let cam = format!("{}d FOV{}", cfg.camera_tilt_deg, cfg.fov_deg);
                self.ui_draw_text(sx, cy + 81, &cam, ui.muted, 1, false);
                self.ui_fill_rect(sx, cy + 93, 5, 5, ctrl_dot);
                self.ui_draw_text(sx + 8, cy + 92, ctrl_short, ui.title, 1, false);

                self.ui_fill_rect(cx + 6, cy + 106, cw - 12, 13, blend_rgb565(ui.header_bg, ui.ok, 2));
                self.ui_draw_rect(cx + 6, cy + 106, cw - 12, 13, ui.ok);
                self.ui_draw_text(cx + 34, cy + 109, "[ENTER] LAUNCH FLIGHT", ui.title, 1, false);
            }
            1 => {
                self.ui_draw_text(cx + 6, cy + 5, "TRACK SELECTOR", ui.tag, 1, false);
                let idx_tag = format!("{}/{}", cfg.track_index + 1, TRACKS.len());
                self.ui_draw_text(cx + cw - (idx_tag.len() as i32) * 6 - 5, cy + 5, &idx_tag, ui.sel_border, 1, false);

                self.ui_draw_text(cx + 6, cy + 21, track.name, ui.title, 1, false);
                let sub: String = track.subtitle.chars().take(30).collect();
                self.ui_draw_text(cx + 6, cy + 31, &sub, ui.val, 1, false);
                self.draw_track_minimap(cx + 6, cy + 42, cw - 12, 64, track, ui);
                self.ui_draw_text(cx + 6, cy + 111, "[Z/C] Switch Track", ui.sel_label, 1, false);
                let mode_lbl = cfg.game_mode.name();
                self.ui_draw_text(cx + cw - (mode_lbl.len() as i32) * 6 - 6, cy + 111, mode_lbl, ui.tag, 1, false);
            }
            2 => {
                self.ui_draw_text(cx + 6, cy + 5, "DRONE WORKBENCH", ui.tag, 1, false);
                let twr_tag = format!("{:.1}:1", cfg.effective_twr());
                self.ui_draw_text(cx + cw - (twr_tag.len() as i32) * 6 - 5, cy + 5, &twr_tag, ui.ok, 1, false);

                self.ui_draw_text(cx + 6, cy + 21, cfg.quad.name(), ui.title, 1, false);
                self.ui_draw_text(cx + 6, cy + 32, cfg.battery_pack.name(), ui.val, 1, false);

                let stats: [(&str, f32, String, u16); 4] = [
                    ("TWR ", (cfg.effective_twr() / 9.0).clamp(0.0, 1.0), format!("{:.1}:1", cfg.effective_twr()), ui.ok),
                    ("AUW ", ((cfg.weight_g as f32 - 15.0) / 1485.0).clamp(0.02, 1.0).sqrt(), format!("{}g", cfg.weight_g), ui.tag),
                    ("GRIP", (cfg.rotor_grip as f32 / 100.0).clamp(0.0, 1.0), format!("{:.2}", cfg.rotor_grip as f32 / 100.0), ui.accent),
                    ("RESP", (1.0 - (cfg.motor_lag_ms as f32 - 5.0) / 40.0).clamp(0.05, 1.0), format!("{}ms", cfg.motor_lag_ms), ui.val),
                ];
                for (i, (lbl, frac, val, col)) in stats.iter().enumerate() {
                    let sy = cy + 47 + (i as i32) * 15;
                    self.ui_draw_text(cx + 6, sy, lbl, ui.label, 1, false);
                    self.draw_slider_bar(cx + 38, sy, 102, 7, *frac, *col, ui, false);
                    self.ui_draw_text(cx + 146, sy, val, ui.title, 1, false);
                }
                self.ui_draw_text(cx + 6, cy + 111, "[Z/C] Quad | [Ent] Tune", ui.sel_label, 1, false);
            }
            3 => {
                self.ui_draw_text(cx + 6, cy + 5, "ENVIRONMENT", ui.tag, 1, false);
                self.ui_draw_text(cx + cw - 48, cy + 5, "LIVE 3D", ui.ok, 1, false);

                let themes = [
                    (VisualTheme::Summer, "SUM"),
                    (VisualTheme::Evening, "EVE"),
                    (VisualTheme::Night, "NGT"),
                    (VisualTheme::Wireframe, "WIR"),
                ];
                for (i, (th, short)) in themes.iter().enumerate() {
                    let sx = cx + 6 + (i as i32) * 46;
                    let sy = cy + 23;
                    let p = palette(*th);
                    let active = cfg.theme == *th;
                    self.ui_fill_rect(sx, sy, 42, 16, p.sky_top);
                    self.ui_fill_rect(sx, sy + 16, 42, 12, p.ground_near);
                    self.ui_draw_hline(sx, sx + 41, sy + 16, p.active_gate);
                    self.ui_draw_rect(sx, sy, 42, 28, if active { ui.sel_border } else { ui.muted });
                    if active {
                        self.ui_draw_rect(sx + 1, sy + 1, 40, 26, ui.sel_border);
                    }
                    self.ui_draw_text(sx + 12, sy + 32, short, if active { ui.sel_label } else { ui.muted }, 1, false);
                }

                let cur = format!("Active: {}", cfg.theme.name());
                self.ui_draw_text(cx + 6, cy + 72, &cur, ui.title, 1, false);
                let cam = format!("Tilt: {} deg | FOV: {} deg", cfg.camera_tilt_deg, cfg.fov_deg);
                self.ui_draw_text(cx + 6, cy + 86, &cam, ui.val, 1, false);
                let fx = format!("Analog FX: {} | Sky: {}", if cfg.analog_filter { "ON" } else { "OFF" }, if cfg.sky_detail { "ON" } else { "OFF" });
                self.ui_draw_text(cx + 6, cy + 98, &fx, ui.muted, 1, false);
                self.ui_draw_text(cx + 6, cy + 111, "[Z/C] Cycle Env Theme", ui.sel_label, 1, false);
            }
            4 => {
                self.ui_draw_text(cx + 6, cy + 5, "SIM SETTINGS", ui.tag, 1, false);
                self.ui_draw_text(cx + cw - 42, cy + 5, "4 TABS", ui.sel_border, 1, false);

                let fm = if input.control_source() == FlightControlSource::ImuSimplified {
                    "IMU EASY"
                } else {
                    cfg.flight_mode.name()
                };
                let rows: [(&str, String); 4] = [
                    ("FLIGHT", format!("{} / {}", cfg.game_mode.name(), fm)),
                    ("RATES ", format!("{} {}/{}", cfg.rates.name(), cfg.rate_center, cfg.rate_max)),
                    ("CAMERA", format!("{}d FOV{} FX:{}", cfg.camera_tilt_deg, cfg.fov_deg, if cfg.analog_filter { "ON" } else { "OFF" })),
                    ("SYSTEM", format!("{} | {}", cfg.ui_theme.name(), cfg.hdmi_resolution.name())),
                ];
                for (i, (k, v)) in rows.iter().enumerate() {
                    let ry = cy + 25 + (i as i32) * 20;
                    self.ui_fill_rect(cx + 6, ry - 2, cw - 12, 16, ui.header_bg);
                    self.ui_draw_text(cx + 10, ry + 2, k, ui.sel_border, 1, false);
                    let vt: String = v.chars().take(20).collect();
                    self.ui_draw_text(cx + 52, ry + 2, &vt, ui.title, 1, false);
                }
                self.ui_draw_text(cx + 6, cy + 111, "[Enter] Open Settings", ui.sel_label, 1, false);
            }
            _ => {
                self.ui_draw_text(cx + 6, cy + 5, "RADIO / INPUT", ui.tag, 1, false);
                self.ui_draw_text(cx + cw - (ctrl_short.len() as i32) * 6 - 5, cy + 5, ctrl_short, ctrl_dot, 1, false);

                match input.control_source() {
                    FlightControlSource::Controller => {
                        let cname: String = input.controller_name().chars().take(30).collect();
                        self.ui_draw_text(cx + 6, cy + 21, &cname, ui.title, 1, false);

                        let sticks = input.flight_sticks(cfg);
                        let s = self.ui_scale();
                        let (ox, oy) = self.ui_origin();
                        self.draw_stick_box(ox + (cx + 30) * s, oy + (cy + 36) * s, 44 * s, sticks.yaw, sticks.throttle * 2.0 - 1.0, s);
                        self.draw_stick_box(ox + (cx + 114) * s, oy + (cy + 36) * s, 44 * s, sticks.roll, sticks.pitch, s);
                        self.ui_draw_text(cx + 38, cy + 84, "L-STK", ui.muted, 1, false);
                        self.ui_draw_text(cx + 122, cy + 84, "R-STK", ui.muted, 1, false);

                        let aux_s = format!(
                            "ARM:{} MOD:{} TRT:{}",
                            cfg.aux_arm.short_label(),
                            cfg.aux_mode.short_label(),
                            cfg.aux_turtle.short_label()
                        );
                        self.ui_draw_text(cx + 6, cy + 97, &aux_s, ui.val, 1, false);
                        self.ui_draw_text(cx + 6, cy + 111, "[Enter] Calibrate & AUX", ui.sel_label, 1, false);
                    }
                    FlightControlSource::ImuSimplified => {
                        self.ui_draw_text(cx + 6, cy + 21, "CardputerZero IMU", ui.title, 1, false);
                        let sticks = input.flight_sticks(cfg);
                        let s = self.ui_scale();
                        let (ox, oy) = self.ui_origin();
                        let thr_pct = (sticks.throttle * 100.0).round() as u8;
                        self.draw_imu_throttle_bar(ox + (cx + 38) * s, oy + (cy + 36) * s, 16 * s, 44 * s, thr_pct, input.imu_climb_cmd(), false, s);
                        self.draw_imu_tilt_box(ox + (cx + 102) * s, oy + (cy + 36) * s, 44 * s, sticks.roll, sticks.pitch, false, s);
                        self.ui_draw_text(cx + 38, cy + 84, "ALT", ui.muted, 1, false);
                        self.ui_draw_text(cx + 112, cy + 84, "TILT", ui.tag, 1, false);
                        self.ui_draw_text(cx + 6, cy + 97, "Tilt Steer | [F/X] Alt", ui.val, 1, false);
                        self.ui_draw_text(cx + 6, cy + 111, "[Enter] IMU & Scan USB/BT", ui.sel_label, 1, false);
                    }
                    FlightControlSource::None => {
                        let hdr = if input.is_cardputer_lite() {
                            "CardputerZero Lite (No IMU)"
                        } else {
                            "No BT/USB Controller"
                        };
                        self.ui_draw_text(cx + 6, cy + 24, hdr, ui.warn, 1, false);
                        self.ui_draw_text(cx + 6, cy + 41, "Plug USB-C ELRS / Radio", ui.title, 1, false);
                        self.ui_draw_text(cx + 6, cy + 54, "or pair a BT Gamepad.", ui.title, 1, false);
                        self.ui_draw_hline(cx + 6, cx + cw - 7, cy + 71, ui.muted);
                        self.ui_draw_text(cx + 6, cy + 79, "Keyboard fallback:", ui.tag, 1, false);
                        self.ui_draw_text(cx + 6, cy + 91, "[FXZC] L-Stk | [KBNM] R-Stk", ui.muted, 1, false);
                        self.ui_draw_text(cx + 6, cy + 111, "[Enter] Controller Setup", ui.sel_label, 1, false);
                    }
                }
            }
        }

        let footer = match input.control_source() {
            FlightControlSource::Controller => "[F/X] Nav | [Z/C] Opt | [Ent] OK | [Hold Esc] Exit",
            FlightControlSource::ImuSimplified => "IMU | [F/X] Nav | [Ent] Select | [Hold Esc] Exit",
            FlightControlSource::None => "Plug USB/BT | [Ent] Select | [Hold Esc] Exit",
        };
        self.draw_footer(footer, Some(ctrl_dot), ui);
    }

    /// Launch warning modal shown when no IMU sensor is detected and/or running on CardputerZero Lite
    /// without a connected USB-C or Bluetooth controller.
    pub fn render_controller_warning(
        &mut self,
        cfg: &Config,
        input: &InputManager,
        hdmi_tag: Option<&str>,
    ) {
        self.render_main_menu(cfg, 0, input, hdmi_tag, 0.5);
        let ui = ui_colors(cfg.ui_theme);
        self.fill_rect_alpha(0, 0, self.w(), self.h(), ui.header_bg, 4);

        let mx = 18;
        let my = 24;
        let mw = UI_W - 36;
        let mh = UI_H - 42;
        let cx = UI_W / 2;
        let warn_border = rgb565(255, 110, 45);
        let banner_bg = rgb565(75, 24, 20);
        let banner_txt = rgb565(255, 215, 70);

        self.ui_fill_rect(mx, my, mw, mh, ui.header_bg);
        self.ui_draw_rect(mx, my, mw, mh, warn_border);
        self.ui_draw_rect(mx + 1, my + 1, mw - 2, mh - 2, warn_border);

        self.ui_fill_rect(mx + 2, my + 2, mw - 4, 19, banner_bg);
        self.ui_draw_hline(mx + 2, mx + mw - 3, my + 21, warn_border);
        let title = "! CONTROLLER REQUIRED !";
        self.ui_draw_text(cx - (title.len() as i32) * 3, my + 8, title, banner_txt, 1, false);

        let reason = if input.is_cardputer_lite() {
            "CardputerZero Lite detected (no IMU)"
        } else {
            "No IMU sensor detected"
        };
        self.ui_draw_text(cx - (reason.len() as i32) * 3, my + 29, reason, ui.title, 1, false);

        let l1 = "A USB-C or Bluetooth controller";
        let l2 = "is required to fly StickZero.";
        self.ui_draw_text(cx - (l1.len() as i32) * 3, my + 45, l1, ui.label, 1, false);
        self.ui_draw_text(cx - (l2.len() as i32) * 3, my + 57, l2, ui.label, 1, false);

        let l3 = "Pair a BT gamepad / ELRS radio";
        let l4 = "or connect via USB-C Host mode.";
        self.ui_draw_text(cx - (l3.len() as i32) * 3, my + 74, l3, ui.tag, 1, false);
        self.ui_draw_text(cx - (l4.len() as i32) * 3, my + 86, l4, ui.tag, 1, false);

        self.ui_draw_hline(mx + 10, mx + mw - 11, my + 103, ui.muted);
        let footer = "[Ent/Esc] Continue | [Hold Esc] Exit";
        self.ui_draw_text(cx - (footer.len() as i32) * 3, my + 111, footer, ui.sel_label, 1, false);
    }

    pub fn render_drone_tuning_menu(&mut self, cfg: &Config, selected: usize, menu_time: f32) {
        let ui = ui_colors(cfg.ui_theme);
        self.render_menu_backdrop(cfg, menu_time, 5);

        let badge = format!("{} | {}g", cfg.quad.craft_name(), cfg.weight_g);
        self.draw_topbar("WORKBENCH", &badge, Some(ui.ok), ui);

        let rows: [(&str, String, Option<f32>); 11] = [
            ("Quad", cfg.quad.craft_name().to_string(), None),
            ("Battery", cfg.battery_pack.short_name().to_string(), None),
            ("Prop Wash", cfg.prop_wash.name().to_string(), None),
            ("Weight", format!("{}g", cfg.weight_g), Some(((cfg.weight_g as f32 - 15.0) / 1485.0).clamp(0.0, 1.0).sqrt())),
            ("Thrust", format!("{}g", cfg.thrust_g), Some(((cfg.thrust_g as f32 - 40.0) / 7960.0).clamp(0.0, 1.0).sqrt())),
            ("Air Drag", format!("{:.2}", cfg.air_drag as f32 / 100.0), Some(((cfg.air_drag as f32 - 5.0) / 95.0).clamp(0.0, 1.0))),
            ("Rotor Grip", format!("{:.2}", cfg.rotor_grip as f32 / 100.0), Some((cfg.rotor_grip as f32 / 100.0).clamp(0.0, 1.0))),
            ("Motor Lag", format!("{}ms", cfg.motor_lag_ms), Some(((cfg.motor_lag_ms as f32 - 5.0) / 40.0).clamp(0.0, 1.0))),
            ("Thr Mid", format!("{:.2}", cfg.thr_mid as f32 / 100.0), Some(((cfg.thr_mid as f32 - 20.0) / 50.0).clamp(0.0, 1.0))),
            ("Thr Expo", format!("{:.2}", cfg.thr_expo as f32 / 100.0), Some((cfg.thr_expo as f32 / 80.0).clamp(0.0, 1.0))),
            ("Reset", "[Enter] Stock".to_string(), None),
        ];

        let lx = 6;
        let ly = 24;
        let lw = 190;
        let lh = 128;
        self.ui_draw_panel(lx, ly, lw, lh, ui.bg, ui.muted, 7);

        const VISIBLE: usize = 7;
        let start = if selected >= VISIBLE {
            (selected + 1 - VISIBLE).min(rows.len() - VISIBLE)
        } else {
            0
        };

        for (slot, i) in (start..(start + VISIBLE).min(rows.len())).enumerate() {
            let (k, ref v, frac) = rows[i];
            let ry = ly + 5 + (slot as i32) * 17;
            let is_sel = i == selected;
            if is_sel {
                self.ui_fill_rect(lx + 3, ry - 2, lw - 11, 15, ui.sel_bg);
                self.ui_draw_rect(lx + 3, ry - 2, lw - 11, 15, ui.sel_border);
                self.ui_fill_rect(lx + 3, ry - 2, 3, 15, ui.header_line);
            }
            let kcol = if is_sel { ui.sel_label } else { ui.label };
            let vcol = if is_sel { ui.sel_val } else { ui.val };
            self.ui_draw_text(lx + 9, ry + 2, k, kcol, 1, false);

            if let Some(f) = frac {
                self.draw_slider_bar(lx + 78, ry + 2, 54, 7, f, if is_sel { ui.ok } else { ui.tag }, ui, is_sel);
                self.ui_draw_text(lx + 137, ry + 2, v, vcol, 1, false);
            } else {
                let disp = if is_sel && i < 10 { format!("< {v} >") } else { v.clone() };
                self.ui_draw_text(lx + 78, ry + 2, &disp, vcol, 1, false);
            }
        }

        let sb_x = lx + lw - 5;
        let sb_y = ly + 5;
        let sb_h = lh - 10;
        self.ui_fill_rect(sb_x, sb_y, 2, sb_h, ui.header_bg);
        let thumb_h = ((sb_h * VISIBLE as i32) / rows.len() as i32).max(12);
        let thumb_y = sb_y + ((sb_h - thumb_h) * start as i32) / (rows.len() - VISIBLE).max(1) as i32;
        self.ui_fill_rect(sb_x, thumb_y, 2, thumb_h, ui.tag);

        let rx = 202;
        let ry = 24;
        let rw = 112;
        let rh = 128;
        self.ui_draw_panel(rx, ry, rw, rh, ui.header_bg, ui.muted, 7);
        self.ui_draw_text(rx + 6, ry + 5, "TELEMETRY", ui.tag, 1, false);

        let twr = cfg.effective_twr();
        let hover_pct = ((1.0 / twr.max(1.0)) * 100.0).round().clamp(10.0, 95.0) as u8;
        let twr_s = format!("TWR  {:.2}:1", twr);
        let twr_col = if twr >= 5.0 { ui.ok } else if twr >= 2.8 { ui.tag } else { ui.warn };
        self.ui_draw_text(rx + 6, ry + 18, &twr_s, ui.title, 1, false);
        self.draw_slider_bar(rx + 6, ry + 28, rw - 12, 6, (twr / 9.0).clamp(0.0, 1.0), twr_col, ui, false);

        let auw_s = format!("AUW {}g", cfg.weight_g);
        let hov_s = format!("HOV {}%", hover_pct);
        self.ui_draw_text(rx + 6, ry + 40, &auw_s, ui.label, 1, false);
        self.ui_draw_text(rx + 62, ry + 40, &hov_s, ui.val, 1, false);

        self.ui_draw_hline(rx + 4, rx + rw - 5, ry + 52, ui.muted);
        self.ui_draw_text(rx + 6, ry + 57, "THR CURVE", ui.tag, 1, false);
        self.draw_throttle_curve(rx + 6, ry + 68, rw - 12, 54, cfg, ui);

        self.draw_footer("[F/X] Nav | [Z/C] Tune | [Ent] Reset | [Esc] Save", None, ui);
    }

    pub fn render_pause_menu(&mut self, cfg: &Config, quad: &QuadState, selected: usize) {
        let ui = ui_colors(cfg.ui_theme);
        self.fill_rect_alpha(0, 0, self.w(), self.h(), ui.header_bg, 4);

        let mx = 52;
        let my = 18;
        let mw = 216;
        let mh = 134;
        self.ui_draw_panel(mx, my, mw, mh, ui.bg, ui.sel_border, 7);
        self.ui_fill_rect(mx + 1, my + 1, mw - 2, 20, ui.header_bg);
        self.ui_fill_rect(mx + 1, my + 1, 3, 20, ui.header_line);
        self.ui_draw_hline(mx + 1, mx + mw - 2, my + 21, ui.header_line);
        self.ui_draw_text(mx + 8, my + 4, "PAUSED", ui.title, 2, false);

        let mins = (quad.flight_time as u32) / 60;
        let secs = (quad.flight_time as u32) % 60;
        let telem = format!("{:02}:{:02} {:.2}V", mins, secs, quad.cell_voltage);
        self.ui_draw_text(mx + mw - (telem.len() as i32) * 6 - 6, my + 7, &telem, ui.tag, 1, false);

        let items = [
            "RESUME FLIGHT",
            "RESTART RUN",
            "DRONE WORKBENCH",
            "SETTINGS",
            "MAIN MENU",
        ];
        for (i, label) in items.iter().enumerate() {
            let y = my + 28 + (i as i32) * 18;
            let is_sel = i == selected;
            if is_sel {
                self.ui_fill_rect(mx + 10, y - 2, mw - 20, 15, ui.sel_bg);
                self.ui_draw_rect(mx + 10, y - 2, mw - 20, 15, ui.sel_border);
                self.ui_fill_rect(mx + 10, y - 2, 3, 15, ui.ok);
            } else {
                self.ui_fill_rect(mx + 10, y - 2, mw - 20, 15, ui.header_bg);
            }
            let col = if is_sel { ui.sel_label } else { ui.label };
            self.ui_draw_text(mx + 20, y + 2, label, col, 1, false);
        }

        self.ui_draw_text(mx + 27, my + mh - 11, "[Esc] Resume | [Ent] Select", ui.muted, 1, false);
    }

    pub fn render_settings_menu(
        &mut self,
        cfg: &Config,
        tab: SettingsTab,
        row: usize,
        focus_tabs: bool,
        imu_mode: bool,
        hdmi_tag: Option<&str>,
        menu_time: f32,
    ) {
        let ui = ui_colors(cfg.ui_theme);
        self.render_menu_backdrop(cfg, menu_time, 5);

        let right_badge = match hdmi_tag {
            Some(tag) => format!("HDMI {tag}"),
            None => tab.label().to_string(),
        };
        self.draw_topbar("SETTINGS", &right_badge, None, ui);

        // --- Horizontal Category Tab Strip ---
        let tab_y = 23;
        let tab_h = 15;
        let tab_w = 75;
        for (i, t) in SettingsTab::ALL.iter().enumerate() {
            let tx = 6 + (i as i32) * (tab_w + 2);
            let active = *t == tab;
            let bg = if active { ui.sel_bg } else { ui.header_bg };
            let border = if active && focus_tabs {
                ui.sel_border
            } else if active {
                ui.header_line
            } else {
                ui.muted
            };
            self.ui_draw_panel(tx, tab_y, tab_w, tab_h, bg, border, 7);
            if active {
                self.ui_fill_rect(tx + 1, tab_y + tab_h - 2, tab_w - 2, 2, if focus_tabs { ui.sel_border } else { ui.header_line });
            }
            let lbl = if active && focus_tabs {
                format!("<{}>", t.label())
            } else {
                t.label().to_string()
            };
            let lx = tx + (tab_w - (lbl.len() as i32) * 6) / 2;
            let lcol = if active { ui.title } else { ui.muted };
            self.ui_draw_text(lx, tab_y + 4, &lbl, lcol, 1, false);
        }

        // --- Tab Content ---
        match tab {
            SettingsTab::Flight => {
                self.ui_draw_panel(6, 42, UI_W - 12, 82, ui.bg, ui.muted, 7);
                let flight_mode_val = if imu_mode {
                    "IMU EASY (Locked)".to_string()
                } else {
                    cfg.flight_mode.name().to_string()
                };
                let rows: [(&str, String, &str, bool); 4] = [
                    ("Game Mode", cfg.game_mode.name().to_string(), "Race: timed gate laps | Freestyle: trick combos", false),
                    ("Flight Mode", flight_mode_val, if imu_mode { "Locked to IMU Easy while no USB/BT controller" } else { "ACRO: manual | HORIZ: level+flip | ANGLE: 45d cap" }, imu_mode),
                    ("Stick Mode", cfg.stick_mode.name().to_string(), "Mode 2: Thr left, Pit right | Mode 1: Thr right", false),
                    ("Throttle Mode", cfg.throttle_mode.name().to_string(), "RC Full: 0-100% stick | Gamepad: spring-centered", false),
                ];
                for (i, (k, v, _, locked)) in rows.iter().enumerate() {
                    let y = 47 + (i as i32) * 18;
                    let is_sel = !focus_tabs && i == row;
                    if is_sel {
                        self.ui_fill_rect(10, y - 2, UI_W - 20, 15, ui.sel_bg);
                        self.ui_draw_rect(10, y - 2, UI_W - 20, 15, ui.sel_border);
                        self.ui_fill_rect(10, y - 2, 3, 15, ui.header_line);
                    }
                    let kcol = if *locked { ui.muted } else if is_sel { ui.sel_label } else { ui.label };
                    let vcol = if *locked { ui.muted } else if is_sel { ui.sel_val } else { ui.val };
                    self.ui_draw_text(18, y + 2, k, kcol, 1, false);
                    let disp = if is_sel && !*locked { format!("< {v} >") } else { v.clone() };
                    self.ui_draw_text(132, y + 2, &disp, vcol, 1, false);
                }
                self.ui_draw_panel(6, 128, UI_W - 12, 24, ui.header_bg, ui.muted, 7);
                let desc = if focus_tabs {
                    "[Z/C] Switch tab | [X/Ent] Edit items"
                } else {
                    rows[row.min(rows.len() - 1)].2
                };
                let desc_trim: String = desc.chars().take(50).collect();
                self.ui_draw_text(12, 136, &desc_trim, ui.tag, 1, false);
            }
            SettingsTab::Rates => {
                let lx = 6;
                let lw = 178;
                self.ui_draw_panel(lx, 42, lw, 110, ui.bg, ui.muted, 7);

                let items: [(&str, String, Option<f32>); 4] = [
                    ("Preset", cfg.rates.name().to_string(), None),
                    ("Center", format!("{} d/s", cfg.rate_center), Some(((cfg.rate_center as f32 - 50.0) / 350.0).clamp(0.0, 1.0))),
                    ("Max Rate", format!("{} d/s", cfg.rate_max), Some(((cfg.rate_max as f32 - 300.0) / 900.0).clamp(0.0, 1.0))),
                    ("Expo", format!("{:.2}", cfg.rate_expo as f32 / 100.0), Some((cfg.rate_expo as f32 / 90.0).clamp(0.0, 1.0))),
                ];
                for (i, (k, v, frac)) in items.iter().enumerate() {
                    let y = 47 + (i as i32) * 25;
                    let is_sel = !focus_tabs && i == row;
                    if is_sel {
                        self.ui_fill_rect(lx + 4, y - 2, lw - 8, 22, ui.sel_bg);
                        self.ui_draw_rect(lx + 4, y - 2, lw - 8, 22, ui.sel_border);
                        self.ui_fill_rect(lx + 4, y - 2, 3, 22, ui.header_line);
                    }
                    let kcol = if is_sel { ui.sel_label } else { ui.label };
                    let vcol = if is_sel { ui.sel_val } else { ui.val };
                    self.ui_draw_text(lx + 11, y + 1, k, kcol, 1, false);
                    let disp = if is_sel && frac.is_none() { format!("< {v} >") } else { v.clone() };
                    self.ui_draw_text(lx + lw - (disp.len() as i32) * 6 - 8, y + 1, &disp, vcol, 1, false);
                    if let Some(f) = frac {
                        self.draw_slider_bar(lx + 11, y + 11, lw - 20, 6, *f, if is_sel { ui.ok } else { ui.tag }, ui, is_sel);
                    } else {
                        self.ui_draw_text(lx + 11, y + 11, "Betaflight Actual Rates", ui.muted, 1, false);
                    }
                }

                self.draw_rates_curve(190, 42, 124, 110, cfg, ui);
            }
            SettingsTab::Camera => {
                let lx = 6;
                let lw = 190;
                self.ui_draw_panel(lx, 42, lw, 110, ui.bg, ui.muted, 7);

                let labels = ["Cam Tilt", "FOV", "Analog FX", "Stick OSD", "Debug OSD"];
                for (i, k) in labels.iter().enumerate() {
                    let y = 48 + (i as i32) * 20;
                    let is_sel = !focus_tabs && i == row;
                    if is_sel {
                        self.ui_fill_rect(lx + 4, y - 2, lw - 8, 17, ui.sel_bg);
                        self.ui_draw_rect(lx + 4, y - 2, lw - 8, 17, ui.sel_border);
                        self.ui_fill_rect(lx + 4, y - 2, 3, 17, ui.header_line);
                    }
                    let kcol = if is_sel { ui.sel_label } else { ui.label };
                    let vcol = if is_sel { ui.sel_val } else { ui.val };
                    self.ui_draw_text(lx + 10, y + 3, k, kcol, 1, false);

                    match i {
                        0 => {
                            let f = (cfg.camera_tilt_deg as f32 / 50.0).clamp(0.0, 1.0);
                            self.draw_slider_bar(lx + 72, y + 3, 72, 7, f, if is_sel { ui.ok } else { ui.tag }, ui, is_sel);
                            let v = format!("{}d", cfg.camera_tilt_deg);
                            self.ui_draw_text(lx + 150, y + 3, &v, vcol, 1, false);
                        }
                        1 => {
                            let f = ((cfg.fov_deg as f32 - 75.0) / 30.0).clamp(0.0, 1.0);
                            self.draw_slider_bar(lx + 72, y + 3, 72, 7, f, if is_sel { ui.ok } else { ui.tag }, ui, is_sel);
                            let v = format!("{}d", cfg.fov_deg);
                            self.ui_draw_text(lx + 150, y + 3, &v, vcol, 1, false);
                        }
                        2 => {
                            self.draw_toggle_pill(lx + 146, y + 1, cfg.analog_filter, ui, is_sel);
                        }
                        3 => {
                            let s = cfg.stick_overlay.name();
                            let disp = if is_sel { format!("<{s}>") } else { s.to_string() };
                            self.ui_draw_text(lx + lw - (disp.len() as i32) * 6 - 8, y + 3, &disp, vcol, 1, false);
                        }
                        _ => {
                            self.draw_toggle_pill(lx + 146, y + 1, cfg.debug_osd, ui, is_sel);
                        }
                    }
                }

                self.draw_camera_preview(202, 42, 112, 110, cfg, ui);
            }
            SettingsTab::System => {
                self.ui_draw_panel(6, 42, UI_W - 12, 110, ui.bg, ui.muted, 7);
                let display_val = match (cfg.display_output, hdmi_tag) {
                    (DisplayOutput::Auto, Some(tag)) => format!("Auto [{tag}]"),
                    (DisplayOutput::Auto, None) => "Auto [LCD]".to_string(),
                    (DisplayOutput::HdmiOnly, Some(tag)) => format!("HDMI [{tag}]"),
                    (DisplayOutput::HdmiOnly, None) => "HDMI Only [No HDMI]".to_string(),
                    (DisplayOutput::LcdOnly, _) => "LCD Only".to_string(),
                };
                let labels = [
                    "UI Theme",
                    "Gfx Detail",
                    "Sky Detail",
                    "Sound",
                    "Display",
                    "HDMI Res",
                    "LCD on HDMI",
                ];
                for (i, k) in labels.iter().enumerate() {
                    let y = 46 + (i as i32) * 15;
                    let is_sel = !focus_tabs && i == row;
                    if is_sel {
                        self.ui_fill_rect(10, y - 1, UI_W - 20, 13, ui.sel_bg);
                        self.ui_draw_rect(10, y - 1, UI_W - 20, 13, ui.sel_border);
                        self.ui_fill_rect(10, y - 1, 3, 13, ui.header_line);
                    }
                    let kcol = if is_sel { ui.sel_label } else { ui.label };
                    let vcol = if is_sel { ui.sel_val } else { ui.val };
                    self.ui_draw_text(18, y + 2, k, kcol, 1, false);

                    match i {
                        0 => {
                            let s = cfg.ui_theme.name();
                            let d = if is_sel { format!("< {s} >") } else { s.to_string() };
                            self.ui_draw_text(126, y + 2, &d, vcol, 1, false);
                        }
                        1 => {
                            let s = if cfg.detail_high { "High" } else { "Low" };
                            let d = if is_sel { format!("< {s} >") } else { s.to_string() };
                            self.ui_draw_text(126, y + 2, &d, vcol, 1, false);
                        }
                        2 => {
                            self.draw_toggle_pill(126, y, cfg.sky_detail, ui, is_sel);
                            self.ui_draw_text(168, y + 2, "Sun/Stars", ui.muted, 1, false);
                        }
                        3 => {
                            self.draw_toggle_pill(126, y, cfg.sound_on, ui, is_sel);
                            self.ui_draw_text(168, y + 2, "Whoop Audio", ui.muted, 1, false);
                        }
                        4 => {
                            let d = if is_sel { format!("< {display_val} >") } else { display_val.clone() };
                            self.ui_draw_text(126, y + 2, &d, vcol, 1, false);
                        }
                        5 => {
                            let s = cfg.hdmi_resolution.name();
                            let d = if is_sel { format!("< {s} >") } else { s.to_string() };
                            self.ui_draw_text(126, y + 2, &d, vcol, 1, false);
                        }
                        _ => {
                            let s = cfg.hdmi_lcd_backlight.name();
                            let d = if is_sel { format!("< {s} >") } else { s.to_string() };
                            self.ui_draw_text(126, y + 2, &d, vcol, 1, false);
                        }
                    }
                }
            }
        }

        let footer = if focus_tabs {
            "[Z/C] Tab | [X/Ent] Edit | [Tab] Next | [Esc] Save"
        } else {
            "[Tab] Next | [F/X] Nav | [Z/C] Edit | [Esc] Save"
        };
        self.draw_footer(footer, None, ui);
    }

    /// Static status card drawn once on the built-in 1.9" LCD when `HDMI Only` mode is active.
    pub fn render_hdmi_lcd_notice(&mut self, cfg: &Config, hdmi_info: &str) {
        let ui = ui_colors(cfg.ui_theme);
        self.clear(ui.bg);
        self.draw_topbar("STICKZERO", "HDMI ONLY", Some(ui.ok), ui);
        let cx = UI_W / 2;

        self.ui_fill_rect(18, 34, UI_W - 36, 98, ui.header_bg);
        self.ui_draw_rect(18, 34, UI_W - 36, 98, ui.sel_border);

        let title = "HDMI OUTPUT ACTIVE";
        self.ui_draw_text(cx - (title.len() as i32) * 6, 46, title, ui.sel_label, 2, false);

        self.ui_draw_text(
            cx - (hdmi_info.len() as i32) * 3,
            74,
            hdmi_info,
            ui.tag,
            1,
            false,
        );

        let l1 = if cfg.hdmi_lcd_backlight == HdmiLcdBacklight::Dim {
            "LCD dimmed to save battery & SPI"
        } else {
            "LCD paused to save battery & SPI"
        };
        let l2 = "Unplug HDMI to return to LCD";
        self.ui_draw_text(cx - (l1.len() as i32) * 3, 96, l1, ui.label, 1, false);
        self.ui_draw_text(cx - (l2.len() as i32) * 3, 110, l2, ui.muted, 1, false);

        self.draw_footer("Settings -> System -> Display / HDMI LCD", None, ui);
    }

    pub fn render_controller_screen(
        &mut self,
        cfg: &Config,
        input: &InputManager,
        selected: usize,
        raw_debug: bool,
        status_note: Option<&str>,
        menu_time: f32,
    ) {
        let ui = ui_colors(cfg.ui_theme);
        self.render_menu_backdrop(cfg, menu_time, 5);

        let ctrl_src = input.control_source();
        let (dot_col, header_tag) = match ctrl_src {
            FlightControlSource::Controller => (ui.ok, input.controller_bus_tag()),
            FlightControlSource::ImuSimplified => (ui.tag, "IMU EASY"),
            FlightControlSource::None => (ui.warn, "NO INPUT"),
        };
        self.draw_topbar("CONTROLLER", header_tag, Some(dot_col), ui);

        let multi_dev = input.candidate_count > 1;
        let actions = controller_actions(ctrl_src, multi_dev);
        let sel = selected.min(actions.len().saturating_sub(1));
        let wizard_active = input.calib.is_some() || input.aux_wiz.is_some();

        // --- Left Action Rail ---
        let nav_x = 6;
        let nav_y = 24;
        let nav_w = 108;
        let nav_h = 128;
        self.ui_draw_panel(nav_x, nav_y, nav_w, nav_h, ui.bg, ui.muted, 7);

        let compact = actions.len() >= 6;
        let row_step = if compact { 20 } else { 23 };
        let row_h = if compact { 17 } else { 19 };
        let start_y = nav_y + 4;

        for (i, &act) in actions.iter().enumerate() {
            let y = start_y + (i as i32) * row_step;
            let is_sel = !wizard_active && i == sel;
            let is_primary = i == 0 && ctrl_src != FlightControlSource::Controller;
            let bg = if is_sel && is_primary {
                blend_rgb565(ui.sel_bg, ui.ok, 2)
            } else if is_sel {
                ui.sel_bg
            } else {
                ui.header_bg
            };
            let border = if is_sel && is_primary {
                ui.ok
            } else if is_sel {
                ui.sel_border
            } else {
                blend_rgb565(ui.bg, ui.muted, 4)
            };
            self.ui_fill_rect(nav_x + 4, y, nav_w - 8, row_h, bg);
            self.ui_draw_rect(nav_x + 4, y, nav_w - 8, row_h, border);
            if is_sel {
                self.ui_fill_rect(
                    nav_x + 4,
                    y,
                    3,
                    row_h,
                    if is_primary { ui.ok } else { ui.header_line },
                );
            }

            let ty = y + (row_h - 8) / 2;
            let lcol = if is_sel {
                if is_primary { ui.title } else { ui.sel_label }
            } else if is_primary {
                ui.ok
            } else {
                ui.label
            };
            self.ui_draw_text(nav_x + 11, ty, act.label(), lcol, 1, false);

            match act {
                ControllerAction::ToggleThrottleMode => {
                    let badge = match cfg.throttle_mode {
                        crate::config::ThrottleMode::RcFull => "RC",
                        crate::config::ThrottleMode::GamepadCenter => "PAD",
                    };
                    let bx = nav_x + nav_w - 9 - (badge.len() as i32) * 6;
                    self.ui_draw_text(bx, ty, badge, if is_sel { ui.sel_val } else { ui.tag }, 1, false);
                }
                ControllerAction::ToggleRawDebug => {
                    let badge = if raw_debug { "ON" } else { "OFF" };
                    let bcol = if raw_debug { ui.ok } else { ui.muted };
                    let bx = nav_x + nav_w - 9 - (badge.len() as i32) * 6;
                    self.ui_draw_text(bx, ty, badge, bcol, 1, false);
                }
                _ => {}
            }
        }

        // --- Right Contextual Setup / Monitor Card ---
        let cx = 120;
        let cy = 24;
        let cw = 194;
        let ch = 128;
        self.ui_draw_panel(cx, cy, cw, ch, ui.bg, ui.muted, 7);
        self.ui_fill_rect_alpha(cx + 1, cy + 1, cw - 2, 15, ui.header_bg, 7);
        self.ui_draw_hline(cx + 1, cx + cw - 2, cy + 16, ui.muted);

        if let Some(ref wiz) = input.calib {
            self.ui_draw_text(cx + 6, cy + 5, "STICK CALIBRATION", ui.tag, 1, false);
            let step_tag = format!("{}/4", (wiz.step + 1).min(4));
            self.ui_draw_text(cx + cw - (step_tag.len() as i32) * 6 - 6, cy + 5, &step_tag, ui.sel_border, 1, false);

            // 4-step segmented progress bar
            for s_idx in 0..4 {
                let sx = cx + 7 + (s_idx as i32) * 46;
                let col = if s_idx < wiz.step {
                    ui.ok
                } else if s_idx == wiz.step {
                    ui.sel_border
                } else {
                    ui.header_bg
                };
                self.ui_fill_rect(sx, cy + 22, 42, 5, col);
                self.ui_draw_rect(sx, cy + 22, 42, 5, ui.muted);
            }

            self.ui_draw_text(cx + 7, cy + 36, wiz.step_prompt(), ui.sel_label, 1, false);
            self.ui_fill_rect(cx + 7, cy + 52, cw - 14, 44, ui.header_bg);
            self.ui_draw_rect(cx + 7, cy + 52, cw - 14, 44, ui.accent);
            let m1: String = wiz.message.chars().take(28).collect();
            let m2: String = wiz.message.chars().skip(28).take(28).collect();
            self.ui_draw_text(cx + 12, cy + 61, &m1, ui.ok, 1, false);
            if !m2.is_empty() {
                self.ui_draw_text(cx + 12, cy + 74, &m2, ui.ok, 1, false);
            }
            self.ui_draw_text(cx + 7, cy + 106, "Push stick fully to detect", ui.title, 1, false);
            self.ui_draw_text(cx + 7, cy + 117, "[Esc] Cancel Calibration", ui.muted, 1, false);
        } else if let Some(ref aw) = input.aux_wiz {
            self.ui_draw_text(cx + 6, cy + 5, "AUX SWITCH BINDER", ui.ok, 1, false);
            let step_tag = format!("{}/3", (aw.step + 1).min(3));
            self.ui_draw_text(cx + cw - (step_tag.len() as i32) * 6 - 6, cy + 5, &step_tag, ui.sel_border, 1, false);

            // 3-step segmented progress bar
            for s_idx in 0..3 {
                let sx = cx + 7 + (s_idx as i32) * 61;
                let col = if s_idx < aw.step {
                    ui.ok
                } else if s_idx == aw.step {
                    ui.sel_border
                } else {
                    ui.header_bg
                };
                self.ui_fill_rect(sx, cy + 22, 57, 5, col);
                self.ui_draw_rect(sx, cy + 22, 57, 5, ui.muted);
            }

            self.ui_draw_text(cx + 7, cy + 36, aw.step_prompt(), ui.sel_label, 1, false);
            self.ui_fill_rect(cx + 7, cy + 52, cw - 14, 44, ui.header_bg);
            self.ui_draw_rect(cx + 7, cy + 52, cw - 14, 44, ui.ok);
            let m1: String = aw.message.chars().take(28).collect();
            let m2: String = aw.message.chars().skip(28).take(28).collect();
            self.ui_draw_text(cx + 12, cy + 61, &m1, ui.tag, 1, false);
            if !m2.is_empty() {
                self.ui_draw_text(cx + 12, cy + 74, &m2, ui.tag, 1, false);
            }
            self.ui_draw_text(cx + 7, cy + 106, "Flip radio switch or button", ui.title, 1, false);
            self.ui_draw_text(cx + 7, cy + 117, "[Ent] Skip | [Esc] Cancel", ui.muted, 1, false);
        } else {
            match ctrl_src {
                FlightControlSource::Controller if raw_debug => {
                    self.ui_draw_text(cx + 6, cy + 5, "RAW DIAGNOSTICS", ui.tag, 1, false);
                    let dev_tag = if multi_dev {
                        format!("{} DEVS", input.candidate_count)
                    } else {
                        "EVDEV".to_string()
                    };
                    self.ui_draw_text(cx + cw - (dev_tag.len() as i32) * 6 - 6, cy + 5, &dev_tag, ui.ok, 1, false);

                    let sticks = input.flight_sticks(cfg);
                    let bars: [(&str, f32, String); 4] = [
                        ("THR", sticks.throttle * 2.0 - 1.0, format!("{:3.0}%", sticks.throttle * 100.0)),
                        ("YAW", sticks.yaw, format!("{:+3.0}%", sticks.yaw * 100.0)),
                        ("PIT", sticks.pitch, format!("{:+3.0}%", sticks.pitch * 100.0)),
                        ("ROL", sticks.roll, format!("{:+3.0}%", sticks.roll * 100.0)),
                    ];
                    for (i, (lbl, val, pct)) in bars.iter().enumerate() {
                        let y = cy + 21 + (i as i32) * 14;
                        self.ui_draw_text(cx + 6, y + 1, lbl, ui.label, 1, false);
                        let bx = cx + 28;
                        let bw = 122;
                        self.ui_fill_rect(bx, y, bw, 9, ui.header_bg);
                        self.ui_draw_rect(bx, y, bw, 9, ui.muted);
                        self.ui_draw_vline(bx + bw / 2, y + 1, y + 7, ui.muted);
                        let pos = bx + ((clampf(*val, -1.0, 1.0) + 1.0) * 0.5 * (bw - 4) as f32) as i32;
                        self.ui_fill_rect(pos, y + 2, 4, 5, ui.ok);
                        self.ui_draw_text(cx + 154, y + 1, pct, ui.title, 1, false);
                    }

                    self.ui_draw_hline(cx + 6, cx + cw - 7, cy + 79, ui.muted);
                    self.ui_draw_text(cx + 6, cy + 83, "RAW EVDEV ABS:", ui.tag, 1, false);
                    let raw = input.raw_axes_summary();
                    let l1: String = raw.chars().take(30).collect();
                    let l2: String = raw.chars().skip(30).take(30).collect();
                    self.ui_draw_text(cx + 6, cy + 94, &l1, ui.ok, 1, false);
                    if !l2.is_empty() {
                        self.ui_draw_text(cx + 6, cy + 104, &l2, ui.ok, 1, false);
                    }
                    let note = status_note.unwrap_or("Toggle DEBUG off for Gimbals");
                    self.ui_draw_text(cx + 6, cy + 116, note, ui.muted, 1, false);
                }
                FlightControlSource::Controller => {
                    let cname: String = input.controller_name().chars().take(24).collect();
                    self.ui_draw_text(cx + 6, cy + 5, &cname, ui.sel_label, 1, false);
                    if multi_dev {
                        let dt = format!("[{}]", input.candidate_count);
                        self.ui_draw_text(cx + cw - (dt.len() as i32) * 6 - 6, cy + 5, &dt, ui.tag, 1, false);
                    }

                    let sticks = input.flight_sticks(cfg);
                    let s = self.ui_scale();
                    let (ox, oy) = self.ui_origin();
                    self.draw_stick_box(ox + (cx + 26) * s, oy + (cy + 22) * s, 48 * s, sticks.yaw, sticks.throttle * 2.0 - 1.0, s);
                    self.draw_stick_box(ox + (cx + 116) * s, oy + (cy + 22) * s, 48 * s, sticks.roll, sticks.pitch, s);
                    self.ui_draw_text(cx + 35, cy + 73, "L-STK", ui.muted, 1, false);
                    self.ui_draw_text(cx + 125, cy + 73, "R-STK", ui.muted, 1, false);

                    let thr_txt = match cfg.throttle_mode {
                        crate::config::ThrottleMode::RcFull => "Throttle: RC Full (0-100%)",
                        crate::config::ThrottleMode::GamepadCenter => "Throttle: Gamepad (Spring)",
                    };
                    self.ui_draw_text(cx + 6, cy + 86, thr_txt, ui.title, 1, false);

                    // 3 clean AUX switch status pills
                    let pills = [
                        ("ARM", cfg.aux_arm),
                        ("MOD", cfg.aux_mode),
                        ("TRT", cfg.aux_turtle),
                    ];
                    for (i, (lbl, binding)) in pills.iter().enumerate() {
                        let px = cx + 6 + (i as i32) * 62;
                        let py = cy + 98;
                        let bound = *binding != AuxBinding::None;
                        let pbg = if bound { blend_rgb565(ui.header_bg, ui.ok, 2) } else { ui.header_bg };
                        let pbd = if bound { ui.ok } else { ui.muted };
                        self.ui_fill_rect(px, py, 58, 13, pbg);
                        self.ui_draw_rect(px, py, 58, 13, pbd);
                        let txt = format!("{}:{}", lbl, binding.short_label());
                        let trim: String = txt.chars().take(9).collect();
                        self.ui_draw_text(px + 3, py + 3, &trim, if bound { ui.title } else { ui.muted }, 1, false);
                    }

                    let note = status_note.unwrap_or("Move sticks to test response");
                    let ncol = if status_note.is_some() { ui.ok } else { ui.muted };
                    self.ui_draw_text(cx + 6, cy + 116, note, ncol, 1, false);
                }
                FlightControlSource::ImuSimplified if raw_debug => {
                    self.ui_draw_text(cx + 6, cy + 5, "IMU DIAGNOSTICS", ui.tag, 1, false);
                    self.ui_draw_text(cx + cw - 42, cy + 5, "SENSOR", ui.ok, 1, false);

                    let sticks = input.flight_sticks(cfg);
                    let s = self.ui_scale();
                    let (ox, oy) = self.ui_origin();
                    let thr_pct = (sticks.throttle * 100.0).round() as u8;
                    self.draw_imu_throttle_bar(ox + (cx + 40) * s, oy + (cy + 22) * s, 16 * s, 40 * s, thr_pct, input.imu_climb_cmd(), false, s);
                    self.draw_imu_tilt_box(ox + (cx + 108) * s, oy + (cy + 22) * s, 40 * s, sticks.roll, sticks.pitch, false, s);

                    let num = format!("R:{:+.2} P:{:+.2} A:{:3}%", sticks.roll, sticks.pitch, thr_pct);
                    self.ui_draw_text(cx + 6, cy + 68, &num, ui.title, 1, false);
                    self.ui_draw_hline(cx + 6, cx + cw - 7, cy + 79, ui.muted);
                    self.ui_draw_text(cx + 6, cy + 84, "RAW IIO SENSOR:", ui.tag, 1, false);
                    let raw = input.imu.summary();
                    let l1: String = raw.chars().take(30).collect();
                    let l2: String = raw.chars().skip(30).take(30).collect();
                    self.ui_draw_text(cx + 6, cy + 95, &l1, ui.ok, 1, false);
                    if !l2.is_empty() {
                        self.ui_draw_text(cx + 6, cy + 105, &l2, ui.ok, 1, false);
                    }
                    let note = status_note.unwrap_or("ZERO IMU to re-level");
                    self.ui_draw_text(cx + 6, cy + 116, note, ui.muted, 1, false);
                }
                FlightControlSource::ImuSimplified => {
                    self.ui_draw_text(cx + 6, cy + 5, "BUILT-IN IMU", ui.tag, 1, false);
                    self.ui_draw_text(cx + cw - 36, cy + 5, "READY", ui.ok, 1, false);

                    let sticks = input.flight_sticks(cfg);
                    let s = self.ui_scale();
                    let (ox, oy) = self.ui_origin();
                    let thr_pct = (sticks.throttle * 100.0).round() as u8;
                    self.draw_imu_throttle_bar(ox + (cx + 38) * s, oy + (cy + 23) * s, 18 * s, 48 * s, thr_pct, input.imu_climb_cmd(), false, s);
                    self.draw_imu_tilt_box(ox + (cx + 104) * s, oy + (cy + 23) * s, 48 * s, sticks.roll, sticks.pitch, false, s);
                    self.ui_draw_text(cx + 38, cy + 74, "ALT", ui.muted, 1, false);
                    self.ui_draw_text(cx + 116, cy + 74, "TILT", ui.tag, 1, false);

                    self.ui_draw_hline(cx + 6, cx + cw - 7, cy + 86, ui.muted);
                    self.ui_draw_text(cx + 6, cy + 92, "Tilt Pitch/Roll | [Z/C] Yaw", ui.title, 1, false);
                    self.ui_draw_text(cx + 6, cy + 104, "[F/X] Alt | Plug USB/BT radio", ui.muted, 1, false);
                    if let Some(note) = status_note {
                        self.ui_draw_text(cx + 6, cy + 116, note, ui.ok, 1, false);
                    } else {
                        self.ui_draw_text(cx + 6, cy + 116, "ZERO IMU sets level hover", ui.sel_label, 1, false);
                    }
                }
                FlightControlSource::None => {
                    self.ui_draw_text(cx + 6, cy + 5, "CONNECT CONTROLLER", ui.tag, 1, false);
                    self.ui_draw_text(cx + cw - 30, cy + 5, "NONE", ui.warn, 1, false);

                    let reason = if input.is_cardputer_lite() {
                        "CardputerZero Lite (No IMU)"
                    } else {
                        "No BT/USB Controller Found"
                    };
                    self.ui_draw_text(cx + 6, cy + 23, reason, ui.warn, 1, false);

                    self.ui_draw_text(cx + 6, cy + 39, "1. USB-C: Plug ELRS / Radio", ui.title, 1, false);
                    self.ui_draw_text(cx + 6, cy + 52, "2. BT: Pair Gamepad / Radio", ui.title, 1, false);
                    self.ui_draw_text(cx + 6, cy + 65, "3. Press SCAN DEV on left", ui.sel_label, 1, false);

                    self.ui_draw_hline(cx + 6, cx + cw - 7, cy + 80, ui.muted);
                    self.ui_draw_text(cx + 6, cy + 86, "Keyboard Fallback (Mode 2):", ui.tag, 1, false);
                    self.ui_draw_text(cx + 6, cy + 98, "[FXZC] Thr/Yaw [KBNM] Pit/Rol", ui.muted, 1, false);

                    if let Some(note) = status_note {
                        self.ui_draw_text(cx + 6, cy + 115, note, ui.ok, 1, false);
                    }
                }
            }
        }

        let footer = if wizard_active {
            "Follow prompt above | [Esc] Cancel Wizard"
        } else {
            "[F/X] Nav | [Ent] OK | [Z/C] Opt | [Esc] Back"
        };
        self.draw_footer(footer, Some(dot_col), ui);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{Config, UiTheme, VisualTheme};
    use crate::input::InputManager;
    use crate::physics::QuadState;
    use crate::tracks::TRACKS;

    #[test]
    fn test_settings_tab_cycle_roundtrip() {
        let mut t = SettingsTab::Flight;
        for expected in [SettingsTab::Rates, SettingsTab::Camera, SettingsTab::System, SettingsTab::Flight] {
            t = t.next();
            assert_eq!(t, expected);
        }
        for expected in [SettingsTab::System, SettingsTab::Camera, SettingsTab::Rates, SettingsTab::Flight] {
            t = t.prev();
            assert_eq!(t, expected);
        }
        assert_eq!(SettingsTab::Flight.row_count(), 4);
        assert_eq!(SettingsTab::Rates.row_count(), 4);
        assert_eq!(SettingsTab::Camera.row_count(), 5);
        assert_eq!(SettingsTab::System.row_count(), 7);
    }

    #[test]
    fn test_controller_actions_adapt_to_hardware() {
        let none_acts = controller_actions(FlightControlSource::None, false);
        assert!(!none_acts.contains(&ControllerAction::ToggleRawDebug));
        assert!(!none_acts.contains(&ControllerAction::CalibrateSticks));
        assert!(none_acts.contains(&ControllerAction::ScanDevices));

        let imu_acts = controller_actions(FlightControlSource::ImuSimplified, false);
        assert!(imu_acts.contains(&ControllerAction::ZeroImu));
        assert!(imu_acts.contains(&ControllerAction::ToggleRawDebug));
        assert!(!imu_acts.contains(&ControllerAction::BindAux));

        let ctrl_single = controller_actions(FlightControlSource::Controller, false);
        assert!(ctrl_single.contains(&ControllerAction::CalibrateSticks));
        assert!(ctrl_single.contains(&ControllerAction::BindAux));
        assert!(!ctrl_single.contains(&ControllerAction::NextDevice));

        let ctrl_multi = controller_actions(FlightControlSource::Controller, true);
        assert!(ctrl_multi.contains(&ControllerAction::NextDevice));
    }

    #[test]
    fn test_steam_sim_menus_render_across_resolutions() {
        let mut cfg = Config::default();
        let input = InputManager::new();
        let quad = QuadState::spawn(&TRACKS[0], &cfg);

        for (w, h) in [(320, 170), (640, 360)] {
            let mut r = Renderer::with_size(w, h);
            for ui_theme in [UiTheme::Monokai, UiTheme::StickTime] {
                cfg.ui_theme = ui_theme;
                for vis in [VisualTheme::Summer, VisualTheme::Evening, VisualTheme::Night, VisualTheme::Wireframe] {
                    cfg.theme = vis;
                    for sel in 0..6 {
                        r.render_main_menu(&cfg, sel, &input, Some("720p"), 1.2);
                    }
                }
                for tab in SettingsTab::ALL {
                    r.render_settings_menu(&cfg, tab, 0, true, false, None, 0.5);
                    for row in 0..tab.row_count() {
                        r.render_settings_menu(&cfg, tab, row, false, false, None, 0.5);
                    }
                }
                for row in 0..11 {
                    r.render_drone_tuning_menu(&cfg, row, 0.5);
                }
                r.render_pause_menu(&cfg, &quad, 0);
                r.render_controller_screen(&cfg, &input, 0, false, None, 0.5);
                r.render_controller_screen(&cfg, &input, 0, true, Some("Devices rescanned"), 0.5);
            }
            assert!(r.fb.iter().any(|&px| px != r.fb[0]));
        }
    }
}
