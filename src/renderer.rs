//! Ultra-fast zero-allocation 320x170 RGB565 3D software rasterizer with a
//! per-pixel 16-bit perspective-correct Z-buffer, solid 3D racing gates,
//! authentic Betaflight OSD + post-flight statistics, and CardputerZero 1.9" menus.

use crate::config::{
    AuxBinding, BatteryPack, Config, DisplayOutput, GameMode, StickOverlay, UiTheme, VisualTheme,
};
use crate::display::{SCREEN_H, SCREEN_W};
use crate::font::{glyph, GLYPH_H, GLYPH_W};
use crate::input::{FlightControlSource, InputManager};
use crate::math::{clampf, Quat, Vec3};
use crate::physics::{QuadState, StickInput};
use crate::tracks::{GateKind, StructureColor, Track, TRACKS};

const W: i32 = SCREEN_W as i32;
const H: i32 = SCREEN_H as i32;
const HALF_W: f32 = (SCREEN_W as f32) * 0.5;
const HALF_H: f32 = (SCREEN_H as f32) * 0.5;
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

#[derive(Copy, Clone)]
struct UiColors {
    bg: u16,
    header_bg: u16,
    header_line: u16,
    title: u16,
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
    pub fb: Box<[u16; SCREEN_W * SCREEN_H]>,
    zbuf: Box<[u16; SCREEN_W * SCREEN_H]>,
    focal: f32,
    rng_state: u32,
    /// Zero-allocation ring buffer storing recent (lx, ly, rx, ry) stick positions for OSD trails.
    stick_trail: [(f32, f32, f32, f32); STICK_TRAIL_LEN],
    trail_head: usize,
    trail_len: usize,
}

impl Renderer {
    pub fn new() -> Self {
        Self {
            fb: Box::new([0u16; SCREEN_W * SCREEN_H]),
            zbuf: Box::new([0u16; SCREEN_W * SCREEN_H]),
            focal: 160.0,
            rng_state: 0x51A7_1C09,
            stick_trail: [(0.0, 0.0, 0.0, 0.0); STICK_TRAIL_LEN],
            trail_head: 0,
            trail_len: 0,
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
        if x >= 0 && x < W && y >= 0 && y < H {
            self.fb[(y as usize) * SCREEN_W + (x as usize)] = color;
        }
    }

    pub fn clear(&mut self, color: u16) {
        self.fb.fill(color);
    }

    pub fn fill_rect(&mut self, x: i32, y: i32, w: i32, h: i32, color: u16) {
        let x0 = x.max(0);
        let y0 = y.max(0);
        let x1 = (x + w).min(W);
        let y1 = (y + h).min(H);
        if x0 >= x1 || y0 >= y1 {
            return;
        }
        for py in y0..y1 {
            let row = (py as usize) * SCREEN_W;
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

    #[inline]
    fn draw_hline(&mut self, x0: i32, x1: i32, y: i32, color: u16) {
        if y < 0 || y >= H {
            return;
        }
        let lx = x0.min(x1).max(0);
        let rx = x0.max(x1).min(W - 1);
        if lx <= rx {
            let row = (y as usize) * SCREEN_W;
            self.fb[row + (lx as usize)..=row + (rx as usize)].fill(color);
        }
    }

    #[inline]
    fn draw_vline(&mut self, x: i32, y0: i32, y1: i32, color: u16) {
        if x < 0 || x >= W {
            return;
        }
        let ty = y0.min(y1).max(0);
        let by = y0.max(y1).min(H - 1);
        for y in ty..=by {
            self.fb[(y as usize) * SCREEN_W + (x as usize)] = color;
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
            if x0 >= 0 && x0 < W && y0 >= 0 && y0 < H {
                let idx = (y0 as usize) * SCREEN_W + (x0 as usize);
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

        let y_start = (v[0].y.ceil() as i32).max(0);
        let y_end = (v[2].y.floor() as i32).min(H - 1);
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
            let x_end = (xb.floor() as i32).min(W - 1);
            if x_start > x_end {
                continue;
            }

            let dx = xb - xa;
            let diz_dx = if dx > 1e-4 { (izb - iza) / dx } else { 0.0 };
            let mut iz = iza + diz_dx * (x_start as f32 - xa);
            let row = (y as usize) * SCREEN_W;

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
        for &b in text.as_bytes() {
            let g = glyph(b);
            if outline {
                for (col_idx, &col) in g.iter().enumerate() {
                    for row in 0..GLYPH_H {
                        if (col & (1 << row)) != 0 {
                            let px = cx + (col_idx as i32) * scale;
                            let py = y + row * scale;
                            self.fill_rect(px - 1, py, scale + 2, scale, black);
                            self.fill_rect(px, py - 1, scale, scale + 2, black);
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
            x: HALF_W + c.x * self.focal * inv_z,
            y: HALF_H - c.y * self.focal * inv_z,
            iz: (inv_z * IZ_SCALE).clamp(1.0, 65000.0),
        }
    }

    /// Exact zero-safe Cohen-Sutherland 2D line clipper that also linearly clips `iz`.
    fn clip_line_2d_z(
        mut x0: f32,
        mut y0: f32,
        mut iz0: f32,
        mut x1: f32,
        mut y1: f32,
        mut iz1: f32,
    ) -> Option<(i32, i32, f32, i32, i32, f32)> {
        const XMIN: f32 = 0.0;
        const YMIN: f32 = 0.0;
        const XMAX: f32 = (W - 1) as f32;
        const YMAX: f32 = (H - 1) as f32;

        let outcode = |x: f32, y: f32| -> u8 {
            let mut c = 0u8;
            if x < XMIN { c |= 1; } else if x > XMAX { c |= 2; }
            if y < YMIN { c |= 4; } else if y > YMAX { c |= 8; }
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
                let t = (YMAX - y0) / dy;
                (x0 + (x1 - x0) * t, YMAX, t)
            } else if (out & 4) != 0 {
                let dy = y1 - y0;
                if dy.abs() < 1e-6 { return None; }
                let t = (YMIN - y0) / dy;
                (x0 + (x1 - x0) * t, YMIN, t)
            } else if (out & 2) != 0 {
                let dx = x1 - x0;
                if dx.abs() < 1e-6 { return None; }
                let t = (XMAX - x0) / dx;
                (XMAX, y0 + (y1 - y0) * t, t)
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
        if let Some((x0, y0, _, x1, y1, _)) = Self::clip_line_2d_z(sa.x, sa.y, sa.iz, sb.x, sb.y, sb.iz) {
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
        if let Some((x0, y0, iz0, x1, y1, iz1)) = Self::clip_line_2d_z(sa.x, sa.y, sa.iz, sb.x, sb.y, sb.iz) {
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
        if max_x < 0.0 || min_x >= W as f32 || max_y < 0.0 || min_y >= H as f32 {
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
                    Self::clip_line_2d_z(sv[k].x, sv[k].y, sv[k].iz, sv[n].x, sv[n].y, sv[n].iz)
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
                    Self::clip_line_2d_z(sv[k].x, sv[k].y, sv[k].iz, sv[n].x, sv[n].y, sv[n].iz)
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
        let (cam_right, cam_up, cam_fwd) = cam_rot.basis();
        let ux = cam_right.y;
        let uy = cam_up.y;
        let uz = cam_fwd.y * self.focal;
        let inv_f = 1.0 / self.focal;

        if !detail_high {
            // Fast 2-tone flat mode when Gfx Detail = Low
            if ux.abs() < 0.002 {
                let mut prev_sky: Option<bool> = None;
                for y in 0..SCREEN_H {
                    let ry = HALF_H - (y as f32);
                    let is_sky = (uy * ry + uz) >= 0.0;
                    let row = y * SCREEN_W;
                    self.fb[row..row + SCREEN_W].fill(if is_sky { pal.sky_top } else { pal.ground_near });
                    if let (Some(was_sky), Some(hcol)) = (prev_sky, pal.horizon_line) {
                        if was_sky != is_sky {
                            self.fb[row..row + SCREEN_W].fill(hcol);
                        }
                    }
                    prev_sky = Some(is_sky);
                }
            } else {
                let inv_ux = 1.0 / ux;
                let sky_on_right = ux > 0.0;
                for y in 0..SCREEN_H {
                    let ry = HALF_H - (y as f32);
                    let x_hor = HALF_W - (uy * ry + uz) * inv_ux;
                    let x_split = (x_hor.round() as i32).clamp(0, W) as usize;
                    let row = y * SCREEN_W;
                    let (left_col, right_col) = if sky_on_right {
                        (pal.ground_near, pal.sky_top)
                    } else {
                        (pal.sky_top, pal.ground_near)
                    };
                    if x_split > 0 {
                        self.fb[row..row + x_split].fill(left_col);
                    }
                    if x_split < SCREEN_W {
                        self.fb[row + x_split..row + SCREEN_W].fill(right_col);
                    }
                    if let Some(hcol) = pal.horizon_line {
                        let x_hor_next = HALF_W - (uy * (ry - 1.0) + uz) * inv_ux;
                        let lx = x_hor.min(x_hor_next).round() as i32;
                        let rx = x_hor.max(x_hor_next).round() as i32;
                        if rx >= 0 && lx < W {
                            self.draw_hline(lx.max(0), rx.min(W - 1), y as i32, hcol);
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
        let step_total = step_fp * ((SCREEN_W - 1) as i32);
        let rolled = ux.abs() >= 0.002;
        let inv_ux = if rolled { 1.0 / ux } else { 0.0 };
        let mut prev_sky: Option<bool> = None;

        for y in 0..SCREEN_H {
            let ry = HALF_H - (y as f32);
            let base = uy * ry + uz;
            let row = y * SCREEN_W;
            let e_left = (ux * (-HALF_W) + base) * inv_f;
            let mut acc: i32 = ((64.0 + e_left * LUT_SCALE) * 65536.0) as i32;
            let acc_right = acc + step_total;

            // Fast-path uniform scanlines (flat/upside-down or fully clamped sky/ground)
            if step_fp == 0 {
                let idx = (acc >> 16).clamp(0, 127) as usize;
                self.fb[row..row + SCREEN_W].fill(lut[idx]);
            } else if acc <= 0 && acc_right <= 0 {
                self.fb[row..row + SCREEN_W].fill(lut[0]);
            } else if acc >= (127 << 16) && acc_right >= (127 << 16) {
                self.fb[row..row + SCREEN_W].fill(lut[127]);
            } else {
                // Smooth per-pixel rolled gradient using pure integer fixed-point LUT lookup (zero floats!)
                for px in &mut self.fb[row..row + SCREEN_W] {
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
                            self.fb[row..row + SCREEN_W].fill(hcol);
                        }
                    }
                    prev_sky = Some(is_sky);
                } else {
                    let x0 = HALF_W - base * inv_ux;
                    let x1 = HALF_W - (uy * (ry - 1.0) + uz) * inv_ux;
                    let lx = x0.min(x1).round() as i32;
                    let rx = x0.max(x1).round() as i32;
                    if rx >= 0 && lx < W {
                        self.draw_hline(lx.max(0), rx.min(W - 1), y as i32, hcol);
                    }
                }
            }
        }
    }

    fn draw_sky_detail(&mut self, cam_inv: Quat, theme: VisualTheme) {
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
                self.fill_rect(sx - 5, sy - 5, 11, 11, halo_col);
                self.fill_rect(sx - 3, sy - 3, 7, 7, sun_col);
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
            for (idx, &s) in STARS.iter().enumerate() {
                let c = cam_inv.rotate(s);
                if c.z > 0.2 {
                    let p = self.project_sv(c);
                    let px = p.x as i32;
                    let py = p.y as i32;
                    self.put_pixel(px, py, star_col);
                    if idx % 3 == 0 {
                        self.put_pixel(px + 1, py, star_col);
                    }
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
        let pal = palette(cfg.theme);
        let half_fov_rad = ((cfg.fov_deg as f32).clamp(70.0, 115.0) * 0.5).to_radians();
        self.focal = HALF_W / half_fov_rad.tan();

        // Clear 16-bit per-pixel Z-buffer (0 = farthest)
        self.zbuf.fill(0);

        let tilt = -(cfg.camera_tilt_deg as f32).to_radians();
        let cam_rot = quad.rot.mul(Quat::from_axis_angle(Vec3::RIGHT, tilt)).normalized();
        let cam_inv = cam_rot.conjugate();
        let cam_pos = quad.pos;

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
                        rgb565(255, 188, 68), // Neon Amber barriers
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
                }
            }
        };

        // Pass 3A: Rasterize all visible structure faces into framebuffer + Z-buffer
        for s in track.structures {
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
            let is_next = cfg.game_mode == GameMode::Freestyle || idx == quad.next_gate;
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

        // 5. Hit flash border
        if quad.hit_flash > 0.0 {
            let red = rgb565(255, 45, 45);
            self.draw_rect(0, 0, W, H, red);
            self.draw_rect(1, 1, W - 2, H - 2, red);
        }

        // 6. Next-gate screen-space beacon / off-screen edge pointer (Race mode)
        if cfg.game_mode == GameMode::Race && !track.gates.is_empty() {
            let target = track.gates[quad.next_gate].center();
            let tc = Self::world_to_cam(target, cam_pos, cam_inv);
            let marker_col = rgb565(255, 230, 40);
            if tc.z > 0.5 {
                let p = self.project_sv(tc);
                let sx = p.x as i32;
                let sy = p.y as i32;
                if sx >= 8 && sx < W - 8 && sy >= 14 && sy < H - 14 {
                    self.draw_line(sx - 4, sy - 9, sx, sy - 5, marker_col);
                    self.draw_line(sx + 4, sy - 9, sx, sy - 5, marker_col);
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
        for y in 0..SCREEN_H {
            let row = y * SCREEN_W;
            let scanline = (y & 1) == 1;
            let y_edge = y < 12 || y >= SCREEN_H - 12;
            for x in 0..SCREEN_W {
                let mut c = self.fb[row + x];
                if scanline {
                    c = ((c >> 1) & 0x7BEF) + ((c >> 2) & 0x39E7) + ((c >> 3) & 0x18E3);
                }
                if y_edge && (x < 18 || x >= SCREEN_W - 18) {
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
        let dir_x = tc.x;
        let dir_y = -tc.y;
        let len = (dir_x * dir_x + dir_y * dir_y).sqrt().max(0.001);
        let nx = dir_x / len;
        let ny = dir_y / len;
        let ax = clampf(HALF_W + nx * (HALF_W - 14.0), 10.0, (W - 10) as f32) as i32;
        let ay = clampf(HALF_H + ny * (HALF_H - 16.0), 16.0, (H - 16) as f32) as i32;
        self.fill_rect(ax - 2, ay - 2, 5, 5, rgb565(0, 0, 0));
        self.fill_rect(ax - 1, ay - 1, 3, 3, color);
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

        let cx = W / 2;
        let cy = H / 2;

        // Post-Flight Betaflight Stats Card when Disarmed after flying or when Battery is Empty!
        if (!quad.armed && (quad.has_flown || quad.battery_empty)) || quad.battery_empty {
            self.draw_post_flight_stats(quad, cfg);
            return;
        }

        // Betaflight crosshair
        self.draw_hline(cx - 5, cx - 2, cy, white);
        self.draw_hline(cx + 2, cx + 5, cy, white);
        self.draw_vline(cx, cy - 3, cy - 2, white);

        // Top-left: Mode (IMU EASY / ACRO / HORIZON / ANGLE / TURTLE) + Gate / Points
        let imu_mode = quad.control_source == FlightControlSource::ImuSimplified;
        let (mode_str, mode_col) = if quad.turtle_mode {
            ("TURTLE", yellow)
        } else if imu_mode {
            ("IMU EASY", cyan)
        } else {
            (cfg.flight_mode.name(), white)
        };
        self.draw_text(4, 3, mode_str, mode_col, 1, true);
        if cfg.game_mode == GameMode::Race {
            let g_str = format!("G {}/{}", quad.next_gate + 1, track.gates.len());
            self.draw_text(4, 13, &g_str, yellow, 1, true);
            let s_str = format!("PTS {}", quad.score);
            self.draw_text(4, 23, &s_str, green, 1, true);
        } else {
            let s_str = format!("PTS {}", quad.score);
            self.draw_text(4, 13, &s_str, yellow, 1, true);
        }

        // Top-center: Betaflight Flight Timer (Freestyle) or Lap Timer (Time Trial)
        if cfg.game_mode == GameMode::Race {
            let secs = quad.lap_time;
            let mins = (secs / 60.0) as u32;
            let rem = secs - (mins as f32) * 60.0;
            let t_str = format!("{mins}:{rem:05.2}");
            self.draw_text(cx - (t_str.len() as i32) * 3, 3, &t_str, white, 1, true);
            let best = cfg.best_laps_ms[cfg.track_index.min(3)];
            if best > 0 {
                let bs = best as f32 / 1000.0;
                let bm = (bs / 60.0) as u32;
                let br = bs - (bm as f32) * 60.0;
                let b_str = format!("B {bm}:{br:05.2}");
                self.draw_text(cx - (b_str.len() as i32) * 3, 13, &b_str, green, 1, true);
            }
        } else {
            let ft = quad.flight_time as u32;
            let mins = ft / 60;
            let secs = ft % 60;
            let t_str = format!("{mins:02}:{secs:02}");
            self.draw_text(cx - 15, 3, &t_str, white, 1, true);
        }

        // Top-right: Betaflight Throttle % + mAh used (+ optional Debug FPS/ms)
        let thr_str = format!("THR {:3}%", quad.throttle_pct);
        self.draw_text(W - 52, 3, &thr_str, white, 1, true);
        if batt_on {
            let mah_str = format!("{:3.0}mAh", quad.mah_used);
            self.draw_text(W - 40, 13, &mah_str, cyan, 1, true);
        }
        if cfg.debug_osd {
            let perf_str = format!("{}F {:.1}ms", fps, frame_ms);
            let pw = (perf_str.len() as i32) * 6;
            self.draw_text(W - pw - 4, 23, &perf_str, rgb565(180, 195, 210), 1, true);
        }

        // Bottom-left: Betaflight Cell Voltage
        let batt_y = if cfg.stick_overlay.is_visible() { H - 41 } else { H - 11 };
        if batt_on {
            let v_col = if quad.cell_voltage < 3.38 {
                red
            } else if quad.cell_voltage < 3.55 {
                yellow
            } else {
                green
            };
            let v_str = format!("{:.2}V", quad.cell_voltage);
            self.draw_text(4, batt_y, &v_str, v_col, 1, true);
        }

        // Bottom-center: Betaflight Craft Name + Speed / Altitude
        let craft = cfg.quad.craft_name();
        self.draw_text(cx - (craft.len() as i32) * 3, H - 20, craft, rgb565(215, 225, 235), 1, true);
        let spd = format!("{:3.0}KPH {:2.0}M", quad.speed_kmh(), (quad.pos.y - 0.22).max(0.0));
        self.draw_text(cx - (spd.len() as i32) * 3, H - 10, &spd, white, 1, true);

        // Center Betaflight Warnings & Score Popups
        if quad.arm_blocked_throttle {
            self.draw_text(cx - 42, cy - 22, "THROTTLE HIGH", red, 1, true);
        } else if quad.arm_aux_connected && !quad.armed {
            self.draw_text(cx - 24, cy - 22, "DISARMED", yellow, 1, true);
            let hint = match cfg.aux_arm {
                AuxBinding::Button { .. } => "PRESS ARM BUTTON",
                _ => "FLIP ARM SWITCH",
            };
            self.draw_text(cx - (hint.len() as i32) * 3, cy - 11, hint, white, 1, true);
        } else if !quad.arm_aux_connected
            && quad.on_ground
            && !quad.has_flown
            && !quad.inverted
            && !quad.battery_empty
        {
            match quad.control_source {
                FlightControlSource::ImuSimplified => {
                    self.draw_text(cx - 15, cy - 22, "READY", green, 1, true);
                    let hint = "F/UP:FLY TILT:STEER SPC:ZERO";
                    self.draw_text(cx - (hint.len() as i32) * 3, cy - 11, hint, white, 1, true);
                }
                FlightControlSource::None => {
                    let title = "CONTROLLER REQUIRED";
                    self.draw_text(cx - (title.len() as i32) * 3, cy - 22, title, red, 1, true);
                    let hint = "CONNECT USB/BT CONTROLLER";
                    self.draw_text(cx - (hint.len() as i32) * 3, cy - 11, hint, white, 1, true);
                }
                FlightControlSource::Controller => {
                    self.draw_text(cx - 15, cy - 22, "READY", green, 1, true);
                }
            }
        } else if quad.turtle_active && quad.turtle_mode {
            self.draw_text(cx - 30, cy - 22, "CRASH FLIP", yellow, 1, true);
            let sub = if quad.inverted {
                "PUSH ROLL / PITCH"
            } else if quad.turtle_aux_connected {
                "RELEASE CRASH FLIP"
            } else {
                "PRESS T TO EXIT"
            };
            self.draw_text(cx - (sub.len() as i32) * 3, cy - 11, sub, if quad.inverted { white } else { green }, 1, true);
        } else if quad.combo_timer > 0.0 && !quad.combo_text.is_empty() {
            let tw = (quad.combo_text.len() as i32) * 6;
            self.draw_text(cx - tw / 2, 32, quad.combo_text, yellow, 1, true);
        } else if quad.on_ground && quad.inverted {
            let prompt = if quad.turtle_aux_connected {
                "TURTLE: PRESS CRASH FLIP"
            } else {
                "TURTLE: ROLL TO FLIP"
            };
            self.draw_text(cx - (prompt.len() as i32) * 3, 32, prompt, red, 1, true);
        } else if batt_on && quad.cell_voltage < 3.35 {
            self.draw_text(cx - 24, 32, "LAND NOW", red, 1, true);
        } else if batt_on && quad.cell_voltage < 3.48 {
            self.draw_text(cx - 33, 32, "LOW BATTERY", yellow, 1, true);
        }

        // Compact live stick / IMU control overlay in bottom corners (Off / Dot / Trails)
        if cfg.stick_overlay.is_visible() {
            if imu_mode {
                let show_trails = cfg.stick_overlay == StickOverlay::Trails;
                if show_trails {
                    self.push_stick_trail(0.0, quad.throttle_pct as f32 / 50.0 - 1.0, sticks.roll, sticks.pitch);
                }
                self.draw_imu_throttle_bar(4, H - 30, 14, 26, quad.throttle_pct, quad.imu_climb_cmd, quad.on_ground);
                self.draw_text(W - 26, H - 39, "IMU", cyan, 1, true);
                self.draw_imu_tilt_box(W - 30, H - 30, 26, sticks.roll, sticks.pitch, show_trails);
            } else {
                let lx = sticks.yaw;
                let ly = sticks.throttle * 2.0 - 1.0;
                let rx = sticks.roll;
                let ry = sticks.pitch;
                if cfg.stick_overlay == StickOverlay::Trails {
                    self.push_stick_trail(lx, ly, rx, ry);
                    self.draw_stick_box_trails(4, H - 30, 26, lx, ly, true);
                    self.draw_stick_box_trails(W - 30, H - 30, 26, rx, ry, false);
                } else {
                    self.draw_stick_box(4, H - 30, 26, lx, ly);
                    self.draw_stick_box(W - 30, H - 30, 26, rx, ry);
                }
            }
        }
    }

    fn draw_post_flight_stats(&mut self, quad: &QuadState, cfg: &Config) {
        let ui = ui_colors(cfg.ui_theme);
        let mx = 46;
        let my = 20;
        let mw = W - 92;
        let mh = H - 40;
        self.fill_rect(mx, my, mw, mh, ui.header_bg);
        self.draw_rect(mx, my, mw, mh, ui.sel_border);

        let title = if quad.battery_empty {
            "BATTERY EMPTY"
        } else {
            "DISARMED - STATS"
        };
        let tcol = if quad.battery_empty { rgb565(255, 80, 80) } else { ui.sel_label };
        self.draw_text(W / 2 - (title.len() as i32) * 3, my + 8, title, tcol, 1, false);
        self.draw_hline(mx + 8, mx + mw - 9, my + 19, ui.header_line);

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
            self.draw_text(mx + 14, y, k, ui.label, 1, false);
            self.draw_text(mx + 124, y, v, ui.sel_val, 1, false);
        }

        let footer = if quad.battery_empty {
            "R: FRESH PACK | ESC: MENU"
        } else if quad.arm_aux_connected {
            "ARM:FLY | R:PACK | ESC:MENU"
        } else {
            "R: FRESH PACK | ESC: MENU"
        };
        self.draw_text(W / 2 - (footer.len() as i32) * 3, my + mh - 14, footer, ui.tag, 1, false);
    }

    fn draw_stick_box(&mut self, x: i32, y: i32, size: i32, sx: f32, sy: f32) {
        let border = rgb565(150, 165, 185);
        let bg = rgb565(18, 22, 30);
        let dot = rgb565(255, 190, 40);
        self.fill_rect(x, y, size, size, bg);
        self.draw_rect(x, y, size, size, border);
        let cx = x + size / 2;
        let cy = y + size / 2;
        self.put_pixel(cx, cy, rgb565(80, 90, 105));
        let half = (size / 2 - 2) as f32;
        let dx = cx + (clampf(sx, -1.0, 1.0) * half) as i32;
        let dy = cy - (clampf(sy, -1.0, 1.0) * half) as i32;
        self.fill_rect(dx - 1, dy - 1, 3, 3, dot);
    }

    /// Zero-allocation fading stick-trail gimbal box (~0.002 ms per frame).
    fn draw_stick_box_trails(&mut self, x: i32, y: i32, size: i32, sx: f32, sy: f32, left_stick: bool) {
        let border = rgb565(150, 165, 185);
        let bg = rgb565(18, 22, 30);
        let dot = rgb565(255, 232, 90);
        let (trail_old, trail_new) = if left_stick {
            (rgb565(28, 78, 108), rgb565(102, 232, 255)) // Deep -> bright cyan trail for throttle/yaw
        } else {
            (rgb565(115, 52, 18), rgb565(255, 165, 50))  // Deep -> bright orange trail for pitch/roll
        };
        self.fill_rect(x, y, size, size, bg);
        self.draw_rect(x, y, size, size, border);
        let cx = x + size / 2;
        let cy = y + size / 2;
        self.put_pixel(cx, cy, rgb565(75, 85, 100));
        let half = (size / 2 - 2) as f32;

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
                } else {
                    self.put_pixel(px, py, col);
                }
                prev = Some((px, py));
            }
        }

        let dx = cx + (clampf(sx, -1.0, 1.0) * half) as i32;
        let dy = cy - (clampf(sy, -1.0, 1.0) * half) as i32;
        self.fill_rect(dx - 1, dy - 1, 3, 3, dot);
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
        self.draw_rect(x, y, w, h, border);

        // Center auto-hover tick reference
        let mid_y = y + h / 2;
        self.draw_hline(x + 2, x + w - 3, mid_y, hover_tick);

        // Live vertical throttle fill
        let inner_h = (h - 4).max(1);
        let frac = (throttle_pct as f32 / 100.0).clamp(0.0, 1.0);
        let fill_h = (frac * inner_h as f32).round() as i32;
        if fill_h > 0 {
            let fy = y + h - 2 - fill_h;
            self.fill_rect(x + 2, fy, (w - 4).max(1), fill_h, fill_col);
            self.draw_hline(x + 2, x + w - 3, fy, rgb565(255, 245, 150));
        }

        // Compact climb / hover / descend indicator right next to gauge
        self.draw_text(x + w + 3, y + h / 2 - 3, cue, cue_col, 1, true);
    }

    /// Simplified 2D IMU tilt reticle box (replaces right gimbal with an attitude/tilt horizon pad).
    fn draw_imu_tilt_box(&mut self, x: i32, y: i32, size: i32, roll: f32, pitch: f32, trails: bool) {
        let border = rgb565(102, 217, 239);
        let bg = rgb565(18, 22, 30);
        let guide = rgb565(46, 62, 78);
        let dz_col = rgb565(72, 108, 128);
        let dot = rgb565(255, 232, 90);
        let trail_old = rgb565(35, 92, 118);
        let trail_new = rgb565(255, 165, 50);

        self.fill_rect(x, y, size, size, bg);
        self.draw_rect(x, y, size, size, border);

        let cx = x + size / 2;
        let cy = y + size / 2;
        // Crosshair axes + small center deadzone box so it visually reads as an IMU tilt pad
        self.draw_hline(x + 2, x + size - 3, cy, guide);
        self.draw_vline(cx, y + 2, y + size - 3, guide);
        self.draw_rect(cx - 2, cy - 2, 5, 5, dz_col);

        let half = (size / 2 - 2) as f32;
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
                } else {
                    self.put_pixel(px, py, col);
                }
                prev = Some((px, py));
            }
        }

        let dx = cx + (clampf(roll, -1.0, 1.0) * half) as i32;
        let dy = cy - (clampf(pitch, -1.0, 1.0) * half) as i32;
        self.fill_rect(dx - 1, dy - 1, 3, 3, dot);
    }

    // -----------------------------------------------------------------------
    // Menu UIs (320 x 170 tailored for CardputerZero 1.9" display)
    // -----------------------------------------------------------------------

    fn draw_header(&mut self, title: &str, right_tag: &str, ui: UiColors) {
        self.clear(ui.bg);
        self.fill_rect(0, 0, W, 22, ui.header_bg);
        self.draw_hline(0, W - 1, 22, ui.header_line);
        self.draw_text(8, 4, title, ui.title, 2, false);
        let rw = (right_tag.len() as i32) * 6;
        self.draw_text(W - rw - 6, 8, right_tag, ui.tag, 1, false);
    }

    pub fn render_main_menu(
        &mut self,
        cfg: &Config,
        selected: usize,
        input: &InputManager,
        hdmi_tag: Option<&str>,
    ) {
        let ui = ui_colors(cfg.ui_theme);
        let right_tag = match hdmi_tag {
            Some(tag) => format!("HDMI {tag}"),
            None => "RUST FPV".to_string(),
        };
        self.draw_header("STICKZERO", &right_tag, ui);

        let track = &TRACKS[cfg.track_index.min(TRACKS.len() - 1)];
        let items: [(&str, String); 6] = [
            ("FLY NOW", format!("{} [{}]", track.name, cfg.game_mode.name())),
            ("TRACK", format!("{} ({})", track.name, track.subtitle)),
            (
                "DRONE",
                format!("{} | {}g {}", cfg.quad.craft_name(), cfg.weight_g, cfg.battery_pack.short_name()),
            ),
            ("THEME", format!("{} / {} deg", cfg.theme.name(), cfg.camera_tilt_deg)),
            (
                "SETTINGS",
                format!(
                    "{} | Rates, Gfx, OSD",
                    if input.control_source() == FlightControlSource::ImuSimplified {
                        "IMU EASY"
                    } else {
                        cfg.flight_mode.name()
                    }
                ),
            ),
            ("CONTROLLER", input.controller_name().chars().take(24).collect()),
        ];

        for (i, (label, val)) in items.iter().enumerate() {
            let y = 28 + (i as i32) * 20;
            let is_sel = i == selected;
            if is_sel {
                self.fill_rect(6, y - 2, W - 12, 17, ui.sel_bg);
                self.draw_rect(6, y - 2, W - 12, 17, ui.sel_border);
            }
            let lcol = if is_sel { ui.sel_label } else { ui.label };
            let vcol = if is_sel { ui.sel_val } else { ui.val };
            self.draw_text(12, y + 3, label, lcol, 1, false);
            self.draw_text(86, y + 3, val, vcol, 1, false);
        }

        let (dot_col, hint) = match input.control_source() {
            FlightControlSource::Controller => (ui.ok, "BT/USB Ready | Ent:Select | Hold Esc:Exit"),
            FlightControlSource::ImuSimplified => (ui.tag, "IMU Tilt Ready | Ent:Select | Hold Esc:Exit"),
            FlightControlSource::None => (ui.warn, "Controller Req! | Ent:Select | Hold Esc:Exit"),
        };
        self.fill_rect(8, H - 14, 6, 6, dot_col);
        self.draw_text(18, H - 14, hint, ui.muted, 1, false);
    }

    /// Launch warning modal shown when no IMU sensor is detected and/or running on CardputerZero Lite
    /// without a connected USB-C or Bluetooth controller.
    pub fn render_controller_warning(
        &mut self,
        cfg: &Config,
        input: &InputManager,
        hdmi_tag: Option<&str>,
    ) {
        self.render_main_menu(cfg, 0, input, hdmi_tag);
        let ui = ui_colors(cfg.ui_theme);

        let mx = 18;
        let my = 24;
        let mw = W - 36;
        let mh = H - 42;
        let cx = W / 2;
        let warn_border = rgb565(255, 110, 45);
        let banner_bg = rgb565(75, 24, 20);
        let banner_txt = rgb565(255, 215, 70);

        self.fill_rect(mx, my, mw, mh, ui.header_bg);
        self.draw_rect(mx, my, mw, mh, warn_border);
        self.draw_rect(mx + 1, my + 1, mw - 2, mh - 2, warn_border);

        self.fill_rect(mx + 2, my + 2, mw - 4, 19, banner_bg);
        self.draw_hline(mx + 2, mx + mw - 3, my + 21, warn_border);
        let title = "! CONTROLLER REQUIRED !";
        self.draw_text(cx - (title.len() as i32) * 3, my + 8, title, banner_txt, 1, false);

        let reason = if input.is_cardputer_lite() {
            "CardputerZero Lite detected (no IMU)"
        } else {
            "No IMU sensor detected"
        };
        self.draw_text(cx - (reason.len() as i32) * 3, my + 29, reason, ui.title, 1, false);

        let l1 = "A USB-C or Bluetooth controller";
        let l2 = "is required to fly StickZero.";
        self.draw_text(cx - (l1.len() as i32) * 3, my + 45, l1, ui.label, 1, false);
        self.draw_text(cx - (l2.len() as i32) * 3, my + 57, l2, ui.label, 1, false);

        let l3 = "Pair a BT gamepad / ELRS radio";
        let l4 = "or connect via USB-C Host mode.";
        self.draw_text(cx - (l3.len() as i32) * 3, my + 74, l3, ui.tag, 1, false);
        self.draw_text(cx - (l4.len() as i32) * 3, my + 86, l4, ui.tag, 1, false);

        self.draw_hline(mx + 10, mx + mw - 11, my + 103, ui.muted);
        let footer = "Enter/Esc: Continue | Hold Esc: Exit";
        self.draw_text(cx - (footer.len() as i32) * 3, my + 111, footer, ui.sel_label, 1, false);
    }

    pub fn render_drone_tuning_menu(&mut self, cfg: &Config, selected: usize) {
        let ui = ui_colors(cfg.ui_theme);
        let rows: [(&str, String); 11] = [
            ("Quad Preset", cfg.quad.name().to_string()),
            ("Battery Pack", cfg.battery_pack.name().to_string()),
            ("Prop Wash", cfg.prop_wash.name().to_string()),
            ("Weight (g)", format!("{}", cfg.weight_g)),
            ("Thrust (g)", format!("{} ({:.1}:1)", cfg.thrust_g, cfg.effective_twr())),
            ("Air Drag", format!("{:.2}", cfg.air_drag as f32 / 100.0)),
            ("Rotor Grip", format!("{:.2}", cfg.rotor_grip as f32 / 100.0)),
            ("Motor Lag (ms)", format!("{}", cfg.motor_lag_ms)),
            ("Thr Mid", format!("{:.2}", cfg.thr_mid as f32 / 100.0)),
            ("Thr Expo", format!("{:.2}", cfg.thr_expo as f32 / 100.0)),
            ("Reset Tuning", "Enter -> Stock".to_string()),
        ];

        let tag = format!("{}/{}", selected + 1, rows.len());
        self.draw_header("DRONE TUNING", &tag, ui);

        const VISIBLE: usize = 8;
        let start = if selected >= VISIBLE {
            (selected + 1 - VISIBLE).min(rows.len() - VISIBLE)
        } else {
            0
        };

        for (slot, i) in (start..(start + VISIBLE).min(rows.len())).enumerate() {
            let (k, ref v) = rows[i];
            let y = 27 + (slot as i32) * 16;
            let is_sel = i == selected;
            if is_sel {
                self.fill_rect(6, y - 2, W - 12, 14, ui.sel_bg);
                self.draw_rect(6, y - 2, W - 12, 14, ui.sel_border);
            }
            let kcol = if is_sel { ui.sel_label } else { ui.label };
            let vcol = if is_sel { ui.sel_val } else { ui.val };
            self.draw_text(12, y + 1, k, kcol, 1, false);
            self.draw_text(128, y + 1, v, vcol, 1, false);
        }

        self.draw_text(8, H - 12, "F/X:Scroll | Z/C:Tune | Esc:Save & Back", ui.muted, 1, false);
    }

    pub fn render_pause_menu(&mut self, cfg: &Config, selected: usize) {
        let ui = ui_colors(cfg.ui_theme);
        let mx = 68;
        let my = 24;
        let mw = 184;
        let mh = 120;
        self.fill_rect(mx, my, mw, mh, ui.header_bg);
        self.draw_rect(mx, my, mw, mh, ui.sel_border);
        self.draw_text(mx + 56, my + 8, "PAUSED", ui.title, 2, false);

        let items = ["RESUME FLIGHT", "RESPAWN START", "DRONE TUNING", "SETTINGS", "MAIN MENU"];
        for (i, label) in items.iter().enumerate() {
            let y = my + 32 + (i as i32) * 16;
            if i == selected {
                self.fill_rect(mx + 10, y - 2, mw - 20, 13, ui.sel_bg);
                self.draw_rect(mx + 10, y - 2, mw - 20, 13, ui.sel_border);
            }
            let col = if i == selected { ui.sel_label } else { ui.label };
            self.draw_text(mx + 22, y + 1, label, col, 1, false);
        }
    }

    pub fn render_settings_menu(
        &mut self,
        cfg: &Config,
        selected: usize,
        imu_mode: bool,
        hdmi_tag: Option<&str>,
    ) {
        let ui = ui_colors(cfg.ui_theme);
        let flight_mode_val = if imu_mode {
            "IMU EASY (Locked)".to_string()
        } else {
            cfg.flight_mode.name().to_string()
        };
        let display_val = match (cfg.display_output, hdmi_tag) {
            (DisplayOutput::Auto, Some(tag)) => format!("Auto [{tag}]"),
            (DisplayOutput::Auto, None) => "Auto [LCD]".to_string(),
            (DisplayOutput::HdmiOnly, Some(tag)) => format!("HDMI [{tag}]"),
            (DisplayOutput::HdmiOnly, None) => "HDMI Only [No HDMI]".to_string(),
            (DisplayOutput::LcdOnly, _) => "LCD Only".to_string(),
        };
        let rows: [(&str, String); 18] = [
            ("Game Mode", cfg.game_mode.name().to_string()),
            ("Flight Mode", flight_mode_val),
            ("Rate Preset", cfg.rates.name().to_string()),
            ("Center Sens", format!("{} deg/s", cfg.rate_center)),
            ("Max Rate", format!("{} deg/s", cfg.rate_max)),
            ("Rate Expo", format!("{:.2}", cfg.rate_expo as f32 / 100.0)),
            ("Camera Tilt", format!("{} deg", cfg.camera_tilt_deg)),
            ("FOV", format!("{} deg", cfg.fov_deg)),
            ("Analog FX", if cfg.analog_filter { "On (NTSC/RF)" } else { "Off" }.to_string()),
            ("UI Theme", cfg.ui_theme.name().to_string()),
            ("Stick OSD", cfg.stick_overlay.name().to_string()),
            ("Sound", if cfg.sound_on { "On (Whoop Audio)" } else { "Off" }.to_string()),
            ("Debug OSD", if cfg.debug_osd { "On (FPS/ms)" } else { "Off" }.to_string()),
            ("Gfx Detail", if cfg.detail_high { "High" } else { "Low" }.to_string()),
            ("Sky Detail", if cfg.sky_detail { "On (Sun/Stars)" } else { "Off" }.to_string()),
            ("Stick Mode", cfg.stick_mode.name().to_string()),
            ("Throttle Mode", cfg.throttle_mode.name().to_string()),
            ("Display", display_val),
        ];

        let tag = format!("{}/{}", selected + 1, rows.len());
        self.draw_header("SETTINGS", &tag, ui);

        const VISIBLE: usize = 8;
        let start = if selected >= VISIBLE {
            (selected + 1 - VISIBLE).min(rows.len() - VISIBLE)
        } else {
            0
        };

        for (slot, i) in (start..(start + VISIBLE).min(rows.len())).enumerate() {
            let (k, ref v) = rows[i];
            let y = 27 + (slot as i32) * 16;
            let is_sel = i == selected;
            let is_locked = imu_mode && i == 1;
            if is_sel {
                self.fill_rect(6, y - 2, W - 12, 14, ui.sel_bg);
                self.draw_rect(6, y - 2, W - 12, 14, ui.sel_border);
            }
            let kcol = if is_locked {
                ui.muted
            } else if is_sel {
                ui.sel_label
            } else {
                ui.label
            };
            let vcol = if is_locked {
                ui.muted
            } else if is_sel {
                ui.sel_val
            } else {
                ui.val
            };
            self.draw_text(12, y + 1, k, kcol, 1, false);
            self.draw_text(126, y + 1, v, vcol, 1, false);
        }

        self.draw_text(8, H - 12, "F/X:Scroll | Z/C:Adjust | Esc:Save", ui.muted, 1, false);
    }

    /// Static status card drawn once on the built-in 1.9" LCD when `HDMI Only` mode is active.
    pub fn render_hdmi_lcd_notice(&mut self, cfg: &Config, hdmi_info: &str) {
        let ui = ui_colors(cfg.ui_theme);
        self.draw_header("STICKZERO", "HDMI ONLY", ui);
        let cx = W / 2;

        self.fill_rect(18, 34, W - 36, 98, ui.header_bg);
        self.draw_rect(18, 34, W - 36, 98, ui.sel_border);

        let title = "HDMI OUTPUT ACTIVE";
        self.draw_text(cx - (title.len() as i32) * 6, 46, title, ui.sel_label, 2, false);

        self.draw_text(
            cx - (hdmi_info.len() as i32) * 3,
            74,
            hdmi_info,
            ui.tag,
            1,
            false,
        );

        let l1 = "LCD paused to save battery & SPI";
        let l2 = "Unplug HDMI to return to LCD";
        self.draw_text(cx - (l1.len() as i32) * 3, 96, l1, ui.label, 1, false);
        self.draw_text(cx - (l2.len() as i32) * 3, 110, l2, ui.muted, 1, false);

        let footer = "Settings -> Display: Auto / LCD";
        self.draw_text(cx - (footer.len() as i32) * 3, H - 14, footer, ui.muted, 1, false);
    }

    pub fn render_controller_screen(&mut self, cfg: &Config, input: &InputManager) {
        let ui = ui_colors(cfg.ui_theme);
        let ctrl_src = input.control_source();
        let header_tag = match ctrl_src {
            FlightControlSource::Controller => "BT / USB-C",
            FlightControlSource::ImuSimplified => "IMU EASY",
            FlightControlSource::None => "NO INPUT",
        };
        self.draw_header("CONTROLLER", header_tag, ui);

        let name: String = input.controller_name().chars().take(36).collect();
        let dev_tag = match ctrl_src {
            FlightControlSource::Controller if input.candidate_count > 1 => {
                format!("{name} [{} devs]", input.candidate_count)
            }
            FlightControlSource::ImuSimplified => "CardputerZero IMU (Simplified)".to_string(),
            FlightControlSource::None if input.is_cardputer_lite() => {
                "CardputerZero Lite (No IMU)".to_string()
            }
            FlightControlSource::None => "No Controller / No IMU".to_string(),
            _ => name,
        };
        let sub_summary = match ctrl_src {
            FlightControlSource::Controller => format!(
                "Thr:{} | ARM:{} MOD:{} TRT:{}",
                cfg.throttle_mode.name(),
                cfg.aux_arm.short_label(),
                cfg.aux_mode.short_label(),
                cfg.aux_turtle.short_label(),
            ),
            FlightControlSource::ImuSimplified => "Tilt:Steer | F/X:Alt | Z/C:Yaw".to_string(),
            FlightControlSource::None => "F/X/Z/C:L-Stk | K/B/N/M:R-Stk".to_string(),
        };

        let dev_col = if ctrl_src == FlightControlSource::None { ui.warn } else { ui.sel_label };
        self.draw_text(8, 25, &dev_tag, dev_col, 1, false);
        self.draw_text(8, 35, &sub_summary, ui.tag, 1, false);

        if let Some(ref wiz) = input.calib {
            self.fill_rect(10, 48, W - 20, 74, ui.header_bg);
            self.draw_rect(10, 48, W - 20, 74, ui.accent);
            self.draw_text(18, 56, "CALIBRATE STICKS", ui.tag, 1, false);
            self.draw_text(18, 72, wiz.step_prompt(), ui.sel_label, 1, false);
            let msg: String = wiz.message.chars().take(44).collect();
            self.draw_text(18, 90, &msg, ui.ok, 1, false);
            self.draw_text(18, 106, "Esc: Cancel Calibration", ui.muted, 1, false);
        } else if let Some(ref aw) = input.aux_wiz {
            self.fill_rect(10, 48, W - 20, 74, ui.header_bg);
            self.draw_rect(10, 48, W - 20, 74, ui.ok);
            self.draw_text(18, 56, "BIND RADIO AUX SWITCHES", ui.ok, 1, false);
            self.draw_text(18, 72, aw.step_prompt(), ui.sel_label, 1, false);
            let msg: String = aw.message.chars().take(44).collect();
            self.draw_text(18, 90, &msg, ui.tag, 1, false);
            self.draw_text(18, 106, "Enter: Skip Step | Esc: Cancel", ui.muted, 1, false);
        } else {
            let sticks = input.flight_sticks(cfg);
            let bars: [(&str, f32); 4] = [
                ("THR", sticks.throttle * 2.0 - 1.0),
                ("YAW", sticks.yaw),
                ("PIT", sticks.pitch),
                ("ROL", sticks.roll),
            ];

            for (i, (lbl, val)) in bars.iter().enumerate() {
                let y = 49 + (i as i32) * 16;
                self.draw_text(8, y + 2, lbl, ui.label, 1, false);
                let bx = 38;
                let bw = 152;
                self.fill_rect(bx, y, bw, 10, ui.header_bg);
                self.draw_rect(bx, y, bw, 10, ui.muted);
                let mid = bx + bw / 2;
                self.draw_vline(mid, y + 1, y + 8, ui.muted);
                let pos = bx + ((clampf(*val, -1.0, 1.0) + 1.0) * 0.5 * (bw - 4) as f32) as i32;
                self.fill_rect(pos, y + 2, 4, 6, ui.ok);
            }

            if ctrl_src == FlightControlSource::ImuSimplified {
                let thr_pct = (sticks.throttle * 100.0).round() as u8;
                self.draw_imu_throttle_bar(204, 51, 18, 46, thr_pct, input.imu_climb_cmd(), false);
                self.draw_imu_tilt_box(260, 51, 46, sticks.roll, sticks.pitch, false);
                self.draw_text(204, 101, "ALT", ui.muted, 1, false);
                self.draw_text(274, 101, "IMU", ui.tag, 1, false);
            } else {
                self.draw_stick_box(204, 51, 46, sticks.yaw, sticks.throttle * 2.0 - 1.0);
                self.draw_stick_box(260, 51, 46, sticks.roll, sticks.pitch);
                self.draw_text(214, 101, "L-STK", ui.muted, 1, false);
                self.draw_text(270, 101, "R-STK", ui.muted, 1, false);
            }
        }

        let raw = match ctrl_src {
            FlightControlSource::Controller => format!("ABS {}", input.raw_axes_summary()),
            FlightControlSource::ImuSimplified => format!("IMU {}", input.imu.summary()),
            FlightControlSource::None => input.imu.summary(),
        };
        let raw_trim: String = raw.chars().take(50).collect();
        self.draw_text(8, 127, &raw_trim, if ctrl_src == FlightControlSource::None { ui.warn } else { ui.ok }, 1, false);

        match ctrl_src {
            FlightControlSource::Controller => {
                self.draw_text(8, 141, "C:Sticks | B:Bind AUX | Ent:RC/Pad", ui.sel_val, 1, false);
                self.draw_text(8, 154, "N:NextDev | R:Reset Auto | Esc:Back", ui.muted, 1, false);
            }
            FlightControlSource::ImuSimplified => {
                self.draw_text(8, 141, "C:Zero IMU | F/X:Alt | R:Rescan BT/USB", ui.sel_val, 1, false);
                self.draw_text(8, 154, "Connect USB/BT radio for full sticks", ui.muted, 1, false);
            }
            FlightControlSource::None => {
                self.draw_text(8, 141, "Connect USB-C Host or BT Controller", ui.sel_val, 1, false);
                self.draw_text(8, 154, "R:Rescan | N:NextDev | Esc:Back", ui.muted, 1, false);
            }
        }
    }
}
