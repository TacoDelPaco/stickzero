//! Ultra-fast zero-allocation 320x170 RGB565 3D software rasterizer with a
//! per-pixel 16-bit perspective-correct Z-buffer, solid 3D racing gates,
//! authentic Betaflight OSD + post-flight statistics, and CardputerZero 1.9" menus.

use crate::config::{AuxBinding, BatteryPack, Config, GameMode, VisualTheme};
use crate::display::{SCREEN_H, SCREEN_W};
use crate::font::{glyph, GLYPH_H, GLYPH_W};
use crate::input::{ControllerKind, InputManager};
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
        // High-contrast Night: deep indigo sky + twilight horizon glow vs dark teal-olive ground!
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
        // High-contrast Wireframe: jet-black sky vs deep emerald ground + bright neon horizon!
        VisualTheme::Wireframe => ThemePalette {
            sky_top: rgb565(0, 0, 0),
            sky_horizon: rgb565(4, 12, 24),
            ground_near: rgb565(8, 38, 20),
            ground_far: rgb565(12, 54, 28),
            horizon_line: Some(rgb565(95, 255, 185)),
            grid_line: rgb565(64, 240, 124),
            active_gate: rgb565(80, 255, 130),
            active_gate_hi: rgb565(225, 255, 125),
            idle_gate: rgb565(42, 155, 96),
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
}

impl Renderer {
    pub fn new() -> Self {
        Self {
            fb: Box::new([0u16; SCREEN_W * SCREEN_H]),
            zbuf: Box::new([0u16; SCREEN_W * SCREEN_H]),
            focal: 160.0,
            rng_state: 0x51A7_1C09,
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

    /// Z-buffered Bresenham line with linear `1/z` interpolation.
    fn draw_line_z(
        &mut self,
        mut x0: i32,
        mut y0: i32,
        iz0: f32,
        x1: i32,
        y1: i32,
        iz1: f32,
        color: u16,
        z_bias: f32,
    ) {
        let dx = (x1 - x0).abs();
        let dy_abs = (y1 - y0).abs();
        let steps = dx.max(dy_abs);
        let sx = if x0 < x1 { 1 } else { -1 };
        let dy = -dy_abs;
        let sy = if y0 < y1 { 1 } else { -1 };
        let mut err = dx + dy;
        let diz = if steps > 0 { (iz1 - iz0) / (steps as f32) } else { 0.0 };
        let mut iz = iz0 + z_bias;

        for _ in 0..=steps {
            if x0 >= 0 && x0 < W && y0 >= 0 && y0 < H {
                let idx = (y0 as usize) * SCREEN_W + (x0 as usize);
                let zu = (iz as i32).clamp(1, 65535) as u16;
                if zu >= self.zbuf[idx] {
                    self.zbuf[idx] = zu;
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

    /// Draw a Z-buffered 3D line (optional 2px thickness for wireframe gates/edges).
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
            self.draw_line_z(x0, y0, iz0, x1, y1, iz1, color, 28.0);
            if thick {
                self.draw_line_z(x0 + 1, y0, iz0, x1 + 1, y1, iz1, color, 28.0);
                self.draw_line_z(x0, y0 + 1, iz0, x1, y1 + 1, iz1, color, 28.0);
            }
        }
    }

    /// Clip and rasterize a 3D quad directly into the framebuffer + 16-bit Z-buffer.
    fn draw_3d_quad_z(&mut self, verts: [Vec3; 4], cam_pos: Vec3, cam_inv: Quat, fill: u16, edge: u16, wireframe: bool) {
        if wireframe {
            for i in 0..4 {
                self.draw_3d_line_z(verts[i], verts[(i + 1) & 3], cam_pos, cam_inv, edge, false);
            }
            return;
        }
        let mut c = [Vec3::ZERO; 4];
        for i in 0..4 {
            c[i] = Self::world_to_cam(verts[i], cam_pos, cam_inv);
        }

        // Sutherland-Hodgman clip against z >= NEAR_Z (yields 0..=5 vertices)
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
            return;
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
            return;
        }

        for k in 1..(count - 1) {
            self.fill_tri_z(sv[0], sv[k], sv[k + 1], fill);
        }
        for k in 0..count {
            let n = (k + 1) % count;
            if let Some((x0, y0, iz0, x1, y1, iz1)) =
                Self::clip_line_2d_z(sv[k].x, sv[k].y, sv[k].iz, sv[n].x, sv[n].y, sv[n].iz)
            {
                self.draw_line_z(x0, y0, iz0, x1, y1, iz1, edge, 24.0);
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
        if wireframe {
            self.draw_3d_line_z(a, b, cam_pos, cam_inv, edge, true);
            return;
        }
        let u = axis_u.scale(half_w);
        let v = axis_v.scale(half_w);
        self.draw_3d_quad_z([a.sub(u), a.add(u), b.add(u), b.sub(u)], cam_pos, cam_inv, fill, edge, false);
        self.draw_3d_quad_z([a.sub(v), a.add(v), b.add(v), b.sub(v)], cam_pos, cam_inv, fill, edge, false);
    }

    /// Ultra-fast zero-per-pixel-float horizon renderer using scanline `slice::fill()`,
    /// plus crisp horizon glow line in Night / Wireframe / Evening themes for instant flip orientation!
    fn draw_sky_and_ground(&mut self, cam_rot: Quat, pal: ThemePalette, detail_high: bool) {
        let (cam_right, cam_up, cam_fwd) = cam_rot.basis();
        let ux = cam_right.y;
        let uy = cam_up.y;
        let uz = cam_fwd.y * self.focal;
        let inv_f = 1.0 / self.focal;

        let sky_mid = if detail_high {
            lerp_rgb565(pal.sky_horizon, pal.sky_top, 0.45)
        } else {
            pal.sky_top
        };
        let gnd_mid = if detail_high {
            lerp_rgb565(pal.ground_far, pal.ground_near, 0.45)
        } else {
            pal.ground_near
        };

        if ux.abs() < 0.002 {
            let mut prev_sky: Option<bool> = None;
            for y in 0..SCREEN_H {
                let ry = HALF_H - (y as f32);
                let elev = (uy * ry + uz) * inv_f;
                let is_sky = elev >= 0.0;
                let color = if is_sky {
                    if detail_high {
                        lerp_rgb565(pal.sky_horizon, pal.sky_top, (elev * 1.8).min(1.0))
                    } else {
                        pal.sky_top
                    }
                } else if detail_high {
                    lerp_rgb565(pal.ground_far, pal.ground_near, (-elev * 2.2).min(1.0))
                } else {
                    pal.ground_near
                };
                let row = y * SCREEN_W;
                self.fb[row..row + SCREEN_W].fill(color);
                if let (Some(was_sky), Some(hcol)) = (prev_sky, pal.horizon_line) {
                    if was_sky != is_sky {
                        self.fb[row..row + SCREEN_W].fill(hcol);
                    }
                }
                prev_sky = Some(is_sky);
            }
            return;
        }

        let inv_ux = 1.0 / ux;
        let band_dx = (0.14 * self.focal * inv_ux).abs();
        let sky_on_right = ux > 0.0;

        for y in 0..SCREEN_H {
            let ry = HALF_H - (y as f32);
            let base = uy * ry + uz;
            let row = y * SCREEN_W;

            let x_hor = HALF_W - base * inv_ux;
            let x_split = (x_hor.round() as i32).clamp(0, W) as usize;

            if sky_on_right {
                if x_split > 0 {
                    self.fb[row..row + x_split].fill(pal.ground_near);
                    if detail_high {
                        let g_start = ((x_hor - band_dx).round() as i32).clamp(0, x_split as i32) as usize;
                        if g_start < x_split {
                            self.fb[row + g_start..row + x_split].fill(gnd_mid);
                        }
                    }
                }
                if x_split < SCREEN_W {
                    self.fb[row + x_split..row + SCREEN_W].fill(pal.sky_top);
                    if detail_high {
                        let s_end = ((x_hor + band_dx).round() as i32).clamp(x_split as i32, W) as usize;
                        if s_end > x_split {
                            self.fb[row + x_split..row + s_end].fill(sky_mid);
                        }
                    }
                }
            } else {
                if x_split > 0 {
                    self.fb[row..row + x_split].fill(pal.sky_top);
                    if detail_high {
                        let s_start = ((x_hor - band_dx).round() as i32).clamp(0, x_split as i32) as usize;
                        if s_start < x_split {
                            self.fb[row + s_start..row + x_split].fill(sky_mid);
                        }
                    }
                }
                if x_split < SCREEN_W {
                    self.fb[row + x_split..row + SCREEN_W].fill(pal.ground_near);
                    if detail_high {
                        let g_end = ((x_hor + band_dx).round() as i32).clamp(x_split as i32, W) as usize;
                        if g_end > x_split {
                            self.fb[row + x_split..row + g_end].fill(gnd_mid);
                        }
                    }
                }
            }

            // Crisp horizon boundary line only where the horizon actually crosses this scanline
            if let Some(hcol) = pal.horizon_line {
                let x_hor_i = x_hor.round() as i32;
                if x_hor_i >= 0 && x_hor_i < W {
                    let hx = x_hor_i as usize;
                    self.fb[row + hx] = hcol;
                    if hx + 1 < SCREEN_W {
                        self.fb[row + hx + 1] = hcol;
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

        // 3. Structures (solid shaded 3D boxes rendered directly with per-pixel Z-buffer!)
        for s in track.structures {
            let (c_top, c_side_a, c_side_b, c_edge) = match s.color {
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
            };
            let (x0, y0, z0) = (s.min.x, s.min.y, s.min.z);
            let (x1, y1, z1) = (s.max.x, s.max.y, s.max.z);
            let wf = pal.wireframe;
            if cam_pos.y > y1 {
                self.draw_3d_quad_z(
                    [Vec3::new(x0, y1, z0), Vec3::new(x1, y1, z0), Vec3::new(x1, y1, z1), Vec3::new(x0, y1, z1)],
                    cam_pos, cam_inv, c_top, c_edge, wf,
                );
            }
            if cam_pos.y < y0 {
                self.draw_3d_quad_z(
                    [Vec3::new(x0, y0, z0), Vec3::new(x1, y0, z0), Vec3::new(x1, y0, z1), Vec3::new(x0, y0, z1)],
                    cam_pos, cam_inv, c_side_b, c_edge, wf,
                );
            }
            if cam_pos.z < z0 {
                self.draw_3d_quad_z(
                    [Vec3::new(x0, y0, z0), Vec3::new(x1, y0, z0), Vec3::new(x1, y1, z0), Vec3::new(x0, y1, z0)],
                    cam_pos, cam_inv, c_side_a, c_edge, wf,
                );
            }
            if cam_pos.z > z1 {
                self.draw_3d_quad_z(
                    [Vec3::new(x0, y0, z1), Vec3::new(x1, y0, z1), Vec3::new(x1, y1, z1), Vec3::new(x0, y1, z1)],
                    cam_pos, cam_inv, c_side_a, c_edge, wf,
                );
            }
            if cam_pos.x < x0 {
                self.draw_3d_quad_z(
                    [Vec3::new(x0, y0, z0), Vec3::new(x0, y0, z1), Vec3::new(x0, y1, z1), Vec3::new(x0, y1, z0)],
                    cam_pos, cam_inv, c_side_b, c_edge, wf,
                );
            }
            if cam_pos.x > x1 {
                self.draw_3d_quad_z(
                    [Vec3::new(x1, y0, z0), Vec3::new(x1, y0, z1), Vec3::new(x1, y1, z1), Vec3::new(x1, y1, z0)],
                    cam_pos, cam_inv, c_side_b, c_edge, wf,
                );
            }
        }

        // 4. Solid 3D Racing Gates (Z-buffered volume posts & beams so they never disappear!)
        for (idx, g) in track.gates.iter().enumerate() {
            let is_next = cfg.game_mode == GameMode::Freestyle || idx == quad.next_gate;
            let main_col = if is_next { pal.active_gate } else { pal.idle_gate };
            let hi_col = if is_next { pal.active_gate_hi } else { rgb565(225, 230, 235) };
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

        // Top-left: Mode (ACRO / ANGLE / TURTLE) + Gate / Points
        let mode_str = if quad.turtle_mode {
            "TURTLE"
        } else {
            cfg.flight_mode.name()
        };
        self.draw_text(4, 3, mode_str, if quad.turtle_mode { yellow } else { white }, 1, true);
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

        // Bottom-left: Betaflight Cell Voltage + ELRS LQ
        let batt_y = if cfg.show_sticks { H - 41 } else { H - 11 };
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
        let lq_y = if cfg.show_sticks { H - 41 } else { H - 11 };
        self.draw_text(W - 44, lq_y, "1:100%", white, 1, true);

        // Bottom-center: Betaflight Craft Name + Speed / Altitude
        let craft = cfg.quad.craft_name();
        self.draw_text(cx - (craft.len() as i32) * 3, H - 20, craft, rgb565(215, 225, 235), 1, true);
        let spd = format!("{:3.0}KPH {:2.0}M", quad.speed_kmh(), (quad.pos.y - 0.22).max(0.0));
        self.draw_text(cx - (spd.len() as i32) * 3, H - 10, &spd, white, 1, true);

        // Center Betaflight Warnings & Score Popups
        if quad.arm_blocked_throttle {
            self.draw_text(cx - 42, cy - 22, "THROTTLE HIGH", red, 1, true);
        } else if !quad.armed {
            self.draw_text(cx - 24, cy - 22, "DISARMED", yellow, 1, true);
            let hint = if cfg.aux_arm != AuxBinding::None {
                "FLIP ARM SWITCH"
            } else {
                "PRESS TAB TO ARM"
            };
            self.draw_text(cx - (hint.len() as i32) * 3, cy - 11, hint, white, 1, true);
        } else if quad.turtle_mode && quad.on_ground {
            self.draw_text(cx - 30, cy - 22, "CRASH FLIP", yellow, 1, true);
        } else if quad.combo_timer > 0.0 && !quad.combo_text.is_empty() {
            let tw = (quad.combo_text.len() as i32) * 6;
            self.draw_text(cx - tw / 2, 32, quad.combo_text, yellow, 1, true);
        } else if quad.on_ground && quad.inverted {
            self.draw_text(cx - 54, 32, "TURTLE: PUSH ROLL", red, 1, true);
        } else if batt_on && quad.cell_voltage < 3.35 {
            self.draw_text(cx - 24, 32, "LAND NOW", red, 1, true);
        } else if batt_on && quad.cell_voltage < 3.48 {
            self.draw_text(cx - 33, 32, "LOW BATTERY", yellow, 1, true);
        }

        // Optional compact live gimbal stick overlay in bottom corners
        if cfg.show_sticks {
            self.draw_stick_box(4, H - 30, 26, sticks.yaw, sticks.throttle * 2.0 - 1.0);
            self.draw_stick_box(W - 30, H - 30, 26, sticks.roll, sticks.pitch);
        }
    }

    fn draw_post_flight_stats(&mut self, quad: &QuadState, cfg: &Config) {
        let mx = 46;
        let my = 20;
        let mw = W - 92;
        let mh = H - 40;
        self.fill_rect(mx, my, mw, mh, rgb565(10, 14, 24));
        self.draw_rect(mx, my, mw, mh, rgb565(90, 190, 255));

        let title = if quad.battery_empty {
            "BATTERY EMPTY"
        } else {
            "DISARMED - STATS"
        };
        let tcol = if quad.battery_empty { rgb565(255, 80, 80) } else { rgb565(255, 230, 75) };
        self.draw_text(W / 2 - (title.len() as i32) * 3, my + 8, title, tcol, 1, false);
        self.draw_hline(mx + 8, mx + mw - 9, my + 19, rgb565(55, 95, 145));

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
            self.draw_text(mx + 14, y, k, rgb565(175, 195, 220), 1, false);
            self.draw_text(mx + 124, y, v, rgb565(255, 255, 255), 1, false);
        }

        let footer = if quad.battery_empty {
            "R: FRESH PACK | ESC: MENU"
        } else if cfg.aux_arm != AuxBinding::None {
            "ARM:FLY | R:PACK | ESC:MENU"
        } else {
            "TAB:ARM | R:PACK | ESC:MENU"
        };
        self.draw_text(W / 2 - (footer.len() as i32) * 3, my + mh - 14, footer, rgb565(115, 225, 255), 1, false);
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

    // -----------------------------------------------------------------------
    // Menu UIs (320 x 170 tailored for CardputerZero 1.9" display)
    // -----------------------------------------------------------------------

    fn draw_header(&mut self, title: &str, right_tag: &str) {
        self.clear(rgb565(12, 16, 26));
        self.fill_rect(0, 0, W, 22, rgb565(22, 32, 52));
        self.draw_hline(0, W - 1, 22, rgb565(65, 140, 240));
        self.draw_text(8, 4, title, rgb565(255, 255, 255), 2, false);
        let rw = (right_tag.len() as i32) * 6;
        self.draw_text(W - rw - 6, 8, right_tag, rgb565(120, 220, 255), 1, false);
    }

    pub fn render_main_menu(&mut self, cfg: &Config, selected: usize, input: &InputManager) {
        self.draw_header("STICKZERO", "RUST FPV");

        let track = &TRACKS[cfg.track_index.min(TRACKS.len() - 1)];
        let items: [(&str, String); 6] = [
            ("FLY NOW", format!("{} [{}]", track.name, cfg.game_mode.name())),
            ("TRACK", format!("{} ({})", track.name, track.subtitle)),
            ("QUAD", format!("{} / {}", cfg.quad.name(), cfg.flight_mode.name())),
            ("THEME", format!("{} / {} deg", cfg.theme.name(), cfg.camera_tilt_deg)),
            ("SETTINGS", "Rates, Batt, Gfx, OSD".to_string()),
            ("CONTROLLER", input.controller_name().chars().take(24).collect()),
        ];

        for (i, (label, val)) in items.iter().enumerate() {
            let y = 28 + (i as i32) * 20;
            let is_sel = i == selected;
            if is_sel {
                self.fill_rect(6, y - 2, W - 12, 17, rgb565(36, 66, 118));
                self.draw_rect(6, y - 2, W - 12, 17, rgb565(90, 190, 255));
            }
            let lcol = if is_sel { rgb565(255, 230, 80) } else { rgb565(180, 195, 215) };
            let vcol = if is_sel { rgb565(255, 255, 255) } else { rgb565(135, 150, 170) };
            self.draw_text(12, y + 3, label, lcol, 1, false);
            self.draw_text(86, y + 3, val, vcol, 1, false);
        }

        let ctrl_ok = input.controller_kind() != ControllerKind::None;
        let dot_col = if ctrl_ok { rgb565(70, 240, 110) } else { rgb565(240, 160, 50) };
        self.fill_rect(8, H - 14, 6, 6, dot_col);
        let hint = if ctrl_ok {
            "BT/USB Ready | Ent:Select | Hold Esc:Exit"
        } else {
            "FXZC/WASD | Ent:Select | Hold Esc:Exit"
        };
        self.draw_text(18, H - 14, hint, rgb565(145, 160, 180), 1, false);
    }

    pub fn render_pause_menu(&mut self, selected: usize) {
        let mx = 70;
        let my = 32;
        let mw = 180;
        let mh = 104;
        self.fill_rect(mx, my, mw, mh, rgb565(14, 18, 30));
        self.draw_rect(mx, my, mw, mh, rgb565(90, 190, 255));
        self.draw_text(mx + 56, my + 8, "PAUSED", rgb565(255, 255, 255), 2, false);

        let items = ["RESUME FLIGHT", "RESPAWN START", "SETTINGS", "MAIN MENU"];
        for (i, label) in items.iter().enumerate() {
            let y = my + 32 + (i as i32) * 16;
            if i == selected {
                self.fill_rect(mx + 10, y - 2, mw - 20, 13, rgb565(40, 74, 130));
            }
            let col = if i == selected { rgb565(255, 230, 80) } else { rgb565(190, 200, 215) };
            self.draw_text(mx + 22, y + 1, label, col, 1, false);
        }
    }

    pub fn render_settings_menu(&mut self, cfg: &Config, selected: usize) {
        let rows: [(&str, String); 17] = [
            ("Game Mode", cfg.game_mode.name().to_string()),
            ("Flight Mode", cfg.flight_mode.name().to_string()),
            ("Battery", cfg.battery_pack.name().to_string()),
            ("Prop Wash", if cfg.prop_wash { "Stock" } else { "Off" }.to_string()),
            ("Sound", if cfg.sound_on { "On (Whoop Audio)" } else { "Off" }.to_string()),
            ("Rate Preset", cfg.rates.name().to_string()),
            ("Center Sens", format!("{} deg/s", cfg.rate_center)),
            ("Max Rate", format!("{} deg/s", cfg.rate_max)),
            ("Rate Expo", format!("{:.2}", cfg.rate_expo as f32 / 100.0)),
            ("Camera Tilt", format!("{} deg", cfg.camera_tilt_deg)),
            ("FOV", format!("{} deg", cfg.fov_deg)),
            ("Analog FX", if cfg.analog_filter { "On (NTSC/RF)" } else { "Off" }.to_string()),
            ("Debug OSD", if cfg.debug_osd { "On (FPS/ms)" } else { "Off" }.to_string()),
            ("Gfx Detail", if cfg.detail_high { "High" } else { "Low" }.to_string()),
            ("Sky Detail", if cfg.sky_detail { "On (Sun/Stars)" } else { "Off" }.to_string()),
            ("Stick Mode", cfg.stick_mode.name().to_string()),
            (
                "Thr / Sticks",
                format!(
                    "{} | Stk:{}",
                    cfg.throttle_mode.name(),
                    if cfg.show_sticks { "On" } else { "Off" }
                ),
            ),
        ];

        let tag = format!("{}/{}", selected + 1, rows.len());
        self.draw_header("SETTINGS", &tag);

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
                self.fill_rect(6, y - 2, W - 12, 14, rgb565(36, 66, 118));
            }
            let kcol = if is_sel { rgb565(255, 230, 80) } else { rgb565(180, 195, 215) };
            let vcol = if is_sel { rgb565(255, 255, 255) } else { rgb565(135, 200, 235) };
            self.draw_text(12, y + 1, k, kcol, 1, false);
            self.draw_text(126, y + 1, v, vcol, 1, false);
        }

        self.draw_text(8, H - 12, "F/X:Scroll | Z/C:Adjust | Esc:Save", rgb565(145, 160, 180), 1, false);
    }

    pub fn render_controller_screen(&mut self, cfg: &Config, input: &InputManager) {
        self.draw_header("CONTROLLER", "BT / USB-C");

        let name: String = input.controller_name().chars().take(36).collect();
        let dev_tag = if input.candidate_count > 1 {
            format!("{name} [{} devs]", input.candidate_count)
        } else {
            name
        };
        let aux_summary = format!(
            "Thr:{} | ARM:{} MOD:{} TRT:{}",
            cfg.throttle_mode.name(),
            cfg.aux_arm.short_label(),
            cfg.aux_mode.short_label(),
            cfg.aux_turtle.short_label(),
        );

        self.draw_text(8, 25, &dev_tag, rgb565(255, 230, 80), 1, false);
        self.draw_text(8, 35, &aux_summary, rgb565(150, 215, 255), 1, false);

        if let Some(ref wiz) = input.calib {
            self.fill_rect(10, 48, W - 20, 74, rgb565(20, 34, 62));
            self.draw_rect(10, 48, W - 20, 74, rgb565(90, 220, 255));
            self.draw_text(18, 56, "CALIBRATE STICKS", rgb565(120, 225, 255), 1, false);
            self.draw_text(18, 72, wiz.step_prompt(), rgb565(255, 235, 70), 1, false);
            let msg: String = wiz.message.chars().take(44).collect();
            self.draw_text(18, 90, &msg, rgb565(190, 240, 190), 1, false);
            self.draw_text(18, 106, "Esc: Cancel Calibration", rgb565(160, 175, 195), 1, false);
        } else if let Some(ref aw) = input.aux_wiz {
            self.fill_rect(10, 48, W - 20, 74, rgb565(20, 34, 62));
            self.draw_rect(10, 48, W - 20, 74, rgb565(90, 255, 180));
            self.draw_text(18, 56, "BIND RADIO AUX SWITCHES", rgb565(120, 255, 190), 1, false);
            self.draw_text(18, 72, aw.step_prompt(), rgb565(255, 235, 70), 1, false);
            let msg: String = aw.message.chars().take(44).collect();
            self.draw_text(18, 90, &msg, rgb565(190, 240, 190), 1, false);
            self.draw_text(18, 106, "Enter: Skip Step | Esc: Cancel", rgb565(160, 175, 195), 1, false);
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
                self.draw_text(8, y + 2, lbl, rgb565(210, 220, 230), 1, false);
                let bx = 38;
                let bw = 152;
                self.fill_rect(bx, y, bw, 10, rgb565(24, 30, 44));
                self.draw_rect(bx, y, bw, 10, rgb565(75, 95, 125));
                let mid = bx + bw / 2;
                self.draw_vline(mid, y + 1, y + 8, rgb565(90, 105, 130));
                let pos = bx + ((clampf(*val, -1.0, 1.0) + 1.0) * 0.5 * (bw - 4) as f32) as i32;
                self.fill_rect(pos, y + 2, 4, 6, rgb565(80, 255, 140));
            }

            self.draw_stick_box(204, 51, 46, sticks.yaw, sticks.throttle * 2.0 - 1.0);
            self.draw_stick_box(260, 51, 46, sticks.roll, sticks.pitch);
            self.draw_text(214, 101, "L-STK", rgb565(160, 175, 195), 1, false);
            self.draw_text(270, 101, "R-STK", rgb565(160, 175, 195), 1, false);
        }

        let raw = format!("ABS {}", input.raw_axes_summary());
        let raw_trim: String = raw.chars().take(50).collect();
        self.draw_text(8, 127, &raw_trim, rgb565(120, 210, 150), 1, false);

        self.draw_text(8, 141, "C:Sticks | B:Bind AUX | Ent:RC/Pad", rgb565(255, 255, 255), 1, false);
        self.draw_text(8, 154, "N:NextDev | R:Reset Auto | Esc:Back", rgb565(145, 160, 180), 1, false);
    }
}
