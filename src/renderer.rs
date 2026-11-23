//! Ultra-fast zero-allocation 320x170 RGB565 3D software rasterizer,
//! Betaflight-style OSD, and CardputerZero 1.9" menu UI.

use crate::config::{Config, GameMode, VisualTheme};
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
const NEAR_Z: f32 = 0.18;

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
    grid_line: u16,
    active_gate: u16,
    active_gate_hi: u16,
    idle_gate: u16,
    wireframe: bool,
}

fn palette(theme: VisualTheme) -> ThemePalette {
    match theme {
        VisualTheme::Summer => ThemePalette {
            sky_top: rgb565(32, 108, 210),
            sky_horizon: rgb565(155, 206, 240),
            ground_near: rgb565(56, 122, 54),
            ground_far: rgb565(118, 162, 122),
            grid_line: rgb565(42, 98, 42),
            active_gate: rgb565(255, 110, 20),
            active_gate_hi: rgb565(255, 220, 60),
            idle_gate: rgb565(180, 190, 205),
            wireframe: false,
        },
        VisualTheme::Evening => ThemePalette {
            sky_top: rgb565(42, 34, 88),
            sky_horizon: rgb565(235, 130, 74),
            ground_near: rgb565(58, 66, 46),
            ground_far: rgb565(118, 92, 72),
            grid_line: rgb565(42, 48, 34),
            active_gate: rgb565(255, 210, 40),
            active_gate_hi: rgb565(255, 255, 180),
            idle_gate: rgb565(160, 145, 165),
            wireframe: false,
        },
        VisualTheme::Night => ThemePalette {
            sky_top: rgb565(6, 10, 26),
            sky_horizon: rgb565(22, 30, 64),
            ground_near: rgb565(16, 22, 30),
            ground_far: rgb565(24, 30, 44),
            grid_line: rgb565(30, 56, 78),
            active_gate: rgb565(0, 255, 180),
            active_gate_hi: rgb565(160, 255, 240),
            idle_gate: rgb565(80, 110, 210),
            wireframe: false,
        },
        VisualTheme::Wireframe => ThemePalette {
            sky_top: rgb565(4, 8, 12),
            sky_horizon: rgb565(8, 22, 18),
            ground_near: rgb565(4, 10, 8),
            ground_far: rgb565(8, 22, 18),
            grid_line: rgb565(18, 95, 58),
            active_gate: rgb565(80, 255, 120),
            active_gate_hi: rgb565(220, 255, 120),
            idle_gate: rgb565(36, 130, 82),
            wireframe: true,
        },
    }
}

#[derive(Copy, Clone)]
enum DrawCmd {
    Poly {
        depth: f32,
        count: usize,
        pts: [(i32, i32); 5],
        fill: u16,
        edge: u16,
    },
    Line {
        depth: f32,
        x0: i32,
        y0: i32,
        x1: i32,
        y1: i32,
        color: u16,
        thick: bool,
    },
}

impl DrawCmd {
    #[inline(always)]
    fn depth(&self) -> f32 {
        match *self {
            DrawCmd::Poly { depth, .. } => depth,
            DrawCmd::Line { depth, .. } => depth,
        }
    }
}

pub struct Renderer {
    pub fb: Box<[u16; SCREEN_W * SCREEN_H]>,
    cmds: Vec<DrawCmd>,
    focal: f32,
}

impl Renderer {
    pub fn new() -> Self {
        Self {
            fb: Box::new([0u16; SCREEN_W * SCREEN_H]),
            cmds: Vec::with_capacity(512),
            focal: 150.0,
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
        // Quick reject far off-screen
        if (x0 < -400 && x1 < -400)
            || (x0 > W + 400 && x1 > W + 400)
            || (y0 < -400 && y1 < -400)
            || (y0 > H + 400 && y1 > H + 400)
        {
            return;
        }
        let dx = (x1 - x0).abs();
        let sx = if x0 < x1 { 1 } else { -1 };
        let dy = -(y1 - y0).abs();
        let sy = if y0 < y1 { 1 } else { -1 };
        let mut err = dx + dy;
        let mut steps = 0;
        loop {
            self.put_pixel(x0, y0, color);
            if x0 == x1 && y0 == y1 {
                break;
            }
            steps += 1;
            if steps > 1200 {
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

    fn fill_tri(&mut self, p0: (i32, i32), p1: (i32, i32), p2: (i32, i32), color: u16) {
        let mut pts = [p0, p1, p2];
        if pts[0].1 > pts[1].1 { pts.swap(0, 1); }
        if pts[1].1 > pts[2].1 { pts.swap(1, 2); }
        if pts[0].1 > pts[1].1 { pts.swap(0, 1); }

        let (x0, y0) = pts[0];
        let (x1, y1) = pts[1];
        let (x2, y2) = pts[2];
        let total_h = y2 - y0;
        if total_h <= 0 {
            return;
        }

        let y_start = y0.max(0);
        let y_end = y2.min(H - 1);
        for y in y_start..=y_end {
            let second_half = y > y1 || y1 == y0;
            let seg_h = if second_half { y2 - y1 } else { y1 - y0 };
            if seg_h == 0 {
                continue;
            }
            let alpha = (y - y0) as f32 / total_h as f32;
            let beta = if second_half {
                (y - y1) as f32 / seg_h as f32
            } else {
                (y - y0) as f32 / seg_h as f32
            };
            let ax = x0 + ((x2 - x0) as f32 * alpha) as i32;
            let bx = if second_half {
                x1 + ((x2 - x1) as f32 * beta) as i32
            } else {
                x0 + ((x1 - x0) as f32 * beta) as i32
            };
            self.draw_hline(ax, bx, y, color);
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

    /// Transform world point into camera space (+X right, +Y up, +Z forward).
    #[inline(always)]
    fn world_to_cam(p: Vec3, cam_pos: Vec3, cam_rot_inv: Quat) -> Vec3 {
        cam_rot_inv.rotate(p.sub(cam_pos))
    }

    #[inline(always)]
    fn project(&self, c: Vec3) -> (i32, i32) {
        let inv_z = self.focal / c.z;
        let sx = HALF_W + c.x * inv_z;
        let sy = HALF_H - c.y * inv_z;
        (sx.clamp(-4000.0, 4000.0) as i32, sy.clamp(-4000.0, 4000.0) as i32)
    }

    fn push_3d_line(&mut self, a: Vec3, b: Vec3, cam_pos: Vec3, cam_inv: Quat, color: u16, thick: bool) {
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
        let (x0, y0) = self.project(ca);
        let (x1, y1) = self.project(cb);
        let depth = (ca.z + cb.z) * 0.5;
        self.cmds.push(DrawCmd::Line { depth, x0, y0, x1, y1, color, thick });
    }

    /// Clip a 4-vertex camera-space quad against `z >= NEAR_Z` using Sutherland-Hodgman.
    /// Keeps solid walls 100% visible even when touching or scraping right against them!
    fn push_3d_quad(&mut self, verts: [Vec3; 4], cam_pos: Vec3, cam_inv: Quat, fill: u16, edge: u16, wireframe: bool) {
        if wireframe {
            for i in 0..4 {
                self.push_3d_line(verts[i], verts[(i + 1) & 3], cam_pos, cam_inv, edge, false);
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

        let mut pts = [(0i32, 0i32); 5];
        let mut min_x = i32::MAX;
        let mut max_x = i32::MIN;
        let mut min_y = i32::MAX;
        let mut max_y = i32::MIN;
        let mut depth_sum = 0.0f32;
        for i in 0..count {
            let p = self.project(clipped[i]);
            pts[i] = p;
            min_x = min_x.min(p.0);
            max_x = max_x.max(p.0);
            min_y = min_y.min(p.1);
            max_y = max_y.max(p.1);
            depth_sum += clipped[i].z;
        }
        if max_x < 0 || min_x >= W || max_y < 0 || min_y >= H {
            return;
        }
        let depth = depth_sum / (count as f32);
        self.cmds.push(DrawCmd::Poly {
            depth,
            count,
            pts,
            fill,
            edge,
        });
    }

    /// Render the sky and ground background accurately tilted by camera pitch and roll.
    fn draw_sky_and_ground(&mut self, cam_rot: Quat, pal: ThemePalette, detail_high: bool) {
        let (cam_right, cam_up, cam_fwd) = cam_rot.basis();
        let ux = cam_right.y;
        let uy = cam_up.y;
        let uz = cam_fwd.y * self.focal;
        let inv_f = 1.0 / self.focal;

        for y in 0..SCREEN_H {
            let ry = HALF_H - (y as f32);
            let base = uy * ry + uz;
            let row = y * SCREEN_W;
            if ux.abs() < 0.002 {
                let elev = base * inv_f;
                let color = if elev >= 0.0 {
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
                self.fb[row..row + SCREEN_W].fill(color);
            } else {
                for x in 0..SCREEN_W {
                    let rx = (x as f32) - HALF_W;
                    let elev = (ux * rx + base) * inv_f;
                    let color = if elev >= 0.0 {
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
                    self.fb[row + x] = color;
                }
            }
        }
    }

    fn draw_sky_detail(&mut self, cam_inv: Quat, theme: VisualTheme) {
        if theme == VisualTheme::Wireframe {
            return;
        }
        // Sun / Moon directional disc in world sky
        let (sun_dir, sun_col, halo_col) = match theme {
            VisualTheme::Summer => (
                Vec3::new(0.45, 0.48, 0.75),
                rgb565(255, 252, 200),
                rgb565(255, 235, 140),
            ),
            VisualTheme::Evening => (
                Vec3::new(0.35, 0.16, 0.92),
                rgb565(255, 210, 95),
                rgb565(245, 130, 60),
            ),
            VisualTheme::Night => (
                Vec3::new(-0.40, 0.52, 0.75),
                rgb565(225, 235, 255),
                rgb565(120, 140, 190),
            ),
            VisualTheme::Wireframe => return,
        };
        let sc = cam_inv.rotate(sun_dir);
        if sc.z > 0.15 {
            let (sx, sy) = self.project(sc);
            self.fill_rect(sx - 5, sy - 5, 11, 11, halo_col);
            self.fill_rect(sx - 3, sy - 3, 7, 7, sun_col);
        }
        // Night starfield
        if theme == VisualTheme::Night {
            const STARS: [Vec3; 10] = [
                Vec3::new(0.6, 0.5, 0.6),
                Vec3::new(-0.7, 0.4, 0.55),
                Vec3::new(0.2, 0.75, 0.62),
                Vec3::new(-0.25, 0.65, -0.7),
                Vec3::new(0.75, 0.35, -0.55),
                Vec3::new(-0.65, 0.55, -0.5),
                Vec3::new(0.1, 0.45, -0.88),
                Vec3::new(-0.85, 0.3, 0.42),
                Vec3::new(0.42, 0.68, -0.6),
                Vec3::new(-0.15, 0.32, 0.93),
            ];
            let star_col = rgb565(200, 220, 255);
            for &s in &STARS {
                let c = cam_inv.rotate(s);
                if c.z > 0.2 {
                    let (px, py) = self.project(c);
                    self.put_pixel(px, py, star_col);
                }
            }
        }
    }

    /// Render the full 3D FPV scene + Betaflight OSD!
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

        // Camera orientation = quad orientation pitched up by camera_tilt_deg
        let tilt = -(cfg.camera_tilt_deg as f32).to_radians();
        let cam_rot = quad.rot.mul(Quat::from_axis_angle(Vec3::RIGHT, tilt)).normalized();
        let cam_inv = cam_rot.conjugate();
        let cam_pos = quad.pos;

        // 1. Sky & Ground + optional Sun/Moon/Stars
        self.draw_sky_and_ground(cam_rot, pal, cfg.detail_high);
        if cfg.sky_detail {
            self.draw_sky_detail(cam_inv, cfg.theme);
        }

        self.cmds.clear();

        // 2. Ground speed grid around quad (10m spacing on High, 20m spacing on Low)
        let step = if cfg.detail_high { 10 } else { 20 };
        let half_cnt = if cfg.detail_high { 4 } else { 2 };
        let gx0 = (cam_pos.x / step as f32).floor() as i32 * step;
        let gz0 = (cam_pos.z / step as f32).floor() as i32 * step;
        for i in -half_cnt..=half_cnt {
            let x = (gx0 + i * step) as f32;
            let z_start = (gz0 - 40) as f32;
            let z_end = (gz0 + 40) as f32;
            self.push_3d_line(
                Vec3::new(x, 0.0, z_start),
                Vec3::new(x, 0.0, z_end),
                cam_pos,
                cam_inv,
                pal.grid_line,
                false,
            );
            let z = (gz0 + i * step) as f32;
            let x_start = (gx0 - 40) as f32;
            let x_end = (gx0 + 40) as f32;
            self.push_3d_line(
                Vec3::new(x_start, 0.0, z),
                Vec3::new(x_end, 0.0, z),
                cam_pos,
                cam_inv,
                pal.grid_line,
                false,
            );
        }

        // 3. Structures (solid shaded 3D boxes with true near-plane clipping)
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
                self.push_3d_quad(
                    [Vec3::new(x0, y1, z0), Vec3::new(x1, y1, z0), Vec3::new(x1, y1, z1), Vec3::new(x0, y1, z1)],
                    cam_pos, cam_inv, c_top, c_edge, wf,
                );
            }
            if cam_pos.z < z0 {
                self.push_3d_quad(
                    [Vec3::new(x0, y0, z0), Vec3::new(x1, y0, z0), Vec3::new(x1, y1, z0), Vec3::new(x0, y1, z0)],
                    cam_pos, cam_inv, c_side_a, c_edge, wf,
                );
            }
            if cam_pos.z > z1 {
                self.push_3d_quad(
                    [Vec3::new(x0, y0, z1), Vec3::new(x1, y0, z1), Vec3::new(x1, y1, z1), Vec3::new(x0, y1, z1)],
                    cam_pos, cam_inv, c_side_a, c_edge, wf,
                );
            }
            if cam_pos.x < x0 {
                self.push_3d_quad(
                    [Vec3::new(x0, y0, z0), Vec3::new(x0, y0, z1), Vec3::new(x0, y1, z1), Vec3::new(x0, y1, z0)],
                    cam_pos, cam_inv, c_side_b, c_edge, wf,
                );
            }
            if cam_pos.x > x1 {
                self.push_3d_quad(
                    [Vec3::new(x1, y0, z0), Vec3::new(x1, y0, z1), Vec3::new(x1, y1, z1), Vec3::new(x1, y1, z0)],
                    cam_pos, cam_inv, c_side_b, c_edge, wf,
                );
            }
        }

        // 4. Racing Gates (all gates highlighted brightly in Freestyle so loops pop!)
        for (idx, g) in track.gates.iter().enumerate() {
            let is_next = cfg.game_mode == GameMode::Freestyle || idx == quad.next_gate;
            let main_col = if is_next { pal.active_gate } else { pal.idle_gate };
            let hi_col = if is_next { pal.active_gate_hi } else { rgb565(225, 230, 235) };

            match g.kind {
                GateKind::Box => {
                    let r = g.right().scale(g.width * 0.5);
                    let p_bl = g.pos.sub(r);
                    let p_br = g.pos.add(r);
                    let p_tl = Vec3::new(p_bl.x, g.pos.y + g.height, p_bl.z);
                    let p_tr = Vec3::new(p_br.x, g.pos.y + g.height, p_br.z);

                    self.push_3d_line(p_bl, p_tl, cam_pos, cam_inv, main_col, true);
                    self.push_3d_line(p_br, p_tr, cam_pos, cam_inv, main_col, true);
                    self.push_3d_line(p_tl, p_tr, cam_pos, cam_inv, hi_col, true);
                    self.push_3d_line(p_bl, p_br, cam_pos, cam_inv, main_col, false);

                    if is_next && cfg.detail_high && !pal.wireframe {
                        let b_tl = Vec3::new(p_tl.x, p_tl.y + 0.55, p_tl.z);
                        let b_tr = Vec3::new(p_tr.x, p_tr.y + 0.55, p_tr.z);
                        self.push_3d_quad([p_tl, p_tr, b_tr, b_tl], cam_pos, cam_inv, main_col, hi_col, false);
                    }
                    if g.pos.y > 0.2 {
                        let g_bl = Vec3::new(p_bl.x, 0.0, p_bl.z);
                        let g_br = Vec3::new(p_br.x, 0.0, p_br.z);
                        self.push_3d_line(g_bl, p_bl, cam_pos, cam_inv, rgb565(90, 95, 105), false);
                        self.push_3d_line(g_br, p_br, cam_pos, cam_inv, rgb565(90, 95, 105), false);
                    }
                }
                GateKind::Dive => {
                    let hw = g.width * 0.5;
                    let c0 = Vec3::new(g.pos.x - hw, g.pos.y, g.pos.z - hw);
                    let c1 = Vec3::new(g.pos.x + hw, g.pos.y, g.pos.z - hw);
                    let c2 = Vec3::new(g.pos.x + hw, g.pos.y, g.pos.z + hw);
                    let c3 = Vec3::new(g.pos.x - hw, g.pos.y, g.pos.z + hw);
                    self.push_3d_line(c0, c1, cam_pos, cam_inv, hi_col, true);
                    self.push_3d_line(c1, c2, cam_pos, cam_inv, main_col, true);
                    self.push_3d_line(c2, c3, cam_pos, cam_inv, hi_col, true);
                    self.push_3d_line(c3, c0, cam_pos, cam_inv, main_col, true);
                }
                GateKind::Flag => {
                    let base = g.pos;
                    let top = Vec3::new(g.pos.x, g.height, g.pos.z);
                    let tip = Vec3::new(g.pos.x + 1.2, g.height - 0.8, g.pos.z);
                    self.push_3d_line(base, top, cam_pos, cam_inv, hi_col, true);
                    self.push_3d_line(top, tip, cam_pos, cam_inv, main_col, true);
                }
            }
        }

        // 5. Painter's algorithm sort (far to near) and rasterize!
        self.cmds.sort_unstable_by(|a, b| b.depth().total_cmp(&a.depth()));
        for i in 0..self.cmds.len() {
            match self.cmds[i] {
                DrawCmd::Poly { count, pts, fill, edge, .. } => {
                    for k in 1..(count - 1) {
                        self.fill_tri(pts[0], pts[k], pts[k + 1], fill);
                    }
                    for k in 0..count {
                        let n = (k + 1) % count;
                        self.draw_line(pts[k].0, pts[k].1, pts[n].0, pts[n].1, edge);
                    }
                }
                DrawCmd::Line { x0, y0, x1, y1, color, thick, .. } => {
                    self.draw_line(x0, y0, x1, y1, color);
                    if thick {
                        self.draw_line(x0 + 1, y0, x1 + 1, y1, color);
                        self.draw_line(x0, y0 + 1, x1, y1 + 1, color);
                    }
                }
            }
        }

        // 6. Hit flash border
        if quad.hit_flash > 0.0 {
            let red = rgb565(255, 45, 45);
            self.draw_rect(0, 0, W, H, red);
            self.draw_rect(1, 1, W - 2, H - 2, red);
        }

        // 7. Next-gate screen-space beacon / off-screen edge pointer (Race mode)
        if cfg.game_mode == GameMode::Race && !track.gates.is_empty() {
            let target = track.gates[quad.next_gate].center();
            let tc = Self::world_to_cam(target, cam_pos, cam_inv);
            let marker_col = rgb565(255, 230, 40);
            if tc.z > 0.5 {
                let (sx, sy) = self.project(tc);
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

        // 8. Betaflight-style OSD
        self.draw_osd(quad, cfg, track, sticks, fps, frame_ms);
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
        let cyan = rgb565(90, 240, 255);
        let yellow = rgb565(255, 225, 60);
        let green = rgb565(80, 255, 110);
        let red = rgb565(255, 70, 70);

        // Center crosshair
        let cx = W / 2;
        let cy = H / 2;
        self.draw_hline(cx - 5, cx - 2, cy, white);
        self.draw_hline(cx + 2, cx + 5, cy, white);
        self.draw_vline(cx, cy - 3, cy - 2, white);

        // Top-left: Mode + Gate / Live Points
        let mode_str = cfg.flight_mode.name();
        self.draw_text(4, 3, mode_str, cyan, 1, true);
        if cfg.game_mode == GameMode::Race {
            let g_str = format!("G {}/{}", quad.next_gate + 1, track.gates.len());
            self.draw_text(4, 13, &g_str, yellow, 1, true);
            let s_str = format!("PTS {}", quad.score);
            self.draw_text(4, 23, &s_str, green, 1, true);
        } else {
            let s_str = format!("PTS {}", quad.score);
            self.draw_text(4, 13, &s_str, yellow, 1, true);
        }

        // Top-center: Lap timer
        if cfg.game_mode == GameMode::Race {
            let secs = quad.lap_time;
            let mins = (secs / 60.0) as u32;
            let rem = secs - (mins as f32) * 60.0;
            let t_str = format!("{mins}:{rem:05.2}");
            self.draw_text(cx - 21, 3, &t_str, white, 1, true);
            let best = cfg.best_laps_ms[cfg.track_index.min(3)];
            if best > 0 {
                let bs = best as f32 / 1000.0;
                let bm = (bs / 60.0) as u32;
                let br = bs - (bm as f32) * 60.0;
                let b_str = format!("B {bm}:{br:05.2}");
                self.draw_text(cx - 27, 13, &b_str, green, 1, true);
            }
        }

        // Top-right: Battery + FPS
        if cfg.battery_sim {
            let v_col = if quad.cell_voltage < 3.4 {
                red
            } else if quad.cell_voltage < 3.6 {
                yellow
            } else {
                green
            };
            let v_str = format!("{:.2}V", quad.cell_voltage);
            self.draw_text(W - 34, 3, &v_str, v_col, 1, true);
        }
        let perf_str = format!("{}F {:.1}m", fps, frame_ms);
        self.draw_text(W - 58, 13, &perf_str, rgb565(190, 200, 215), 1, true);

        // Bottom-center: Speed & Altitude
        let spd = format!("{:3.0}KPH  {:2.0}M", quad.speed_kmh(), quad.pos.y.max(0.0));
        self.draw_text(cx - 36, H - 11, &spd, white, 1, true);

        // Combo / Lap / Turtle alerts
        if quad.combo_timer > 0.0 && !quad.combo_text.is_empty() {
            let tw = (quad.combo_text.len() as i32) * 6;
            self.draw_text(cx - tw / 2, 34, quad.combo_text, yellow, 1, true);
        } else if quad.on_ground && quad.inverted {
            self.draw_text(cx - 54, 34, "TURTLE: PUSH ROLL", red, 1, true);
        } else if quad.battery_empty && cfg.battery_sim {
            self.draw_text(cx - 48, 34, "BATT EMPTY (R)", red, 1, true);
        }

        // Optional compact live gimbal stick overlay in bottom corners
        if cfg.show_sticks {
            self.draw_stick_box(4, H - 30, 26, sticks.yaw, sticks.throttle * 2.0 - 1.0);
            self.draw_stick_box(W - 30, H - 30, 26, sticks.roll, sticks.pitch);
        }
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
            ("SETTINGS", "Rates, Sticks, Battery".to_string()),
            ("CONTROLLER", input.controller_name().chars().take(22).collect()),
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

        // Status footer
        let ctrl_ok = input.controller_kind() != ControllerKind::None;
        let dot_col = if ctrl_ok { rgb565(70, 240, 110) } else { rgb565(240, 160, 50) };
        self.fill_rect(8, H - 14, 6, 6, dot_col);
        let hint = if ctrl_ok {
            "BT/USB Ready | A/Ent:Select | Hold Esc:Exit"
        } else {
            "USB-C/BT | KB:WASD+Arr | Hold Esc:Exit"
        };
        self.draw_text(18, H - 14, hint, rgb565(145, 160, 180), 1, false);
    }

    pub fn render_pause_menu(&mut self, selected: usize) {
        // Dim center modal over live 3D frame
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
        let rows: [(&str, String); 13] = [
            ("Game Mode", cfg.game_mode.name().to_string()),
            ("Flight Mode", cfg.flight_mode.name().to_string()),
            ("Rate Preset", cfg.rates.name().to_string()),
            ("Center Sens", format!("{} deg/s", cfg.rate_center)),
            ("Max Rate", format!("{} deg/s", cfg.rate_max)),
            ("Rate Expo", format!("{:.2}", cfg.rate_expo as f32 / 100.0)),
            ("Camera Tilt", format!("{} deg", cfg.camera_tilt_deg)),
            ("FOV", format!("{} deg", cfg.fov_deg)),
            ("Gfx Detail", if cfg.detail_high { "High" } else { "Low" }.to_string()),
            ("Sky Detail", if cfg.sky_detail { "On (Sun/Stars)" } else { "Off" }.to_string()),
            ("Stick Mode", cfg.stick_mode.name().to_string()),
            ("Throttle", cfg.throttle_mode.name().to_string()),
            (
                "Batt / OSD",
                format!(
                    "Batt:{} Stk:{}",
                    if cfg.battery_sim { "On" } else { "Off" },
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
            self.draw_text(134, y + 1, v, vcol, 1, false);
        }

        self.draw_text(8, H - 12, "Up/Dn:Scroll | L/R:Adjust | Esc:Save", rgb565(145, 160, 180), 1, false);
    }

    pub fn render_controller_screen(&mut self, cfg: &Config, input: &InputManager) {
        self.draw_header("CONTROLLER", "BT / USB-C");

        let name: String = input.controller_name().chars().take(36).collect();
        let dev_tag = if input.candidate_count > 1 {
            format!("{name} [{} devs]", input.candidate_count)
        } else {
            name
        };
        let kind_str = match input.controller_kind() {
            ControllerKind::RcRadio => "RC Radio | Thr:",
            ControllerKind::Gamepad => "Gamepad  | Thr:",
            ControllerKind::None => "No Pad   | Thr:",
        };
        let mapped_tag = if cfg.axis_thr.is_some() { "CUSTOM" } else { "AUTO" };
        let sub = format!("{kind_str}{} ({mapped_tag})", cfg.throttle_mode.name());

        self.draw_text(8, 26, &dev_tag, rgb565(255, 230, 80), 1, false);
        self.draw_text(8, 36, &sub, rgb565(150, 215, 255), 1, false);

        if let Some(ref wiz) = input.calib {
            // Interactive 4-step calibration overlay
            self.fill_rect(10, 50, W - 20, 72, rgb565(20, 34, 62));
            self.draw_rect(10, 50, W - 20, 72, rgb565(90, 220, 255));
            self.draw_text(18, 58, "CALIBRATE STICKS", rgb565(120, 225, 255), 1, false);
            self.draw_text(18, 74, wiz.step_prompt(), rgb565(255, 235, 70), 1, false);
            let msg: String = wiz.message.chars().take(44).collect();
            self.draw_text(18, 92, &msg, rgb565(190, 240, 190), 1, false);
            self.draw_text(18, 108, "Esc: Cancel Calibration", rgb565(160, 175, 195), 1, false);
        } else {
            let sticks = input.flight_sticks(cfg);
            let bars: [(&str, f32); 4] = [
                ("THR", sticks.throttle * 2.0 - 1.0),
                ("YAW", sticks.yaw),
                ("PIT", sticks.pitch),
                ("ROL", sticks.roll),
            ];

            for (i, (lbl, val)) in bars.iter().enumerate() {
                let y = 50 + (i as i32) * 16;
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

            // Live dual gimbal preview on the right
            self.draw_stick_box(204, 52, 46, sticks.yaw, sticks.throttle * 2.0 - 1.0);
            self.draw_stick_box(260, 52, 46, sticks.roll, sticks.pitch);
            self.draw_text(214, 102, "L-STK", rgb565(160, 175, 195), 1, false);
            self.draw_text(270, 102, "R-STK", rgb565(160, 175, 195), 1, false);
        }

        // Live raw EV_ABS axes diagnostic strip
        let raw = format!("ABS {}", input.raw_axes_summary());
        let raw_trim: String = raw.chars().take(50).collect();
        self.draw_text(8, 128, &raw_trim, rgb565(120, 210, 150), 1, false);

        self.draw_text(8, 142, "C:Detect Sticks | Ent:RC/Pad | N:NextDev", rgb565(255, 255, 255), 1, false);
        self.draw_text(8, 155, "R:Rescan+Auto | Esc:Back", rgb565(145, 160, 180), 1, false);
    }
}
