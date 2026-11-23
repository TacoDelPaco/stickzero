//! 3D FPV race tracks and freestyle environments.

use crate::math::Vec3;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum GateKind {
    /// Standard upright rectangular racing gate.
    Box,
    /// Horizontal dive gate (fly down through it).
    Dive,
    /// Tall race flag (pass to either side within radius).
    Flag,
}

#[derive(Copy, Clone, Debug)]
pub struct Gate {
    pub pos: Vec3,
    /// Yaw angle in radians (0 = facing +Z).
    pub yaw: f32,
    pub width: f32,
    pub height: f32,
    pub kind: GateKind,
}

impl Gate {
    pub const fn box_gate(x: f32, y: f32, z: f32, yaw_deg: f32, w: f32, h: f32) -> Self {
        Self {
            pos: Vec3::new(x, y, z),
            yaw: yaw_deg * std::f32::consts::PI / 180.0,
            width: w,
            height: h,
            kind: GateKind::Box,
        }
    }

    pub const fn dive_gate(x: f32, y: f32, z: f32, yaw_deg: f32, size: f32) -> Self {
        Self {
            pos: Vec3::new(x, y, z),
            yaw: yaw_deg * std::f32::consts::PI / 180.0,
            width: size,
            height: size,
            kind: GateKind::Dive,
        }
    }

    pub const fn flag(x: f32, z: f32, height: f32) -> Self {
        Self {
            pos: Vec3::new(x, 0.0, z),
            yaw: 0.0,
            width: 1.2,
            height,
            kind: GateKind::Flag,
        }
    }

    /// Forward normal of the gate plane.
    #[inline]
    pub fn forward(&self) -> Vec3 {
        match self.kind {
            GateKind::Dive => Vec3::new(0.0, -1.0, 0.0),
            _ => Vec3::new(self.yaw.sin(), 0.0, self.yaw.cos()),
        }
    }

    #[inline]
    pub fn right(&self) -> Vec3 {
        Vec3::new(self.yaw.cos(), 0.0, -self.yaw.sin())
    }

    /// Center of the gate opening in world coordinates.
    #[inline]
    pub fn center(&self) -> Vec3 {
        match self.kind {
            GateKind::Dive => self.pos,
            GateKind::Box => Vec3::new(self.pos.x, self.pos.y + self.height * 0.5, self.pos.z),
            GateKind::Flag => Vec3::new(self.pos.x, self.height * 0.5, self.pos.z),
        }
    }

    /// Check if movement from `prev` to `curr` passed through this gate.
    pub fn check_pass(&self, prev: Vec3, curr: Vec3) -> bool {
        match self.kind {
            GateKind::Box => {
                let c = self.center();
                let n = self.forward();
                let d0 = prev.sub(c).dot(n);
                let d1 = curr.sub(c).dot(n);
                // Crossed plane in either direction (generous for fun) or forward
                if (d0 <= 0.0 && d1 > 0.0) || (d0 >= 0.0 && d1 < 0.0) {
                    let denom = d0 - d1;
                    let t = if denom.abs() > 1e-6 { d0 / denom } else { 0.5 };
                    let hit = prev.add(curr.sub(prev).scale(t));
                    let rel = hit.sub(c);
                    let r = self.right();
                    let lx = rel.dot(r).abs();
                    let ly = rel.y.abs();
                    lx <= self.width * 0.58 && ly <= self.height * 0.58
                } else {
                    false
                }
            }
            GateKind::Dive => {
                let c = self.pos;
                if prev.y >= c.y && curr.y < c.y {
                    let dy = prev.y - curr.y;
                    let t = if dy > 1e-6 { (prev.y - c.y) / dy } else { 0.5 };
                    let hit = prev.add(curr.sub(prev).scale(t));
                    let dx = (hit.x - c.x).abs();
                    let dz = (hit.z - c.z).abs();
                    dx <= self.width * 0.6 && dz <= self.height * 0.6
                } else {
                    false
                }
            }
            GateKind::Flag => {
                let dx = curr.x - self.pos.x;
                let dz = curr.z - self.pos.z;
                let dist_sq = dx * dx + dz * dz;
                dist_sq < 6.5 * 6.5 && curr.y <= self.height + 3.0
            }
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub enum StructureColor {
    Concrete,
    Steel,
    RedWhite,
    Wood,
}

/// Axis-aligned 3D solid box obstacle (tower, bando wall, column, roof beam).
#[derive(Copy, Clone, Debug)]
pub struct Structure {
    pub min: Vec3,
    pub max: Vec3,
    pub color: StructureColor,
}

impl Structure {
    pub const fn aabb(x0: f32, y0: f32, z0: f32, x1: f32, y1: f32, z1: f32, color: StructureColor) -> Self {
        Self {
            min: Vec3::new(x0, y0, z0),
            max: Vec3::new(x1, y1, z1),
            color,
        }
    }

    pub fn center(&self) -> Vec3 {
        Vec3::new(
            (self.min.x + self.max.x) * 0.5,
            (self.min.y + self.max.y) * 0.5,
            (self.min.z + self.max.z) * 0.5,
        )
    }
}

pub struct Track {
    pub name: &'static str,
    pub subtitle: &'static str,
    pub start_pos: Vec3,
    pub start_yaw: f32,
    pub gates: &'static [Gate],
    pub structures: &'static [Structure],
}

// ---------------------------------------------------------------------------
// Track 0: Meadow Loop (friendly sweeping race course)
// ---------------------------------------------------------------------------
static MEADOW_GATES: [Gate; 8] = [
    Gate::box_gate(0.0, 0.0, 22.0, 0.0, 4.8, 3.6),
    Gate::box_gate(16.0, 0.0, 46.0, 38.0, 4.8, 3.6),
    Gate::box_gate(42.0, 2.0, 56.0, 90.0, 4.8, 3.6),
    Gate::box_gate(62.0, 0.0, 36.0, 165.0, 4.8, 3.6),
    Gate::box_gate(56.0, 0.0, 8.0, 200.0, 4.8, 3.6),
    Gate::box_gate(36.0, 1.5, -14.0, 245.0, 4.8, 3.6),
    Gate::box_gate(10.0, 0.0, -18.0, 285.0, 4.8, 3.6),
    Gate::box_gate(-4.0, 0.0, 2.0, 345.0, 4.8, 3.6),
];

static MEADOW_STRUCTURES: [Structure; 4] = [
    // Start tent / canopy posts
    Structure::aabb(-10.0, 0.0, -2.0, -7.0, 3.2, 2.0, StructureColor::RedWhite),
    // Mid-field landmark pylon
    Structure::aabb(28.0, 0.0, 20.0, 31.0, 14.0, 23.0, StructureColor::Concrete),
    // Outer chicane barrier
    Structure::aabb(50.0, 0.0, 22.0, 52.0, 6.0, 25.0, StructureColor::Wood),
    // Backstretch tower
    Structure::aabb(18.0, 0.0, -28.0, 21.0, 10.0, -25.0, StructureColor::Steel),
];

// ---------------------------------------------------------------------------
// Track 1: Figure 8 (fast crossover + elevated split-S gate)
// ---------------------------------------------------------------------------
static FIG8_GATES: [Gate; 8] = [
    Gate::box_gate(0.0, 0.0, 20.0, 0.0, 4.6, 3.5),
    Gate::box_gate(14.0, 2.0, 44.0, 35.0, 4.6, 3.5),
    Gate::box_gate(38.0, 4.0, 46.0, 115.0, 4.6, 3.5),
    Gate::box_gate(24.0, 1.0, 20.0, 215.0, 4.6, 3.5),
    Gate::box_gate(-14.0, 0.0, -4.0, 225.0, 4.6, 3.5),
    Gate::box_gate(-34.0, 3.0, 14.0, 315.0, 4.6, 3.5),
    Gate::box_gate(-24.0, 5.5, 36.0, 35.0, 4.6, 3.5),
    Gate::box_gate(-4.0, 0.0, 4.0, 15.0, 4.6, 3.5),
];

static FIG8_STRUCTURES: [Structure; 4] = [
    // Center crossover bridge pillars
    Structure::aabb(8.0, 0.0, 16.0, 10.5, 11.0, 18.5, StructureColor::Steel),
    Structure::aabb(-12.0, 0.0, 16.0, -9.5, 11.0, 18.5, StructureColor::Steel),
    // High turn marker tower
    Structure::aabb(46.0, 0.0, 48.0, 49.0, 16.0, 51.0, StructureColor::RedWhite),
    Structure::aabb(-42.0, 0.0, 22.0, -39.0, 15.0, 25.0, StructureColor::RedWhite),
];

// ---------------------------------------------------------------------------
// Track 2: Dive Tower (28m central tower + vertical dive gate!)
// ---------------------------------------------------------------------------
static DIVE_GATES: [Gate; 7] = [
    Gate::box_gate(0.0, 0.0, 20.0, 0.0, 4.8, 3.8),
    Gate::box_gate(0.0, 10.0, 40.0, 0.0, 5.2, 4.2),
    Gate::dive_gate(0.0, 16.0, 54.0, 0.0, 6.5),
    Gate::box_gate(16.0, 0.0, 54.0, 90.0, 4.8, 3.6),
    Gate::box_gate(28.0, 0.0, 28.0, 175.0, 4.8, 3.6),
    Gate::box_gate(14.0, 0.0, 0.0, 240.0, 4.8, 3.6),
    Gate::box_gate(-6.0, 0.0, 6.0, 340.0, 4.8, 3.6),
];

static DIVE_STRUCTURES: [Structure; 5] = [
    // 28m Dive Tower core
    Structure::aabb(-3.0, 0.0, 44.0, 3.0, 28.0, 50.0, StructureColor::Concrete),
    // Tower top crown
    Structure::aabb(-4.5, 26.0, 42.5, 4.5, 28.5, 51.5, StructureColor::RedWhite),
    // Side underpass pillars
    Structure::aabb(14.0, 0.0, 48.0, 16.0, 8.0, 50.0, StructureColor::Steel),
    Structure::aabb(14.0, 0.0, 58.0, 16.0, 8.0, 60.0, StructureColor::Steel),
    Structure::aabb(13.5, 7.5, 47.5, 16.5, 9.5, 60.5, StructureColor::Concrete),
];

// ---------------------------------------------------------------------------
// Track 3: Bando Freestyle (concrete ruin, gaps, pillars, roof dive)
// ---------------------------------------------------------------------------
static BANDO_GATES: [Gate; 6] = [
    Gate::box_gate(0.0, 0.0, 18.0, 0.0, 5.4, 4.2),
    Gate::box_gate(0.0, 7.0, 42.0, 0.0, 5.4, 4.2),
    Gate::dive_gate(18.0, 12.0, 44.0, 90.0, 6.5),
    Gate::box_gate(18.0, 0.0, 24.0, 180.0, 5.4, 4.2),
    Gate::box_gate(-16.0, 3.0, 26.0, 270.0, 5.4, 4.2),
    Gate::box_gate(-14.0, 0.0, 6.0, 160.0, 5.4, 4.2),
];

static BANDO_STRUCTURES: [Structure; 8] = [
    // Main bando hall left & right walls
    Structure::aabb(-10.0, 0.0, 22.0, -7.5, 13.0, 54.0, StructureColor::Concrete),
    Structure::aabb(7.5, 0.0, 22.0, 10.0, 13.0, 54.0, StructureColor::Concrete),
    // Roof beams across the hall (leave a huge powerloop gap in the middle!)
    Structure::aabb(-10.0, 11.5, 22.0, 10.0, 13.5, 28.0, StructureColor::Concrete),
    Structure::aabb(-10.0, 11.5, 48.0, 10.0, 13.5, 54.0, StructureColor::Concrete),
    // Smokestack tower
    Structure::aabb(24.0, 0.0, 40.0, 28.5, 32.0, 44.5, StructureColor::Concrete),
    // Exterior concrete pillars for slalom / trippy spins
    Structure::aabb(-22.0, 0.0, 14.0, -19.5, 14.0, 16.5, StructureColor::Steel),
    Structure::aabb(-22.0, 0.0, 34.0, -19.5, 14.0, 36.5, StructureColor::Steel),
    Structure::aabb(18.0, 0.0, 12.0, 20.5, 12.0, 14.5, StructureColor::Wood),
];

pub static TRACKS: [Track; 4] = [
    Track {
        name: "Meadow Loop",
        subtitle: "8 gates - wide oval",
        start_pos: Vec3::new(0.0, 0.35, 0.0),
        start_yaw: 0.0,
        gates: &MEADOW_GATES,
        structures: &MEADOW_STRUCTURES,
    },
    Track {
        name: "Figure 8",
        subtitle: "8 gates - crossover",
        start_pos: Vec3::new(0.0, 0.35, 0.0),
        start_yaw: 0.0,
        gates: &FIG8_GATES,
        structures: &FIG8_STRUCTURES,
    },
    Track {
        name: "Dive Tower",
        subtitle: "28m tower + dive gate",
        start_pos: Vec3::new(0.0, 0.35, 0.0),
        start_yaw: 0.0,
        gates: &DIVE_GATES,
        structures: &DIVE_STRUCTURES,
    },
    Track {
        name: "Bando",
        subtitle: "Concrete ruin + gaps",
        start_pos: Vec3::new(0.0, 0.35, 0.0),
        start_yaw: 0.0,
        gates: &BANDO_GATES,
        structures: &BANDO_STRUCTURES,
    },
];
