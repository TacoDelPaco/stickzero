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
                    // Must pass strictly INSIDE the inner gate opening (never outside the posts/crossbar!)
                    let inner_hw = (self.width * 0.5 - 0.14).max(0.4);
                    let inner_hh = (self.height * 0.5 - 0.14).max(0.4);
                    lx <= inner_hw && ly <= inner_hh
                } else {
                    false
                }
            }
            GateKind::Dive => {
                let c = self.pos;
                if (prev.y >= c.y && curr.y < c.y) || (prev.y <= c.y && curr.y > c.y) {
                    let dy = (prev.y - curr.y).abs();
                    let t = if dy > 1e-6 { (prev.y - c.y).abs() / dy } else { 0.5 };
                    let hit = prev.add(curr.sub(prev).scale(t));
                    let dx = (hit.x - c.x).abs();
                    let dz = (hit.z - c.z).abs();
                    let inner_hw = (self.width * 0.5 - 0.14).max(0.4);
                    let inner_hd = (self.height * 0.5 - 0.14).max(0.4);
                    dx <= inner_hw && dz <= inner_hd
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

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum StructureColor {
    Concrete,
    Steel,
    RedWhite,
    Wood,
    DarkWood,
    Foliage,
    Hay,
}

/// Axis-aligned 3D solid box obstacle (tower, tree trunk/canopy, fence, hay bale, wall, gantry).
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
// Track 0: Meadow Loop (farmstead oval packed with fences, trees, hay & barn)
// ---------------------------------------------------------------------------
static MEADOW_GATES: [Gate; 8] = [
    Gate::box_gate(0.0, 0.0, 22.0, 0.0, 4.8, 3.6),
    Gate::box_gate(18.0, 0.0, 54.0, 34.0, 4.8, 3.6),
    Gate::box_gate(52.0, 2.0, 66.0, 90.0, 4.8, 3.6),
    Gate::box_gate(72.0, 0.0, 34.0, 162.0, 4.8, 3.6),
    Gate::box_gate(64.0, 0.0, -12.0, 196.0, 4.8, 3.6),
    Gate::box_gate(34.0, 1.8, -42.0, 245.0, 4.8, 3.6),
    Gate::box_gate(-14.0, 0.0, -38.0, 292.0, 4.8, 3.6),
    Gate::box_gate(-26.0, 0.0, -4.0, 342.0, 4.8, 3.6),
];

static MEADOW_STRUCTURES: [Structure; 44] = [
    // Start paddock tent & wooden fence line on left of start straight
    Structure::aabb(-13.0, 0.0, -4.0, -9.0, 3.4, 2.0, StructureColor::RedWhite),
    Structure::aabb(-8.5, 0.0, 6.0, -8.0, 1.35, 24.0, StructureColor::DarkWood),
    Structure::aabb(8.0, 0.0, 10.0, 8.5, 1.35, 28.0, StructureColor::DarkWood),

    // Hay bale chicane & stack between Gate 0 -> Gate 1
    Structure::aabb(5.5, 0.0, 34.0, 8.2, 1.4, 36.2, StructureColor::Hay),
    Structure::aabb(8.4, 0.0, 34.0, 11.1, 1.4, 36.2, StructureColor::Hay),
    Structure::aabb(6.9, 1.4, 34.1, 9.6, 2.7, 36.1, StructureColor::Hay),
    Structure::aabb(-4.5, 0.0, 42.0, -2.0, 1.4, 44.5, StructureColor::Hay),

    // Tree 1 (left of Gate 0->1 straight — fly under canopy or powerloop over!)
    Structure::aabb(-11.5, 0.0, 36.0, -10.5, 4.2, 37.0, StructureColor::DarkWood),
    Structure::aabb(-13.8, 4.2, 33.7, -8.2, 7.8, 39.3, StructureColor::Foliage),

    // Tree 2 (infield oak between Gate 1 & center pylon)
    Structure::aabb(24.5, 0.0, 28.0, 25.6, 4.6, 29.1, StructureColor::DarkWood),
    Structure::aabb(21.8, 4.6, 25.3, 28.3, 8.8, 31.8, StructureColor::Foliage),

    // Mid-field landmark concrete silo + hay bales at base
    Structure::aabb(26.0, 0.0, 6.0, 30.0, 15.0, 10.0, StructureColor::Concrete),
    Structure::aabb(31.0, 0.0, 7.0, 33.6, 1.4, 9.2, StructureColor::Hay),
    Structure::aabb(22.2, 0.0, 6.5, 24.8, 1.4, 8.8, StructureColor::Hay),

    // North meadow tower + paddock fence guiding Gate 1 -> Gate 2
    Structure::aabb(32.0, 0.0, 74.0, 36.0, 16.0, 78.0, StructureColor::RedWhite),
    Structure::aabb(26.0, 0.0, 66.5, 42.0, 1.35, 67.0, StructureColor::DarkWood),
    Structure::aabb(35.0, 0.0, 50.0, 37.8, 1.4, 52.2, StructureColor::Hay),

    // Tree 3 (north-east corner grove between Gate 2 -> Gate 3)
    Structure::aabb(66.0, 0.0, 56.0, 67.1, 4.4, 57.1, StructureColor::DarkWood),
    Structure::aabb(63.4, 4.4, 53.4, 69.7, 8.4, 59.7, StructureColor::Foliage),

    // Tree 4 (inside east bend — great orbit / gap tree)
    Structure::aabb(49.5, 0.0, 42.0, 50.5, 4.0, 43.0, StructureColor::DarkWood),
    Structure::aabb(47.2, 4.0, 39.7, 52.8, 7.6, 45.3, StructureColor::Foliage),

    // East paddock fence & chicane barrier between Gate 3 -> Gate 4
    Structure::aabb(57.0, 0.0, 10.0, 59.5, 7.5, 14.0, StructureColor::Wood),
    Structure::aabb(78.5, 0.0, 4.0, 79.0, 1.4, 24.0, StructureColor::DarkWood),
    Structure::aabb(66.5, 0.0, 12.0, 69.2, 1.4, 14.2, StructureColor::Hay),
    Structure::aabb(66.5, 1.4, 12.1, 69.0, 2.7, 14.1, StructureColor::Hay),

    // Tree 5 (southeast grove between Gate 4 -> Gate 5)
    Structure::aabb(56.0, 0.0, -32.0, 57.1, 4.8, -30.9, StructureColor::DarkWood),
    Structure::aabb(53.2, 4.8, -34.8, 59.9, 9.2, -28.1, StructureColor::Foliage),

    // Southeast steel mast & south paddock fence
    Structure::aabb(72.0, 0.0, -34.0, 75.5, 13.0, -30.5, StructureColor::Steel),
    Structure::aabb(10.0, 0.0, -48.5, 28.0, 1.35, -48.0, StructureColor::DarkWood),

    // Hay pyramid on south backstretch (Gate 5 -> Gate 6)
    Structure::aabb(10.0, 0.0, -36.0, 12.8, 1.4, -33.8, StructureColor::Hay),
    Structure::aabb(13.0, 0.0, -36.0, 15.8, 1.4, -33.8, StructureColor::Hay),
    Structure::aabb(11.5, 1.4, -35.9, 14.3, 2.7, -33.9, StructureColor::Hay),

    // Tree 6 (south infield tree)
    Structure::aabb(16.0, 0.0, -18.0, 17.0, 4.2, -17.0, StructureColor::DarkWood),
    Structure::aabb(13.5, 4.2, -20.5, 19.5, 8.0, -14.5, StructureColor::Foliage),

    // South backstretch tower
    Structure::aabb(4.0, 0.0, -56.0, 7.5, 12.5, -52.5, StructureColor::Steel),

    // West meadow wooden barn (with raised gable roof tier) + fence
    Structure::aabb(-46.0, 0.0, -24.0, -38.0, 6.5, -14.0, StructureColor::Wood),
    Structure::aabb(-45.0, 6.5, -23.5, -39.0, 8.8, -14.5, StructureColor::RedWhite),
    Structure::aabb(-34.0, 0.0, -26.0, -33.5, 1.35, -10.0, StructureColor::DarkWood),

    // Hay stack near Gate 6 -> Gate 7
    Structure::aabb(-26.0, 0.0, -22.0, -23.2, 1.4, -19.8, StructureColor::Hay),

    // Tree 7 (west flank between Gate 7 -> Gate 0)
    Structure::aabb(-34.0, 0.0, 14.0, -33.0, 4.4, 15.0, StructureColor::DarkWood),
    Structure::aabb(-36.4, 4.4, 11.6, -30.6, 8.2, 17.4, StructureColor::Foliage),

    // Tree 8 (northwest corner tall tree)
    Structure::aabb(-24.0, 0.0, 44.0, -22.9, 5.0, 45.1, StructureColor::DarkWood),
    Structure::aabb(-26.8, 5.0, 41.2, -20.1, 9.5, 47.9, StructureColor::Foliage),

    // Northwest concrete pylon
    Structure::aabb(-40.0, 0.0, 38.0, -36.5, 14.0, 41.5, StructureColor::Concrete),
];

// ---------------------------------------------------------------------------
// Track 1: Figure 8 (tight crossover bridge, park trees, fences & hay bales)
// ---------------------------------------------------------------------------
static FIG8_GATES: [Gate; 8] = [
    Gate::box_gate(0.0, 0.0, 22.0, 0.0, 4.6, 3.5),
    Gate::box_gate(22.0, 2.2, 52.0, 38.0, 4.6, 3.5),
    Gate::box_gate(62.0, 4.0, 54.0, 115.0, 4.6, 3.5),
    Gate::box_gate(36.0, 1.5, 14.0, 215.0, 4.6, 3.5),
    Gate::box_gate(-24.0, 0.0, -32.0, 225.0, 4.6, 3.5),
    Gate::box_gate(-64.0, 3.0, 10.0, 315.0, 4.6, 3.5),
    Gate::box_gate(-42.0, 5.0, 54.0, 35.0, 4.6, 3.5),
    Gate::box_gate(-12.0, 0.0, -6.0, 15.0, 4.6, 3.5),
];

static FIG8_STRUCTURES: [Structure; 40] = [
    // Center crossover bridge (two steel pillars + overhead bridge deck at y=9.5..11.2)
    Structure::aabb(10.0, 0.0, 26.0, 12.5, 9.5, 29.0, StructureColor::Steel),
    Structure::aabb(-14.5, 0.0, 26.0, -12.0, 9.5, 29.0, StructureColor::Steel),
    Structure::aabb(-15.0, 9.5, 25.5, 13.0, 11.2, 29.5, StructureColor::Concrete),

    // Split-rail wooden fences lining the start & crossover chutes
    Structure::aabb(-8.0, 0.0, 6.0, -7.5, 1.35, 20.0, StructureColor::DarkWood),
    Structure::aabb(7.5, 0.0, 6.0, 8.0, 1.35, 20.0, StructureColor::DarkWood),
    Structure::aabb(16.0, 0.0, 38.0, 30.0, 1.35, 38.5, StructureColor::DarkWood),
    Structure::aabb(-34.0, 0.0, 38.0, -20.0, 1.35, 38.5, StructureColor::DarkWood),

    // East lobe apex tower + hay barriers
    Structure::aabb(74.0, 0.0, 56.0, 77.5, 18.0, 59.5, StructureColor::RedWhite),
    Structure::aabb(46.0, 0.0, 36.0, 48.8, 1.4, 38.2, StructureColor::Hay),
    Structure::aabb(49.0, 0.0, 36.0, 51.8, 1.4, 38.2, StructureColor::Hay),
    Structure::aabb(47.5, 1.4, 36.1, 50.3, 2.7, 38.1, StructureColor::Hay),

    // East lobe trees (center of east loop + outer flank)
    Structure::aabb(38.0, 0.0, 34.0, 39.0, 4.4, 35.0, StructureColor::DarkWood),
    Structure::aabb(35.4, 4.4, 31.4, 41.6, 8.4, 37.6, StructureColor::Foliage),
    Structure::aabb(66.0, 0.0, 24.0, 67.1, 4.8, 25.1, StructureColor::DarkWood),
    Structure::aabb(63.2, 4.8, 21.2, 69.9, 9.2, 27.9, StructureColor::Foliage),
    Structure::aabb(44.0, 0.0, 64.0, 45.0, 4.2, 65.0, StructureColor::DarkWood),
    Structure::aabb(41.6, 4.2, 61.6, 47.4, 7.8, 67.4, StructureColor::Foliage),

    // West lobe apex tower + hay barriers
    Structure::aabb(-76.0, 0.0, 14.0, -72.5, 17.5, 17.5, StructureColor::RedWhite),
    Structure::aabb(-46.0, 0.0, 24.0, -43.2, 1.4, 26.2, StructureColor::Hay),
    Structure::aabb(-43.0, 0.0, 24.0, -40.2, 1.4, 26.2, StructureColor::Hay),

    // West lobe trees (center of west loop + flank)
    Structure::aabb(-42.0, 0.0, 16.0, -41.0, 4.6, 17.0, StructureColor::DarkWood),
    Structure::aabb(-44.6, 4.6, 13.4, -38.4, 8.8, 19.6, StructureColor::Foliage),
    Structure::aabb(-62.0, 0.0, 42.0, -60.9, 4.5, 43.1, StructureColor::DarkWood),
    Structure::aabb(-64.8, 4.5, 39.2, -58.1, 8.8, 45.9, StructureColor::Foliage),
    Structure::aabb(-26.0, 0.0, 62.0, -25.0, 4.0, 63.0, StructureColor::DarkWood),
    Structure::aabb(-28.4, 4.0, 59.6, -22.6, 7.6, 65.4, StructureColor::Foliage),

    // South lobe outer pylon, wooden fence & hay stack
    Structure::aabb(-36.0, 0.0, -54.0, -32.5, 14.5, -50.5, StructureColor::Concrete),
    Structure::aabb(-46.0, 0.0, -26.0, -34.0, 1.35, -25.5, StructureColor::DarkWood),
    Structure::aabb(-6.0, 0.0, -30.0, -3.2, 1.4, -27.8, StructureColor::Hay),
    Structure::aabb(-6.0, 1.4, -29.9, -3.4, 2.7, -27.9, StructureColor::Hay),

    // South trees (framing Gate 3 -> Gate 4 -> Gate 5)
    Structure::aabb(16.0, 0.0, -14.0, 17.0, 4.4, -13.0, StructureColor::DarkWood),
    Structure::aabb(13.5, 4.4, -16.5, 19.5, 8.2, -10.5, StructureColor::Foliage),
    Structure::aabb(-46.0, 0.0, -12.0, -45.0, 4.2, -11.0, StructureColor::DarkWood),
    Structure::aabb(-48.4, 4.2, -14.4, -42.6, 7.8, -8.6, StructureColor::Foliage),

    // Southeast timber shed & fence
    Structure::aabb(46.0, 0.0, -28.0, 52.0, 6.5, -22.0, StructureColor::Wood),
    Structure::aabb(32.0, 0.0, -22.0, 44.0, 1.35, -21.5, StructureColor::DarkWood),

    // North apex marker & northwest pylon
    Structure::aabb(2.0, 0.0, 74.0, 5.5, 16.0, 77.5, StructureColor::Concrete),
    Structure::aabb(-58.0, 0.0, 68.0, -54.5, 13.5, 71.5, StructureColor::Steel),

    // Extra infield tree near north crossover
    Structure::aabb(6.0, 0.0, 48.0, 7.0, 4.2, 49.0, StructureColor::DarkWood),
    Structure::aabb(3.6, 4.2, 45.6, 9.4, 7.8, 51.4, StructureColor::Foliage),
];

// ---------------------------------------------------------------------------
// Track 2: Dive Tower (28m central tower + forest grove, ranch fences & hay)
// ---------------------------------------------------------------------------
static DIVE_GATES: [Gate; 7] = [
    Gate::box_gate(0.0, 0.0, 20.0, 0.0, 4.8, 3.8),
    Gate::box_gate(0.0, 12.0, 38.0, 0.0, 5.2, 4.2),
    Gate::dive_gate(0.0, 16.0, 56.0, 0.0, 6.5),
    Gate::box_gate(32.0, 0.0, 60.0, 82.0, 4.8, 3.6),
    Gate::box_gate(58.0, 1.8, 12.0, 175.0, 4.8, 3.6),
    Gate::box_gate(24.0, 0.0, -38.0, 240.0, 4.8, 3.6),
    Gate::box_gate(-34.0, 1.5, -12.0, 335.0, 4.8, 3.6),
];

static DIVE_STRUCTURES: [Structure; 43] = [
    // 26m Dive Tower core (sits cleanly beneath the crown — anchored at z=44..50)
    Structure::aabb(-3.0, 0.0, 44.0, 3.0, 26.0, 50.0, StructureColor::Concrete),
    // Tower top crown (sits cleanly on top of core at y=26.0..28.5)
    Structure::aabb(-4.5, 26.0, 42.5, 4.5, 28.5, 51.5, StructureColor::RedWhite),

    // Northeast underpass right off the dive pullout (pillars y=0..8.5, roof y=8.5..10.5)
    Structure::aabb(29.5, 0.0, 52.0, 32.0, 8.5, 54.5, StructureColor::Steel),
    Structure::aabb(29.5, 0.0, 65.5, 32.0, 8.5, 68.0, StructureColor::Steel),
    Structure::aabb(29.0, 8.5, 51.5, 32.5, 10.5, 68.5, StructureColor::Concrete),

    // Start approach fences & hay bales framing the tower run-up
    Structure::aabb(-9.5, 0.0, 8.0, -9.0, 1.35, 28.0, StructureColor::DarkWood),
    Structure::aabb(9.0, 0.0, 8.0, 9.5, 1.35, 28.0, StructureColor::DarkWood),
    Structure::aabb(10.5, 0.0, 32.0, 13.3, 1.4, 34.2, StructureColor::Hay),
    Structure::aabb(-13.5, 0.0, 34.0, -10.7, 1.4, 36.2, StructureColor::Hay),
    Structure::aabb(-13.5, 1.4, 34.1, -10.9, 2.7, 36.1, StructureColor::Hay),

    // Dive pullout hay stacks & fence along Gate 2 -> Gate 3 -> Gate 4
    Structure::aabb(14.0, 0.0, 64.0, 16.8, 1.4, 66.2, StructureColor::Hay),
    Structure::aabb(17.0, 0.0, 64.0, 19.8, 1.4, 66.2, StructureColor::Hay),
    Structure::aabb(42.0, 0.0, 44.0, 42.5, 1.35, 60.0, StructureColor::DarkWood),

    // East forest grove (4 trees between Gate 3 -> Gate 4 -> Gate 5 for awesome slalom!)
    Structure::aabb(48.0, 0.0, 42.0, 49.1, 4.8, 43.1, StructureColor::DarkWood),
    Structure::aabb(45.2, 4.8, 39.2, 51.9, 9.2, 45.9, StructureColor::Foliage),
    Structure::aabb(36.0, 0.0, 26.0, 37.0, 4.2, 27.0, StructureColor::DarkWood),
    Structure::aabb(33.5, 4.2, 23.5, 39.5, 8.0, 29.5, StructureColor::Foliage),
    Structure::aabb(68.0, 0.0, 28.0, 69.1, 5.2, 29.1, StructureColor::DarkWood),
    Structure::aabb(65.0, 5.2, 25.0, 72.1, 10.0, 32.1, StructureColor::Foliage),
    Structure::aabb(44.0, 0.0, -8.0, 45.0, 4.5, -7.0, StructureColor::DarkWood),
    Structure::aabb(41.4, 4.5, -10.6, 47.6, 8.6, -4.4, StructureColor::Foliage),

    // East relay tower + wooden supply barn + hay stack
    Structure::aabb(68.0, 0.0, -14.0, 71.5, 20.0, -10.5, StructureColor::RedWhite),
    Structure::aabb(54.0, 0.0, -26.0, 61.0, 6.2, -19.0, StructureColor::Wood),
    Structure::aabb(48.0, 0.0, -24.0, 50.8, 1.4, -21.8, StructureColor::Hay),
    Structure::aabb(48.0, 1.4, -23.9, 50.6, 2.7, -21.9, StructureColor::Hay),

    // South perimeter monolith, ranch fence & trees
    Structure::aabb(6.0, 0.0, -56.0, 10.5, 16.5, -51.5, StructureColor::Concrete),
    Structure::aabb(-14.0, 0.0, -40.5, 8.0, 1.35, -40.0, StructureColor::DarkWood),
    Structure::aabb(14.0, 0.0, -16.0, 15.0, 4.2, -15.0, StructureColor::DarkWood),
    Structure::aabb(11.6, 4.2, -18.4, 17.4, 7.8, -12.6, StructureColor::Foliage),
    Structure::aabb(-12.0, 0.0, -26.0, -10.9, 4.8, -24.9, StructureColor::DarkWood),
    Structure::aabb(-14.8, 4.8, -28.8, -8.1, 9.2, -22.1, StructureColor::Foliage),

    // Hay chicane between Gate 5 -> Gate 6
    Structure::aabb(-2.0, 0.0, -28.0, 0.8, 1.4, -25.8, StructureColor::Hay),
    Structure::aabb(1.0, 0.0, -28.0, 3.8, 1.4, -25.8, StructureColor::Hay),
    Structure::aabb(-0.5, 1.4, -27.9, 2.3, 2.7, -25.9, StructureColor::Hay),

    // West flank tower, ranch fence & trees between Gate 6 -> Gate 0
    Structure::aabb(-54.0, 0.0, 16.0, -49.5, 18.0, 20.5, StructureColor::Steel),
    Structure::aabb(-44.5, 0.0, -28.0, -44.0, 1.35, -8.0, StructureColor::DarkWood),
    Structure::aabb(-26.0, 0.0, 16.0, -25.0, 4.4, 17.0, StructureColor::DarkWood),
    Structure::aabb(-28.5, 4.4, 13.5, -22.5, 8.2, 19.5, StructureColor::Foliage),
    Structure::aabb(-38.0, 0.0, 42.0, -36.9, 5.0, 43.1, StructureColor::DarkWood),
    Structure::aabb(-40.8, 5.0, 39.2, -34.1, 9.5, 45.9, StructureColor::Foliage),

    // Northwest timber tower + hay stack
    Structure::aabb(-42.0, 0.0, 68.0, -38.0, 14.5, 72.0, StructureColor::Wood),
    Structure::aabb(-18.0, 0.0, 56.0, -15.2, 1.4, 58.2, StructureColor::Hay),
    Structure::aabb(-18.0, 0.0, 58.4, -15.2, 1.4, 60.6, StructureColor::Hay),
];

// ---------------------------------------------------------------------------
// Track 3: Bando Freestyle (concrete ruin, overgrown trees, barriers & crates)
// ---------------------------------------------------------------------------
static BANDO_GATES: [Gate; 6] = [
    Gate::box_gate(0.0, 0.0, 18.0, 0.0, 5.4, 4.2),
    Gate::box_gate(0.0, 7.5, 46.0, 0.0, 5.4, 4.2),
    Gate::dive_gate(32.0, 13.5, 52.0, 90.0, 6.5),
    Gate::box_gate(46.0, 0.0, -8.0, 180.0, 5.4, 4.2),
    Gate::box_gate(-38.0, 3.2, 26.0, 270.0, 5.4, 4.2),
    Gate::box_gate(-28.0, 0.0, -32.0, 155.0, 5.4, 4.2),
];

static BANDO_STRUCTURES: [Structure; 44] = [
    // Main bando hall left & right walls (y = 0..12.0; right wall anchored at x=11..14, z=24..64)
    Structure::aabb(-14.0, 0.0, 24.0, -11.0, 12.0, 64.0, StructureColor::Concrete),
    Structure::aabb(11.0, 0.0, 24.0, 14.0, 12.0, 64.0, StructureColor::Concrete),
    // Roof beams resting cleanly on top of the hall walls (y = 12.0..14.0, huge powerloop gap in middle!)
    Structure::aabb(-14.0, 12.0, 24.0, 14.0, 14.0, 30.0, StructureColor::Concrete),
    Structure::aabb(-14.0, 12.0, 58.0, 14.0, 14.0, 64.0, StructureColor::Concrete),

    // Concrete jersey barriers & wooden crates framing the hall entrance
    Structure::aabb(-10.0, 0.0, 10.0, -5.0, 1.3, 11.4, StructureColor::Concrete),
    Structure::aabb(5.0, 0.0, 10.0, 10.0, 1.3, 11.4, StructureColor::Concrete),
    Structure::aabb(-9.5, 0.0, 34.0, -7.2, 1.8, 36.5, StructureColor::Wood),
    Structure::aabb(7.2, 0.0, 48.0, 9.5, 2.0, 50.5, StructureColor::Wood),
    Structure::aabb(7.4, 2.0, 48.2, 9.3, 3.5, 50.3, StructureColor::Wood),

    // Tall concrete smokestack beside the dive gate (30m)
    Structure::aabb(40.0, 0.0, 49.0, 44.5, 30.0, 53.5, StructureColor::Concrete),

    // Steel powerloop gantry in the east yard (between dive gate & Gate 3)
    Structure::aabb(24.0, 0.0, 20.0, 26.2, 9.0, 22.2, StructureColor::Steel),
    Structure::aabb(38.0, 0.0, 20.0, 40.2, 9.0, 22.2, StructureColor::Steel),
    Structure::aabb(23.5, 9.0, 19.8, 40.7, 10.8, 22.4, StructureColor::Steel),

    // East yard concrete ruin wall, jersey barriers & stacked wooden pallets
    Structure::aabb(34.0, 0.0, -28.0, 37.5, 10.5, -14.0, StructureColor::Concrete),
    Structure::aabb(56.0, 0.0, 8.0, 60.0, 19.0, 12.0, StructureColor::Wood),
    Structure::aabb(48.0, 0.0, 24.0, 56.0, 1.4, 25.2, StructureColor::Concrete),
    Structure::aabb(26.0, 0.0, 4.0, 28.8, 1.6, 6.5, StructureColor::Wood),
    Structure::aabb(26.2, 1.6, 4.2, 28.6, 3.0, 6.3, StructureColor::Hay),

    // Overgrown courtyard trees (east & northeast ruins)
    Structure::aabb(24.0, 0.0, 34.0, 25.0, 4.4, 35.0, StructureColor::DarkWood),
    Structure::aabb(21.5, 4.4, 31.5, 27.5, 8.2, 37.5, StructureColor::Foliage),
    Structure::aabb(52.0, 0.0, 36.0, 53.1, 4.8, 37.1, StructureColor::DarkWood),
    Structure::aabb(49.2, 4.8, 33.2, 55.9, 9.2, 39.9, StructureColor::Foliage),
    Structure::aabb(24.0, 0.0, -24.0, 25.0, 4.2, -23.0, StructureColor::DarkWood),
    Structure::aabb(21.6, 4.2, -26.4, 27.4, 7.8, -20.6, StructureColor::Foliage),

    // West compound slalom steel pillars + chain-link/wood perimeter barricade
    Structure::aabb(-32.0, 0.0, 8.0, -29.2, 14.0, 10.8, StructureColor::Steel),
    Structure::aabb(-48.0, 0.0, 42.0, -45.2, 15.0, 44.8, StructureColor::Steel),
    Structure::aabb(-26.0, 0.0, 44.0, -23.5, 11.0, 46.5, StructureColor::Concrete),
    Structure::aabb(-50.5, 0.0, 2.0, -50.0, 1.6, 22.0, StructureColor::DarkWood),
    Structure::aabb(-28.0, 0.0, -10.0, -20.0, 1.4, -8.8, StructureColor::Concrete),

    // Overgrown west courtyard trees
    Structure::aabb(-36.0, 0.0, 46.0, -35.0, 4.4, 47.0, StructureColor::DarkWood),
    Structure::aabb(-38.5, 4.4, 43.5, -32.5, 8.2, 49.5, StructureColor::Foliage),
    Structure::aabb(-46.0, 0.0, -16.0, -45.0, 4.6, -15.0, StructureColor::DarkWood),
    Structure::aabb(-48.6, 4.6, -18.6, -42.4, 8.8, -12.4, StructureColor::Foliage),

    // South ruin outposts, low walls, crates & hay bales
    Structure::aabb(-14.0, 0.0, -48.0, -9.5, 13.0, -43.5, StructureColor::Concrete),
    Structure::aabb(14.0, 0.0, -46.0, 18.0, 10.5, -42.0, StructureColor::Steel),
    Structure::aabb(-8.0, 0.0, -36.0, 6.0, 1.5, -34.6, StructureColor::Concrete),
    Structure::aabb(8.0, 0.0, -22.0, 10.8, 1.4, -19.8, StructureColor::Hay),
    Structure::aabb(11.0, 0.0, -22.0, 13.8, 1.4, -19.8, StructureColor::Hay),
    Structure::aabb(-16.0, 0.0, -18.0, -13.2, 1.6, -15.5, StructureColor::Wood),

    // North crane pylon, fence & trees behind bando hall
    Structure::aabb(-14.0, 0.0, 78.0, -10.0, 18.5, 82.0, StructureColor::RedWhite),
    Structure::aabb(6.0, 0.0, 74.0, 22.0, 1.4, 74.6, StructureColor::DarkWood),
    Structure::aabb(12.0, 0.0, 80.0, 13.1, 4.8, 81.1, StructureColor::DarkWood),
    Structure::aabb(9.2, 4.8, 77.2, 15.9, 9.2, 83.9, StructureColor::Foliage),
    Structure::aabb(-26.0, 0.0, 66.0, -23.2, 1.6, 68.5, StructureColor::Wood),
];

pub static TRACKS: [Track; 4] = [
    Track {
        name: "Meadow Loop",
        subtitle: "8 gates - farm, trees & hay",
        start_pos: Vec3::new(0.0, 0.35, 0.0),
        start_yaw: 0.0,
        gates: &MEADOW_GATES,
        structures: &MEADOW_STRUCTURES,
    },
    Track {
        name: "Figure 8",
        subtitle: "8 gates - bridge & park",
        start_pos: Vec3::new(0.0, 0.35, 0.0),
        start_yaw: 0.0,
        gates: &FIG8_GATES,
        structures: &FIG8_STRUCTURES,
    },
    Track {
        name: "Dive Tower",
        subtitle: "28m tower, grove & ranch",
        start_pos: Vec3::new(0.0, 0.35, 0.0),
        start_yaw: 0.0,
        gates: &DIVE_GATES,
        structures: &DIVE_STRUCTURES,
    },
    Track {
        name: "Bando",
        subtitle: "Ruin hall, trees & yard",
        start_pos: Vec3::new(0.0, 0.35, 0.0),
        start_yaw: 0.0,
        gates: &BANDO_GATES,
        structures: &BANDO_STRUCTURES,
    },
];

#[cfg(test)]
mod tests {
    use super::*;

    fn point_inside_expanded(p: Vec3, s: &Structure, pad: f32) -> bool {
        p.x >= s.min.x - pad
            && p.x <= s.max.x + pad
            && p.y >= s.min.y - pad
            && p.y <= s.max.y + pad
            && p.z >= s.min.z - pad
            && p.z <= s.max.z + pad
    }

    #[test]
    fn test_all_tracks_have_dense_obstacles_with_clear_spawn_and_gates() {
        for track in &TRACKS {
            assert!(
                track.structures.len() >= 40,
                "Track '{}' should be richly populated with obstacles, got {}",
                track.name,
                track.structures.len()
            );
            let has_trees = track.structures.iter().any(|s| s.color == StructureColor::Foliage);
            let has_fences = track.structures.iter().any(|s| s.color == StructureColor::DarkWood);
            let has_hay = track.structures.iter().any(|s| s.color == StructureColor::Hay);
            assert!(has_trees, "Track '{}' should include trees", track.name);
            assert!(has_fences, "Track '{}' should include fences/wood posts", track.name);
            assert!(has_hay, "Track '{}' should include hay bales", track.name);

            // Spawn point must have clean takeoff clearance
            for s in track.structures {
                assert!(
                    !point_inside_expanded(track.start_pos, s, 3.0),
                    "Track '{}' structure {:?} intrudes on start pad",
                    track.name,
                    s
                );
            }

            // Every gate center must remain unobstructed
            for (idx, g) in track.gates.iter().enumerate() {
                let c = g.center();
                for s in track.structures {
                    assert!(
                        !point_inside_expanded(c, s, 1.2),
                        "Track '{}' gate {} at {:?} is blocked by structure {:?}",
                        track.name,
                        idx,
                        c,
                        s
                    );
                }
            }
        }
    }
}
