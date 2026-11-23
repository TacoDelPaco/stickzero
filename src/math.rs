//! Minimal 3D math primitives (Vec3, Quat, Mat3) optimized for Cortex-A53 f32.

#[derive(Copy, Clone, Debug, Default)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub const ZERO: Self = Self { x: 0.0, y: 0.0, z: 0.0 };
    pub const UP: Self = Self { x: 0.0, y: 1.0, z: 0.0 };
    pub const RIGHT: Self = Self { x: 1.0, y: 0.0, z: 0.0 };
    pub const FORWARD: Self = Self { x: 0.0, y: 0.0, z: 1.0 };

    #[inline(always)]
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    #[inline(always)]
    pub fn add(self, rhs: Self) -> Self {
        Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }

    #[inline(always)]
    pub fn sub(self, rhs: Self) -> Self {
        Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }

    #[inline(always)]
    pub fn scale(self, s: f32) -> Self {
        Self::new(self.x * s, self.y * s, self.z * s)
    }

    #[inline(always)]
    pub fn dot(self, rhs: Self) -> f32 {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }

    #[inline(always)]
    pub fn cross(self, rhs: Self) -> Self {
        Self::new(
            self.y * rhs.z - self.z * rhs.y,
            self.z * rhs.x - self.x * rhs.z,
            self.x * rhs.y - self.y * rhs.x,
        )
    }

    #[inline(always)]
    pub fn len_sq(self) -> f32 {
        self.dot(self)
    }

    #[inline(always)]
    pub fn len(self) -> f32 {
        self.len_sq().sqrt()
    }

    #[inline(always)]
    pub fn normalized(self) -> Self {
        let l = self.len();
        if l > 1e-6 {
            self.scale(1.0 / l)
        } else {
            Self::ZERO
        }
    }
}

/// Unit quaternion representing orientation from body space to world space.
/// Coordinate system (right-handed, Y-up):
///   +X = right, +Y = up, +Z = forward.
#[derive(Copy, Clone, Debug)]
pub struct Quat {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Quat {
    pub const IDENTITY: Self = Self { w: 1.0, x: 0.0, y: 0.0, z: 0.0 };

    #[inline(always)]
    pub const fn new(w: f32, x: f32, y: f32, z: f32) -> Self {
        Self { w, x, y, z }
    }

    #[inline(always)]
    pub fn from_axis_angle(axis: Vec3, angle: f32) -> Self {
        let half = angle * 0.5;
        let (s, c) = half.sin_cos();
        Self::new(c, axis.x * s, axis.y * s, axis.z * s)
    }

    #[inline(always)]
    pub fn mul(self, rhs: Self) -> Self {
        Self {
            w: self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
            x: self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            y: self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            z: self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
        }
    }

    #[inline(always)]
    pub fn normalized(self) -> Self {
        let n = (self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z).sqrt();
        if n > 1e-6 {
            let inv = 1.0 / n;
            Self::new(self.w * inv, self.x * inv, self.y * inv, self.z * inv)
        } else {
            Self::IDENTITY
        }
    }

    #[inline(always)]
    pub fn conjugate(self) -> Self {
        Self::new(self.w, -self.x, -self.y, -self.z)
    }

    /// Rotate a vector from body space into world space.
    #[inline(always)]
    pub fn rotate(self, v: Vec3) -> Vec3 {
        let qv = Vec3::new(self.x, self.y, self.z);
        let t = qv.cross(v).scale(2.0);
        v.add(t.scale(self.w)).add(qv.cross(t))
    }

    /// Rotate a vector from world space into body space.
    #[inline(always)]
    pub fn inv_rotate(self, v: Vec3) -> Vec3 {
        self.conjugate().rotate(v)
    }

    /// Integrate body-frame angular velocity omega (rad/s) over dt.
    #[inline(always)]
    pub fn integrate_body_omega(self, omega: Vec3, dt: f32) -> Self {
        let half = 0.5 * dt;
        let dq = Quat::new(0.0, omega.x * half, omega.y * half, omega.z * half);
        let prod = self.mul(dq);
        Quat::new(
            self.w + prod.w,
            self.x + prod.x,
            self.y + prod.y,
            self.z + prod.z,
        )
        .normalized()
    }

    /// Extract basis vectors (right, up, forward) in world space.
    #[inline(always)]
    pub fn basis(self) -> (Vec3, Vec3, Vec3) {
        (
            self.rotate(Vec3::RIGHT),
            self.rotate(Vec3::UP),
            self.rotate(Vec3::FORWARD),
        )
    }
}

#[inline(always)]
pub fn clampf(v: f32, lo: f32, hi: f32) -> f32 {
    if v < lo {
        lo
    } else if v > hi {
        hi
    } else {
        v
    }
}
