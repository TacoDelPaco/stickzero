//! 240 Hz 6-DOF FPV quadcopter physics engine with Betaflight Actual rates,
//! Acro & Angle modes, motor spool lag, ground effect, obstacle collisions,
//! turtle mode, battery sag simulation, and freestyle trick tracking.

use crate::config::{Config, FlightMode};
use crate::math::{clampf, Quat, Vec3};
use crate::tracks::Track;

pub const PHYSICS_HZ: f32 = 240.0;
pub const PHYSICS_DT: f32 = 1.0 / PHYSICS_HZ;
const GRAVITY: f32 = 9.81;
const QUAD_RADIUS: f32 = 0.25;

/// Normalized RC stick inputs.
#[derive(Copy, Clone, Debug, Default)]
pub struct StickInput {
    /// Throttle: 0.0 ..= 1.0
    pub throttle: f32,
    /// Yaw: -1.0 (left) ..= +1.0 (right)
    pub yaw: f32,
    /// Pitch: -1.0 (nose up / pull back) ..= +1.0 (nose down / push forward)
    pub pitch: f32,
    /// Roll: -1.0 (left) ..= +1.0 (right)
    pub roll: f32,
}

#[derive(Clone, Debug)]
pub struct QuadState {
    pub pos: Vec3,
    pub vel: Vec3,
    pub rot: Quat,
    /// Body-frame angular velocity (rad/s): x=pitch, y=yaw, z=roll
    pub omega: Vec3,
    /// Smoothed effective motor throttle (0..1)
    pub motor_thr: f32,
    pub on_ground: bool,
    pub inverted: bool,
    pub hit_flash: f32,

    // Battery simulation
    pub cell_voltage: f32,
    pub mah_used: f32,
    pub battery_empty: bool,

    // Race / lap state
    pub next_gate: usize,
    pub lap: u32,
    pub lap_time: f32,
    pub last_lap_ms: u32,
    pub lap_started: bool,

    // Freestyle trick state
    pub score: u32,
    pub combo_text: &'static str,
    pub combo_timer: f32,
    pub flip_accum: f32,
    pub roll_accum: f32,
    pub inverted_time: f32,
}

impl QuadState {
    pub fn spawn(track: &Track) -> Self {
        Self {
            pos: track.start_pos,
            vel: Vec3::ZERO,
            rot: Quat::from_axis_angle(Vec3::UP, track.start_yaw),
            omega: Vec3::ZERO,
            motor_thr: 0.0,
            on_ground: true,
            inverted: false,
            hit_flash: 0.0,
            cell_voltage: 4.20,
            mah_used: 0.0,
            battery_empty: false,
            next_gate: 0,
            lap: 1,
            lap_time: 0.0,
            last_lap_ms: 0,
            lap_started: false,
            score: 0,
            combo_text: "",
            combo_timer: 0.0,
            flip_accum: 0.0,
            roll_accum: 0.0,
            inverted_time: 0.0,
        }
    }

    pub fn respawn_at_start(&mut self, track: &Track) {
        let best_score = self.score;
        *self = Self::spawn(track);
        self.score = best_score;
    }

    pub fn speed_kmh(&self) -> f32 {
        self.vel.len() * 3.6
    }

    /// Betaflight Actual rates formula: returns target angular rate in rad/s.
    #[inline]
    pub fn betaflight_actual(stick: f32, center_sens: f32, max_rate: f32, expo: f32) -> f32 {
        let s = clampf(stick, -1.0, 1.0);
        let a = s.abs();
        let expof = a * (a.powi(3) * expo + a * (1.0 - expo));
        let deg_s = center_sens * s + (max_rate - center_sens) * expof.copysign(s);
        deg_s * (std::f32::consts::PI / 180.0)
    }

    /// Advance physics by one fixed substep (`PHYSICS_DT`).
    pub fn step(&mut self, input: StickInput, cfg: &mut Config, track: &Track) {
        let dt = PHYSICS_DT;
        let params = cfg.quad.params();

        if self.hit_flash > 0.0 {
            self.hit_flash = (self.hit_flash - dt).max(0.0);
        }
        if self.combo_timer > 0.0 {
            self.combo_timer = (self.combo_timer - dt).max(0.0);
        }

        let prev_pos = self.pos;

        // 1. Motor spool lag (first-order filter)
        let target_thr = if self.battery_empty && cfg.battery_sim {
            0.0
        } else {
            clampf(input.throttle, 0.0, 1.0)
        };
        let alpha = dt / (params.motor_tau + dt);
        self.motor_thr += (target_thr - self.motor_thr) * alpha;

        // 2. Battery voltage sag & depletion
        if cfg.battery_sim {
            let amps = 1.5 + params.full_amp * self.motor_thr * self.motor_thr;
            self.mah_used += amps * (dt / 3.6);
            let soc = (1.0 - self.mah_used / params.mah).clamp(0.0, 1.0);
            let resting = 3.30 + 0.90 * soc;
            let sag = 0.35 * self.motor_thr * self.motor_thr;
            self.cell_voltage = (resting - sag).max(3.0);
            if soc <= 0.0 {
                self.battery_empty = true;
            }
        } else {
            self.cell_voltage = 4.20;
            self.battery_empty = false;
        }

        // 3. Determine orientation & ground status
        let (right, up, forward) = self.rot.basis();
        self.inverted = up.y < -0.25;

        // Turtle mode: if upside down on ground, pushing roll or pitch flips the quad upright!
        if self.on_ground && self.inverted && (input.roll.abs() > 0.65 || input.pitch.abs() > 0.65) {
            let yaw = forward.x.atan2(forward.z);
            self.rot = Quat::from_axis_angle(Vec3::UP, yaw);
            self.pos.y = 0.5;
            self.vel = Vec3::new(0.0, 2.2, 0.0);
            self.omega = Vec3::ZERO;
            self.inverted = false;
            self.combo_text = "TURTLE FLIP";
            self.combo_timer = 1.2;
        }

        // 4. Target body angular rates (rad/s)
        let (center_sens, max_rate, expo) = cfg.rates.actual_rates();
        let target_omega = match cfg.flight_mode {
            FlightMode::Acro => {
                // In our Y-up +Z-forward right-handed frame:
                // +X is right (positive rotation around +X pitches nose DOWN)
                // +Y is up (positive rotation around +Y yaws RIGHT? Actually right-hand rule around +Y turns +Z toward +X = RIGHT! Wait: +Y x +Z = +X, so +Y rot turns +Z toward +X = RIGHT!)
                // +Z is forward (positive rotation around +Z rolls LEFT; so roll right is -Z)
                let pitch_rate = Self::betaflight_actual(input.pitch, center_sens, max_rate, expo);
                let yaw_rate = Self::betaflight_actual(input.yaw, center_sens * 0.85, max_rate * 0.82, expo);
                let roll_rate = -Self::betaflight_actual(input.roll, center_sens, max_rate, expo);
                Vec3::new(pitch_rate, yaw_rate, roll_rate)
            }
            FlightMode::Angle => {
                // Self-leveling: max 45 deg tilt
                let max_tilt = 45.0_f32.to_radians();
                let desired_pitch = clampf(input.pitch, -1.0, 1.0) * max_tilt;
                let desired_roll = clampf(input.roll, -1.0, 1.0) * max_tilt;

                // Measure current pitch/roll relative to world up
                let cur_pitch = (-forward.y).asin();
                let cur_roll = (-right.y).asin();
                let pitch_err = desired_pitch - cur_pitch;
                let roll_err = desired_roll - cur_roll;

                let yaw_rate = Self::betaflight_actual(input.yaw, center_sens * 0.85, max_rate * 0.7, expo);
                Vec3::new(
                    clampf(pitch_err * 7.5, -6.0, 6.0),
                    yaw_rate,
                    clampf(-roll_err * 7.5, -6.0, 6.0),
                )
            }
        };

        // Fast PID-like rotational tracking (airmode authority even at low throttle)
        let rot_resp = if self.on_ground && self.motor_thr < 0.08 {
            2.0
        } else {
            24.0
        };
        let rot_alpha = (rot_resp * dt).min(1.0);
        self.omega = self.omega.add(target_omega.sub(self.omega).scale(rot_alpha));

        // Integrate quaternion
        if !self.on_ground || self.motor_thr > 0.08 {
            self.rot = self.rot.integrate_body_omega(self.omega, dt);
        }

        // 5. Forces: Thrust + Ground Effect + Quadratic Drag + Gravity
        let (_, up_now, _) = self.rot.basis();
        // Nonlinear prop thrust curve (feels like real Betaflight throttle)
        let thr_curve = self.motor_thr.powf(1.35);
        let sag_factor = if cfg.battery_sim {
            (self.cell_voltage / 4.0).clamp(0.72, 1.05)
        } else {
            1.0
        };
        // Ground effect cushion below 1.4 m
        let ge = if self.pos.y < 1.4 && up_now.y > 0.2 {
            1.0 + 0.16 * (1.0 - self.pos.y / 1.4)
        } else {
            1.0
        };
        let thrust_mag = (params.idle_accel * self.motor_thr.signum()
            + params.max_accel * thr_curve)
            * sag_factor
            * ge;

        let thrust_accel = up_now.scale(thrust_mag);
        let speed = self.vel.len();
        let drag_accel = self.vel.scale(-params.drag * speed - 0.12);
        let gravity_accel = Vec3::new(0.0, -GRAVITY, 0.0);

        let total_accel = thrust_accel.add(drag_accel).add(gravity_accel);
        self.vel = self.vel.add(total_accel.scale(dt));
        self.pos = self.pos.add(self.vel.scale(dt));

        // 6. Ground collision
        if self.pos.y <= QUAD_RADIUS {
            self.pos.y = QUAD_RADIUS;
            if self.vel.y < -6.0 {
                self.hit_flash = 0.25;
            }
            if self.vel.y < 0.0 {
                self.vel.y = -self.vel.y * 0.15;
                if self.vel.y.abs() < 0.4 {
                    self.vel.y = 0.0;
                }
            }
            // Ground friction
            self.vel.x *= 1.0 - (6.0 * dt).min(0.9);
            self.vel.z *= 1.0 - (6.0 * dt).min(0.9);
            self.on_ground = true;
            self.flip_accum = 0.0;
            self.roll_accum = 0.0;
            self.inverted_time = 0.0;
        } else {
            self.on_ground = false;
        }

        // 7. Structure AABB collisions
        for s in track.structures {
            let ex_min = Vec3::new(s.min.x - QUAD_RADIUS, s.min.y - QUAD_RADIUS, s.min.z - QUAD_RADIUS);
            let ex_max = Vec3::new(s.max.x + QUAD_RADIUS, s.max.y + QUAD_RADIUS, s.max.z + QUAD_RADIUS);
            if self.pos.x > ex_min.x
                && self.pos.x < ex_max.x
                && self.pos.y > ex_min.y
                && self.pos.y < ex_max.y
                && self.pos.z > ex_min.z
                && self.pos.z < ex_max.z
            {
                // Push out along minimum penetration axis
                let dx0 = self.pos.x - ex_min.x;
                let dx1 = ex_max.x - self.pos.x;
                let dy0 = self.pos.y - ex_min.y;
                let dy1 = ex_max.y - self.pos.y;
                let dz0 = self.pos.z - ex_min.z;
                let dz1 = ex_max.z - self.pos.z;
                let min_pen = dx0.min(dx1).min(dy0).min(dy1).min(dz0).min(dz1);
                if min_pen == dx0 {
                    self.pos.x = ex_min.x;
                    self.vel.x = -self.vel.x * 0.35;
                } else if min_pen == dx1 {
                    self.pos.x = ex_max.x;
                    self.vel.x = -self.vel.x * 0.35;
                } else if min_pen == dy0 {
                    self.pos.y = ex_min.y;
                    self.vel.y = -self.vel.y * 0.35;
                } else if min_pen == dy1 {
                    self.pos.y = ex_max.y;
                    self.vel.y = -self.vel.y * 0.35;
                } else if min_pen == dz0 {
                    self.pos.z = ex_min.z;
                    self.vel.z = -self.vel.z * 0.35;
                } else {
                    self.pos.z = ex_max.z;
                    self.vel.z = -self.vel.z * 0.35;
                }
                self.hit_flash = 0.28;
            }
        }

        // World boundary soft fence (180m radius)
        let r_sq = self.pos.x * self.pos.x + self.pos.z * self.pos.z;
        if r_sq > 180.0 * 180.0 {
            let r = r_sq.sqrt();
            let scale = 180.0 / r;
            self.pos.x *= scale;
            self.pos.z *= scale;
            self.vel.x *= -0.3;
            self.vel.z *= -0.3;
        }
        if self.pos.y > 95.0 {
            self.pos.y = 95.0;
            self.vel.y = self.vel.y.min(0.0);
        }

        // 8. Race gate progression & lap timing
        if !self.lap_started && self.motor_thr > 0.15 && self.pos.y > 0.45 {
            self.lap_started = true;
        }
        if self.lap_started {
            self.lap_time += dt;
        }
        if !track.gates.is_empty() {
            let g = &track.gates[self.next_gate];
            if g.check_pass(prev_pos, self.pos) {
                self.next_gate += 1;
                self.score += 100;
                if self.next_gate >= track.gates.len() {
                    self.next_gate = 0;
                    let ms = (self.lap_time * 1000.0) as u32;
                    self.last_lap_ms = ms;
                    let ti = cfg.track_index.min(3);
                    if ms > 1000 && (cfg.best_laps_ms[ti] == 0 || ms < cfg.best_laps_ms[ti]) {
                        cfg.best_laps_ms[ti] = ms;
                        cfg.save();
                        self.combo_text = "NEW BEST LAP!";
                        self.combo_timer = 2.0;
                    } else {
                        self.combo_text = "LAP COMPLETE";
                        self.combo_timer = 1.6;
                    }
                    self.lap += 1;
                    self.lap_time = 0.0;
                }
            }
        }

        // 9. Freestyle trick detection
        if !self.on_ground && self.pos.y > 1.5 {
            self.flip_accum += self.omega.x * dt;
            self.roll_accum += self.omega.z * dt;
            if self.flip_accum.abs() >= std::f32::consts::TAU * 0.92 {
                self.flip_accum = 0.0;
                self.score += 250;
                self.combo_text = "POWER FLIP +250";
                self.combo_timer = 1.2;
            }
            if self.roll_accum.abs() >= std::f32::consts::TAU * 0.92 {
                self.roll_accum = 0.0;
                self.score += 200;
                self.combo_text = "BARREL ROLL +200";
                self.combo_timer = 1.2;
            }
            if self.inverted {
                self.inverted_time += dt;
                if self.inverted_time >= 1.2 {
                    self.inverted_time = 0.0;
                    self.score += 150;
                    self.combo_text = "INVERTED +150";
                    self.combo_timer = 1.0;
                }
            } else {
                self.inverted_time = 0.0;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tracks::TRACKS;

    #[test]
    fn test_hover_and_climb() {
        let mut cfg = Config::default();
        let track = &TRACKS[0];
        let mut quad = QuadState::spawn(track);
        let input = StickInput {
            throttle: 0.7,
            yaw: 0.0,
            pitch: 0.0,
            roll: 0.0,
        };
        for _ in 0..240 {
            quad.step(input, &mut cfg, track);
        }
        assert!(quad.pos.y > 5.0, "Quad should climb with 70% throttle, got y={}", quad.pos.y);
    }

    #[test]
    fn test_acro_flip() {
        let mut cfg = Config::default();
        let track = &TRACKS[0];
        let mut quad = QuadState::spawn(track);
        quad.pos.y = 20.0;
        let input = StickInput {
            throttle: 0.4,
            yaw: 0.0,
            pitch: 1.0,
            roll: 0.0,
        };
        for _ in 0..180 {
            quad.step(input, &mut cfg, track);
        }
        assert!(quad.score >= 250, "Full pitch should register a flip trick");
    }
}
