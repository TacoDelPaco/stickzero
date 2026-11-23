//! 240 Hz 6-DOF FPV quadcopter physics engine with Betaflight Actual rates,
//! Acro & Angle modes, motor spool lag, ground effect, obstacle collisions,
//! turtle mode, battery sag simulation, and freestyle trick tracking.

use crate::config::{BatteryPack, Config, FlightMode, GameMode};
use crate::math::{clampf, Quat, Vec3};
use crate::tracks::{GateKind, Track};

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
    pub prop_wash_phase: f32,
    pub prop_wash_level: f32,

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
    pub gate_cooldowns: [f32; 16],

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
            prop_wash_phase: 0.0,
            prop_wash_level: 0.0,
            cell_voltage: 4.35, // 1S LiHV fresh charge (4.35V)
            mah_used: 0.0,
            battery_empty: false,
            next_gate: 0,
            lap: 1,
            lap_time: 0.0,
            last_lap_ms: 0,
            lap_started: false,
            gate_cooldowns: [0.0; 16],
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
        let batt_on = cfg.battery_pack != BatteryPack::Off;
        let (pack_mah, pack_mass_g, pack_sag) = cfg.battery_pack.specs(&params);
        let mass_ratio = (params.dry_mass_g + pack_mass_g) / (params.dry_mass_g + params.ref_batt_g);

        // 1. Motor spool lag (first-order filter)
        let target_thr = if self.battery_empty && batt_on {
            0.0
        } else {
            clampf(input.throttle, 0.0, 1.0)
        };
        let alpha = dt / (params.motor_tau + dt);
        self.motor_thr += (target_thr - self.motor_thr) * alpha;

        // 2. Battery voltage sag & depletion (LiHV 4.35V -> 3.30V for Lava II packs)
        if batt_on {
            let amps = (1.2 + params.full_amp * self.motor_thr * self.motor_thr) * mass_ratio.sqrt();
            self.mah_used += amps * (dt / 3.6);
            let soc = (1.0 - self.mah_used / pack_mah).clamp(0.0, 1.0);
            let full_v = match cfg.battery_pack {
                BatteryPack::LavaII580 | BatteryPack::LavaII680 => 4.35,
                _ => 4.20,
            };
            let resting = 3.30 + (full_v - 3.30) * soc;
            let sag = pack_sag * self.motor_thr * self.motor_thr;
            self.cell_voltage = (resting - sag).max(3.0);
            if soc <= 0.0 {
                self.battery_empty = true;
            }
        } else {
            self.cell_voltage = 4.35;
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
        let (center_sens, max_rate, expo) = cfg.actual_rates();
        let target_omega = match cfg.flight_mode {
            FlightMode::Acro => {
                let pitch_rate = Self::betaflight_actual(input.pitch, center_sens, max_rate, expo);
                let yaw_rate = Self::betaflight_actual(input.yaw, center_sens * 0.85, max_rate * 0.82, expo);
                let roll_rate = -Self::betaflight_actual(input.roll, center_sens, max_rate, expo);
                Vec3::new(pitch_rate, yaw_rate, roll_rate)
            }
            FlightMode::Angle => {
                let max_tilt = 45.0_f32.to_radians();
                let desired_pitch = clampf(input.pitch, -1.0, 1.0) * max_tilt;
                let desired_roll = clampf(input.roll, -1.0, 1.0) * max_tilt;

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

        // Fast PID-like rotational tracking (slightly heavier pack = slightly softer snap)
        let rot_resp = if self.on_ground && self.motor_thr < 0.08 {
            2.0
        } else {
            24.0 / mass_ratio.sqrt()
        };
        let rot_alpha = (rot_resp * dt).min(1.0);
        self.omega = self.omega.add(target_omega.sub(self.omega).scale(rot_alpha));

        // 4b. Realistic Stock Prop Wash (descending through own dirty air / dive pullout)
        let mut wash_thrust_loss = 1.0_f32;
        self.prop_wash_level = 0.0;
        if cfg.prop_wash && !self.on_ground && self.vel.y < -1.8 && up.y > 0.25 {
            let horiz_speed = (self.vel.x * self.vel.x + self.vel.z * self.vel.z).sqrt();
            let descent = ((-self.vel.y - 1.8) / 8.0).clamp(0.0, 1.0);
            let slow_factor = (1.0 - horiz_speed / 16.0).clamp(0.15, 1.0);
            let intensity = descent * slow_factor * (0.35 + 0.65 * self.motor_thr);
            self.prop_wash_level = intensity;
            self.prop_wash_phase += dt * 115.0; // ~18 Hz prop wash wobble
            let wobble_p = self.prop_wash_phase.sin() * intensity * 0.42;
            let wobble_r = (self.prop_wash_phase * 1.37).cos() * intensity * 0.55;
            self.omega.x += wobble_p;
            self.omega.z += wobble_r;
            wash_thrust_loss = 1.0 - intensity * 0.11;
        }

        // Integrate quaternion
        if !self.on_ground || self.motor_thr > 0.08 {
            self.rot = self.rot.integrate_body_omega(self.omega, dt);
        }

        // 5. Forces: Thrust (scaled by battery mass ratio & sag) + Ground Effect + Drag + Gravity
        let (_, up_now, _) = self.rot.basis();
        let thr_curve = self.motor_thr.powf(1.35);
        let sag_factor = if batt_on {
            (self.cell_voltage / 4.0).clamp(0.72, 1.06)
        } else {
            1.0
        };
        let ge = if self.pos.y < 1.4 && up_now.y > 0.2 {
            1.0 + 0.16 * (1.0 - self.pos.y / 1.4)
        } else {
            1.0
        };
        let thrust_mag = ((params.idle_accel * self.motor_thr.signum()
            + params.max_accel * thr_curve)
            / mass_ratio)
            * sag_factor
            * ge
            * wash_thrust_loss;

        let thrust_accel = up_now.scale(thrust_mag);
        let speed = self.vel.len();
        let drag_accel = self.vel.scale(-(params.drag / mass_ratio) * speed - 0.12);
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

        // 8. Gate / loop scoring & lap timing
        for cd in &mut self.gate_cooldowns {
            if *cd > 0.0 {
                *cd = (*cd - dt).max(0.0);
            }
        }
        if !self.lap_started && self.motor_thr > 0.15 && self.pos.y > 0.45 {
            self.lap_started = true;
        }
        if self.lap_started {
            self.lap_time += dt;
        }
        if !track.gates.is_empty() {
            match cfg.game_mode {
                GameMode::Race => {
                    let g = &track.gates[self.next_gate];
                    if g.check_pass(prev_pos, self.pos) {
                        self.next_gate += 1;
                        self.score += 100;
                        if self.next_gate >= track.gates.len() {
                            self.next_gate = 0;
                            self.score += 500;
                            let ms = (self.lap_time * 1000.0) as u32;
                            self.last_lap_ms = ms;
                            let ti = cfg.track_index.min(3);
                            if ms > 1000 && (cfg.best_laps_ms[ti] == 0 || ms < cfg.best_laps_ms[ti]) {
                                cfg.best_laps_ms[ti] = ms;
                                cfg.save();
                                self.combo_text = "NEW BEST LAP! +500";
                                self.combo_timer = 2.0;
                            } else {
                                self.combo_text = "LAP COMPLETE +500";
                                self.combo_timer = 1.6;
                            }
                            self.lap += 1;
                            self.lap_time = 0.0;
                        } else {
                            self.combo_text = "GATE +100";
                            self.combo_timer = 0.8;
                        }
                    }
                }
                GameMode::Freestyle => {
                    for (idx, g) in track.gates.iter().enumerate() {
                        if idx < self.gate_cooldowns.len()
                            && self.gate_cooldowns[idx] <= 0.0
                            && g.check_pass(prev_pos, self.pos)
                        {
                            self.gate_cooldowns[idx] = 1.2;
                            match g.kind {
                                GateKind::Dive => {
                                    self.score += 200;
                                    self.combo_text = "DIVE GATE +200";
                                }
                                _ => {
                                    self.score += 100;
                                    self.combo_text = "GATE +100";
                                }
                            }
                            self.combo_timer = 1.1;
                        }
                    }
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

    #[test]
    fn test_freestyle_gate_scoring() {
        let mut cfg = Config::default();
        cfg.game_mode = GameMode::Freestyle;
        let track = &TRACKS[0];
        let mut quad = QuadState::spawn(track);
        // Place quad right before gate 0 (at z=22) moving forward through it
        quad.pos = Vec3::new(0.0, 1.8, 21.7);
        quad.vel = Vec3::new(0.0, 0.0, 15.0);
        let input = StickInput { throttle: 0.3, yaw: 0.0, pitch: 0.0, roll: 0.0 };
        for _ in 0..30 {
            quad.step(input, &mut cfg, track);
        }
        assert!(quad.score >= 100, "Flying through a loop/gate in Freestyle should award points, got {}", quad.score);
    }
}
