//! 240 Hz 6-DOF FPV quadcopter physics engine with Betaflight Actual rates,
//! Acro / Horizon / Angle modes, StickTime-inspired aerodynamic inflow damping (climb thrust loss,
//! descent cushion, translational lift, rotor H-force grip), custom Drone Tuning sliders,
//! radio AUX Arm/Mode/Turtle support, solid gate & structure collisions, and post-flight stats.

use crate::config::{AuxBinding, BatteryPack, Config, FlightMode, GameMode, PropWashLevel};
use crate::input::AuxInputs;
use crate::math::{clampf, Quat, Vec3};
use crate::tracks::{GateKind, Track};

pub const PHYSICS_HZ: f32 = 240.0;
pub const PHYSICS_DT: f32 = 1.0 / PHYSICS_HZ;
/// ~1.08g planted gravity feel (similar to StickTime's 1.1g tuning)
const GRAVITY: f32 = 10.6;
const QUAD_RADIUS: f32 = 0.22;

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
    pub turtle_cooldown: f32,

    // Betaflight arming & mode state
    pub armed: bool,
    pub arm_blocked_throttle: bool,
    pub turtle_mode: bool,
    pub has_flown: bool,
    pub throttle_pct: u8,

    // Battery & post-flight telemetry
    pub cell_voltage: f32,
    pub min_cell_voltage: f32,
    pub mah_used: f32,
    pub battery_empty: bool,
    pub flight_time: f32,
    pub max_speed_kmh: f32,

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
    pub fn spawn(track: &Track, cfg: &Config) -> Self {
        let start_v = match cfg.battery_pack {
            BatteryPack::LavaII580 | BatteryPack::LavaII680 | BatteryPack::Off => 4.35,
            BatteryPack::Stock => 4.20,
        };
        let start_armed = cfg.aux_arm == AuxBinding::None;
        Self {
            pos: Vec3::new(track.start_pos.x, QUAD_RADIUS, track.start_pos.z),
            vel: Vec3::ZERO,
            rot: Quat::from_axis_angle(Vec3::UP, track.start_yaw),
            omega: Vec3::ZERO,
            motor_thr: 0.0,
            on_ground: true,
            inverted: false,
            hit_flash: 0.0,
            prop_wash_phase: 0.0,
            prop_wash_level: 0.0,
            turtle_cooldown: 0.0,
            armed: start_armed,
            arm_blocked_throttle: false,
            turtle_mode: false,
            has_flown: false,
            throttle_pct: 0,
            cell_voltage: start_v,
            min_cell_voltage: start_v,
            mah_used: 0.0,
            battery_empty: false,
            flight_time: 0.0,
            max_speed_kmh: 0.0,
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

    pub fn respawn_at_start(&mut self, track: &Track, cfg: &Config) {
        let best_score = self.score;
        *self = Self::spawn(track, cfg);
        self.score = best_score;
    }

    pub fn speed_kmh(&self) -> f32 {
        self.vel.len() * 3.6
    }

    /// Apply radio AUX switches / buttons and keyboard flight toggles.
    /// Returns `true` if the quad just transitioned from disarmed to armed.
    pub fn apply_aux(&mut self, aux: AuxInputs, input: StickInput, cfg: &mut Config) -> bool {
        let mut just_armed = false;

        // 1. Flight Mode switch / toggle (Acro / Horizon / Angle)
        if let Some(target) = aux.mode_switch {
            if cfg.flight_mode != target {
                cfg.flight_mode = target;
                self.combo_text = target.name();
                self.combo_timer = 0.8;
            }
        } else if aux.mode_toggle {
            cfg.flight_mode = cfg.flight_mode.next();
            self.combo_text = cfg.flight_mode.name();
            self.combo_timer = 0.8;
        }

        // 2. Turtle Mode switch / toggle
        if let Some(on) = aux.turtle_switch {
            self.turtle_mode = on;
        } else if aux.turtle_toggle {
            self.turtle_mode = !self.turtle_mode;
            self.combo_text = if self.turtle_mode { "TURTLE ARMED" } else { "TURTLE OFF" };
            self.combo_timer = 0.8;
        }

        // 3. Arm / Disarm switch / toggle with Betaflight throttle-low safety
        let want_arm = if let Some(sw) = aux.arm_switch {
            Some(sw)
        } else if aux.arm_toggle {
            Some(!self.armed)
        } else {
            None
        };

        if input.throttle <= 0.15 {
            self.arm_blocked_throttle = false;
        }

        if let Some(want) = want_arm {
            if want && !self.armed {
                if self.battery_empty && cfg.battery_pack != BatteryPack::Off {
                    // Wait for fresh pack
                } else if input.throttle > 0.15 && !self.turtle_mode {
                    self.arm_blocked_throttle = true;
                } else {
                    self.armed = true;
                    self.arm_blocked_throttle = false;
                    just_armed = true;
                }
            } else if !want && self.armed {
                self.armed = false;
                self.arm_blocked_throttle = false;
            }
        }

        just_armed
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

    /// Betaflight throttle mid + expo curve (flattens sensitivity around hover `mid=0.40`).
    #[inline]
    fn apply_throttle_expo(raw: f32, expo_pct: u8) -> f32 {
        let x = clampf(raw, 0.0, 1.0);
        let e = (expo_pct as f32 / 100.0).clamp(0.0, 0.65);
        if e < 1e-3 {
            return x;
        }
        let mid = 0.40_f32;
        if x < mid {
            let u = (x - mid) / mid; // -1..0
            mid + mid * ((1.0 - e) * u + e * u * u * u)
        } else {
            let u = (x - mid) / (1.0 - mid); // 0..1
            mid + (1.0 - mid) * ((1.0 - e) * u + e * u * u * u)
        }
    }

    /// Collide quad sphere against a 3D line segment (gate posts, crossbars, stilts).
    fn collide_segment(&mut self, a: Vec3, b: Vec3, tube_radius: f32) {
        let ab = b.sub(a);
        let len_sq = ab.len_sq();
        if len_sq < 1e-6 {
            return;
        }
        let t = clampf(self.pos.sub(a).dot(ab) / len_sq, 0.0, 1.0);
        let closest = a.add(ab.scale(t));
        let diff = self.pos.sub(closest);
        let dist_sq = diff.len_sq();
        let min_dist = QUAD_RADIUS + tube_radius;
        if dist_sq < min_dist * min_dist {
            let dist = dist_sq.sqrt();
            let normal = if dist > 1e-4 {
                diff.scale(1.0 / dist)
            } else {
                Vec3::UP
            };
            self.pos = closest.add(normal.scale(min_dist));
            let vn = self.vel.dot(normal);
            if vn < 0.0 {
                self.vel = self.vel.sub(normal.scale(vn * 1.30));
                self.vel = self.vel.scale(0.75);
                if vn < -2.0 {
                    self.omega.x += normal.z * 3.5;
                    self.omega.z -= normal.x * 3.5;
                    self.hit_flash = 0.25;
                }
            }
        }
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
        if self.turtle_cooldown > 0.0 {
            self.turtle_cooldown = (self.turtle_cooldown - dt).max(0.0);
        }

        let prev_pos = self.pos;
        let batt_on = cfg.battery_pack != BatteryPack::Off;
        let (pack_mah, pack_mass_g, pack_sag) = cfg.battery_pack.specs(&params);

        // Apply custom Drone Tuning multipliers ("The Zone"-style physics customization)
        let thrust_scale = cfg.thrust_pct as f32 / 100.0;
        let weight_scale = cfg.weight_pct as f32 / 100.0;
        let drag_scale = cfg.drag_pct as f32 / 100.0;
        let grip_scale = cfg.grip_pct as f32 / 100.0;
        let motor_scale = cfg.motor_pct as f32 / 100.0;

        let effective_dry_g = params.dry_mass_g * weight_scale;
        let mass_ratio = (effective_dry_g + pack_mass_g) / (params.dry_mass_g + params.ref_batt_g);
        let eff_motor_tau = (params.motor_tau / motor_scale).clamp(0.005, 0.05);

        self.throttle_pct = (clampf(input.throttle, 0.0, 1.0) * 100.0).round() as u8;

        // 1. Betaflight throttle expo + motor spool lag
        let shaped_thr = Self::apply_throttle_expo(input.throttle, cfg.thr_expo);
        let target_thr = if !self.armed || (self.battery_empty && batt_on) {
            0.0
        } else {
            shaped_thr
        };
        let alpha = dt / (eff_motor_tau + dt);
        self.motor_thr += (target_thr - self.motor_thr) * alpha;
        if !self.armed && self.motor_thr < 0.01 {
            self.motor_thr = 0.0;
        }

        // 2. Battery voltage sag & depletion (LiHV 4.35V -> 3.30V for Lava II packs)
        if batt_on {
            let draw_active = self.armed && !self.battery_empty;
            let amps = if draw_active {
                (1.1 + params.full_amp * thrust_scale * self.motor_thr * self.motor_thr) * mass_ratio.sqrt()
            } else {
                0.35
            };
            if draw_active {
                self.mah_used += amps * (dt / 3.6);
            }
            let soc = (1.0 - self.mah_used / pack_mah).clamp(0.0, 1.0);
            let full_v = match cfg.battery_pack {
                BatteryPack::LavaII580 | BatteryPack::LavaII680 => 4.35,
                _ => 4.20,
            };
            let resting = 3.30 + (full_v - 3.30) * soc;
            let sag = if draw_active {
                pack_sag * self.motor_thr * self.motor_thr
            } else {
                0.0
            };
            self.cell_voltage = (resting - sag).max(3.0);
            if self.armed && self.cell_voltage < self.min_cell_voltage {
                self.min_cell_voltage = self.cell_voltage;
            }
            if soc <= 0.0 {
                self.battery_empty = true;
                self.armed = false;
            }
        } else {
            self.cell_voltage = 4.35;
            self.battery_empty = false;
        }

        if self.armed {
            self.flight_time += dt;
            let spd = self.speed_kmh();
            if spd > self.max_speed_kmh {
                self.max_speed_kmh = spd;
            }
            if !self.on_ground && self.pos.y > QUAD_RADIUS + 0.25 {
                self.has_flown = true;
            }
        }

        // 3. Orientation & body-frame airflow decomposition (StickTime aerodynamic model!)
        let (right, up, forward) = self.rot.basis();
        self.inverted = up.y < -0.35;

        let v_up = self.vel.dot(up);
        let v_right = self.vel.dot(right);
        let v_fwd = self.vel.dot(forward);
        let edge_sq = v_right * v_right + v_fwd * v_fwd;
        let edge_speed = edge_sq.sqrt();

        // Turtle Mode ONLY activates when genuinely on the ground!
        let auto_turtle = cfg.aux_turtle == AuxBinding::None
            && self.on_ground
            && up.y < -0.55
            && input.throttle < 0.12;
        let turtle_ready = self.armed && self.on_ground && (self.turtle_mode || auto_turtle);
        let turtle_flipping = turtle_ready && (input.roll.abs() > 0.40 || input.pitch.abs() > 0.40);

        // Compute static + aerodynamic thrust (StickTime-style climb loss, descent cushion, translational lift)
        let thr_curve = self.motor_thr.powf(1.18); // Closer to StickTime whoop curve (0.92..1.4) + thr_expo around hover!
        let sag_factor = if batt_on {
            (self.cell_voltage / 4.0).clamp(0.72, 1.05)
        } else {
            1.0
        };
        let ge = if self.pos.y < 0.60 && up.y > 0.3 {
            let h = (self.pos.y - QUAD_RADIUS).max(0.0);
            1.0 + 0.06 * (1.0 - h / 0.38).clamp(0.0, 1.0) / (1.0 + edge_sq / 80.0)
        } else {
            1.0
        };
        let base_thrust = if !self.armed || turtle_ready {
            0.0
        } else {
            ((params.idle_accel * self.motor_thr.signum()
                + params.max_accel * thrust_scale * thr_curve)
                / mass_ratio)
                * sag_factor
                * ge
        };

        // StickTime-inspired axial inflow damping & translational lift:
        // Makes hovering feel naturally planted instead of slippery!
        let inflow_factor = if v_up > 0.0 {
            // Climbing through air reduces effective prop angle of attack -> self-damps upward drift
            1.0 / (1.0 + v_up * 0.045)
        } else {
            // Gentle descent cushions thrust slightly (peaks near -2.5 m/s), fades in fast dives
            let sink = -v_up;
            let x = (sink / 5.5).min(1.8);
            let cushion = 1.0 + 0.10 * (2.0 * x - x * x).max(-0.12);
            cushion.clamp(0.88, 1.11)
        };
        // Translational lift when flying forward/edgewise
        let trans_lift = 1.0 + 0.11 * (edge_sq / (edge_sq + 36.0)) * (1.0 - 0.5 * self.motor_thr.powi(3));

        // Smooth prop wash buildup (0.2s lag when descending vertically at low edgewise speed)
        let target_wash = match cfg.prop_wash {
            PropWashLevel::Off => 0.0,
            PropWashLevel::Stock | PropWashLevel::Max => {
                if self.armed && !self.on_ground && v_up < -0.6 && up.y > 0.25 {
                    let sink_f = ((-v_up - 0.6) / 6.0).clamp(0.0, 1.0);
                    let clear_f = 1.0 / (1.0 + edge_speed * 0.32);
                    sink_f * clear_f
                } else {
                    0.0
                }
            }
        };
        let wash_alpha = (dt / 0.20).min(1.0);
        self.prop_wash_level += (target_wash - self.prop_wash_level) * wash_alpha;
        let (wash_loss_coeff, wash_shake_coeff) = match cfg.prop_wash {
            PropWashLevel::Off => (0.0, 0.0),
            PropWashLevel::Stock => (0.09, 0.26),
            PropWashLevel::Max => (0.18, 0.52),
        };
        let wash_thrust_mult = 1.0 - self.prop_wash_level * wash_loss_coeff;

        let aero_thrust = base_thrust * inflow_factor * trans_lift * wash_thrust_mult;
        let vertical_lift = up.y * aero_thrust;
        let pinned_on_ground = self.on_ground && !turtle_flipping && (!self.armed || up.y < 0.25 || vertical_lift <= GRAVITY * 1.01);

        // 4. Rotational control (Acro / Horizon / Angle / Grounded Yaw / Turtle)
        let (center_sens, max_rate, expo) = cfg.actual_rates();

        if turtle_flipping {
            if self.turtle_cooldown <= 0.0 {
                self.combo_text = "TURTLE FLIP";
                self.combo_timer = 0.8;
                self.turtle_cooldown = 0.9;
            }
            if self.on_ground && self.vel.y < 0.45 {
                self.vel.y = 0.65;
            }
            let r_cmd = if input.roll.abs() > 0.40 { -input.roll.signum() * 5.8 } else { 0.0 };
            let p_cmd = if input.pitch.abs() > 0.40 { input.pitch.signum() * 5.8 } else { 0.0 };
            let target_omega = Vec3::new(p_cmd, 0.0, r_cmd);
            let rot_alpha = (16.0 * dt).min(1.0);
            self.omega = self.omega.add(target_omega.sub(self.omega).scale(rot_alpha));
            self.rot = self.rot.integrate_body_omega(self.omega, dt);
        } else if pinned_on_ground {
            let target_up = if up.y >= 0.0 { 1.0f32 } else { -1.0f32 };
            let cur_pitch = clampf(-forward.y * target_up, -1.0, 1.0).asin();
            let cur_roll = clampf(-right.y * target_up, -1.0, 1.0).asin();

            let yaw_rate = if self.armed && up.y >= 0.25 && !turtle_ready {
                Self::betaflight_actual(input.yaw, center_sens * 0.85, max_rate * 0.70, expo)
            } else {
                0.0
            };

            let ground_omega = Vec3::new(-cur_pitch * 18.0, yaw_rate, cur_roll * 18.0);
            self.rot = self.rot.integrate_body_omega(ground_omega, dt);
            self.omega = Vec3::new(0.0, yaw_rate, 0.0);
        } else {
            let acro_omega = Vec3::new(
                Self::betaflight_actual(input.pitch, center_sens, max_rate, expo),
                Self::betaflight_actual(input.yaw, center_sens * 0.85, max_rate * 0.82, expo),
                -Self::betaflight_actual(input.roll, center_sens, max_rate, expo),
            );

            let angle_omega = {
                let max_tilt = 45.0_f32.to_radians();
                let desired_pitch = clampf(input.pitch, -1.0, 1.0) * max_tilt;
                let desired_roll = clampf(input.roll, -1.0, 1.0) * max_tilt;
                // Handle inverted recovery cleanly in Horizon/Angle
                let (cur_pitch, cur_roll) = if up.y >= 0.0 {
                    (
                        clampf(-forward.y, -1.0, 1.0).asin(),
                        clampf(-right.y, -1.0, 1.0).asin(),
                    )
                } else {
                    // Upside down -> command fast roll back upright
                    let roll_sign = if right.y >= 0.0 { -1.0 } else { 1.0 };
                    (0.0, roll_sign * std::f32::consts::PI * 0.75)
                };
                let pitch_err = desired_pitch - cur_pitch;
                let roll_err = desired_roll - cur_roll;
                Vec3::new(
                    clampf(pitch_err * 7.5, -7.0, 7.0),
                    Self::betaflight_actual(input.yaw, center_sens * 0.85, max_rate * 0.75, expo),
                    clampf(-roll_err * 7.5, -7.0, 7.0),
                )
            };

            let target_omega = match cfg.flight_mode {
                FlightMode::Acro => acro_omega,
                FlightMode::Angle => angle_omega,
                FlightMode::Horizon => {
                    // Authentic Betaflight Horizon: self-level below 55% stick, full Acro flip above 85%!
                    let def = input.pitch.abs().max(input.roll.abs());
                    let blend = ((def - 0.55) / 0.30).clamp(0.0, 1.0);
                    Vec3::new(
                        angle_omega.x + (acro_omega.x - angle_omega.x) * blend,
                        acro_omega.y,
                        angle_omega.z + (acro_omega.z - angle_omega.z) * blend,
                    )
                }
            };

            let rot_resp = if self.armed {
                (24.0 * motor_scale.sqrt()) / mass_ratio.sqrt()
            } else {
                1.0
            };
            let rot_alpha = (rot_resp * dt).min(1.0);
            self.omega = self.omega.add(target_omega.sub(self.omega).scale(rot_alpha));

            // Prop wash high-frequency micro-shudder (tinywhoop ~38 Hz)
            if self.prop_wash_level > 0.05 && wash_shake_coeff > 0.0 {
                self.prop_wash_phase += dt * 235.0;
                let amp = self.prop_wash_level * wash_shake_coeff * (0.4 + 0.6 * self.motor_thr);
                self.omega.x += self.prop_wash_phase.sin() * amp;
                self.omega.z += (self.prop_wash_phase * 1.31).cos() * amp * 1.2;
            }

            if self.armed || !self.on_ground {
                self.rot = self.rot.integrate_body_omega(self.omega, dt);
            }
        }

        // 5. Linear Forces: Thrust + Aerodynamic Body Drag + In-Plane Rotor Grip + Gravity
        let (right_now, up_now, fwd_now) = self.rot.basis();
        let thrust_mag = if pinned_on_ground { 0.0 } else { aero_thrust };

        let thrust_accel = up_now.scale(thrust_mag);
        let speed = self.vel.len();
        let eff_drag = (params.drag * drag_scale) / mass_ratio;
        let drag_accel = self.vel.scale(-eff_drag * speed - 0.04 * drag_scale);

        // In-plane rotor H-force grip (StickTime Grip setting: carves turns cleanly without icy drift)
        let v_plane = right_now
            .scale(self.vel.dot(right_now))
            .add(fwd_now.scale(self.vel.dot(fwd_now)));
        let grip_coeff = (params.grip * grip_scale / mass_ratio) * ((thrust_mag / (5.0 * GRAVITY) + 0.04).sqrt());
        let grip_accel = v_plane.scale(-grip_coeff);

        let gravity_accel = Vec3::new(0.0, -GRAVITY, 0.0);

        let total_accel = thrust_accel
            .add(drag_accel)
            .add(grip_accel)
            .add(gravity_accel);
        self.vel = self.vel.add(total_accel.scale(dt));
        self.pos = self.pos.add(self.vel.scale(dt));

        // 6. Ground collision (smooth zero-bounce touchdown + realistic skids drag)
        if self.pos.y <= QUAD_RADIUS {
            self.pos.y = QUAD_RADIUS;
            if self.vel.y < -5.5 {
                self.hit_flash = 0.25;
                self.vel.y = -self.vel.y * 0.08;
            } else if self.vel.y < 0.0 {
                self.vel.y = 0.0;
            }
            let friction = (11.0 * dt).min(0.95);
            self.vel.x *= 1.0 - friction;
            self.vel.z *= 1.0 - friction;
            if self.vel.x.abs() < 0.03 { self.vel.x = 0.0; }
            if self.vel.z.abs() < 0.03 { self.vel.z = 0.0; }
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
                let dx0 = self.pos.x - ex_min.x;
                let dx1 = ex_max.x - self.pos.x;
                let dy0 = self.pos.y - ex_min.y;
                let dy1 = ex_max.y - self.pos.y;
                let dz0 = self.pos.z - ex_min.z;
                let dz1 = ex_max.z - self.pos.z;
                let min_pen = dx0.min(dx1).min(dy0).min(dy1).min(dz0).min(dz1);
                if min_pen == dx0 {
                    self.pos.x = ex_min.x;
                    self.vel.x = -self.vel.x * 0.28;
                } else if min_pen == dx1 {
                    self.pos.x = ex_max.x;
                    self.vel.x = -self.vel.x * 0.28;
                } else if min_pen == dy0 {
                    self.pos.y = ex_min.y;
                    self.vel.y = -self.vel.y * 0.28;
                } else if min_pen == dy1 {
                    self.pos.y = ex_max.y;
                    self.vel.y = -self.vel.y * 0.20;
                } else if min_pen == dz0 {
                    self.pos.z = ex_min.z;
                    self.vel.z = -self.vel.z * 0.28;
                } else {
                    self.pos.z = ex_max.z;
                    self.vel.z = -self.vel.z * 0.28;
                }
                self.hit_flash = 0.25;
            }
        }

        // 7b. Solid 3D Gate Frame Collisions
        for g in track.gates {
            match g.kind {
                GateKind::Box => {
                    let r = g.right().scale(g.width * 0.5);
                    let p_bl = g.pos.sub(r);
                    let p_br = g.pos.add(r);
                    let p_tl = Vec3::new(p_bl.x, g.pos.y + g.height, p_bl.z);
                    let p_tr = Vec3::new(p_br.x, g.pos.y + g.height, p_br.z);
                    self.collide_segment(p_bl, p_tl, 0.16);
                    self.collide_segment(p_br, p_tr, 0.16);
                    self.collide_segment(p_tl, p_tr, 0.20);
                    if g.pos.y > 0.2 {
                        let g_bl = Vec3::new(p_bl.x, 0.0, p_bl.z);
                        let g_br = Vec3::new(p_br.x, 0.0, p_br.z);
                        self.collide_segment(g_bl, p_bl, 0.12);
                        self.collide_segment(g_br, p_br, 0.12);
                    }
                }
                GateKind::Dive => {
                    let hw = g.width * 0.5;
                    let c0 = Vec3::new(g.pos.x - hw, g.pos.y, g.pos.z - hw);
                    let c1 = Vec3::new(g.pos.x + hw, g.pos.y, g.pos.z - hw);
                    let c2 = Vec3::new(g.pos.x + hw, g.pos.y, g.pos.z + hw);
                    let c3 = Vec3::new(g.pos.x - hw, g.pos.y, g.pos.z + hw);
                    self.collide_segment(c0, c1, 0.18);
                    self.collide_segment(c1, c2, 0.18);
                    self.collide_segment(c2, c3, 0.18);
                    self.collide_segment(c3, c0, 0.18);
                }
                GateKind::Flag => {
                    let top = Vec3::new(g.pos.x, g.height, g.pos.z);
                    self.collide_segment(g.pos, top, 0.14);
                }
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
        if !self.lap_started && self.armed && self.motor_thr > 0.15 && self.pos.y > 0.45 {
            self.lap_started = true;
        }
        if self.lap_started && self.armed {
            self.lap_time += dt;
        }
        if !track.gates.is_empty() && self.armed {
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
        if self.armed && !self.on_ground && self.pos.y > 1.5 {
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
        let mut quad = QuadState::spawn(track, &cfg);
        let input = StickInput {
            throttle: 0.75,
            yaw: 0.0,
            pitch: 0.0,
            roll: 0.0,
        };
        for _ in 0..240 {
            quad.step(input, &mut cfg, track);
        }
        assert!(quad.pos.y > 3.0, "Quad should climb with 75% throttle, got y={}", quad.pos.y);
    }

    #[test]
    fn test_acro_flip() {
        let mut cfg = Config::default();
        let track = &TRACKS[0];
        let mut quad = QuadState::spawn(track, &cfg);
        quad.pos.y = 20.0;
        quad.on_ground = false;
        let input = StickInput {
            throttle: 0.4,
            yaw: 0.0,
            pitch: 1.0,
            roll: 0.0,
        };
        for _ in 0..240 {
            quad.step(input, &mut cfg, track);
        }
        assert!(quad.score >= 250, "Full pitch should register a flip trick");
    }

    #[test]
    fn test_horizon_levels_and_flips() {
        let mut cfg = Config::default();
        cfg.flight_mode = FlightMode::Horizon;
        let track = &TRACKS[0];
        let mut quad = QuadState::spawn(track, &cfg);
        quad.pos.y = 20.0;
        quad.on_ground = false;
        // Start tilted 35 deg, release sticks -> should self-level!
        quad.rot = Quat::from_axis_angle(Vec3::FORWARD, 35.0_f32.to_radians());
        let neutral = StickInput { throttle: 0.45, yaw: 0.0, pitch: 0.0, roll: 0.0 };
        for _ in 0..180 {
            quad.step(neutral, &mut cfg, track);
        }
        let (_, up, _) = quad.rot.basis();
        assert!(up.y > 0.96, "Horizon mode must self-level at center stick, up.y={}", up.y);

        // Now push full pitch -> should flip!
        let flip = StickInput { throttle: 0.45, yaw: 0.0, pitch: 1.0, roll: 0.0 };
        for _ in 0..240 {
            quad.step(flip, &mut cfg, track);
        }
        assert!(quad.score >= 250, "Horizon mode must allow full flips at max stick!");
    }

    #[test]
    fn test_freestyle_gate_scoring() {
        let mut cfg = Config::default();
        cfg.game_mode = GameMode::Freestyle;
        let track = &TRACKS[0];
        let mut quad = QuadState::spawn(track, &cfg);
        quad.pos = Vec3::new(0.0, 1.8, 21.7);
        quad.vel = Vec3::new(0.0, 0.0, 15.0);
        quad.on_ground = false;
        let input = StickInput { throttle: 0.3, yaw: 0.0, pitch: 0.0, roll: 0.0 };
        for _ in 0..30 {
            quad.step(input, &mut cfg, track);
        }
        assert!(quad.score >= 100, "Flying through a loop/gate in Freestyle should award points, got {}", quad.score);
    }

    #[test]
    fn test_gate_post_collision() {
        let mut cfg = Config::default();
        let track = &TRACKS[0];
        let mut quad = QuadState::spawn(track, &cfg);
        quad.pos = Vec3::new(-2.4, 1.8, 21.5);
        quad.vel = Vec3::new(0.0, 0.0, 12.0);
        quad.on_ground = false;
        let input = StickInput { throttle: 0.45, yaw: 0.0, pitch: 0.0, roll: 0.0 };
        for _ in 0..20 {
            quad.step(input, &mut cfg, track);
        }
        assert!(quad.vel.z < 5.0, "Hitting a gate post should bounce/slow the quad, got vz={}", quad.vel.z);
    }

    #[test]
    fn test_yaw_while_grounded_and_armed() {
        let mut cfg = Config::default();
        let track = &TRACKS[0];
        let mut quad = QuadState::spawn(track, &cfg);
        let (_, _, fwd_before) = quad.rot.basis();
        let input = StickInput { throttle: 0.0, yaw: 1.0, pitch: 0.8, roll: 0.8 };
        for _ in 0..60 {
            quad.step(input, &mut cfg, track);
        }
        let (_, up_after, fwd_after) = quad.rot.basis();
        assert!(quad.on_ground, "Quad should remain grounded at 0 throttle");
        assert!(up_after.y > 0.98, "Quad must stay flat on ground even if pitch/roll pushed, up.y={}", up_after.y);
        assert!(fwd_before.dot(fwd_after) < 0.95, "Quad should yaw to look around while grounded and armed");
    }

    #[test]
    fn test_sideways_low_flight_does_not_trigger_turtle() {
        let mut cfg = Config::default();
        let track = &TRACKS[0];
        let mut quad = QuadState::spawn(track, &cfg);
        quad.pos = Vec3::new(0.0, 0.45, 0.0);
        quad.on_ground = false;
        quad.rot = Quat::from_axis_angle(Vec3::FORWARD, 70.0_f32.to_radians());
        let input = StickInput { throttle: 0.85, yaw: 0.0, pitch: 0.0, roll: 0.8 };
        for _ in 0..10 {
            quad.step(input, &mut cfg, track);
        }
        assert!(quad.motor_thr > 0.3, "Sideways low flight must NOT cut throttle via turtle mode!");
    }
}
