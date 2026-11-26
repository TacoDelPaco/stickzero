//! 240 Hz 6-DOF FPV quadcopter physics engine with Betaflight Actual rates,
//! Acro / Horizon / Angle modes, StickTime-inspired aerodynamic inflow damping (climb thrust loss,
//! descent cushion, translational lift, rotor H-force grip), custom Drone Tuning sliders,
//! radio AUX Arm/Mode/Turtle support, solid gate & structure collisions, and post-flight stats.

use crate::config::{AuxBinding, BatteryPack, Config, FlightMode, GameMode, PropWashLevel};
use crate::input::{AuxInputs, FlightControlSource};
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

    // Object-collision angled wall-slide state
    pub wall_slide: bool,
    pub wall_normal: Vec3,
    pub wall_slide_time: f32,
    pub wall_contact_age: f32,
    pub wall_escape_cooldown: f32,

    // Betaflight arming & mode state
    pub armed: bool,
    pub arm_blocked_throttle: bool,
    pub arm_aux_connected: bool,
    pub turtle_mode: bool,
    pub turtle_active: bool,
    pub turtle_aux_connected: bool,
    pub horizon_smooth: f32,
    pub has_flown: bool,
    pub throttle_pct: u8,

    // Active input mode (Controller vs CardputerZero IMU simplified flight)
    pub control_source: FlightControlSource,
    /// Vertical climb/descend command in IMU simplified mode (-1 descend, 0 auto-hover, +1 climb)
    pub imu_climb_cmd: f32,

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
            wall_slide: false,
            wall_normal: Vec3::ZERO,
            wall_slide_time: 0.0,
            wall_contact_age: 0.0,
            wall_escape_cooldown: 0.0,
            armed: start_armed,
            arm_blocked_throttle: false,
            arm_aux_connected: false,
            turtle_mode: false,
            turtle_active: false,
            turtle_aux_connected: false,
            horizon_smooth: 1.0,
            has_flown: false,
            throttle_pct: 0,
            control_source: FlightControlSource::Controller,
            imu_climb_cmd: 0.0,
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
        let ctrl_src = self.control_source;
        *self = Self::spawn(track, cfg);
        self.score = best_score;
        self.control_source = ctrl_src;
    }

    pub fn speed_kmh(&self) -> f32 {
        self.vel.len() * 3.6
    }

    /// Apply radio AUX switches / buttons and keyboard flight toggles.
    /// Returns `true` if the quad just transitioned from disarmed to armed.
    pub fn apply_aux(&mut self, aux: AuxInputs, input: StickInput, cfg: &mut Config) -> bool {
        let mut just_armed = false;

        // 1. Flight Mode switch / toggle (Acro / Horizon / Angle) — disabled in IMU Simplified mode
        if self.control_source != FlightControlSource::ImuSimplified {
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
        }

        // 2. Turtle Mode switch / toggle
        let was_aux_turtle = self.turtle_aux_connected;
        self.turtle_aux_connected = aux.turtle_switch.is_some();
        if let Some(on) = aux.turtle_switch {
            self.turtle_mode = on;
        } else {
            if was_aux_turtle {
                self.turtle_mode = false;
            }
            if aux.turtle_toggle {
                self.turtle_mode = !self.turtle_mode;
                self.combo_text = if self.turtle_mode { "TURTLE ARMED" } else { "TURTLE OFF" };
                self.combo_timer = 0.8;
            }
        }

        // 3. Arm / Disarm switch / toggle with Betaflight throttle-low safety
        self.arm_aux_connected = aux.arm_aux_connected;
        if input.throttle <= 0.15 {
            self.arm_blocked_throttle = false;
        }

        if !self.arm_aux_connected {
            // No ARM AUX bound on a connected controller: keep quad automatically ready/armed
            // (unless battery is empty) so players don't need extra arming controls.
            if !self.battery_empty && !self.armed {
                self.armed = true;
                self.arm_blocked_throttle = false;
                just_armed = true;
            }
            return just_armed;
        }

        let want_arm = if let Some(sw) = aux.arm_switch {
            Some(sw)
        } else if aux.arm_toggle {
            Some(!self.armed)
        } else {
            None
        };

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

    /// Betaflight throttle mid + expo curve (flattens sensitivity around `thr_mid`).
    #[inline]
    pub fn apply_throttle_expo(raw: f32, mid_pct: u8, expo_pct: u8) -> f32 {
        let x = clampf(raw, 0.0, 1.0);
        let e = (expo_pct as f32 / 100.0).clamp(0.0, 0.80);
        if e < 1e-3 {
            return x;
        }
        let mid = (mid_pct as f32 / 100.0).clamp(0.20, 0.70);
        if x < mid {
            let u = (x - mid) / mid; // -1..0
            mid + mid * ((1.0 - e) * u + e * u * u * u)
        } else {
            let u = (x - mid) / (1.0 - mid); // 0..1
            mid + (1.0 - mid) * ((1.0 - e) * u + e * u * u * u)
        }
    }

    /// Extract horizontal (XZ) right and forward unit vectors from the quad's basis,
    /// avoiding degeneracy when pitched or rolled steeply.
    #[inline]
    fn horizontal_basis(right: Vec3, forward: Vec3) -> (Vec3, Vec3) {
        let f_sq = forward.x * forward.x + forward.z * forward.z;
        let r_sq = right.x * right.x + right.z * right.z;
        if f_sq >= r_sq && f_sq > 1e-4 {
            let fwd_h = Vec3::new(forward.x, 0.0, forward.z).scale(1.0 / f_sq.sqrt());
            let right_h = Vec3::UP.cross(fwd_h);
            (right_h, fwd_h)
        } else if r_sq > 1e-4 {
            let right_h = Vec3::new(right.x, 0.0, right.z).scale(1.0 / r_sq.sqrt());
            let fwd_h = right_h.cross(Vec3::UP);
            (right_h, fwd_h)
        } else {
            (Vec3::RIGHT, Vec3::FORWARD)
        }
    }

    /// Resolve a collision against a solid 3D object (structure face or gate frame).
    /// Side/overhang impacts absorb rebound and enter/refresh the angled wall-slide state
    /// so the drone slowly falls angled towards the ground against the object until grounded,
    /// unless the pilot flies away to regain balance.
    fn resolve_object_contact(
        &mut self,
        normal: Vec3,
        flying_away: bool,
        was_wall_sliding: bool,
        substep_wall_norm: &mut Vec3,
        substep_wall_hits: &mut u32,
    ) {
        let vn = self.vel.dot(normal);

        // Flat top / roof surface: settle cleanly on top instead of wall-sliding
        if normal.y >= 0.70 {
            if vn < -5.5 {
                self.hit_flash = 0.25;
                self.vel = self.vel.sub(normal.scale(vn * 1.08));
            } else if vn < 0.0 {
                self.vel = self.vel.sub(normal.scale(vn));
            }
            let friction = (11.0 * PHYSICS_DT).min(0.95);
            self.vel.x *= 1.0 - friction;
            self.vel.z *= 1.0 - friction;
            if self.vel.x.abs() < 0.03 { self.vel.x = 0.0; }
            if self.vel.z.abs() < 0.03 { self.vel.z = 0.0; }
            if self.vel.y <= 0.1 {
                self.on_ground = true;
                self.wall_slide = false;
                self.wall_slide_time = 0.0;
                self.flip_accum = 0.0;
                self.roll_accum = 0.0;
                self.inverted_time = 0.0;
            }
            return;
        }

        // Side wall / pillar / gate post / crossbar / underside impact
        if vn < -1.5 || (!was_wall_sliding && vn < -0.5) {
            self.hit_flash = 0.25;
        }

        // Absorb inward velocity cleanly so the quad stays against the object instead of bouncing away
        if vn < 0.0 {
            self.vel = self.vel.sub(normal.scale(vn));
        }

        // Already grounded at the base of an object: stay grounded rather than starting a wall-slide
        if self.on_ground || self.pos.y <= QUAD_RADIUS + 0.05 {
            return;
        }

        // Pilot is actively flying away after peeling off: block clipping without re-latching
        // unless slamming hard inward again.
        if self.wall_escape_cooldown > 0.0 && flying_away && vn > -3.0 {
            return;
        }

        // On fresh airborne contact, damp tangential & upward speed so the drone settles into
        // a slow fall against the object rather than shooting sideways or climbing the wall.
        if !was_wall_sliding && *substep_wall_hits == 0 {
            self.vel.x *= 0.45;
            self.vel.z *= 0.45;
            if self.vel.y > 0.0 {
                self.vel.y *= 0.25;
            }
            self.flip_accum = 0.0;
            self.roll_accum = 0.0;
            self.inverted_time = 0.0;
        }

        // Horizontal outward normal pointing away from the object
        let horiz_sq = normal.x * normal.x + normal.z * normal.z;
        let wall_n = if horiz_sq > 0.04 {
            let inv = 1.0 / horiz_sq.sqrt();
            Vec3::new(normal.x * inv, 0.0, normal.z * inv)
        } else {
            let (_, _, fwd) = self.rot.basis();
            let f_sq = fwd.x * fwd.x + fwd.z * fwd.z;
            if f_sq > 1e-4 {
                let inv = 1.0 / f_sq.sqrt();
                Vec3::new(-fwd.x * inv, 0.0, -fwd.z * inv)
            } else {
                Vec3::new(0.0, 0.0, -1.0)
            }
        };

        *substep_wall_norm = substep_wall_norm.add(wall_n);
        *substep_wall_hits += 1;
    }

    /// Collide quad sphere against a 3D line segment (gate posts, crossbars, stilts).
    fn collide_segment(
        &mut self,
        a: Vec3,
        b: Vec3,
        tube_radius: f32,
        flying_away: bool,
        was_wall_sliding: bool,
        substep_wall_norm: &mut Vec3,
        substep_wall_hits: &mut u32,
    ) {
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
            self.resolve_object_contact(
                normal,
                flying_away,
                was_wall_sliding,
                substep_wall_norm,
                substep_wall_hits,
            );
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
        if self.wall_escape_cooldown > 0.0 {
            self.wall_escape_cooldown = (self.wall_escape_cooldown - dt).max(0.0);
        }

        let prev_pos = self.pos;
        let batt_on = cfg.battery_pack != BatteryPack::Off;
        let (pack_mah, _, pack_sag) = cfg.battery_pack.specs(&params);

        // Direct physical Drone Tuning values ("The Zone"-style real units)
        let weight_g = (cfg.weight_g as f32).max(15.0);
        let thrust_g = (cfg.thrust_g as f32).max(40.0);
        let ref_auw_g = (params.dry_mass_g + params.ref_batt_g).max(15.0);
        let mass_ratio = (weight_g / ref_auw_g).clamp(0.35, 3.0);
        let thrust_ratio = (thrust_g / (params.thrust_g as f32).max(40.0)).clamp(0.35, 3.0);
        // Exact Newtonian max upward acceleration from thrust (g) / weight (g) * g
        let max_accel = (thrust_g / weight_g) * GRAVITY;
        let eff_motor_tau = (cfg.motor_lag_ms as f32 / 1000.0).clamp(0.005, 0.050);

        // Orientation & body-frame airflow decomposition (computed first so Crash Flip can silence throttle!)
        let (right, up, forward) = self.rot.basis();
        self.inverted = up.y < -0.35;

        // Track ongoing wall-slide contact and check if pilot is actively flying away to regain balance
        if self.wall_slide {
            self.wall_slide_time += dt;
            self.wall_contact_age += dt;
            if self.on_ground || self.wall_contact_age > 0.22 {
                self.wall_slide = false;
                self.wall_slide_time = 0.0;
            }
        }

        let (right_h, fwd_h) = Self::horizontal_basis(right, forward);
        let stick_world = fwd_h.scale(input.pitch).add(right_h.scale(input.roll));
        let stick_mag = (input.pitch * input.pitch + input.roll * input.roll).sqrt();
        let away_cmd = stick_world.dot(self.wall_normal);
        let can_fly_away = self.armed && !self.turtle_active && (!self.battery_empty || !batt_on);
        let flying_away = can_fly_away
            && input.throttle > 0.20
            && (away_cmd > 0.18 || (stick_mag > 0.35 && away_cmd >= -0.05));

        if self.wall_slide && self.wall_slide_time > 0.08 && flying_away {
            self.wall_slide = false;
            self.wall_slide_time = 0.0;
            self.wall_escape_cooldown = 0.45;
            // Peel cleanly off the object surface and assist un-tipping away from the wall
            let vn = self.vel.dot(self.wall_normal);
            if vn < 1.25 {
                self.vel = self.vel.add(self.wall_normal.scale(1.25 - vn));
            }
            let tx = Vec3::UP.dot(right);
            let tz = Vec3::UP.dot(forward);
            self.omega.x += clampf(tz * 3.5, -2.5, 2.5);
            self.omega.z += clampf(-tx * 3.5, -2.5, 2.5);
        }

        // Crash Flip / Turtle Mode latch:
        // - If Crash Flip is bound on a connected controller (turtle_aux_connected), require that switch/button.
        // - Otherwise automate turtle mode while grounded & inverted so players can simply roll to flip.
        // - Only enters when genuinely on the ground (so bumping the switch midair never drops you!).
        // - Stays latched through the entire flip hop even when on_ground briefly goes false!
        // - Completely disables throttle stick (zero thrust & zero throttle audio) while active.
        let auto_turtle = !self.turtle_aux_connected
            && self.on_ground
            && up.y < -0.55;
        if !self.armed {
            self.turtle_active = false;
        } else if self.turtle_mode {
            if self.on_ground {
                self.turtle_active = true;
            } else if self.turtle_active && self.pos.y > QUAD_RADIUS + 1.4 {
                self.turtle_active = false;
            }
        } else if auto_turtle {
            self.turtle_active = true;
        } else if self.turtle_active {
            // If Crash Flip switch was released, exit immediately (auto-turtle finishes its flip first)
            if self.turtle_aux_connected || up.y > 0.72 || self.on_ground {
                self.turtle_active = false;
            }
        }

        let stick_flip_mag = input.roll.abs().max(input.pitch.abs());
        let turtle_flipping = self.turtle_active && stick_flip_mag > 0.22 && up.y < 0.72;
        let imu_simplified = self.control_source == FlightControlSource::ImuSimplified;

        // 1. Throttle control:
        //    - Controller mode: Betaflight throttle mid + expo + motor spool lag
        //    - IMU Simplified mode: Automatic hover-hold & smooth vertical velocity control
        //      (stays grounded until Climb is pressed, auto-hovers when released, lands cleanly on Descend)
        let target_thr = if !self.armed || (self.battery_empty && batt_on) {
            0.0
        } else if self.turtle_active {
            if turtle_flipping { 0.35 * stick_flip_mag } else { 0.0 }
        } else if imu_simplified {
            let climb = clampf(self.imu_climb_cmd, -1.0, 1.0);
            if self.on_ground && climb <= 0.05 {
                0.0
            } else if !self.on_ground && climb < -0.1 && self.pos.y <= QUAD_RADIUS + 0.30 && self.vel.y <= 0.25 {
                0.0
            } else {
                let up_y_eff = up.y.clamp(0.55, 1.0);
                let sag_est = if batt_on {
                    (self.cell_voltage / 4.0).clamp(0.72, 1.05)
                } else {
                    1.0
                };
                let idle_a = params.idle_accel / mass_ratio;
                let needed_curve = ((GRAVITY / (sag_est * up_y_eff) - idle_a).max(0.5) / max_accel).clamp(0.05, 0.90);
                let hover_thr = needed_curve.powf(1.0 / 1.18);
                let target_vy = if climb > 0.05 {
                    3.2 * climb
                } else if climb < -0.05 {
                    2.8 * climb
                } else {
                    0.0
                };
                let vy_err = target_vy - self.vel.y;
                clampf(hover_thr + vy_err * 0.085, 0.06, 0.92)
            }
        } else {
            Self::apply_throttle_expo(input.throttle, cfg.thr_mid, cfg.thr_expo)
        };

        self.throttle_pct = if self.turtle_active {
            0
        } else if imu_simplified {
            (clampf(target_thr, 0.0, 1.0) * 100.0).round() as u8
        } else {
            (clampf(input.throttle, 0.0, 1.0) * 100.0).round() as u8
        };

        let alpha = dt / (eff_motor_tau + dt);
        self.motor_thr += (target_thr - self.motor_thr) * alpha;
        if (!self.armed || (self.turtle_active && !turtle_flipping)) && self.motor_thr < 0.01 {
            self.motor_thr = 0.0;
        }

        // 2. Battery voltage sag & depletion (LiHV 4.35V -> 3.30V for Lava II packs)
        if batt_on {
            let draw_active = self.armed && !self.battery_empty;
            let amps = if draw_active {
                (1.1 + params.full_amp * thrust_ratio * self.motor_thr * self.motor_thr) * mass_ratio.sqrt()
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
            if self.arm_aux_connected || self.has_flown {
                self.flight_time += dt;
            }
            let spd = self.speed_kmh();
            if spd > self.max_speed_kmh {
                self.max_speed_kmh = spd;
            }
            if !self.on_ground && self.pos.y > QUAD_RADIUS + 0.25 && !self.turtle_active {
                self.has_flown = true;
            }
        }

        let v_up = self.vel.dot(up);
        let v_right = self.vel.dot(right);
        let v_fwd = self.vel.dot(forward);
        let edge_sq = v_right * v_right + v_fwd * v_fwd;
        let edge_speed = edge_sq.sqrt();

        // Compute static + aerodynamic thrust (StickTime-style climb loss, descent cushion, translational lift)
        let thr_curve = self.motor_thr.powf(1.18);
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
        let base_thrust = if !self.armed || self.turtle_active {
            0.0
        } else {
            ((params.idle_accel / mass_ratio) * self.motor_thr.signum() + max_accel * thr_curve)
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
        let pinned_on_ground =
            self.on_ground && !turtle_flipping && (!self.armed || self.turtle_active || up.y < 0.25 || vertical_lift <= GRAVITY * 1.01);

        // 4. Rotational control (Acro / Horizon / Angle / Grounded Yaw / Turtle)
        let (center_sens, max_rate, expo) = cfg.actual_rates();

        if turtle_flipping {
            if self.turtle_cooldown <= 0.0 {
                self.combo_text = "CRASH FLIP";
                self.combo_timer = 0.8;
                self.turtle_cooldown = 0.7;
            }
            // Crisp ground kick so the quad has room to complete a full 180-deg flip upright
            if self.on_ground && self.vel.y < 0.60 {
                self.vel.y = 1.95;
            }
            let r_cmd = if input.roll.abs() > 0.22 { -input.roll.signum() * 13.5 } else { 0.0 };
            let p_cmd = if input.pitch.abs() > 0.22 { input.pitch.signum() * 13.5 } else { 0.0 };
            let target_omega = Vec3::new(p_cmd, 0.0, r_cmd);
            let rot_alpha = (28.0 * dt).min(1.0);
            self.omega = self.omega.add(target_omega.sub(self.omega).scale(rot_alpha));
            self.rot = self.rot.integrate_body_omega(self.omega, dt);
        } else if self.turtle_active && !self.on_ground {
            // Finished flipping upright (or stick released mid-hop): level smoothly as it lands!
            let target_up = if up.y >= 0.0 { 1.0f32 } else { -1.0f32 };
            let cur_pitch = clampf(-forward.y * target_up, -1.0, 1.0).asin();
            let cur_roll = clampf(-right.y * target_up, -1.0, 1.0).asin();
            let level_omega = Vec3::new(-cur_pitch * 12.0, 0.0, cur_roll * 12.0);
            self.omega = self.omega.add(level_omega.sub(self.omega).scale((18.0 * dt).min(1.0)));
            self.rot = self.rot.integrate_body_omega(self.omega, dt);
        } else if pinned_on_ground {
            let target_up = if up.y >= 0.0 { 1.0f32 } else { -1.0f32 };
            let cur_pitch = clampf(-forward.y * target_up, -1.0, 1.0).asin();
            let cur_roll = clampf(-right.y * target_up, -1.0, 1.0).asin();

            let yaw_rate = if self.armed && up.y >= 0.25 && !self.turtle_active {
                Self::betaflight_actual(input.yaw, center_sens * 0.85, max_rate * 0.70, expo)
            } else {
                0.0
            };

            let ground_omega = Vec3::new(-cur_pitch * 18.0, yaw_rate, cur_roll * 18.0);
            self.rot = self.rot.integrate_body_omega(ground_omega, dt);
            self.omega = Vec3::new(0.0, yaw_rate, 0.0);
        } else if self.wall_slide {
            // Angled towards the ground against the object: lean `up` into `-wall_normal`
            // so the wall-facing side tips downward until grounded (unless flying away).
            let tilt_rad = ((cfg.camera_tilt_deg as f32) + 23.0).clamp(42.0, 56.0).to_radians();
            let (sin_t, cos_t) = tilt_rad.sin_cos();
            let target_up = Vec3::new(
                -self.wall_normal.x * sin_t,
                cos_t,
                -self.wall_normal.z * sin_t,
            )
            .normalized();

            let tx = target_up.dot(right);
            let ty = target_up.dot(up);
            let tz = target_up.dot(forward);
            let cross_len = (tz * tz + tx * tx).sqrt();
            let err_angle = clampf(ty, -1.0, 1.0).acos();
            let (err_x, err_z) = if cross_len > 1e-4 {
                let s = err_angle / cross_len;
                (tz * s, -tx * s)
            } else if ty < 0.0 {
                (std::f32::consts::PI, 0.0)
            } else {
                (0.0, 0.0)
            };

            let yaw_cmd = if self.armed && !self.turtle_active {
                Self::betaflight_actual(input.yaw, center_sens * 0.65, max_rate * 0.55, expo)
            } else {
                0.0
            };

            let target_omega = Vec3::new(
                clampf(err_x * 6.0, -5.5, 5.5),
                yaw_cmd,
                clampf(err_z * 6.0, -5.5, 5.5),
            );
            let rot_alpha = (14.0 * dt).min(1.0);
            self.omega = self.omega.add(target_omega.sub(self.omega).scale(rot_alpha));
            self.rot = self.rot.integrate_body_omega(self.omega, dt);
        } else if imu_simplified {
            // Simplified CardputerZero IMU flight: always self-levels (capped at 38 deg total tilt
            // via circular clamping) with smooth yaw so tilting the handheld never accidentally flips
            // upside-down or loses hover authority on diagonal forward+bank turns.
            let max_tilt = 38.0_f32.to_radians();
            let mut px = clampf(input.pitch, -1.0, 1.0);
            let mut rx = clampf(input.roll, -1.0, 1.0);
            let mag = (px * px + rx * rx).sqrt();
            if mag > 1.0 {
                px /= mag;
                rx /= mag;
            }
            let desired_pitch = px * max_tilt;
            let desired_roll = rx * max_tilt;
            let (cur_pitch, cur_roll) = if up.y >= 0.0 {
                (
                    clampf(-forward.y, -1.0, 1.0).asin(),
                    clampf(-right.y, -1.0, 1.0).asin(),
                )
            } else {
                let roll_sign = if right.y >= 0.0 { -1.0 } else { 1.0 };
                (0.0, roll_sign * std::f32::consts::PI * 0.75)
            };
            let pitch_err = desired_pitch - cur_pitch;
            let roll_err = desired_roll - cur_roll;
            let target_omega = Vec3::new(
                clampf(pitch_err * 8.0, -6.5, 6.5),
                clampf(input.yaw, -1.0, 1.0) * 145.0_f32.to_radians(),
                clampf(-roll_err * 8.0, -6.5, 6.5),
            );
            let rot_alpha = (18.0 * dt).min(1.0);
            self.omega = self.omega.add(target_omega.sub(self.omega).scale(rot_alpha));
            self.rot = self.rot.integrate_body_omega(self.omega, dt);
        } else {
            let acro_omega = Vec3::new(
                Self::betaflight_actual(input.pitch, center_sens, max_rate, expo),
                Self::betaflight_actual(input.yaw, center_sens * 0.85, max_rate * 0.82, expo),
                -Self::betaflight_actual(input.roll, center_sens, max_rate, expo),
            );

            let target_omega = match cfg.flight_mode {
                FlightMode::Acro => acro_omega,
                FlightMode::Angle => {
                    // Strict Betaflight ANGLE mode: stick directly commands tilt angle (capped at 55 deg, never flips)
                    let max_tilt = 55.0_f32.to_radians();
                    let desired_pitch = clampf(input.pitch, -1.0, 1.0) * max_tilt;
                    let desired_roll = clampf(input.roll, -1.0, 1.0) * max_tilt;
                    let (cur_pitch, cur_roll) = if up.y >= 0.0 {
                        (
                            clampf(-forward.y, -1.0, 1.0).asin(),
                            clampf(-right.y, -1.0, 1.0).asin(),
                        )
                    } else {
                        let roll_sign = if right.y >= 0.0 { -1.0 } else { 1.0 };
                        (0.0, roll_sign * std::f32::consts::PI * 0.75)
                    };
                    let pitch_err = desired_pitch - cur_pitch;
                    let roll_err = desired_roll - cur_roll;
                    Vec3::new(
                        clampf(pitch_err * 8.5, -7.5, 7.5),
                        Self::betaflight_actual(input.yaw, center_sens * 0.85, max_rate * 0.75, expo),
                        clampf(-roll_err * 8.5, -7.5, 7.5),
                    )
                }
                FlightMode::Horizon => {
                    // Verified Betaflight HORIZON mode (`pidLevel` crossfade + asymmetric PT1 smoothing):
                    // 1. Stick deflection fades `raw_s` from 1.0 (near center) to 0.0 at 75% stick
                    //    (`horizonLimitSticks = 0.75`), dropping instantly on outer stick throws so
                    //    flips/rolls start with zero lag, and ramping smoothly (`tau = 0.12s`) when
                    //    centering sticks so recovery glides level without violent camera jerks.
                    // 2. Crossfades `angle_omega` (targeting up to 65 deg tilt via gimbal-lock-free
                    //    3D shortest-arc `desired_sky x sky`) with `acro_omega` (`acro * (1 - S) + angle * S`)
                    //    so moderate stick carves steeper/livelier banks than Angle mode without ever
                    //    accidentally flipping, while >= 75% stick delivers 100% pure Acro flips/rolls
                    //    and pure pitch flips recover purely along pitch (no sideways roll yank).
                    let def = input.pitch.abs().max(input.roll.abs());
                    let t = clampf((def - 0.15) / 0.60, 0.0, 1.0); // 0.0 at <=15%, 1.0 at >=75% stick
                    let raw_s = 1.0 - t * t;
                    let smooth_alpha = dt / (0.12 + dt);
                    let pt1_s = self.horizon_smooth + (raw_s - self.horizon_smooth) * smooth_alpha;
                    let level_s = raw_s.min(pt1_s);
                    self.horizon_smooth = level_s;

                    let max_tilt = 65.0_f32.to_radians();
                    let des_p = clampf(input.pitch, -1.0, 1.0) * max_tilt;
                    let des_r = clampf(input.roll, -1.0, 1.0) * max_tilt;
                    let desired_sky = Vec3::new(
                        -des_r.sin(),
                        (des_p.cos() * des_r.cos()).max(0.1),
                        -des_p.sin(),
                    )
                    .normalized();
                    let sky_body = Vec3::new(right.y, up.y, forward.y);
                    let cross = desired_sky.cross(sky_body);
                    let cross_len = (cross.x * cross.x + cross.y * cross.y + cross.z * cross.z).sqrt();
                    let dot = clampf(desired_sky.dot(sky_body), -1.0, 1.0);
                    let err_angle = dot.acos();

                    let (err_x, err_z) = if cross_len > 1e-4 {
                        let scale = err_angle / cross_len;
                        (cross.x * scale, cross.z * scale)
                    } else if dot < 0.0 {
                        // Exact 180-deg antipodal fallback: continue along active rotation axis
                        if self.omega.x.abs() > self.omega.z.abs() {
                            let sx = if self.omega.x >= 0.0 { 1.0 } else { -1.0 };
                            (sx * std::f32::consts::PI, 0.0)
                        } else {
                            let sz = if self.omega.z >= 0.0 { 1.0 } else { -1.0 };
                            (0.0, sz * std::f32::consts::PI)
                        }
                    } else {
                        (0.0, 0.0)
                    };

                    let angle_omega_x = clampf(err_x * 6.2, -7.5, 7.5);
                    let angle_omega_z = clampf(err_z * 6.2, -7.5, 7.5);

                    Vec3::new(
                        acro_omega.x * (1.0 - level_s) + angle_omega_x * level_s,
                        acro_omega.y,
                        acro_omega.z * (1.0 - level_s) + angle_omega_z * level_s,
                    )
                }
            };

            let motor_resp_scale =
                (params.motor_lag_ms as f32 / (cfg.motor_lag_ms as f32).max(5.0)).clamp(0.4, 2.5);
            let rot_resp = if self.armed {
                (24.0 * motor_resp_scale.sqrt()) / mass_ratio.sqrt()
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
        if self.wall_slide {
            // Slowly fall against the object with wall-scrape drag and gentle inward lean
            // so the drone stays against the object until grounded (unless flying away).
            let target_fall = if self.armed && !self.turtle_active {
                2.6 - 0.9 * self.motor_thr.clamp(0.0, 1.0)
            } else {
                4.2
            };
            let fall_rate = if self.vel.y > 0.0 { 9.0 } else { 5.5 };
            self.vel.y += (-target_fall - self.vel.y) * (fall_rate * dt).min(1.0);

            let vn = self.vel.dot(self.wall_normal);
            let v_horiz = Vec3::new(self.vel.x, 0.0, self.vel.z);
            let v_tan = v_horiz.sub(self.wall_normal.scale(vn));
            let new_vn = vn + (-0.35 - vn) * (8.0 * dt).min(1.0);
            let new_v_tan = v_tan.scale((1.0 - 6.0 * dt).max(0.0));
            let new_horiz = new_v_tan.add(self.wall_normal.scale(new_vn));
            self.vel.x = new_horiz.x;
            self.vel.z = new_horiz.z;
            self.pos = self.pos.add(self.vel.scale(dt));
        } else {
            let (right_now, up_now, fwd_now) = self.rot.basis();
            let thrust_mag = if pinned_on_ground { 0.0 } else { aero_thrust };

            let mut thrust_accel = up_now.scale(thrust_mag);
            if self.wall_escape_cooldown > 0.0 && flying_away {
                let tn = thrust_accel.dot(self.wall_normal);
                if tn < 0.0 {
                    thrust_accel = thrust_accel.sub(self.wall_normal.scale(tn));
                }
            }
            let speed = self.vel.len();
            let eff_drag = (cfg.air_drag as f32) / 1000.0; // e.g. 29 (0.29) -> 0.029
            let drag_accel = self.vel.scale(-eff_drag * speed - eff_drag * 1.4);

            // In-plane rotor H-force grip (carves turns cleanly without icy drift)
            let v_plane = right_now
                .scale(self.vel.dot(right_now))
                .add(fwd_now.scale(self.vel.dot(fwd_now)));
            let eff_grip = (cfg.rotor_grip as f32) / 100.0; // e.g. 34 -> 0.34
            let grip_coeff = eff_grip * ((thrust_mag / (5.0 * GRAVITY) + 0.04).sqrt());
            let grip_accel = v_plane.scale(-grip_coeff);

            let gravity_accel = Vec3::new(0.0, -GRAVITY, 0.0);

            let total_accel = thrust_accel
                .add(drag_accel)
                .add(grip_accel)
                .add(gravity_accel);
            self.vel = self.vel.add(total_accel.scale(dt));
            self.pos = self.pos.add(self.vel.scale(dt));
        }

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
            self.wall_slide = false;
            self.wall_slide_time = 0.0;
            self.flip_accum = 0.0;
            self.roll_accum = 0.0;
            self.inverted_time = 0.0;
        } else {
            self.on_ground = false;
        }

        // 7. Structure AABB collisions
        let was_wall_sliding = self.wall_slide;
        let mut substep_wall_norm = Vec3::ZERO;
        let mut substep_wall_hits: u32 = 0;

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
                let normal = if min_pen == dx0 {
                    self.pos.x = ex_min.x;
                    Vec3::new(-1.0, 0.0, 0.0)
                } else if min_pen == dx1 {
                    self.pos.x = ex_max.x;
                    Vec3::new(1.0, 0.0, 0.0)
                } else if min_pen == dy0 {
                    self.pos.y = ex_min.y;
                    Vec3::new(0.0, -1.0, 0.0)
                } else if min_pen == dy1 {
                    self.pos.y = ex_max.y;
                    Vec3::new(0.0, 1.0, 0.0)
                } else if min_pen == dz0 {
                    self.pos.z = ex_min.z;
                    Vec3::new(0.0, 0.0, -1.0)
                } else {
                    self.pos.z = ex_max.z;
                    Vec3::new(0.0, 0.0, 1.0)
                };
                self.resolve_object_contact(
                    normal,
                    flying_away,
                    was_wall_sliding,
                    &mut substep_wall_norm,
                    &mut substep_wall_hits,
                );
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
                    self.collide_segment(p_bl, p_tl, 0.16, flying_away, was_wall_sliding, &mut substep_wall_norm, &mut substep_wall_hits);
                    self.collide_segment(p_br, p_tr, 0.16, flying_away, was_wall_sliding, &mut substep_wall_norm, &mut substep_wall_hits);
                    self.collide_segment(p_tl, p_tr, 0.20, flying_away, was_wall_sliding, &mut substep_wall_norm, &mut substep_wall_hits);
                    if g.pos.y > 0.2 {
                        let g_bl = Vec3::new(p_bl.x, 0.0, p_bl.z);
                        let g_br = Vec3::new(p_br.x, 0.0, p_br.z);
                        self.collide_segment(g_bl, p_bl, 0.12, flying_away, was_wall_sliding, &mut substep_wall_norm, &mut substep_wall_hits);
                        self.collide_segment(g_br, p_br, 0.12, flying_away, was_wall_sliding, &mut substep_wall_norm, &mut substep_wall_hits);
                    }
                }
                GateKind::Dive => {
                    let hw = g.width * 0.5;
                    let c0 = Vec3::new(g.pos.x - hw, g.pos.y, g.pos.z - hw);
                    let c1 = Vec3::new(g.pos.x + hw, g.pos.y, g.pos.z - hw);
                    let c2 = Vec3::new(g.pos.x + hw, g.pos.y, g.pos.z + hw);
                    let c3 = Vec3::new(g.pos.x - hw, g.pos.y, g.pos.z + hw);
                    self.collide_segment(c0, c1, 0.18, flying_away, was_wall_sliding, &mut substep_wall_norm, &mut substep_wall_hits);
                    self.collide_segment(c1, c2, 0.18, flying_away, was_wall_sliding, &mut substep_wall_norm, &mut substep_wall_hits);
                    self.collide_segment(c2, c3, 0.18, flying_away, was_wall_sliding, &mut substep_wall_norm, &mut substep_wall_hits);
                    self.collide_segment(c3, c0, 0.18, flying_away, was_wall_sliding, &mut substep_wall_norm, &mut substep_wall_hits);
                }
                GateKind::Flag => {
                    let top = Vec3::new(g.pos.x, g.height, g.pos.z);
                    self.collide_segment(g.pos, top, 0.14, flying_away, was_wall_sliding, &mut substep_wall_norm, &mut substep_wall_hits);
                }
            }
        }

        if substep_wall_hits > 0 {
            let avg_n = substep_wall_norm.normalized();
            if avg_n.len_sq() > 0.5 {
                self.wall_normal = avg_n;
            }
            if !self.wall_slide {
                self.wall_slide = true;
                self.wall_slide_time = 0.0;
            }
            self.wall_contact_age = 0.0;
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

    #[test]
    fn test_angle_capped_while_horizon_flips() {
        let track = &TRACKS[0];

        // 1. ANGLE mode at 100% roll stick must stay capped upright (never flip!)
        let mut cfg_angle = Config::default();
        cfg_angle.flight_mode = FlightMode::Angle;
        let mut q_angle = QuadState::spawn(track, &cfg_angle);
        q_angle.pos = Vec3::new(0.0, 15.0, 0.0);
        q_angle.on_ground = false;
        let full_roll = StickInput { throttle: 0.5, yaw: 0.0, pitch: 0.0, roll: 1.0 };
        for _ in 0..160 {
            q_angle.step(full_roll, &mut cfg_angle, track);
        }
        let (_, up_angle, _) = q_angle.rot.basis();
        assert!(up_angle.y > 0.45, "ANGLE mode must stay capped near 55 deg, got up.y={}", up_angle.y);

        // 2. HORIZON mode at 75% roll stick must freely roll past 90 deg and flip inverted!
        let mut cfg_hor = Config::default();
        cfg_hor.flight_mode = FlightMode::Horizon;
        let mut q_hor = QuadState::spawn(track, &cfg_hor);
        q_hor.pos = Vec3::new(0.0, 15.0, 0.0);
        q_hor.on_ground = false;
        let roll_75 = StickInput { throttle: 0.5, yaw: 0.0, pitch: 0.0, roll: 0.75 };
        let mut went_inverted = false;
        for _ in 0..160 {
            q_hor.step(roll_75, &mut cfg_hor, track);
            if q_hor.rot.basis().1.y < -0.2 {
                went_inverted = true;
            }
        }
        assert!(went_inverted, "HORIZON mode at 75% stick should smoothly flip inverted!");

        // 3. Releasing stick in HORIZON must self-level back upright
        let center = StickInput { throttle: 0.45, yaw: 0.0, pitch: 0.0, roll: 0.0 };
        for _ in 0..240 {
            q_hor.step(center, &mut cfg_hor, track);
        }
        assert!(q_hor.rot.basis().1.y > 0.92, "HORIZON mode should self-level when stick centered!");
    }

    #[test]
    fn test_touching_outside_of_gate_scores_no_points() {
        let mut cfg = Config::default();
        cfg.game_mode = GameMode::Freestyle;
        let track = &TRACKS[0]; // Gate 0 at (0, 0, 22), width=4.8 (posts at x = +-2.4)
        let mut quad = QuadState::spawn(track, &cfg);
        quad.pos = Vec3::new(2.55, 1.8, 21.5); // Outside right post!
        quad.vel = Vec3::new(0.0, 0.0, 12.0);
        quad.on_ground = false;
        let input = StickInput { throttle: 0.40, yaw: 0.0, pitch: 0.0, roll: 0.0 };
        for _ in 0..30 {
            quad.step(input, &mut cfg, track);
        }
        assert_eq!(quad.score, 0, "Touching/flying outside a gate post must NOT score points!");
    }

    #[test]
    fn test_crash_flip_ignores_throttle_and_flips_upright() {
        let mut cfg = Config::default();
        cfg.aux_turtle = AuxBinding::Button { code: 304 };
        let track = &TRACKS[0];
        let mut quad = QuadState::spawn(track, &cfg);
        // Flip quad upside down on the ground and engage Crash Flip via connected AUX
        quad.rot = Quat::from_axis_angle(Vec3::FORWARD, std::f32::consts::PI);
        quad.on_ground = true;
        quad.turtle_aux_connected = true;
        quad.turtle_mode = true;

        // 1. Full throttle with centered roll/pitch must do NOTHING (zero motor_thr, stays on ground)
        let full_thr = StickInput { throttle: 1.0, yaw: 0.0, pitch: 0.0, roll: 0.0 };
        for _ in 0..60 {
            quad.step(full_thr, &mut cfg, track);
        }
        assert!(quad.turtle_active, "Crash Flip should be active on the ground");
        assert!(quad.motor_thr < 0.02, "Throttle must be silenced in Crash Flip, got motor_thr={}", quad.motor_thr);
        assert!(quad.on_ground, "Throttle must not lift quad in Crash Flip");

        // 2. Push roll to flip cleanly 180 deg upright!
        let push_roll = StickInput { throttle: 0.0, yaw: 0.0, pitch: 0.0, roll: 1.0 };
        for _ in 0..120 {
            quad.step(push_roll, &mut cfg, track);
        }
        let (_, up, _) = quad.rot.basis();
        assert!(up.y > 0.75, "Crash Flip must flip inverted quad upright, got up.y={}", up.y);
    }

    #[test]
    fn test_auto_turtle_rolls_upright_without_connected_aux() {
        let mut cfg = Config::default();
        // Even if a binding exists in config, without a connected controller providing it,
        // turtle_aux_connected stays false and roll-to-flip works automatically.
        cfg.aux_turtle = AuxBinding::Button { code: 304 };
        let track = &TRACKS[0];
        let mut quad = QuadState::spawn(track, &cfg);
        quad.rot = Quat::from_axis_angle(Vec3::FORWARD, std::f32::consts::PI);
        quad.on_ground = true;
        let idle = StickInput { throttle: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0 };
        quad.apply_aux(AuxInputs::default(), idle, &mut cfg);
        assert!(!quad.turtle_aux_connected);

        let push_roll = StickInput { throttle: 0.0, yaw: 0.0, pitch: 0.0, roll: 1.0 };
        for _ in 0..120 {
            quad.step(push_roll, &mut cfg, track);
        }
        let (_, up, _) = quad.rot.basis();
        assert!(up.y > 0.75, "Auto-turtle should roll upright without Crash Flip button, got up.y={}", up.y);
        assert!(!quad.turtle_active, "Auto-turtle should exit once flipped upright");
    }

    #[test]
    fn test_auto_arm_ready_without_connected_aux() {
        let mut cfg = Config::default();
        cfg.aux_arm = AuxBinding::Axis { code: 2, invert: false };
        let track = &TRACKS[0];
        let mut quad = QuadState::spawn(track, &cfg);
        let idle = StickInput { throttle: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0 };

        // 1. No connected controller providing ARM AUX -> auto-armed & READY
        quad.apply_aux(AuxInputs::default(), idle, &mut cfg);
        assert!(!quad.arm_aux_connected);
        assert!(quad.armed, "Without a connected ARM AUX, quad should auto-arm immediately");

        // 2. Connected controller providing ARM AUX low -> disarms cleanly
        let mut connected_disarmed = AuxInputs::default();
        connected_disarmed.arm_aux_connected = true;
        connected_disarmed.arm_switch = Some(false);
        quad.apply_aux(connected_disarmed, idle, &mut cfg);
        assert!(quad.arm_aux_connected);
        assert!(!quad.armed, "Connected ARM AUX switch low should disarm quad");
    }

    #[test]
    fn test_horizon_holds_half_stick_bank_and_recovers_pitch_flip_cleanly() {
        let track = &TRACKS[0];
        let half_roll = StickInput { throttle: 0.65, yaw: 0.0, pitch: 0.0, roll: 0.45 };

        // 1. At 45% stick held for 3 seconds (720 steps), Horizon must hold a steady bank
        //    (steeper than Angle mode, yet NEVER accidentally flip inverted!)
        let mut cfg_ang = Config::default();
        cfg_ang.flight_mode = FlightMode::Angle;
        let mut q_ang = QuadState::spawn(track, &cfg_ang);
        q_ang.pos = Vec3::new(0.0, 25.0, 0.0);
        q_ang.on_ground = false;

        let mut cfg_hor = Config::default();
        cfg_hor.flight_mode = FlightMode::Horizon;
        let mut q_hor = QuadState::spawn(track, &cfg_hor);
        q_hor.pos = Vec3::new(0.0, 25.0, 0.0);
        q_hor.on_ground = false;

        for _ in 0..720 {
            q_ang.step(half_roll, &mut cfg_ang, track);
            q_hor.step(half_roll, &mut cfg_hor, track);
            assert!(
                q_hor.rot.basis().1.y > 0.25,
                "Horizon at 45% stick must NEVER flip inverted, got up.y={}",
                q_hor.rot.basis().1.y
            );
        }
        let tilt_ang = q_ang.rot.basis().1.y.acos().to_degrees();
        let tilt_hor = q_hor.rot.basis().1.y.acos().to_degrees();
        assert!(
            tilt_hor > tilt_ang + 8.0 && tilt_hor < 65.0,
            "Horizon 45% stick bank ({tilt_hor:.1} deg) should be livelier than Angle ({tilt_ang:.1} deg) without flipping"
        );

        // 2. Pure 145-deg pitch inverted recovery with centered sticks must recover purely along
        //    pitch without yanking sideways on the roll axis!
        let mut q_flip = QuadState::spawn(track, &cfg_hor);
        q_flip.pos = Vec3::new(0.0, 25.0, 0.0);
        q_flip.on_ground = false;
        q_flip.rot = Quat::from_axis_angle(Vec3::RIGHT, 145.0_f32.to_radians());
        q_flip.horizon_smooth = 0.0; // Just exited a full-stick frontflip
        let centered = StickInput { throttle: 0.50, yaw: 0.0, pitch: 0.0, roll: 0.0 };
        for _ in 0..240 {
            q_flip.step(centered, &mut cfg_hor, track);
            let (right, _, _) = q_flip.rot.basis();
            assert!(
                right.y.abs() < 0.02,
                "Pure pitch recovery in Horizon must not twist sideways onto roll, right.y={}",
                right.y
            );
        }
        assert!(
            q_flip.rot.basis().1.y > 0.96,
            "Horizon should smoothly recover upright from 145-deg pitch, got up.y={}",
            q_flip.rot.basis().1.y
        );
    }

    #[test]
    fn test_object_collision_slow_angled_fall_until_grounded() {
        // Dive Tower core AABB: x=-3..3, y=0..26, z=44..50 (front face at z = 44.0 - QUAD_RADIUS = 43.78)
        let track = &TRACKS[2];

        for mode in [FlightMode::Acro, FlightMode::Horizon, FlightMode::Angle] {
            let mut cfg = Config::default();
            cfg.flight_mode = mode;
            let mut quad = QuadState::spawn(track, &cfg);
            quad.pos = Vec3::new(0.0, 5.0, 43.2);
            quad.vel = Vec3::new(0.0, 0.0, 12.0);
            quad.on_ground = false;

            // Keep neutral sticks & hover-like throttle (pilot does NOT fly away)
            let hold_hover = StickInput { throttle: 0.42, yaw: 0.0, pitch: 0.0, roll: 0.0 };

            // After 0.6s (144 steps), quad should be sliding slowly angled towards the ground against the wall
            for _ in 0..144 {
                quad.step(hold_hover, &mut cfg, track);
            }
            let (_, up, fwd) = quad.rot.basis();
            assert!(
                quad.wall_slide,
                "{:?}: should remain in wall_slide against wall while falling",
                mode
            );
            assert!(
                (quad.pos.z - (44.0 - QUAD_RADIUS)).abs() < 0.12,
                "{:?}: should stay against the object face (z≈43.78), got z={:.3}",
                mode,
                quad.pos.z
            );
            assert!(
                fwd.y < -0.55 && up.y > 0.35,
                "{:?}: should angle nose towards the ground against object, fwd.y={:.3}, up.y={:.3}",
                mode,
                fwd.y,
                up.y
            );
            assert!(
                quad.vel.y < -1.0 && quad.vel.y > -3.2,
                "{:?}: should fall slowly against object, got vy={:.3}",
                mode,
                quad.vel.y
            );
            assert!(
                !quad.on_ground && quad.pos.y < 4.5 && quad.pos.y > 2.0,
                "{:?}: fall from 5m should be gradual at 0.6s, got y={:.3}",
                mode,
                quad.pos.y
            );

            // Continue without flying away: should stay angled against the wall all the way until grounded
            let mut grounded = false;
            for _ in 0..960 {
                quad.step(hold_hover, &mut cfg, track);
                if quad.on_ground {
                    grounded = true;
                    break;
                }
            }
            assert!(
                grounded && quad.on_ground && !quad.wall_slide,
                "{:?}: should slowly fall all the way until grounded, y={:.3}, on_ground={}, wall_slide={}",
                mode,
                quad.pos.y,
                quad.on_ground,
                quad.wall_slide
            );
        }
    }

    #[test]
    fn test_object_collision_fly_away_regains_balance() {
        let track = &TRACKS[2]; // Dive Tower front face at z = 44.0

        for mode in [FlightMode::Acro, FlightMode::Horizon, FlightMode::Angle] {
            let mut cfg = Config::default();
            cfg.flight_mode = mode;
            let mut quad = QuadState::spawn(track, &cfg);
            quad.pos = Vec3::new(0.0, 8.0, 43.2);
            quad.vel = Vec3::new(0.0, 0.0, 12.0);
            quad.on_ground = false;

            let neutral = StickInput { throttle: 0.42, yaw: 0.0, pitch: 0.0, roll: 0.0 };
            for _ in 0..120 {
                quad.step(neutral, &mut cfg, track);
            }
            assert!(quad.wall_slide, "{:?}: expected wall_slide after impact", mode);

            // Pull pitch back + throttle to fly away from the wall
            let pull_away = StickInput { throttle: 0.58, yaw: 0.0, pitch: -0.60, roll: 0.0 };
            for _ in 0..160 {
                quad.step(pull_away, &mut cfg, track);
            }
            assert!(
                !quad.wall_slide,
                "{:?}: flying away should exit wall_slide",
                mode
            );
            assert!(
                quad.pos.z < 42.5,
                "{:?}: should fly clearly away from wall (z=43.78), got z={:.3}",
                mode,
                quad.pos.z
            );
            assert!(
                !quad.on_ground && quad.pos.y > 4.0,
                "{:?}: should stay airborne after flying away, got y={:.3}",
                mode,
                quad.pos.y
            );

            // Center sticks to verify balance recovery
            let level_hover = StickInput { throttle: 0.48, yaw: 0.0, pitch: 0.0, roll: 0.0 };
            for _ in 0..180 {
                quad.step(level_hover, &mut cfg, track);
            }
            let (_, up_after, _) = quad.rot.basis();
            if mode != FlightMode::Acro {
                assert!(
                    up_after.y > 0.92,
                    "{:?}: should regain upright balance after flying away, up.y={:.3}",
                    mode,
                    up_after.y
                );
            }
        }
    }

    #[test]
    fn test_pushing_into_wall_with_throttle_still_slowly_falls() {
        let track = &TRACKS[2]; // Dive Tower front face at z = 44.0
        let mut cfg = Config::default();
        cfg.flight_mode = FlightMode::Angle;
        let mut quad = QuadState::spawn(track, &cfg);
        quad.pos = Vec3::new(0.0, 4.0, 43.2);
        quad.vel = Vec3::new(0.0, 1.5, 10.0);
        quad.on_ground = false;

        // Keep pushing forward into the wall with 65% throttle (NOT flying away)
        let push_into_wall = StickInput { throttle: 0.65, yaw: 0.0, pitch: 0.55, roll: 0.0 };
        for _ in 0..240 {
            quad.step(push_into_wall, &mut cfg, track);
        }
        assert!(
            quad.wall_slide && quad.pos.y < 3.0 && quad.vel.y < -0.5,
            "Pushing into wall with high throttle must still slowly fall (not climb!), y={:.3}, vy={:.3}, wall_slide={}",
            quad.pos.y,
            quad.vel.y,
            quad.wall_slide
        );
    }

    #[test]
    fn test_sideways_wall_slide_and_gate_post_slide() {
        // 1. Sideways hit into Bando right wall (x: 11.0..14.0, y: 0..12, z: 24..72)
        let bando = &TRACKS[3];
        let mut cfg = Config::default();
        cfg.flight_mode = FlightMode::Horizon;
        let mut q = QuadState::spawn(bando, &cfg);
        q.pos = Vec3::new(10.4, 6.0, 35.0);
        q.vel = Vec3::new(10.0, 0.0, 0.0);
        q.on_ground = false;

        let hover = StickInput { throttle: 0.42, yaw: 0.0, pitch: 0.0, roll: 0.0 };
        for _ in 0..120 {
            q.step(hover, &mut cfg, bando);
        }
        let (right, _, _) = q.rot.basis();
        assert!(q.wall_slide, "Sideways wall hit should engage wall_slide");
        assert!(
            right.y < -0.55,
            "Right side hitting wall should dip right side towards ground, right.y={:.3}",
            right.y
        );

        // Roll left away from right wall -> flies away cleanly
        let roll_away = StickInput { throttle: 0.58, yaw: 0.0, pitch: 0.0, roll: -0.60 };
        for _ in 0..160 {
            q.step(roll_away, &mut cfg, bando);
        }
        assert!(!q.wall_slide && q.pos.x < 9.3, "Rolling left should fly away from right wall, x={:.3}", q.pos.x);

        // 2. Gate post hit (Meadow Gate 0 left post at x=-2.4, z=22.0, height=3.6)
        let meadow = &TRACKS[0];
        let mut qg = QuadState::spawn(meadow, &cfg);
        qg.pos = Vec3::new(-2.4, 2.8, 21.3);
        qg.vel = Vec3::new(0.0, 0.0, 10.0);
        qg.on_ground = false;
        let mut reached_ground = false;
        let mut saw_angled_slide = false;
        for _ in 0..600 {
            qg.step(hover, &mut cfg, meadow);
            if qg.wall_slide && qg.rot.basis().2.y < -0.45 && qg.vel.y < -0.8 {
                saw_angled_slide = true;
            }
            if qg.on_ground {
                reached_ground = true;
                break;
            }
        }
        assert!(saw_angled_slide, "Hitting a gate post should trigger slow angled slide");
        assert!(reached_ground, "Sliding down gate post without escaping should ground the quad");
    }

    #[test]
    fn test_roof_landing_does_not_wall_slide() {
        let dive = &TRACKS[2]; // Tower crown top at y = 28.5, x: -4.5..4.5, z: 42.5..51.5
        let mut cfg = Config::default();
        let mut q = QuadState::spawn(dive, &cfg);
        q.pos = Vec3::new(0.0, 29.1, 47.0);
        q.vel = Vec3::new(0.0, -1.5, 0.0);
        q.on_ground = false;
        let idle = StickInput { throttle: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0 };
        for _ in 0..120 {
            q.step(idle, &mut cfg, dive);
        }
        assert!(!q.wall_slide, "Landing on top of a flat roof should NOT trigger wall_slide");
        assert!(q.on_ground, "Drone should settle grounded on top of the flat roof");
        assert!((q.pos.y - (28.5 + QUAD_RADIUS)).abs() < 0.05, "Should rest on roof surface, y={:.3}", q.pos.y);
    }

    #[test]
    fn test_imu_simplified_takeoff_hover_tilt_and_landing() {
        let track = &TRACKS[0];
        let mut cfg = Config::default();
        cfg.flight_mode = FlightMode::Acro; // IMU simplified mode must still self-level even when global mode is Acro!
        let mut q = QuadState::spawn(track, &cfg);
        q.control_source = FlightControlSource::ImuSimplified;
        let neutral = StickInput { throttle: 0.5, yaw: 0.0, pitch: 0.0, roll: 0.0 };

        // 1. Stays grounded until climb is pressed
        q.imu_climb_cmd = 0.0;
        for _ in 0..120 {
            q.step(neutral, &mut cfg, track);
        }
        assert!(q.on_ground && q.throttle_pct == 0, "Should stay idle on pad until climb is pressed");

        // 2. Press Climb -> lifts off cleanly
        q.imu_climb_cmd = 1.0;
        for _ in 0..240 {
            q.step(neutral, &mut cfg, track);
        }
        assert!(!q.on_ground && q.pos.y > 2.0, "Should take off and climb, y={:.2}", q.pos.y);

        // 3. Release Climb -> auto-hovers steadily with near-zero vertical drift
        q.imu_climb_cmd = 0.0;
        for _ in 0..240 {
            q.step(neutral, &mut cfg, track);
        }
        let hover_y = q.pos.y;
        for _ in 0..240 {
            q.step(neutral, &mut cfg, track);
        }
        assert!((q.pos.y - hover_y).abs() < 0.35, "Auto-hover should hold altitude steadily ({hover_y:.2} -> {:.2})", q.pos.y);

        // 4a. Straight forward tilt -> accelerates cleanly forward (+Z) while holding altitude
        let tilt_fwd = StickInput { throttle: 0.5, yaw: 0.0, pitch: 1.0, roll: 0.0 };
        let start_z = q.pos.z;
        for _ in 0..240 {
            q.step(tilt_fwd, &mut cfg, track);
            let (_, up, _) = q.rot.basis();
            assert!(up.y > 0.72, "Forward tilt must stay upright within 38 deg cap (up.y={:.2})", up.y);
        }
        assert!(q.pos.z > start_z + 2.0, "Forward tilt should fly forward, dz={:.2}", q.pos.z - start_z);

        // 4b. Diagonal full forward + right tilt + coordinated yaw -> stays upright & carves right
        let tilt_fwd_right = StickInput { throttle: 0.5, yaw: 0.6, pitch: 1.0, roll: 1.0 };
        for _ in 0..240 {
            q.step(tilt_fwd_right, &mut cfg, track);
            let (_, up, _) = q.rot.basis();
            assert!(up.y > 0.70, "Diagonal IMU tilt must stay upright within 38 deg cap (up.y={:.2})", up.y);
        }

        // 5. Hold Descend -> lands gently back on ground
        q.imu_climb_cmd = -1.0;
        for _ in 0..960 {
            q.step(neutral, &mut cfg, track);
            if q.on_ground {
                break;
            }
        }
        assert!(q.on_ground, "Holding descend should land cleanly on ground, y={:.2}", q.pos.y);
    }

    #[test]
    fn test_imu_mode_disables_flight_mode_switching() {
        let track = &TRACKS[0];
        let mut cfg = Config::default();
        cfg.flight_mode = FlightMode::Acro;
        let mut q = QuadState::spawn(track, &cfg);
        q.control_source = FlightControlSource::ImuSimplified;

        let sticks = StickInput { throttle: 0.5, yaw: 0.0, pitch: 0.0, roll: 0.0 };
        let aux_toggle = AuxInputs { mode_toggle: true, ..AuxInputs::default() };
        q.apply_aux(aux_toggle, sticks, &mut cfg);
        assert_eq!(cfg.flight_mode, FlightMode::Acro, "mode_toggle must be ignored in IMU mode");

        let aux_switch = AuxInputs { mode_switch: Some(FlightMode::Angle), ..AuxInputs::default() };
        q.apply_aux(aux_switch, sticks, &mut cfg);
        assert_eq!(cfg.flight_mode, FlightMode::Acro, "mode_switch must be ignored in IMU mode");

        // Switching back to Controller re-enables mode changes
        q.control_source = FlightControlSource::Controller;
        q.apply_aux(aux_switch, sticks, &mut cfg);
        assert_eq!(cfg.flight_mode, FlightMode::Angle, "mode_switch should work in Controller mode");
    }
}
