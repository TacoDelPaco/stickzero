//! StickZero — Minimal native Rust 3D FPV quadcopter simulator for M5Stack CardputerZero.
//!
//! Designed for low-latency flight on CardputerZero's Raspberry Pi CM0 (Quad Cortex-A53 @ 1GHz)
//! and 320x170 ST7789v3 LCD over Linux fbdev, with USB-C & Bluetooth controller support.

mod audio;
mod config;
mod display;
mod font;
mod hidraw;
mod imu;
mod input;
mod linux_sys;
mod math;
mod physics;
mod png;
mod renderer;
mod tracks;

use audio::AudioEngine;
use config::{AuxBinding, BatteryPack, Config, FlightMode, RatesPreset, StickOverlay, ThrottleMode, UiTheme, VisualTheme};
use display::{FbDisplay, SCREEN_H, SCREEN_W};
use input::{FlightControlSource, InputManager};
use linux_sys::{signal, SIGINT, SIGTERM};
use math::{Quat, Vec3};
use physics::{QuadState, StickInput, PHYSICS_DT};
use renderer::Renderer;
use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};
use tracks::TRACKS;

static RUNNING: AtomicBool = AtomicBool::new(true);

extern "C" fn handle_signal(_sig: i32) {
    RUNNING.store(false, Ordering::SeqCst);
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum AppScreen {
    ControllerWarning,
    MainMenu,
    DroneTuning,
    Flying,
    Paused,
    Settings,
    Controller,
}

fn run_benchmark() {
    let mut cfg = Config::default();
    let mut renderer = Renderer::new();
    let track = &TRACKS[3]; // Bando (heaviest geometry)
    let mut quad = QuadState::spawn(track, &cfg);
    quad.pos = Vec3::new(0.0, 5.0, 10.0);
    quad.vel = Vec3::new(0.0, 0.0, 18.0);
    quad.on_ground = false;

    const FRAMES: usize = 600;
    for overlay in [StickOverlay::Dot, StickOverlay::Trails] {
        cfg.stick_overlay = overlay;
        renderer.reset_stick_trail();
        let t0 = Instant::now();
        for f in 0..FRAMES {
            let phase = (f as f32) * 0.12;
            let sticks = StickInput {
                throttle: 0.52 + 0.12 * phase.sin(),
                yaw: 0.18 * phase.cos(),
                pitch: 0.22 * (phase * 1.3).sin(),
                roll: 0.28 * (phase * 0.9).cos(),
            };
            for _ in 0..4 {
                quad.step(sticks, &mut cfg, track);
            }
            renderer.render_flight(&quad, &cfg, track, sticks, 60, 1.0);
        }
        let elapsed = t0.elapsed();
        let ms_per_frame = elapsed.as_secs_f64() * 1000.0 / (FRAMES as f64);
        let fps = 1000.0 / ms_per_frame;
        println!(
            "StickZero benchmark [Stick OSD: {:8}] ({} frames @ 320x170 Z-buf + 240Hz): {:.3} ms/frame ({:.0} FPS)",
            overlay.name(), FRAMES, ms_per_frame, fps
        );
    }
}

fn seed_demo_stick_trail(renderer: &mut Renderer, end_sticks: StickInput) {
    renderer.reset_stick_trail();
    for i in 0..16 {
        let t = (i as f32) / 15.0;
        let lx = end_sticks.yaw - 0.55 * (1.0 - t);
        let ly = (end_sticks.throttle * 2.0 - 1.0) - 0.55 * (1.0 - t) * (1.0 - 0.35 * t);
        let rx = end_sticks.roll - 0.65 * (1.0 - t) * (1.0 - 0.3 * t);
        let ry = end_sticks.pitch - 0.55 * (1.0 - t) * (t * 2.0).cos();
        renderer.push_stick_trail(lx, ly, rx, ry);
    }
}

fn generate_screenshots(dir: &Path) -> Result<(), String> {
    let mut cfg = Config::default(); // Monokai default + StickOverlay::Trails
    let mut renderer = Renderer::new();
    let input = InputManager::new();

    // 1. Main Menu (Monokai default)
    renderer.render_main_menu(&cfg, 0, &input);
    png::save_rgb565_png(&dir.join("01-main-menu.png"), SCREEN_W, SCREEN_H, &*renderer.fb)?;

    // 2. Summer Meadow Race Flight (with Analog FX + Betaflight OSD + Trailing Sticks)
    cfg.theme = VisualTheme::Summer;
    cfg.track_index = 0;
    cfg.game_mode = config::GameMode::Race;
    let track = &TRACKS[0];
    let mut quad = QuadState::spawn(track, &cfg);
    quad.pos = Vec3::new(-1.2, 2.4, 8.0);
    quad.vel = Vec3::new(1.0, 0.2, 16.5);
    quad.on_ground = false;
    quad.rot = Quat::from_axis_angle(Vec3::FORWARD, -0.18).mul(Quat::from_axis_angle(Vec3::RIGHT, 0.12));
    quad.lap_time = 12.48;
    quad.flight_time = 44.0;
    quad.mah_used = 128.0;
    quad.throttle_pct = 54;
    quad.cell_voltage = 3.94;
    let sticks = StickInput { throttle: 0.54, yaw: 0.12, pitch: 0.28, roll: 0.22 };
    seed_demo_stick_trail(&mut renderer, sticks);
    renderer.render_flight(&quad, &cfg, track, sticks, 60, 0.9);
    png::save_rgb565_png(&dir.join("02-meadow-summer.png"), SCREEN_W, SCREEN_H, &*renderer.fb)?;

    // 3. Night Bando Freestyle (Z-buffered concrete ruin + high-contrast night horizon)
    cfg.theme = VisualTheme::Night;
    cfg.track_index = 3;
    cfg.game_mode = config::GameMode::Freestyle;
    let bando = &TRACKS[3];
    let mut q2 = QuadState::spawn(bando, &cfg);
    q2.pos = Vec3::new(0.0, 6.5, 8.0);
    q2.vel = Vec3::new(0.5, 1.5, 18.0);
    q2.on_ground = false;
    q2.rot = Quat::from_axis_angle(Vec3::FORWARD, 0.28);
    q2.score = 1450;
    q2.flight_time = 82.0;
    q2.mah_used = 245.0;
    q2.throttle_pct = 62;
    q2.combo_text = "POWER FLIP +250";
    q2.combo_timer = 1.0;
    q2.cell_voltage = 3.78;
    let sticks2 = StickInput { throttle: 0.62, yaw: -0.14, pitch: 0.36, roll: -0.32 };
    seed_demo_stick_trail(&mut renderer, sticks2);
    renderer.render_flight(&q2, &cfg, bando, sticks2, 60, 1.1);
    png::save_rgb565_png(&dir.join("03-bando-night.png"), SCREEN_W, SCREEN_H, &*renderer.fb)?;

    // 4. Controller Monitor & AUX Switch Binder screen (Monokai)
    cfg.throttle_mode = ThrottleMode::RcFull;
    renderer.render_controller_screen(&cfg, &input);
    png::save_rgb565_png(&dir.join("04-controller.png"), SCREEN_W, SCREEN_H, &*renderer.fb)?;

    println!("Saved 4 CardputerZero 320x170 screenshots to {}", dir.display());
    Ok(())
}

fn generate_previews(dir: &Path) -> Result<(), String> {
    let mut cfg = Config::default();
    let mut renderer = Renderer::new();
    let input = InputManager::new();

    // 1. Monokai Main Menu (Default)
    cfg.ui_theme = UiTheme::Monokai;
    renderer.render_main_menu(&cfg, 0, &input);
    png::save_rgb565_png(&dir.join("monokai-main-menu.png"), SCREEN_W, SCREEN_H, &*renderer.fb)?;

    // 2. StickTime Main Menu (Alternate)
    cfg.ui_theme = UiTheme::StickTime;
    renderer.render_main_menu(&cfg, 0, &input);
    png::save_rgb565_png(&dir.join("sticktime-main-menu.png"), SCREEN_W, SCREEN_H, &*renderer.fb)?;

    // 3. Monokai Drone Tuning Menu (real physical units: Weight (g): 37 with Lava II 680mAh)
    cfg.ui_theme = UiTheme::Monokai;
    cfg.set_battery_pack(BatteryPack::LavaII680);
    renderer.render_drone_tuning_menu(&cfg, 3);
    png::save_rgb565_png(&dir.join("monokai-drone-tuning.png"), SCREEN_W, SCREEN_H, &*renderer.fb)?;

    // 4. Monokai Settings showing UI Theme + Stick OSD: Trails
    cfg.flight_mode = FlightMode::Horizon;
    cfg.stick_overlay = StickOverlay::Trails;
    renderer.render_settings_menu(&cfg, 10, false);
    png::save_rgb565_png(&dir.join("monokai-settings.png"), SCREEN_W, SCREEN_H, &*renderer.fb)?;

    // 4. Clean zero-overlap Wireframe Bando + Trailing Sticks OSD
    cfg.theme = VisualTheme::Wireframe;
    cfg.track_index = 3;
    cfg.game_mode = config::GameMode::Freestyle;
    cfg.analog_filter = false;
    let bando = &TRACKS[3];
    let mut q = QuadState::spawn(bando, &cfg);
    q.pos = Vec3::new(-4.5, 4.2, 6.0);
    q.vel = Vec3::new(1.5, 0.4, 15.0);
    q.on_ground = false;
    q.rot = Quat::from_axis_angle(Vec3::UP, 0.14)
        .mul(Quat::from_axis_angle(Vec3::FORWARD, -0.14))
        .mul(Quat::from_axis_angle(Vec3::RIGHT, 0.06));
    q.score = 820;
    q.flight_time = 51.0;
    q.mah_used = 165.0;
    q.throttle_pct = 52;
    q.cell_voltage = 3.88;
    let sticks = StickInput { throttle: 0.52, yaw: 0.25, pitch: 0.34, roll: -0.28 };
    seed_demo_stick_trail(&mut renderer, sticks);
    renderer.render_flight(&q, &cfg, bando, sticks, 60, 0.4);
    png::save_rgb565_png(&dir.join("wireframe-bando.png"), SCREEN_W, SCREEN_H, &*renderer.fb)?;

    // 5. Clean zero-overlap Wireframe Dive Tower + Trailing Sticks OSD
    cfg.track_index = 2;
    let dive = &TRACKS[2];
    let mut qd = QuadState::spawn(dive, &cfg);
    qd.pos = Vec3::new(6.0, 9.5, 22.0);
    qd.vel = Vec3::new(-1.0, 0.5, 16.0);
    qd.on_ground = false;
    qd.rot = Quat::from_axis_angle(Vec3::UP, -0.18)
        .mul(Quat::from_axis_angle(Vec3::RIGHT, 0.16));
    qd.score = 640;
    qd.flight_time = 38.0;
    qd.mah_used = 112.0;
    qd.throttle_pct = 58;
    qd.cell_voltage = 3.92;
    let sticks_d = StickInput { throttle: 0.58, yaw: -0.22, pitch: 0.40, roll: 0.35 };
    seed_demo_stick_trail(&mut renderer, sticks_d);
    renderer.render_flight(&qd, &cfg, dive, sticks_d, 60, 0.4);
    png::save_rgb565_png(&dir.join("wireframe-dive-tower.png"), SCREEN_W, SCREEN_H, &*renderer.fb)?;

    // 6. CardputerZero IMU Simplified Flight OSD (Summer Meadow, hover-assist vertical gauge + IMU tilt pad)
    cfg.theme = VisualTheme::Summer;
    cfg.track_index = 0;
    cfg.game_mode = config::GameMode::Race;
    cfg.analog_filter = true;
    let meadow = &TRACKS[0];
    let mut qi = QuadState::spawn(meadow, &cfg);
    qi.control_source = FlightControlSource::ImuSimplified;
    qi.imu_climb_cmd = 0.0;
    qi.pos = Vec3::new(-0.8, 2.2, 8.0);
    qi.vel = Vec3::new(0.8, 0.0, 12.5);
    qi.on_ground = false;
    qi.rot = Quat::from_axis_angle(Vec3::FORWARD, -0.14).mul(Quat::from_axis_angle(Vec3::RIGHT, 0.10));
    qi.lap_time = 9.35;
    qi.flight_time = 24.0;
    qi.mah_used = 64.0;
    qi.throttle_pct = 38;
    qi.cell_voltage = 4.02;
    let sticks_i = StickInput { throttle: 0.50, yaw: 0.16, pitch: 0.30, roll: 0.26 };
    seed_demo_stick_trail(&mut renderer, sticks_i);
    renderer.render_flight(&qi, &cfg, meadow, sticks_i, 60, 0.9);
    png::save_rgb565_png(&dir.join("imu-simplified-osd.png"), SCREEN_W, SCREEN_H, &*renderer.fb)?;

    // 7. Launch Controller Required Warning (No IMU / CardputerZero Lite)
    cfg.ui_theme = UiTheme::Monokai;
    renderer.render_controller_warning(&cfg, &input);
    png::save_rgb565_png(&dir.join("controller-required-warning.png"), SCREEN_W, SCREEN_H, &*renderer.fb)?;

    println!("Saved 7 CardputerZero 320x170 preview PNGs to {}", dir.display());
    Ok(())
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.iter().any(|a| a == "--bench") {
        run_benchmark();
        return;
    }
    if let Some(pos) = args.iter().position(|a| a == "--screenshot") {
        let dir = args.get(pos + 1).map(|s| s.as_str()).unwrap_or("screenshots");
        if let Err(e) = generate_screenshots(Path::new(dir)) {
            eprintln!("screenshot error: {e}");
            std::process::exit(1);
        }
        return;
    }
    if let Some(pos) = args.iter().position(|a| a == "--preview") {
        let dir = args.get(pos + 1).map(|s| s.as_str()).unwrap_or("previews");
        if let Err(e) = generate_previews(Path::new(dir)) {
            eprintln!("preview error: {e}");
            std::process::exit(1);
        }
        return;
    }

    unsafe {
        signal(SIGINT, handle_signal);
        signal(SIGTERM, handle_signal);
    }

    let mut display = match FbDisplay::open_auto() {
        Ok(d) => d,
        Err(e) => {
            eprintln!("StickZero display init failed: {e}");
            eprintln!("Tip: set STICKZERO_FBDEV=/dev/fb1 or run with --bench / --screenshot <dir>");
            std::process::exit(1);
        }
    };

    let mut cfg = Config::load();
    let audio = AudioEngine::start(cfg.sound_on);
    let mut input = InputManager::new();
    let mut renderer = Renderer::new();
    let mut screen = if input.needs_controller_warning() {
        AppScreen::ControllerWarning
    } else {
        AppScreen::MainMenu
    };
    let mut sub_return = AppScreen::MainMenu;
    let mut menu_idx: usize = 0;
    let mut drone_idx: usize = 0;
    let mut pause_idx: usize = 0;
    let mut settings_idx: usize = 0;
    let mut quad = QuadState::spawn(&TRACKS[cfg.track_index], &cfg);
    quad.control_source = input.control_source();

    let target_frame = Duration::from_micros(16_667); // 60 Hz target
    let mut last_frame = Instant::now();
    let mut phys_accum = 0.0_f32;
    let mut fps_counter = 0u32;
    let mut fps_display = 60u32;
    let mut fps_timer = Instant::now();
    let mut last_render_ms = 1.0_f32;

    while RUNNING.load(Ordering::Relaxed) {
        let frame_start = Instant::now();
        let dt = (frame_start - last_frame).as_secs_f32().min(0.05);
        last_frame = frame_start;

        let in_menu = screen != AppScreen::Flying;
        let motors_sounding = !in_menu && quad.armed && (!quad.turtle_active || quad.motor_thr > 0.03);
        audio.set_flying(motors_sounding, cfg.analog_filter);
        input.poll(dt, in_menu, &cfg);
        let act = input.take_menu_actions();

        if act.exit_app {
            break;
        }

        match screen {
            AppScreen::ControllerWarning => {
                if input.has_controller() || !input.needs_controller_warning() || act.select || act.back {
                    screen = AppScreen::MainMenu;
                }
                renderer.render_controller_warning(&cfg, &input);
            }
            AppScreen::MainMenu => {
                if act.up { menu_idx = (menu_idx + 5) % 6; }
                if act.down { menu_idx = (menu_idx + 1) % 6; }
                if act.left || act.right || act.select {
                    match menu_idx {
                        0 if act.select => {
                            quad = QuadState::spawn(&TRACKS[cfg.track_index], &cfg);
                            quad.control_source = input.control_source();
                            input.reset_kb_throttle();
                            if quad.control_source == FlightControlSource::ImuSimplified {
                                input.recenter_imu();
                            }
                            renderer.reset_stick_trail();
                            phys_accum = 0.0;
                            if quad.armed {
                                audio.trigger_arm();
                            }
                            screen = AppScreen::Flying;
                        }
                        1 => {
                            cfg.track_index = if act.left {
                                (cfg.track_index + TRACKS.len() - 1) % TRACKS.len()
                            } else {
                                (cfg.track_index + 1) % TRACKS.len()
                            };
                            cfg.save();
                        }
                        2 => {
                            if act.select {
                                drone_idx = 0;
                                sub_return = AppScreen::MainMenu;
                                screen = AppScreen::DroneTuning;
                            } else {
                                let next_q = if act.left { cfg.quad.prev() } else { cfg.quad.next() };
                                cfg.set_quad(next_q);
                                cfg.save();
                            }
                        }
                        3 => {
                            cfg.theme = cfg.theme.next();
                            cfg.save();
                        }
                        4 if act.select => {
                            settings_idx = 0;
                            sub_return = AppScreen::MainMenu;
                            screen = AppScreen::Settings;
                        }
                        5 if act.select => {
                            input.scan_devices();
                            screen = AppScreen::Controller;
                        }
                        _ => {}
                    }
                }
                renderer.render_main_menu(&cfg, menu_idx, &input);
            }
            AppScreen::DroneTuning => {
                const NUM_DRONE: usize = 11;
                if act.back {
                    cfg.save();
                    screen = sub_return;
                } else {
                    if act.up { drone_idx = (drone_idx + NUM_DRONE - 1) % NUM_DRONE; }
                    if act.down { drone_idx = (drone_idx + 1) % NUM_DRONE; }
                    if act.left || act.right || act.select {
                        match drone_idx {
                            0 => {
                                let next_q = if act.left { cfg.quad.prev() } else { cfg.quad.next() };
                                cfg.set_quad(next_q);
                            }
                            1 => {
                                let next_b = if act.left {
                                    cfg.battery_pack.prev()
                                } else {
                                    cfg.battery_pack.next()
                                };
                                cfg.set_battery_pack(next_b);
                            }
                            2 => {
                                cfg.prop_wash = if act.left {
                                    cfg.prop_wash.prev()
                                } else {
                                    cfg.prop_wash.next()
                                };
                            }
                            3 => {
                                // Weight (g): 1g steps for whoops (<=100g), 5g for micros, 10g for 5"
                                let step: u16 = if cfg.weight_g <= 100 { 1 } else if cfg.weight_g <= 250 { 5 } else { 10 };
                                cfg.weight_g = if act.left {
                                    cfg.weight_g.saturating_sub(step).max(15)
                                } else {
                                    (cfg.weight_g + step).min(1500)
                                };
                            }
                            4 => {
                                // Thrust (g): 5g steps for whoops (<=300g), 20g for micros, 100g for 5"
                                let step: u16 = if cfg.thrust_g <= 300 { 5 } else if cfg.thrust_g <= 1000 { 20 } else { 100 };
                                cfg.thrust_g = if act.left {
                                    cfg.thrust_g.saturating_sub(step).max(40)
                                } else {
                                    (cfg.thrust_g + step).min(8000)
                                };
                            }
                            5 => {
                                // Air Drag (0.05..=1.00, step 0.02)
                                cfg.air_drag = if act.left {
                                    cfg.air_drag.saturating_sub(2).max(5)
                                } else {
                                    (cfg.air_drag + 2).min(100)
                                };
                            }
                            6 => {
                                // Rotor Grip (0.00..=1.00, step 0.02)
                                cfg.rotor_grip = if act.left {
                                    cfg.rotor_grip.saturating_sub(2)
                                } else {
                                    (cfg.rotor_grip + 2).min(100)
                                };
                            }
                            7 => {
                                // Motor Lag (ms): 5..=45 ms, step 1 ms
                                cfg.motor_lag_ms = if act.left {
                                    cfg.motor_lag_ms.saturating_sub(1).max(5)
                                } else {
                                    (cfg.motor_lag_ms + 1).min(45)
                                };
                            }
                            8 => {
                                // Throttle Mid (0.20..=0.70, step 0.02)
                                cfg.thr_mid = if act.left {
                                    cfg.thr_mid.saturating_sub(2).max(20)
                                } else {
                                    (cfg.thr_mid + 2).min(70)
                                };
                            }
                            9 => {
                                // Throttle Expo (0.00..=0.80, step 0.05)
                                cfg.thr_expo = if act.left {
                                    cfg.thr_expo.saturating_sub(5)
                                } else {
                                    (cfg.thr_expo + 5).min(80)
                                };
                            }
                            10 if act.select => {
                                cfg.reset_drone_tuning();
                            }
                            _ => {}
                        }
                        cfg.save();
                    }
                }
                renderer.render_drone_tuning_menu(&cfg, drone_idx);
            }
            AppScreen::Flying => {
                if act.back {
                    pause_idx = 0;
                    screen = AppScreen::Paused;
                } else {
                    let track = &TRACKS[cfg.track_index];
                    let ctrl_src = input.control_source();
                    if ctrl_src != quad.control_source {
                        quad.control_source = ctrl_src;
                        renderer.reset_stick_trail();
                        if ctrl_src == FlightControlSource::ImuSimplified {
                            input.recenter_imu();
                        }
                    }
                    if act.respawn {
                        quad.respawn_at_start(track, &cfg);
                        input.reset_kb_throttle();
                        if quad.control_source == FlightControlSource::ImuSimplified {
                            input.recenter_imu();
                        }
                        renderer.reset_stick_trail();
                        if quad.armed {
                            audio.trigger_arm();
                        }
                    }
                    if act.recenter_imu && quad.control_source == FlightControlSource::ImuSimplified {
                        input.recenter_imu();
                        quad.combo_text = "IMU ZEROED";
                        quad.combo_timer = 0.9;
                    }
                    quad.imu_climb_cmd = input.imu_climb_cmd();
                    let sticks = input.flight_sticks(&cfg);
                    let aux = input.aux_inputs(&cfg);
                    if quad.apply_aux(aux, sticks, &mut cfg) {
                        audio.trigger_arm();
                    }

                    let prev_score = quad.score;
                    let prev_hit = quad.hit_flash;
                    phys_accum += dt;
                    while phys_accum >= PHYSICS_DT {
                        quad.step(sticks, &mut cfg, track);
                        phys_accum -= PHYSICS_DT;
                    }
                    if quad.score > prev_score {
                        audio.trigger_gate();
                    }
                    if quad.hit_flash > prev_hit + 0.10 {
                        audio.trigger_hit();
                    }
                    let low_batt = quad.armed && cfg.battery_pack != BatteryPack::Off && quad.cell_voltage < 3.45;
                    audio.update_telemetry(
                        quad.motor_thr,
                        quad.vel.len(),
                        quad.prop_wash_level,
                        sticks.pitch,
                        sticks.roll,
                        low_batt,
                    );
                    let r0 = Instant::now();
                    renderer.render_flight(&quad, &cfg, track, sticks, fps_display, last_render_ms);
                    last_render_ms = r0.elapsed().as_secs_f32() * 1000.0;
                }
            }
            AppScreen::Paused => {
                const NUM_PAUSE: usize = 5;
                let track = &TRACKS[cfg.track_index];
                let sticks = input.flight_sticks(&cfg);
                if act.back {
                    screen = AppScreen::Flying;
                } else {
                    if act.up { pause_idx = (pause_idx + NUM_PAUSE - 1) % NUM_PAUSE; }
                    if act.down { pause_idx = (pause_idx + 1) % NUM_PAUSE; }
                    if act.select {
                        match pause_idx {
                            0 => screen = AppScreen::Flying,
                            1 => {
                                quad.respawn_at_start(track, &cfg);
                                quad.control_source = input.control_source();
                                input.reset_kb_throttle();
                                if quad.control_source == FlightControlSource::ImuSimplified {
                                    input.recenter_imu();
                                }
                                renderer.reset_stick_trail();
                                if quad.armed {
                                    audio.trigger_arm();
                                }
                                screen = AppScreen::Flying;
                            }
                            2 => {
                                drone_idx = 0;
                                sub_return = AppScreen::Paused;
                                screen = AppScreen::DroneTuning;
                            }
                            3 => {
                                settings_idx = 0;
                                sub_return = AppScreen::Paused;
                                screen = AppScreen::Settings;
                            }
                            4 => {
                                screen = AppScreen::MainMenu;
                            }
                            _ => {}
                        }
                    }
                }
                renderer.render_flight(&quad, &cfg, track, sticks, fps_display, last_render_ms);
                renderer.render_pause_menu(&cfg, pause_idx);
            }
            AppScreen::Settings => {
                const NUM_SETTINGS: usize = 17;
                let imu_mode = input.control_source() == FlightControlSource::ImuSimplified;
                if act.back {
                    cfg.save();
                    screen = sub_return;
                } else {
                    if act.up { settings_idx = (settings_idx + NUM_SETTINGS - 1) % NUM_SETTINGS; }
                    if act.down { settings_idx = (settings_idx + 1) % NUM_SETTINGS; }
                    if act.left || act.right || act.select {
                        match settings_idx {
                            0 => cfg.game_mode = cfg.game_mode.toggle(),
                            1 => {
                                if !imu_mode {
                                    cfg.flight_mode = if act.left {
                                        cfg.flight_mode.prev()
                                    } else {
                                        cfg.flight_mode.next()
                                    };
                                }
                            }
                            2 => {
                                let next = cfg.rates.next_preset();
                                cfg.apply_rate_preset(next);
                            }
                            3 => {
                                cfg.rate_center = if act.left {
                                    cfg.rate_center.saturating_sub(10).max(50)
                                } else {
                                    (cfg.rate_center + 10).min(400)
                                };
                                cfg.rates = RatesPreset::Custom;
                            }
                            4 => {
                                cfg.rate_max = if act.left {
                                    cfg.rate_max.saturating_sub(20).max(300)
                                } else {
                                    (cfg.rate_max + 20).min(1200)
                                };
                                cfg.rates = RatesPreset::Custom;
                            }
                            5 => {
                                cfg.rate_expo = if act.left {
                                    cfg.rate_expo.saturating_sub(5)
                                } else {
                                    (cfg.rate_expo + 5).min(90)
                                };
                                cfg.rates = RatesPreset::Custom;
                            }
                            6 => {
                                if act.left {
                                    cfg.camera_tilt_deg = cfg.camera_tilt_deg.saturating_sub(5);
                                } else {
                                    cfg.camera_tilt_deg = (cfg.camera_tilt_deg + 5).min(50);
                                }
                            }
                            7 => {
                                cfg.fov_deg = if act.left {
                                    cfg.fov_deg.saturating_sub(5).max(75)
                                } else {
                                    (cfg.fov_deg + 5).min(105)
                                };
                            }
                            8 => cfg.analog_filter = !cfg.analog_filter,
                            9 => cfg.ui_theme = cfg.ui_theme.toggle(),
                            10 => {
                                cfg.stick_overlay = if act.left {
                                    cfg.stick_overlay.prev()
                                } else {
                                    cfg.stick_overlay.next()
                                };
                                renderer.reset_stick_trail();
                            }
                            11 => {
                                cfg.sound_on = !cfg.sound_on;
                                audio.set_enabled(cfg.sound_on);
                            }
                            12 => cfg.debug_osd = !cfg.debug_osd,
                            13 => cfg.detail_high = !cfg.detail_high,
                            14 => cfg.sky_detail = !cfg.sky_detail,
                            15 => cfg.stick_mode = cfg.stick_mode.toggle(),
                            16 => cfg.throttle_mode = cfg.throttle_mode.toggle(),
                            _ => {}
                        }
                        cfg.save();
                    }
                }
                renderer.render_settings_menu(&cfg, settings_idx, imu_mode);
            }
            AppScreen::Controller => {
                if input.calib.is_some() {
                    if act.back {
                        input.calib = None;
                    } else {
                        input.update_calibration(&mut cfg);
                    }
                } else if input.aux_wiz.is_some() {
                    if act.back {
                        input.aux_wiz = None;
                    } else {
                        input.update_aux_wizard(&mut cfg, act.select);
                    }
                } else if act.back {
                    cfg.save();
                    screen = AppScreen::MainMenu;
                } else if act.calibrate {
                    if input.has_controller() {
                        input.start_calibration();
                    } else if input.has_imu() {
                        input.recenter_imu();
                    }
                } else if act.bind_aux {
                    input.start_aux_wizard(&cfg);
                } else if act.next_device {
                    input.cycle_device();
                } else if act.select || act.left || act.right {
                    cfg.throttle_mode = cfg.throttle_mode.toggle();
                    cfg.save();
                } else if act.respawn {
                    cfg.axis_thr = None;
                    cfg.axis_yaw = None;
                    cfg.axis_pit = None;
                    cfg.axis_rol = None;
                    cfg.invert_throttle = false;
                    cfg.invert_yaw = false;
                    cfg.invert_pitch = false;
                    cfg.invert_roll = false;
                    cfg.aux_arm = AuxBinding::None;
                    cfg.aux_mode = AuxBinding::None;
                    cfg.aux_turtle = AuxBinding::None;
                    cfg.save();
                    input.scan_devices();
                }
                renderer.render_controller_screen(&cfg, &input);
            }
        }

        display.present(&renderer.fb);

        fps_counter += 1;
        if fps_timer.elapsed() >= Duration::from_secs(1) {
            fps_display = fps_counter;
            fps_counter = 0;
            fps_timer = Instant::now();
        }

        let spent = frame_start.elapsed();
        if spent < target_frame {
            std::thread::sleep(target_frame - spent);
        }
    }

    cfg.save();
}
