//! StickZero — Minimal native Rust 3D FPV quadcopter simulator for M5Stack CardputerZero.
//!
//! Designed for low-latency flight on CardputerZero's Raspberry Pi CM0 (Quad Cortex-A53 @ 1GHz)
//! and 320x170 ST7789v3 LCD over Linux fbdev, with Bluetooth & USB-C controller support.

mod audio;
mod config;
mod display;
mod font;
mod input;
mod linux_sys;
mod math;
mod physics;
mod png;
mod renderer;
mod tracks;

use audio::AudioEngine;
use config::{BatteryPack, Config, RatesPreset, ThrottleMode, VisualTheme};
use display::{FbDisplay, SCREEN_H, SCREEN_W};
use input::InputManager;
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
    MainMenu,
    Flying,
    Paused,
    Settings,
    Controller,
}

fn run_benchmark() {
    let mut cfg = Config::default();
    let mut renderer = Renderer::new();
    let track = &TRACKS[3]; // Bando (heaviest geometry)
    let mut quad = QuadState::spawn(track);
    quad.pos = Vec3::new(0.0, 5.0, 10.0);
    quad.vel = Vec3::new(0.0, 0.0, 18.0);
    let sticks = StickInput { throttle: 0.52, yaw: 0.08, pitch: 0.12, roll: 0.05 };

    const FRAMES: usize = 600;
    let t0 = Instant::now();
    for _ in 0..FRAMES {
        for _ in 0..4 {
            quad.step(sticks, &mut cfg, track);
        }
        renderer.render_flight(&quad, &cfg, track, sticks, 60, 1.0);
    }
    let elapsed = t0.elapsed();
    let ms_per_frame = elapsed.as_secs_f64() * 1000.0 / (FRAMES as f64);
    let fps = 1000.0 / ms_per_frame;
    println!(
        "StickZero benchmark ({} frames @ 320x170 RGB565 + 240Hz physics): {:.3} ms/frame ({:.0} theoretical FPS)",
        FRAMES, ms_per_frame, fps
    );
}

fn generate_screenshots(dir: &Path) -> Result<(), String> {
    let mut cfg = Config::default();
    let mut renderer = Renderer::new();
    let input = InputManager::new();

    // 1. Main Menu
    renderer.render_main_menu(&cfg, 0, &input);
    png::save_rgb565_png(&dir.join("01-main-menu.png"), SCREEN_W, SCREEN_H, &*renderer.fb)?;

    // 2. Summer Meadow Race Flight
    cfg.theme = VisualTheme::Summer;
    cfg.track_index = 0;
    let track = &TRACKS[0];
    let mut quad = QuadState::spawn(track);
    quad.pos = Vec3::new(-1.2, 2.4, 8.0);
    quad.vel = Vec3::new(1.0, 0.2, 19.5);
    quad.rot = Quat::from_axis_angle(Vec3::FORWARD, -0.18).mul(Quat::from_axis_angle(Vec3::RIGHT, 0.12));
    quad.lap_time = 12.48;
    quad.cell_voltage = 3.94;
    let sticks = StickInput { throttle: 0.62, yaw: 0.08, pitch: 0.24, roll: 0.18 };
    renderer.render_flight(&quad, &cfg, track, sticks, 60, 0.9);
    png::save_rgb565_png(&dir.join("02-meadow-summer.png"), SCREEN_W, SCREEN_H, &*renderer.fb)?;

    // 3. Night Bando Freestyle
    cfg.theme = VisualTheme::Night;
    cfg.track_index = 3;
    let bando = &TRACKS[3];
    let mut q2 = QuadState::spawn(bando);
    q2.pos = Vec3::new(0.0, 6.5, 8.0);
    q2.vel = Vec3::new(0.5, 1.5, 22.0);
    q2.rot = Quat::from_axis_angle(Vec3::FORWARD, 0.28);
    q2.score = 1450;
    q2.combo_text = "POWER FLIP +250";
    q2.combo_timer = 1.0;
    q2.cell_voltage = 3.78;
    cfg.game_mode = config::GameMode::Freestyle;
    renderer.render_flight(&q2, &cfg, bando, sticks, 60, 1.1);
    png::save_rgb565_png(&dir.join("03-bando-night.png"), SCREEN_W, SCREEN_H, &*renderer.fb)?;

    // 4. Controller Monitor screen
    cfg.throttle_mode = ThrottleMode::GamepadCenter;
    renderer.render_controller_screen(&cfg, &input);
    png::save_rgb565_png(&dir.join("04-controller.png"), SCREEN_W, SCREEN_H, &*renderer.fb)?;

    println!("Saved 4 CardputerZero 320x170 screenshots to {}", dir.display());
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
    let mut screen = AppScreen::MainMenu;
    let mut menu_idx: usize = 0;
    let mut pause_idx: usize = 0;
    let mut settings_idx: usize = 0;
    let mut quad = QuadState::spawn(&TRACKS[cfg.track_index]);

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
        audio.set_flying(!in_menu, cfg.analog_filter);
        input.poll(dt, in_menu, &cfg);
        let act = input.take_menu_actions();

        if act.exit_app {
            break;
        }

        match screen {
            AppScreen::MainMenu => {
                if act.up { menu_idx = (menu_idx + 5) % 6; }
                if act.down { menu_idx = (menu_idx + 1) % 6; }
                if act.left || act.right || act.select {
                    match menu_idx {
                        0 if act.select => {
                            quad = QuadState::spawn(&TRACKS[cfg.track_index]);
                            phys_accum = 0.0;
                            audio.trigger_arm();
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
                            if act.left {
                                cfg.flight_mode = cfg.flight_mode.toggle();
                            } else {
                                cfg.quad = cfg.quad.next();
                            }
                            cfg.save();
                        }
                        3 => {
                            cfg.theme = cfg.theme.next();
                            cfg.save();
                        }
                        4 if act.select => {
                            settings_idx = 0;
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
            AppScreen::Flying => {
                if act.back {
                    pause_idx = 0;
                    screen = AppScreen::Paused;
                } else {
                    let track = &TRACKS[cfg.track_index];
                    if act.respawn {
                        quad.respawn_at_start(track);
                        audio.trigger_arm();
                    }
                    let sticks = input.flight_sticks(&cfg);
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
                    let low_batt = cfg.battery_pack != BatteryPack::Off && quad.cell_voltage < 3.45;
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
                let track = &TRACKS[cfg.track_index];
                let sticks = input.flight_sticks(&cfg);
                if act.back {
                    screen = AppScreen::Flying;
                } else {
                    if act.up { pause_idx = (pause_idx + 3) % 4; }
                    if act.down { pause_idx = (pause_idx + 1) % 4; }
                    if act.select {
                        match pause_idx {
                            0 => screen = AppScreen::Flying,
                            1 => {
                                quad.respawn_at_start(track);
                                audio.trigger_arm();
                                screen = AppScreen::Flying;
                            }
                            2 => {
                                settings_idx = 0;
                                screen = AppScreen::Settings;
                            }
                            3 => {
                                screen = AppScreen::MainMenu;
                            }
                            _ => {}
                        }
                    }
                }
                renderer.render_flight(&quad, &cfg, track, sticks, fps_display, last_render_ms);
                renderer.render_pause_menu(pause_idx);
            }
            AppScreen::Settings => {
                const NUM_SETTINGS: usize = 16;
                if act.back {
                    cfg.save();
                    screen = AppScreen::MainMenu;
                } else {
                    if act.up { settings_idx = (settings_idx + NUM_SETTINGS - 1) % NUM_SETTINGS; }
                    if act.down { settings_idx = (settings_idx + 1) % NUM_SETTINGS; }
                    if act.left || act.right || act.select {
                        match settings_idx {
                            0 => cfg.game_mode = cfg.game_mode.toggle(),
                            1 => cfg.flight_mode = cfg.flight_mode.toggle(),
                            2 => cfg.battery_pack = cfg.battery_pack.next(),
                            3 => cfg.prop_wash = !cfg.prop_wash,
                            4 => {
                                cfg.sound_on = !cfg.sound_on;
                                audio.set_enabled(cfg.sound_on);
                            }
                            5 => {
                                let next = cfg.rates.next_preset();
                                cfg.apply_rate_preset(next);
                            }
                            6 => {
                                cfg.rate_center = if act.left {
                                    cfg.rate_center.saturating_sub(10).max(50)
                                } else {
                                    (cfg.rate_center + 10).min(400)
                                };
                                cfg.rates = RatesPreset::Custom;
                            }
                            7 => {
                                cfg.rate_max = if act.left {
                                    cfg.rate_max.saturating_sub(20).max(300)
                                } else {
                                    (cfg.rate_max + 20).min(1200)
                                };
                                cfg.rates = RatesPreset::Custom;
                            }
                            8 => {
                                cfg.rate_expo = if act.left {
                                    cfg.rate_expo.saturating_sub(5)
                                } else {
                                    (cfg.rate_expo + 5).min(90)
                                };
                                cfg.rates = RatesPreset::Custom;
                            }
                            9 => {
                                if act.left {
                                    cfg.camera_tilt_deg = cfg.camera_tilt_deg.saturating_sub(5);
                                } else {
                                    cfg.camera_tilt_deg = (cfg.camera_tilt_deg + 5).min(50);
                                }
                            }
                            10 => {
                                cfg.fov_deg = if act.left {
                                    cfg.fov_deg.saturating_sub(5).max(75)
                                } else {
                                    (cfg.fov_deg + 5).min(105)
                                };
                            }
                            11 => cfg.analog_filter = !cfg.analog_filter,
                            12 => cfg.detail_high = !cfg.detail_high,
                            13 => cfg.sky_detail = !cfg.sky_detail,
                            14 => cfg.stick_mode = cfg.stick_mode.toggle(),
                            15 => {
                                if act.left {
                                    cfg.throttle_mode = cfg.throttle_mode.toggle();
                                } else {
                                    cfg.show_sticks = !cfg.show_sticks;
                                }
                            }
                            _ => {}
                        }
                        cfg.save();
                    }
                }
                renderer.render_settings_menu(&cfg, settings_idx);
            }
            AppScreen::Controller => {
                if input.calib.is_some() {
                    if act.back {
                        input.calib = None;
                    } else {
                        input.update_calibration(&mut cfg);
                    }
                } else if act.back {
                    cfg.save();
                    screen = AppScreen::MainMenu;
                } else if act.calibrate {
                    input.start_calibration();
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
