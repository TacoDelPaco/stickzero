//! Zero-dependency real-time analog tinywhoop audio synthesizer.
//!
//! Runs on a dedicated background thread (keeping the 3D render thread at 0ms audio overhead)
//! and streams 22.05 kHz 16-bit mono PCM to ALSA (`aplay`) so it works seamlessly with
//! CardputerZero's ES8389 speaker and 3.5mm headphone jack while keeping a 100% static musl binary.

use std::io::Write;
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::Duration;

const SAMPLE_RATE: f32 = 22050.0;
const CHUNK_SAMPLES: usize = 512; // ~23 ms latency per chunk

/// Lock-free telemetry snapshot shared from physics/main loop to the audio synth thread.
pub struct AudioShared {
    pub enabled: AtomicBool,
    pub flying: AtomicBool,
    pub analog_fx: AtomicBool,
    pub throttle_bits: AtomicU32,
    pub speed_bits: AtomicU32,
    pub prop_wash_bits: AtomicU32,
    pub pitch_mix_bits: AtomicU32,
    pub roll_mix_bits: AtomicU32,
    pub low_batt: AtomicBool,
    pub event_arm: AtomicBool,
    pub event_gate: AtomicBool,
    pub event_hit: AtomicBool,
    pub quit: AtomicBool,
}

impl AudioShared {
    pub fn new(enabled: bool) -> Self {
        Self {
            enabled: AtomicBool::new(enabled),
            flying: AtomicBool::new(false),
            analog_fx: AtomicBool::new(true),
            throttle_bits: AtomicU32::new(0.0f32.to_bits()),
            speed_bits: AtomicU32::new(0.0f32.to_bits()),
            prop_wash_bits: AtomicU32::new(0.0f32.to_bits()),
            pitch_mix_bits: AtomicU32::new(0.0f32.to_bits()),
            roll_mix_bits: AtomicU32::new(0.0f32.to_bits()),
            low_batt: AtomicBool::new(false),
            event_arm: AtomicBool::new(false),
            event_gate: AtomicBool::new(false),
            event_hit: AtomicBool::new(false),
            quit: AtomicBool::new(false),
        }
    }
}

pub struct AudioEngine {
    pub shared: Arc<AudioShared>,
    handle: Option<JoinHandle<()>>,
}

impl AudioEngine {
    pub fn start(enabled: bool) -> Self {
        let shared = Arc::new(AudioShared::new(enabled));
        let bg = Arc::clone(&shared);
        let handle = thread::spawn(move || audio_thread(bg));
        Self {
            shared,
            handle: Some(handle),
        }
    }

    pub fn set_enabled(&self, on: bool) {
        self.shared.enabled.store(on, Ordering::Relaxed);
    }

    pub fn set_flying(&self, flying: bool, analog_fx: bool) {
        self.shared.flying.store(flying, Ordering::Relaxed);
        self.shared.analog_fx.store(analog_fx, Ordering::Relaxed);
    }

    pub fn update_telemetry(
        &self,
        throttle: f32,
        speed_mps: f32,
        prop_wash: f32,
        pitch_cmd: f32,
        roll_cmd: f32,
        low_batt: bool,
    ) {
        self.shared.throttle_bits.store(throttle.to_bits(), Ordering::Relaxed);
        self.shared.speed_bits.store(speed_mps.to_bits(), Ordering::Relaxed);
        self.shared.prop_wash_bits.store(prop_wash.to_bits(), Ordering::Relaxed);
        self.shared.pitch_mix_bits.store(pitch_cmd.to_bits(), Ordering::Relaxed);
        self.shared.roll_mix_bits.store(roll_cmd.to_bits(), Ordering::Relaxed);
        self.shared.low_batt.store(low_batt, Ordering::Relaxed);
    }

    pub fn trigger_arm(&self) {
        self.shared.event_arm.store(true, Ordering::Relaxed);
    }

    pub fn trigger_gate(&self) {
        self.shared.event_gate.store(true, Ordering::Relaxed);
    }

    pub fn trigger_hit(&self) {
        self.shared.event_hit.store(true, Ordering::Relaxed);
    }
}

impl Drop for AudioEngine {
    fn drop(&mut self) {
        self.shared.quit.store(true, Ordering::Relaxed);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

fn spawn_aplay() -> Option<Child> {
    Command::new("aplay")
        .args([
            "-q",
            "-t", "raw",
            "-f", "S16_LE",
            "-c", "1",
            "-r", "22050",
            "-B", "45000",
        ])
        .stdin(Stdio::piped())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .ok()
}

fn audio_thread(shared: Arc<AudioShared>) {
    let mut child: Option<Child> = None;
    let mut pcm = [0u8; CHUNK_SAMPLES * 2];

    // 4 tinywhoop brushless motor oscillators (FL, FR, RL, RR)
    let mut phases = [0.0f32; 4];
    let detune = [1.000f32, 1.019, 0.984, 1.033];
    let mut wash_lfo = 0.0f32;
    let mut lp_noise = 0.0f32;
    let mut rng: u32 = 0x9E37_79B9;

    // Event envelopes
    let mut arm_timer = 0.0f32;
    let mut gate_timer = 0.0f32;
    let mut hit_timer = 0.0f32;
    let mut batt_beep_accum = 0.0f32;
    let mut beep_phase = 0.0f32;

    // Smoothed throttle for click-free RPM transitions
    let mut smooth_thr = 0.0f32;

    while !shared.quit.load(Ordering::Relaxed) {
        let enabled = shared.enabled.load(Ordering::Relaxed);
        let flying = shared.flying.load(Ordering::Relaxed);

        if !enabled {
            if let Some(mut c) = child.take() {
                let _ = c.kill();
                let _ = c.wait();
            }
            thread::sleep(Duration::from_millis(50));
            continue;
        }

        if child.is_none() {
            child = spawn_aplay();
            if child.is_none() {
                // No audio device / aplay unavailable; retry slowly
                thread::sleep(Duration::from_millis(1000));
                continue;
            }
        }

        // Check one-shot triggers
        if shared.event_arm.swap(false, Ordering::Relaxed) {
            arm_timer = 0.42;
        }
        if shared.event_gate.swap(false, Ordering::Relaxed) {
            gate_timer = 0.18;
        }
        if shared.event_hit.swap(false, Ordering::Relaxed) {
            hit_timer = 0.20;
        }

        let target_thr = f32::from_bits(shared.throttle_bits.load(Ordering::Relaxed)).clamp(0.0, 1.0);
        let speed = f32::from_bits(shared.speed_bits.load(Ordering::Relaxed)).clamp(0.0, 40.0);
        let prop_wash = f32::from_bits(shared.prop_wash_bits.load(Ordering::Relaxed)).clamp(0.0, 1.0);
        let pitch_mix = f32::from_bits(shared.pitch_mix_bits.load(Ordering::Relaxed)).clamp(-1.0, 1.0);
        let roll_mix = f32::from_bits(shared.roll_mix_bits.load(Ordering::Relaxed)).clamp(-1.0, 1.0);
        let low_batt = shared.low_batt.load(Ordering::Relaxed);
        let analog_fx = shared.analog_fx.load(Ordering::Relaxed);

        let dt = 1.0 / SAMPLE_RATE;

        for i in 0..CHUNK_SAMPLES {
            // Fast xorshift32 white noise in -1..1
            rng ^= rng << 13;
            rng ^= rng >> 17;
            rng ^= rng << 5;
            let white = ((rng as i32) as f32) * (1.0 / 2147483648.0);
            lp_noise += (white - lp_noise) * 0.14;

            let mut sample = 0.0f32;

            if flying {
                smooth_thr += (target_thr - smooth_thr) * 0.0025;
                // High-KV 1S 75mm tinywhoop blade-pass fundamental: ~290 Hz idle -> ~1320 Hz full
                let base_freq = 290.0 + smooth_thr.powf(0.82) * 1030.0;

                // Prop wash flutter LFO (~19 Hz)
                wash_lfo += dt * 19.5 * std::f32::consts::TAU;
                if wash_lfo > std::f32::consts::TAU {
                    wash_lfo -= std::f32::consts::TAU;
                }
                let wash_mod = 1.0 + wash_lfo.sin() * prop_wash * 0.09;

                // Betaflight-style 4-motor pitch/roll differential
                let mixes = [
                    1.0 + 0.05 * pitch_mix + 0.05 * roll_mix,
                    1.0 + 0.05 * pitch_mix - 0.05 * roll_mix,
                    1.0 - 0.05 * pitch_mix + 0.05 * roll_mix,
                    1.0 - 0.05 * pitch_mix - 0.05 * roll_mix,
                ];

                let mut motor_sum = 0.0f32;
                for m in 0..4 {
                    let f = (base_freq * detune[m] * mixes[m] * wash_mod).clamp(180.0, 2200.0);
                    phases[m] += dt * f * std::f32::consts::TAU;
                    if phases[m] > std::f32::consts::TAU {
                        phases[m] -= std::f32::consts::TAU;
                    }
                    // Fundamental + 2nd harmonic gives authentic brushless tinywhoop whine
                    let s1 = phases[m].sin();
                    let s2 = (phases[m] * 2.0).sin() * 0.35;
                    motor_sum += s1 + s2;
                }
                let motor_vol = 0.055 + smooth_thr * 0.16;
                sample += (motor_sum * 0.25) * motor_vol;

                // Air rush / prop wash turbulence noise
                let wind_vol = (speed / 28.0).clamp(0.0, 1.0) * 0.09 + prop_wash * 0.14;
                sample += lp_noise * wind_vol;

                // Subtle 5.8GHz analog VTX audio hiss floor
                if analog_fx {
                    sample += white * 0.012;
                }

                // Low battery Betaflight double-beep
                if low_batt {
                    batt_beep_accum += dt;
                    if batt_beep_accum > 2.6 {
                        batt_beep_accum = 0.0;
                    }
                    let beep_on = (batt_beep_accum < 0.07)
                        || (batt_beep_accum > 0.13 && batt_beep_accum < 0.20);
                    if beep_on {
                        beep_phase += dt * 2350.0 * std::f32::consts::TAU;
                        if beep_phase > std::f32::consts::TAU {
                            beep_phase -= std::f32::consts::TAU;
                        }
                        sample += beep_phase.sin() * 0.14;
                    }
                } else {
                    batt_beep_accum = 0.5;
                }
            } else {
                smooth_thr *= 0.995;
            }

            // Bluejay / BLHeli tinywhoop arming chirp (3 ascending notes)
            if arm_timer > 0.0 {
                arm_timer = (arm_timer - dt).max(0.0);
                let freq = if arm_timer > 0.28 {
                    1100.0
                } else if arm_timer > 0.14 {
                    1480.0
                } else {
                    1980.0
                };
                beep_phase += dt * freq * std::f32::consts::TAU;
                if beep_phase > std::f32::consts::TAU {
                    beep_phase -= std::f32::consts::TAU;
                }
                sample += beep_phase.sin() * 0.16;
            }

            // Gate / loop score chime (bright dual-tone)
            if gate_timer > 0.0 {
                gate_timer = (gate_timer - dt).max(0.0);
                let freq = if gate_timer > 0.09 { 1320.0 } else { 1760.0 };
                beep_phase += dt * freq * std::f32::consts::TAU;
                if beep_phase > std::f32::consts::TAU {
                    beep_phase -= std::f32::consts::TAU;
                }
                sample += beep_phase.sin() * 0.18;
            }

            // Wall impact thud + analog static crackle
            if hit_timer > 0.0 {
                hit_timer = (hit_timer - dt).max(0.0);
                let env = hit_timer / 0.20;
                sample += (lp_noise * 0.45 + white * 0.22) * env;
            }

            // Soft clip and convert to S16_LE
            let clipped = (sample * 1.4).tanh().clamp(-0.98, 0.98);
            let s16 = (clipped * 30000.0) as i16;
            let bytes = s16.to_le_bytes();
            pcm[i * 2] = bytes[0];
            pcm[i * 2 + 1] = bytes[1];
        }

        if let Some(ref mut c) = child {
            if let Some(ref mut stdin) = c.stdin {
                if stdin.write_all(&pcm).is_err() {
                    let _ = c.kill();
                    let _ = c.wait();
                    child = None;
                }
            }
        }
    }

    if let Some(mut c) = child {
        let _ = c.kill();
        let _ = c.wait();
    }
}
