# StickZero

A minimal, zero-dependency, native **Rust** 3D FPV quadcopter simulator built specifically for the **M5Stack CardputerZero** (Raspberry Pi CM0, Quad-Core ARM Cortex-A53 @ 1 GHz, 512 MB RAM, 1.9" 320×170 ST7789v3 LCD).

Inspired by [StickTime-Pi](https://github.com/juricabi/sticktime-pi), **StickZero** replaces the Lua interpreter and desktop/GUI stack with a single statically-linked `aarch64-unknown-linux-musl` Rust binary that renders directly to CardputerZero's RGB565 framebuffer while reading Bluetooth or USB-C controllers over Linux `evdev`.

## Why Rust on CardputerZero?

StickTime-Pi runs `StickTime.lua` inside an embedded Lua 5.3 interpreter (designed for a Raspberry Pi 4, and noticeably slower on a Pi Zero 2 W / CM0). Writing the simulator natively in Rust gives major advantages on CardputerZero's limited hardware:

- **Zero interpreter overhead**: 6-DOF quaternion physics (240 Hz substeps) and 3D software rasterization compile directly to Cortex-A53 ARMv8 machine code.
- **Zero-allocation flight loop**: Pre-allocated 108.8 KB RGB565 framebuffer (`320 × 170 × 2`) and draw command buffer — no GC pauses, deterministic sub-millisecond frame times.
- **Zero external dependencies**: No Lua, no X11/Wayland, no SDL, no LVGL, no glibc version coupling.
- **Single static AArch64 binary**: ~400 KB stripped, < 4 MB RAM, instant startup.

## Features

- **6-DOF FPV Flight Physics (240 Hz)**:
  - **Betaflight Actual Rates** presets (`Soft`, `Normal`, `Fast`).
  - **Flight Modes**: `ACRO` (full 3D flips/rolls/dives) and `ANGLE` (self-leveling).
  - **4 Quad Classes**: `5" Racer`, `5" Freestyle`, `3" Toothpick`, `65mm Whoop`.
  - Motor spool lag, quadratic aerodynamic drag, ground effect cushion, obstacle collisions, **Turtle Mode** (push roll/pitch when inverted on the ground to flip upright), and optional battery voltage sag simulation.
- **4 Built-in 3D Tracks**:
  - `Meadow Loop` — 8-gate sweeping oval.
  - `Figure 8` — crossover speed track.
  - `Dive Tower` — 28m central tower + vertical dive gate + underpass.
  - `Bando` — concrete ruin with roof gaps and freestyle pillars.
- **4 Visual Themes**: `Summer`, `Evening`, `Night` (glowing neon gates), and `Wireframe`.
- **Betaflight-style OSD**: Outlined 5×7 font, lap timer & persistent best lap, gate progress, next-gate beacon + off-screen edge arrow, speed (KPH), altitude (M), cell voltage, FPS/frame-ms, and optional live dual-gimbal stick overlay.
- **Bluetooth & USB-C Controller Support**:
  - Hot-plugs standard Linux `evdev` controllers automatically every 1.5s.
  - Works with **USB-C Host** (flip CardputerZero's left-side USB switch to **Host**) for EdgeTX/OpenTX USB Joysticks and USB gamepads.
  - Works with **Bluetooth** for ExpressLRS BLE Joysticks, Xbox, PlayStation DualSense/DS4, Nintendo Switch Pro, and 8BitDo controllers.
  - Supports **Mode 2 / Mode 1** and **RC Full-Range Throttle** vs **Gamepad Spring-Centered Throttle** (`Pad Center`, where center stick = hover!).
  - Built-in **Controller Monitor** screen with live stick bars and dual gimbal preview.
  - **CardputerZero Keyboard Fallback**: `W/S` throttle, `A/D` yaw, `Arrows` or `I/J/K/L` pitch/roll, `R` respawn, **Short `Esc`** pause/back, **Long `Esc` (0.7s)** clean exit to APPLaunch.

## Building & Packaging

```bash
# 1. Run unit tests & performance benchmark on host
cargo test
cargo run --release -- --bench

# 2. Cross-compile static AArch64 binary for CardputerZero
cargo build --release --target aarch64-unknown-linux-musl

# 3. Build CardputerZero APPLaunch .deb package
bash ./package.sh
```

## Installing on CardputerZero

### Option A: One-line install directly on CardputerZero (over Wi-Fi)

```bash
curl -fLO https://raw.githubusercontent.com/TacoDelPaco/stickzero/main/dist/stickzero_0.1.0_arm64.deb \
  && sudo dpkg -i stickzero_0.1.0_arm64.deb \
  && systemctl --user restart APPLaunch.service
```

### Option B: Deploy from your PC via SSH or ADB

```bash
./deploy.sh pi@cardputerzero.local
# or
./deploy.sh adb
```

### Display Detection

StickZero never hardcodes `/dev/fb0`. It scans `/proc/fb` for `fb_st7789v` (typically `/dev/fb1` on CardputerZero), and also honors `STICKZERO_FBDEV` or `LV_LINUX_FBDEV_DEVICE` if you want to force HDMI (`/dev/fb0`) or another framebuffer.
