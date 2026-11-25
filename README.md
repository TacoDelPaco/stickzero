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
  - **Flight Modes**: `ACRO` (full 3D flips/rolls/dives), `HORIZON` (self-leveling near center, full acro flips past 55%–85% stick), and `ANGLE` (self-leveling).
  - **6 Quad Presets**: `Air75 II 75mm`, `Meteor75 Pro II`, `65mm Whoop`, `3" Toothpick`, `5" Freestyle`, `5" Racer`.
  - **Drone Tuning Menu** (*The Zone*-style real physical units): **Battery Pack** (`Lava II 580mAh`, `Lava II 680mAh`, `Stock`, `Off`), **Prop Wash** (`Off` / `Stock` / `Max`), **Weight (g)** (`15–1500 g` AUW), **Thrust (g)** (`40–8000 g` with live TWR readout), **Air Drag** (`0.05–1.00`), **Rotor Grip** (`0.00–1.00`), **Motor Lag (ms)** (`5–45 ms`), **Thr Mid** (`0.20–0.70`), and **Thr Expo** (`0.00–0.80`).
  - **StickTime-calibrated Hover Aerodynamics**: Axial inflow climb damping & descent cushion, translational lift, in-plane rotor grip, realistic ground friction & yaw-while-grounded, solid 3D gate collisions, and grounded **Turtle Mode**.
  - **Betaflight Actual Rates**: `Soft` (`100/480/0.20`), `Normal`, `Fast`, or fully customizable `Center Sens`, `Max Rate`, and `Rate Expo`.
- **Per-Pixel 16-bit Z-Buffered 3D Engine & 4 Tracks**:
  - `Meadow Loop`, `Figure 8`, `Dive Tower`, and `Bando` — solid 3D gate posts & beams that never clip or vanish.
- **4 Flight Themes + 2 Menu UI Themes**:
  - Flight: `Summer`, `Evening`, `Night`, and two-pass hidden-line neon `Wireframe` (strictly occluded dark hologram walls with neon edges vs glowing orange/yellow gates and emerald green ground grid).
  - Menus: **Monokai** (default) and **StickTime** UI themes toggleable in Settings.
- **Authentic Betaflight OSD + Trailing Sticks + Post-Flight Stats**: Clean flight timer, throttle %, mAh used, cell voltage, craft name, speed/altitude, toggleable **Stick OSD** (`Off` / `On (Dot)` / `Trails`), toggleable `Debug OSD` (FPS/ms), optional NTSC/RF `Analog FX`, and Betaflight post-flight stats card on disarm or empty pack.
- **Bluetooth & USB-C Controller Support (EdgeTX / ELRS / Gamepads)**:
  - Hot-plugs standard Linux `evdev` controllers automatically every 1.5s, including **RadioMaster Pocket** (`RM Pocket`), EdgeTX USB Joysticks, ELRS BLE Joysticks, Xbox, PlayStation, Switch Pro, and 8BitDo.
  - Built-in 4-step **Stick Calibration (`C`)** and 3-step **AUX Switch/Button Binder (`B`)** for **Arm/Disarm**, **3-pos Flight Mode (`ACRO`/`HORIZON`/`ANGLE`)**, and **Turtle Mode**.
  - **CardputerZero IMU Simplified Flight (No Controller Fallback)**: Automatically reads the onboard **Bosch BMI270** over Linux IIO when no controller is connected — tilt forward/back and left/right to fly with 38° self-leveling, coordinated bank-to-turn yaw, auto-hover altitude hold (`F`/`Up` climb, `X`/`Down` descend/land, `Z`/`C` manual yaw, `Space` re-zero tilt), and a dedicated simplified OSD stick layout (vertical altitude/hover gauge + single `IMU` tilt reticle). On **CardputerZero Lite** or hardware without an IMU sensor, StickZero warns on launch that a controller is required to fly.
  - **Fn-free CardputerZero Keyboard Controls**: `F/X/Z/C` or Arrow keys in menus and for the **Left Gimbal** (Throttle/Yaw) in flight; **`K/B/N/M`** for the **Right Gimbal** (`K` pitch fwd, `N` pitch back, `B` roll left, `M` roll right); `Space` zero IMU, `Tab` arm/disarm, `E` mode cycle (disabled in IMU mode), `T` turtle, `R` respawn, **Short `Esc`** pause/back, **Long `Esc` (0.7s)** clean exit to APPLaunch.

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
curl -fLO https://raw.githubusercontent.com/TacoDelPaco/stickzero/main/dist/stickzero_0.1.14_arm64.deb \
  && sudo dpkg -i stickzero_0.1.14_arm64.deb \
  && systemctl --user restart APPLaunch.service
```

### Option B: Deploy from your PC via SSH or ADB

```bash
./deploy.sh pi@cardputerzero.local
# or
./deploy.sh adb
```

### Low-Latency Bluetooth Controller Setup (ELRS BLE & Gamepads)

StickZero drains controller `evdev` events on a dedicated background thread (`poll()` + 64-event batch reads + `SYN_DROPPED` resync) so the app itself adds < 1 ms input latency. If you still see 1–2 s delay over Bluetooth on CardputerZero, it is caused by BLE packet backlog in the transmitter or BlueZ stack:

1. **ExpressLRS BLE Joystick (RadioMaster Pocket, etc.)**: Before starting **BLE Joystick**, open `SYS → ExpressLRS` and lower **Packet Rate** to **`100Hz Full`** (or `50Hz`). Running 250–500 Hz floods BLE faster than the link can drain and builds up a multi-second queue.
2. **Lower BlueZ's BLE connection interval** (default Linux BlueZ uses 30–50 ms ≈ 22 Hz; tighten to 7.5–11.25 ms ≈ 90–133 Hz):
   ```bash
   sudo sed -i 's/^#*MinConnectionInterval=.*/MinConnectionInterval=6/; s/^#*MaxConnectionInterval=.*/MaxConnectionInterval=9/; s/^#*ConnectionLatency=.*/ConnectionLatency=0/' /etc/bluetooth/main.conf \
     && sudo systemctl restart bluetooth \
     && bluetoothctl scan off
   ```
3. **2.4 GHz Wi-Fi / BT Coexistence**: CardputerZero's Pi CM0 shares one 2.4 GHz antenna for Wi-Fi and Bluetooth — turning Wi-Fi off while flying over BT (`sudo rfkill block wifi`) removes wireless stalls, or flip the USB switch to **Host** and use a USB-C cable for ~2 ms wired latency.

### Display Detection

StickZero never hardcodes `/dev/fb0`. It scans `/proc/fb` for `fb_st7789v` (typically `/dev/fb1` on CardputerZero), and also honors `STICKZERO_FBDEV` or `LV_LINUX_FBDEV_DEVICE` if you want to force HDMI (`/dev/fb0`) or another framebuffer.
