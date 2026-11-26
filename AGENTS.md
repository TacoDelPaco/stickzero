# AGENTS.md — StickZero

Project conventions and release workflow for AI coding agents working on **StickZero** (M5Stack CardputerZero native Rust 3D FPV simulator).

## Core Constraints

- **Zero external crate dependencies**: Keep `Cargo.toml` dependency-free (`std` + raw Linux FFI in `src/linux_sys.rs`) so builds remain a single statically-linked `aarch64-unknown-linux-musl` binary (~400 KB).
- **Target hardware**: M5Stack CardputerZero (Raspberry Pi CM0, quad Cortex-A53 @ 1 GHz, 512 MB LPDDR2, 320×170 ST7789v3 SPI LCD + Mini-HDMI via Wayland/`labwc-pi`, DRM/KMS, or `fbdev`).
- **Performance**: Keep the 60 FPS flight loop and 240 Hz physics zero-allocation per frame.
- **No regressions**: Preserve built-in 1.9" LCD support, IMU simplified flight fallback, and CardputerZero Fn-free keyboard controls (`F/X/Z/C`, `K/B/N/M`, short/long `Esc`).

## Build, Verification & Release Workflow

After every code change, complete **all** steps below before finishing:

1. **Run tests**
   ```bash
   cargo test
   ```
2. **Verify release build / benchmark** (when touching physics, rendering, or display blitting)
   ```bash
   cargo run --release -- --bench
   ```
3. **Cross-compile static ARM64 binary** (install target first if missing: `rustup target add aarch64-unknown-linux-musl`)
   ```bash
   cargo build --release --target aarch64-unknown-linux-musl
   ```
4. **Rebuild the CardputerZero `.deb` package**
   ```bash
   bash ./package.sh
   ```
   Confirm `dist/stickzero_<version>_arm64.deb` is updated (keep version in sync across `Cargo.toml`, `package.sh`, `app-builder.json`, and `README.md` if bumping versions).
5. **Auto-commit and push**
   - Stage all modified/new source files, docs, and the rebuilt `dist/stickzero_*_arm64.deb`.
   - Commit with a clear, concise summary message.
   - Push to the current branch (e.g., `origin/main`) unless explicitly asked to open a PR or work on a separate branch.
