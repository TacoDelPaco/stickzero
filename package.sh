#!/usr/bin/env bash
set -euo pipefail

VERSION="0.1.0"
PKG_NAME="stickzero"
ARCH="arm64"
BIN="target/aarch64-unknown-linux-musl/release/stickzero"
ICON="assets/stickzero.png"
STAGE="build/deb/${PKG_NAME}_${VERSION}_${ARCH}"

if [ ! -f "$BIN" ]; then
    echo "Building static AArch64 binary for CardputerZero (Cortex-A53)..."
    cargo build --release --target aarch64-unknown-linux-musl
fi

if [ ! -f "$ICON" ]; then
    cargo run --release -- --screenshot screenshots
fi

rm -rf "$STAGE"
mkdir -p \
    "$STAGE/DEBIAN" \
    "$STAGE/usr/share/APPLaunch/applications" \
    "$STAGE/usr/share/APPLaunch/bin" \
    "$STAGE/usr/share/APPLaunch/share/icons" \
    "dist"

cp "$BIN" "$STAGE/usr/share/APPLaunch/bin/stickzero"
chmod 755 "$STAGE/usr/share/APPLaunch/bin/stickzero"

cp "$ICON" "$STAGE/usr/share/APPLaunch/share/icons/stickzero.png"
chmod 644 "$STAGE/usr/share/APPLaunch/share/icons/stickzero.png"

cat > "$STAGE/usr/share/APPLaunch/applications/stickzero.desktop" <<'EOF'
[Desktop Entry]
Type=Application
Name=StickZero
Comment=Minimal native Rust 3D FPV quadcopter simulator for CardputerZero
Exec=/usr/share/APPLaunch/bin/stickzero
Icon=share/icons/stickzero.png
Terminal=false
EOF

cat > "$STAGE/DEBIAN/control" <<EOF
Package: ${PKG_NAME}
Version: ${VERSION}
Section: games
Priority: optional
Architecture: ${ARCH}
Maintainer: StickZero Contributors
Description: Minimal native Rust 3D FPV quadcopter simulator for CardputerZero
 Ultra-lightweight 3D FPV drone simulator inspired by StickTime-Pi, written in
 pure Rust with zero external dependencies. Renders directly to CardputerZero's
 320x170 ST7789v3 RGB565 framebuffer and supports Bluetooth and USB-C controllers.
EOF

OUT_DEB="dist/${PKG_NAME}_${VERSION}_${ARCH}.deb"
dpkg-deb --root-owner-group -b "$STAGE" "$OUT_DEB"
echo "Created CardputerZero APPLaunch package: $OUT_DEB"
ls -lh "$OUT_DEB"
