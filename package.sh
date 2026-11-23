#!/usr/bin/env bash
set -euo pipefail

VERSION="0.1.1"
PKG_NAME="stickzero"
ARCH="arm64"
BIN="target/aarch64-unknown-linux-musl/release/stickzero"
ICON="assets/stickzero_100.png"
STAGE="build/deb/${PKG_NAME}_${VERSION}_${ARCH}"

if [ ! -f "$BIN" ]; then
    echo "Building static AArch64 binary for CardputerZero (Cortex-A53)..."
    cargo build --release --target aarch64-unknown-linux-musl
fi

rm -rf "$STAGE"
mkdir -p \
    "$STAGE/DEBIAN" \
    "$STAGE/usr/share/APPLaunch/applications" \
    "$STAGE/usr/share/APPLaunch/bin" \
    "$STAGE/usr/share/APPLaunch/share/images" \
    "dist"

cp "$BIN" "$STAGE/usr/share/APPLaunch/bin/stickzero"
chmod 755 "$STAGE/usr/share/APPLaunch/bin/stickzero"

cp "$ICON" "$STAGE/usr/share/APPLaunch/share/images/stickzero_100.png"
chmod 644 "$STAGE/usr/share/APPLaunch/share/images/stickzero_100.png"

cat > "$STAGE/usr/share/APPLaunch/applications/stickzero.desktop" <<'EOF'
[Desktop Entry]
Name=StickZero
Comment=Minimal native Rust 3D FPV quadcopter simulator for CardputerZero
Exec=/usr/share/APPLaunch/bin/stickzero
Terminal=false
Icon=share/images/stickzero_100.png
Type=Application
EOF

cat > "$STAGE/DEBIAN/control" <<EOF
Package: ${PKG_NAME}
Version: ${VERSION}
Section: games
Priority: optional
Architecture: ${ARCH}
Maintainer: omghax.base.eth <262865+TacoDelPaco@users.noreply.github.com>
Homepage: https://github.com/TacoDelPaco/stickzero
Description: Minimal native Rust 3D FPV quadcopter simulator for CardputerZero
 Ultra-lightweight 3D FPV tinywhoop simulator inspired by StickTime-Pi, written in
 pure Rust with zero external dependencies. Renders directly to CardputerZero's
 320x170 ST7789v3 RGB565 framebuffer and supports USB-C and Bluetooth controllers.
EOF

OUT_DEB="dist/${PKG_NAME}_${VERSION}_${ARCH}.deb"
dpkg-deb --root-owner-group -b "$STAGE" "$OUT_DEB"
echo "Created CardputerZero APPLaunch package: $OUT_DEB"
ls -lh "$OUT_DEB"
