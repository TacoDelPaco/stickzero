#!/usr/bin/env bash
set -euo pipefail

VERSION="0.1.15"
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
    "$STAGE/usr/lib/tmpfiles.d" \
    "$STAGE/usr/lib/udev/rules.d" \
    "$STAGE/usr/share/APPLaunch/applications" \
    "$STAGE/usr/share/APPLaunch/bin" \
    "$STAGE/usr/share/APPLaunch/share/images" \
    "dist"

cp "$BIN" "$STAGE/usr/share/APPLaunch/bin/stickzero"
chmod 755 "$STAGE/usr/share/APPLaunch/bin/stickzero"

cp "$ICON" "$STAGE/usr/share/APPLaunch/share/images/stickzero_100.png"
chmod 644 "$STAGE/usr/share/APPLaunch/share/images/stickzero_100.png"

cat > "$STAGE/usr/lib/udev/rules.d/70-stickzero.rules" <<'EOF'
# Allow StickZero (input group / active seat) to read USB & Bluetooth RC radios / gamepads
# on both evdev (/dev/input/event*) and hidraw (/dev/hidraw*, e.g. ExpressLRS BLE CH8 -> A7+).
ACTION=="add|change", SUBSYSTEM=="input", KERNEL=="event*", ENV{ID_INPUT_JOYSTICK}=="1", GROUP="input", MODE="0660", TAG+="uaccess"
ACTION=="add|change", SUBSYSTEM=="input", KERNEL=="event*", ATTRS{name}=="*EdgeTX*", GROUP="input", MODE="0660", TAG+="uaccess"
ACTION=="add|change", SUBSYSTEM=="input", KERNEL=="event*", ATTRS{name}=="*OpenTX*", GROUP="input", MODE="0660", TAG+="uaccess"
ACTION=="add|change", SUBSYSTEM=="input", KERNEL=="event*", ATTRS{name}=="*RadioMaster*", GROUP="input", MODE="0660", TAG+="uaccess"
ACTION=="add|change", SUBSYSTEM=="input", KERNEL=="event*", ATTRS{name}=="*Radiomaster*", GROUP="input", MODE="0660", TAG+="uaccess"
ACTION=="add|change", SUBSYSTEM=="input", KERNEL=="event*", ATTRS{name}=="*ELRS*", GROUP="input", MODE="0660", TAG+="uaccess"
ACTION=="add|change", SUBSYSTEM=="input", KERNEL=="event*", ATTRS{name}=="*ExpressLRS*", GROUP="input", MODE="0660", TAG+="uaccess"
ACTION=="add|change", SUBSYSTEM=="input", KERNEL=="event*", ATTRS{name}=="*FrSky*", GROUP="input", MODE="0660", TAG+="uaccess"
ACTION=="add|change", SUBSYSTEM=="input", KERNEL=="event*", ATTRS{name}=="*BETAFPV*", GROUP="input", MODE="0660", TAG+="uaccess"
ACTION=="add|change", SUBSYSTEM=="input", KERNEL=="event*", ATTRS{name}=="*Jumper*", GROUP="input", MODE="0660", TAG+="uaccess"
ACTION=="add|change", SUBSYSTEM=="input", KERNEL=="event*", ATTRS{name}=="*TBS*", GROUP="input", MODE="0660", TAG+="uaccess"
ACTION=="add|change", SUBSYSTEM=="input", KERNEL=="event*", ATTRS{name}=="*Joystick*", GROUP="input", MODE="0660", TAG+="uaccess"
ACTION=="add|change", SUBSYSTEM=="input", KERNEL=="event*", ATTRS{name}=="*Gamepad*", GROUP="input", MODE="0660", TAG+="uaccess"
ACTION=="add|change", SUBSYSTEM=="hidraw", KERNELS=="0005:*", GROUP="input", MODE="0660", TAG+="uaccess"
ACTION=="add|change", SUBSYSTEM=="hidraw", KERNELS=="0003:*", ATTRS{name}=="*EdgeTX*", GROUP="input", MODE="0660", TAG+="uaccess"
ACTION=="add|change", SUBSYSTEM=="hidraw", KERNELS=="0003:*", ATTRS{name}=="*OpenTX*", GROUP="input", MODE="0660", TAG+="uaccess"
ACTION=="add|change", SUBSYSTEM=="hidraw", KERNELS=="0003:*", ATTRS{name}=="*RadioMaster*", GROUP="input", MODE="0660", TAG+="uaccess"
ACTION=="add|change", SUBSYSTEM=="hidraw", ATTRS{name}=="*ELRS*", GROUP="input", MODE="0660", TAG+="uaccess"
ACTION=="add|change", SUBSYSTEM=="hidraw", ATTRS{name}=="*ExpressLRS*", GROUP="input", MODE="0660", TAG+="uaccess"
EOF
chmod 644 "$STAGE/usr/lib/udev/rules.d/70-stickzero.rules"

cat > "$STAGE/usr/lib/tmpfiles.d/stickzero.conf" <<'EOF'
# Allow StickZero (video group) to dim/restore CardputerZero's built-in LCD backlight when HDMI is active.
z /sys/class/backlight/*/brightness 0664 root video -
EOF
chmod 644 "$STAGE/usr/lib/tmpfiles.d/stickzero.conf"

cat > "$STAGE/usr/share/APPLaunch/applications/stickzero.desktop" <<'EOF'
[Desktop Entry]
Name=StickZero
Comment=Minimal native Rust 3D FPV quadcopter simulator for CardputerZero
Exec=/usr/share/APPLaunch/bin/stickzero
Terminal=false
Icon=share/images/stickzero_100.png
Type=Application
EOF

cat > "$STAGE/DEBIAN/postinst" <<'EOF'
#!/bin/sh
set -e

# 1. Reload udev rules so USB/BT controllers (/dev/input/event*, /dev/hidraw*) get access immediately
if command -v udevadm >/dev/null 2>&1; then
    udevadm control --reload-rules >/dev/null 2>&1 || true
    udevadm trigger --subsystem-match=input >/dev/null 2>&1 || true
    udevadm trigger --subsystem-match=hidraw >/dev/null 2>&1 || true
fi

# 2. Apply tmpfiles.d rule so /sys/class/backlight/*/brightness is writable by video group
if command -v systemd-tmpfiles >/dev/null 2>&1; then
    systemd-tmpfiles --create /usr/lib/tmpfiles.d/stickzero.conf >/dev/null 2>&1 || true
fi

# 3. Tune BlueZ [LE] connection parameters for low-latency BLE HID controllers (7.5-11.25ms, 0 latency)
BT_CONF="/etc/bluetooth/main.conf"
if [ -f "$BT_CONF" ] && command -v awk >/dev/null 2>&1; then
    TMP_CONF="$(mktemp /tmp/stickzero-bt.XXXXXX)"
    awk '
    function flush_le() {
        if (!s_min) print "MinConnectionInterval=6"
        if (!s_max) print "MaxConnectionInterval=9"
        if (!s_lat) print "ConnectionLatency=0"
        if (!s_sup) print "ConnectionSupervisionTimeout=400"
    }
    BEGIN { in_le = 0; saw_le = 0; s_min = 0; s_max = 0; s_lat = 0; s_sup = 0 }
    /^\[.*\]/ {
        if (in_le) {
            flush_le()
            in_le = 0
        }
        if ($0 ~ /^\[LE\][[:space:]]*$/) {
            in_le = 1
            saw_le = 1
        }
        print
        next
    }
    in_le && /^[[:space:]]*#?[[:space:]]*MinConnectionInterval[[:space:]]*=/ {
        if (!s_min) { print "MinConnectionInterval=6"; s_min = 1 }
        next
    }
    in_le && /^[[:space:]]*#?[[:space:]]*MaxConnectionInterval[[:space:]]*=/ {
        if (!s_max) { print "MaxConnectionInterval=9"; s_max = 1 }
        next
    }
    in_le && /^[[:space:]]*#?[[:space:]]*ConnectionLatency[[:space:]]*=/ {
        if (!s_lat) { print "ConnectionLatency=0"; s_lat = 1 }
        next
    }
    in_le && /^[[:space:]]*#?[[:space:]]*ConnectionSupervisionTimeout[[:space:]]*=/ {
        if (!s_sup) { print "ConnectionSupervisionTimeout=400"; s_sup = 1 }
        next
    }
    { print }
    END {
        if (in_le) {
            flush_le()
        } else if (!saw_le) {
            print ""
            print "[LE]"
            flush_le()
        }
    }
    ' "$BT_CONF" > "$TMP_CONF"

    if ! cmp -s "$BT_CONF" "$TMP_CONF"; then
        cat "$TMP_CONF" > "$BT_CONF"
        if command -v systemctl >/dev/null 2>&1; then
            systemctl try-restart bluetooth.service >/dev/null 2>&1 || true
        fi
    fi
    rm -f "$TMP_CONF"
fi

exit 0
EOF
chmod 755 "$STAGE/DEBIAN/postinst"

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
