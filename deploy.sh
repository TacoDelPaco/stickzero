#!/usr/bin/env bash
set -euo pipefail

# Deploy StickZero directly to a CardputerZero over SSH or ADB.
# Usage:
#   ./deploy.sh pi@cardputerzero.local
#   ./deploy.sh adb

TARGET="${1:-}"
DEB="dist/stickzero_0.1.11_arm64.deb"

if [ ! -f "$DEB" ]; then
    bash ./package.sh
fi

if [ -z "$TARGET" ]; then
    echo "Usage:"
    echo "  ./deploy.sh <user@cardputerzero-ip>   # Install via SSH + dpkg"
    echo "  ./deploy.sh adb                       # Install via ADB"
    exit 1
fi

if [ "$TARGET" = "adb" ]; then
    adb push "$DEB" /tmp/stickzero.deb
    adb shell "sudo dpkg -i /tmp/stickzero.deb && (systemctl --user restart APPLaunch.service 2>/dev/null || true)"
else
    scp "$DEB" "$TARGET:/tmp/stickzero.deb"
    ssh -t "$TARGET" "sudo dpkg -i /tmp/stickzero.deb && (systemctl --user restart APPLaunch.service 2>/dev/null || true)"
fi

echo "StickZero installed and APPLaunch restarted!"
