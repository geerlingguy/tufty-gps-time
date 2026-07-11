#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST_DIR="/Volumes/TUFTY/apps/gps_time"

if [ ! -d "/Volumes/TUFTY" ]; then
    echo "Error: TUFTY volume not found at /Volumes/TUFTY - is it plugged in and in disk mode?" >&2
    exit 1
fi

# Copy contents of __init__.py to /Volumes/TUFTY/apps/gps_time/__init__.py
mkdir -p "$DEST_DIR"
cp "$SCRIPT_DIR/__init__.py" "$DEST_DIR/__init__.py"
echo "Copied __init__.py to $DEST_DIR"

# Eject the 'TUFTY' volume (make this work on macOS)
diskutil eject /Volumes/TUFTY
echo "Ejected TUFTY"
