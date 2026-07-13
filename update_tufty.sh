#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$SCRIPT_DIR/gps_time"
DEST_DIR="/Volumes/TUFTY/apps/gps_time"

if [ ! -d "/Volumes/TUFTY" ]; then
    echo "Error: TUFTY volume not found at /Volumes/TUFTY - is it plugged in and in disk mode?" >&2
    exit 1
fi

if [ ! -d "$SRC_DIR" ]; then
    echo "Error: source directory not found at $SRC_DIR" >&2
    exit 1
fi

# Copy all contents of gps_time/ into $DEST_DIR/
mkdir -p "$DEST_DIR"
cp -R "$SRC_DIR"/. "$DEST_DIR/"
echo "Copied contents of $SRC_DIR to $DEST_DIR"

# Eject the 'TUFTY' volume
diskutil eject /Volumes/TUFTY
echo "Ejected TUFTY"
