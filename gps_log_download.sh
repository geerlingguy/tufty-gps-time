#!/bin/bash
# Pull all GPS log CSVs off the Tufty into the current directory.
# Requires mpremote (pip install mpremote) and Thonny disconnected first,
# since only one program can hold the serial port open at a time.
#
# Usage: ./gps_log_download.sh [--clear]
#   --clear   delete each log file off the device after it's been
#             copied successfully (skips the delete for any file
#             whose copy failed)

PORT="/dev/cu.usbmodem101"   # adjust if your port differs

CLEAR=0
for arg in "$@"; do
    case "$arg" in
        --clear)
            CLEAR=1
            ;;
        *)
            echo "Unknown argument: $arg" >&2
            echo "Usage: $0 [--clear]" >&2
            exit 1
            ;;
    esac
done

mpremote connect "$PORT" fs ls :state | grep -o 'gps_log_[0-9]*\.csv' | sort -u | while read -r file; do
    echo "Copying $file..."
    if mpremote connect "$PORT" fs cp ":state/$file" .; then
        if [ "$CLEAR" -eq 1 ]; then
            echo "Deleting $file from device..."
            mpremote connect "$PORT" fs rm ":state/$file"
        fi
    else
        echo "Copy failed for $file, leaving it on the device." >&2
    fi
done
