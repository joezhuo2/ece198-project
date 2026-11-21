"""Record labeled training data from the Arduino over USB serial.

Usage: python record.py <label> <COM port> [seconds]

Saves the sketch's "D,..." lines to ml/data/<label>_<timestamp>.csv.
The filename carries the label, which train.py reads back.
"""
import csv
import sys
import time
from pathlib import Path

import serial

# Same order as the row[] array the sketch prints
COLUMNS = ["ax", "ay", "az", "gx", "gy", "gz", "dist", "tilt"]
# Allowed labels; rejecting anything else stops a typo from creating a bogus class
LABELS = ["squat_good", "squat_lean", "squat_fast", "lunge", "hinge", "carry", "floor", "idle"]


def main():
    if len(sys.argv) < 3:
        sys.exit("usage: python record.py <label> <COM port> [seconds]\nlabels: " + ", ".join(LABELS))
    label, port = sys.argv[1], sys.argv[2]
    if label not in LABELS:
        sys.exit(f"unknown label '{label}', use one of: {', '.join(LABELS)}")
    seconds = float(sys.argv[3]) if len(sys.argv) > 3 else 60
    out = Path(__file__).parent / "data" / f"{label}_{time.strftime('%Y%m%d-%H%M%S')}.csv"
    out.parent.mkdir(exist_ok=True)

    with serial.Serial(port, 115200, timeout=1) as ser, out.open("w", newline="") as f:
        # Countdown so the user can get into position before recording starts
        for i in (3, 2, 1):
            print(f"starting in {i}...")
            time.sleep(1)
        print(f"recording '{label}' for {seconds:.0f}s")
        ser.reset_input_buffer()  # drop data buffered during the countdown
        writer = csv.writer(f)
        writer.writerow(COLUMNS)
        count = 0
        end = time.time() + seconds
        while time.time() < end:
            # Skip startup messages and partial or corrupted lines
            parts = ser.readline().decode(errors="ignore").strip().split(",")
            if parts[0] != "D" or len(parts) != len(COLUMNS) + 1:
                continue
            try:
                writer.writerow([float(p) for p in parts[1:]])
            except ValueError:
                continue
            count += 1

    print(f"saved {count} samples to {out}")


if __name__ == "__main__":
    main()
