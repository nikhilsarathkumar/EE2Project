#!/usr/bin/env python3
"""
serial_log.py  —  read from a serial/BT COM port, print to console and save to file.

Usage:
    python serial_log.py              # uses defaults: COM5, 115200
    python serial_log.py COM3         # specify port
    python serial_log.py COM3 9600    # specify port and baud
    python serial_log.py COM5 115200 --out my_log.txt   # custom filename

Output file is auto-named  log_YYYYMMDD_HHMMSS.txt  unless --out is given.
Press Ctrl+C to stop.
"""

import sys
import argparse
import datetime
import serial

DEFAULT_PORT = 'COM5'
DEFAULT_BAUD = 115200

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('port', nargs='?', default=DEFAULT_PORT)
    parser.add_argument('baud', nargs='?', type=int, default=DEFAULT_BAUD)
    parser.add_argument('--out', help='Output filename (default: auto timestamped)')
    args = parser.parse_args()

    filename = args.out or f"log_{datetime.datetime.now():%Y%m%d_%H%M%S}.txt"

    print(f"Opening {args.port} @ {args.baud} baud")
    print(f"Logging to {filename}")
    print("Press Ctrl+C to stop.\n")

    with serial.Serial(args.port, args.baud, timeout=1) as ser, \
         open(filename, 'w', encoding='utf-8') as f:
        try:
            while True:
                line = ser.readline()
                if not line:
                    continue
                text = line.decode('utf-8', errors='replace').rstrip('\r\n')
                print(text)
                f.write(text + '\n')
                f.flush()
        except KeyboardInterrupt:
            print(f"\nStopped. Log saved to {filename}")

if __name__ == '__main__':
    main()
