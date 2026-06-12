#!/usr/bin/env python3
"""
plot_log.py  —  plot theta_dot_raw vs theta_dot_filt from a balance-robot log.

Usage:
    python plot_log.py            # reads from clipboard (copy log first)
    python plot_log.py log.txt    # read from file
    python plot_log.py log.txt --save  # save PNG instead of showing
"""

import sys
import argparse
import tkinter as tk
import matplotlib.pyplot as plt

def parse_line(line):
    """Return dict of key→float for a log line, or None if unparseable."""
    line = line.strip()
    if not line or line.startswith('#') or line.startswith('>>'):
        return None
    pairs = {}
    for token in line.split():
        if '=' not in token:
            continue
        k, _, v = token.partition('=')
        try:
            pairs[k] = float(v)
        except ValueError:
            pass  # skip non-numeric values like key=-
    return pairs if 't' in pairs else None

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('file', nargs='?', help='Log file (default: stdin)')
    parser.add_argument('--save', action='store_true', help='Save PNG instead of showing')
    args = parser.parse_args()

    if args.file:
        with open(args.file) as f:
            lines = f.readlines()
    else:
        root = tk.Tk()
        root.withdraw()
        text = root.clipboard_get()
        root.destroy()
        lines = text.splitlines()
        print(f"Read {len(lines)} lines from clipboard.")

    rows = [r for line in lines if (r := parse_line(line)) is not None]

    if not rows:
        sys.exit("No parseable lines found.")

    t_ms = [r['t']                        for r in rows]
    td_r  = [r.get('td_r',  float('nan')) for r in rows]
    th_d  = [r.get('th_d',  float('nan')) for r in rows]
    th_d7 = [r.get('th_d7', float('nan')) for r in rows]
    td_c  = [r.get('td_c',  float('nan')) for r in rows]
    th    = [r.get('th',    float('nan')) for r in rows]

    # Normalise time to start at 0
    t0 = t_ms[0]
    t_s = [(t - t0) / 1000.0 for t in t_ms]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8), sharex=True)

    ax1.plot(t_s, td_r,  color='steelblue', lw=0.8, alpha=0.5, label='td_r (raw)')
    ax1.plot(t_s, th_d,  color='firebrick', lw=1.5,            label='th_d (LPF=0.95)')
    ax1.plot(t_s, th_d7, color='seagreen',  lw=1.5,            label='th_d7 (LPF=0.80)')
    ax1.plot(t_s, td_c,  color='purple',    lw=1.5, ls='--',   label='td_c (active)')
    ax1.axhline(0, color='k', lw=0.5, ls='--')
    ax1.set_ylabel('Angular rate (rad/s)')
    ax1.set_title('θ̇ raw vs filtered')
    ax1.legend()
    ax1.grid(alpha=0.3)

    ax2.plot(t_s, th, color='darkorange', lw=1.5, label='th (CF tilt angle)')
    ax2.axhline(0, color='k', lw=0.5, ls='--')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Angle (rad)')
    ax2.set_title('θ tilt angle')
    ax2.legend()
    ax2.grid(alpha=0.3)

    plt.tight_layout()

    if args.save:
        out = (args.file.rsplit('.', 1)[0] + '_plot.png') if args.file else 'log_plot.png'
        plt.savefig(out, dpi=150)
        print(f"Saved → {out}")
    else:
        plt.show()

if __name__ == '__main__':
    main()
