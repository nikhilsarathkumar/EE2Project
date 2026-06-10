#!/usr/bin/env python3
"""
testing.py  —  a_theta_theta identification from a free-swing IMU log.

Fits a damped sinusoid to both td_r (raw gyro) and th_d (IIR filtered)
and reports a_tt from each for comparison.

Usage:
    python testing.py            # reads from clipboard (copy log first)
    python testing.py log.txt    # read from file
"""

import sys
import argparse
import numpy as np
import tkinter as tk
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from scipy.signal import hilbert

# ── Physical parameters (match firmware) ────────────────────────────────────
RHO   = 0.034
TAU_A = 0.04
G     = 9.81

# ── Crop: set T_START / T_END to isolate the clean free-decay window (s) ───
T_START, T_END = 0.0, None

def parse_line(line):
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
            pass
    return pairs if 't' in pairs else None

def damped(t, A, sigma, wd, phi):
    return A * np.exp(-sigma * t) * np.cos(wd * t + phi)

def fit_signal(t_s, sig):
    """Fit damped sinusoid and return (popt, a_tt, a_err, zeta)."""
    dt  = np.mean(np.diff(t_s))
    f   = np.fft.rfftfreq(len(sig), dt)
    P   = np.abs(np.fft.rfft(sig)); P[f < 0.1] = 0.0
    wd0 = 2 * np.pi * f[np.argmax(P)]

    env  = np.abs(hilbert(sig))
    good = env > 0.05 * env.max()
    sig0 = max(-np.polyfit(t_s[good], np.log(env[good]), 1)[0], 0.0)
    A0   = env.max()

    # Crop to active oscillation window so the noise floor doesn't bias sigma
    last = np.where(good)[0][-1]
    popt, pcov = curve_fit(damped, t_s[:last+1], sig[:last+1], p0=[A0, sig0, wd0, 0.0], maxfev=30000)
    _, sigma, wd, _ = popt
    perr = np.sqrt(np.diag(pcov))

    wn    = np.hypot(wd, sigma)
    zeta  = sigma / wn
    a_tt  = wn ** 2
    a_err = 2 * np.hypot(wd * perr[2], sigma * perr[1])
    return popt, a_tt, a_err, zeta

def report(label, popt, a_tt, a_err, zeta):
    _, sigma, wd, _ = popt
    wn = np.hypot(wd, sigma)
    print(f"\n── {label} ──")
    print(f"  w_d   = {wd:.4f} rad/s   (T_d = {2*np.pi/wd:.4f} s)")
    print(f"  sigma = {sigma:.4f} 1/s")
    print(f"  zeta  = {zeta:.4f}   ({'OK' if zeta < 0.1 else 'HIGH'})")
    print(f"  w_n   = {wn:.4f} rad/s")
    print(f"  a_tt  = {a_tt:.3f} +/- {a_err:.3f}")
    print(f"  c_to  = {a_tt * RHO / (G * TAU_A):.4f}")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('file', nargs='?', help='Log file (default: clipboard)')
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

    t_raw  = np.array([r['t'] / 1000.0              for r in rows])
    td_r   = np.array([r.get('td_r', float('nan'))  for r in rows])
    th_d   = np.array([r.get('th_d', float('nan'))  for r in rows])

    def prepare(t_all, sig_all):
        valid = ~np.isnan(sig_all)
        t, s  = t_all[valid], sig_all[valid]
        t     = t - t[0]
        end   = t[-1] if T_END is None else T_END
        keep  = (t >= T_START) & (t <= end)
        t, s  = t[keep], s[keep]
        return t, s - np.mean(s)

    t_r, sig_r = prepare(t_raw, td_r)
    t_f, sig_f = prepare(t_raw, th_d)

    popt_r, a_tt_r, a_err_r, zeta_r = fit_signal(t_r, sig_r)
    popt_f, a_tt_f, a_err_f, zeta_f = fit_signal(t_f, sig_f)

    report("td_r  (raw gyro)",      popt_r, a_tt_r, a_err_r, zeta_r)
    report("th_d  (IIR filtered)",  popt_f, a_tt_f, a_err_f, zeta_f)

    # ── Plot ─────────────────────────────────────────────────────────────────
    fig, axes = plt.subplots(2, 2, figsize=(13, 7), sharex='col')

    axes[0,0].plot(t_r, sig_r, color='steelblue', lw=0.8, alpha=0.6, label='td_r')
    axes[0,0].plot(t_r, damped(t_r, *popt_r), 'r', lw=1.3, label='fit')
    axes[0,0].set_ylabel('rate (rad/s)')
    axes[0,0].set_title(f'td_r raw — a_tt={a_tt_r:.2f}, ζ={zeta_r:.3f}')
    axes[0,0].legend(); axes[0,0].grid(alpha=0.3)

    axes[1,0].plot(t_r, sig_r - damped(t_r, *popt_r), lw=0.8, color='steelblue')
    axes[1,0].set_ylabel('residual'); axes[1,0].set_xlabel('time (s)')
    axes[1,0].grid(alpha=0.3)

    axes[0,1].plot(t_f, sig_f, color='firebrick', lw=1.0, alpha=0.8, label='th_d')
    axes[0,1].plot(t_f, damped(t_f, *popt_f), color='darkorange', lw=1.3, label='fit')
    axes[0,1].set_title(f'th_d filtered — a_tt={a_tt_f:.2f}, ζ={zeta_f:.3f}')
    axes[0,1].legend(); axes[0,1].grid(alpha=0.3)

    axes[1,1].plot(t_f, sig_f - damped(t_f, *popt_f), lw=0.8, color='firebrick')
    axes[1,1].set_xlabel('time (s)')
    axes[1,1].grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig("swing_fit.png", dpi=130)
    plt.show()

if __name__ == '__main__':
    main()

"""
a_tt  = 48.087 +/- 0.239
a_tt  = 45.655 +/- 0.242

"""