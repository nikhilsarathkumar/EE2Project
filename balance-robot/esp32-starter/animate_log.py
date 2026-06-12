#!/usr/bin/env python3
"""
animate_log.py  —  side-view animation of balance robot from a log.

Usage:
    python animate_log.py               # reads from clipboard
    python animate_log.py log.txt       # reads from file
    python animate_log.py --speed 2.0   # 2x playback speed
    python animate_log.py --save out.gif

Space bar pauses/unpauses.
"""

import sys
import argparse
import itertools
import tkinter as tk
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.animation import FuncAnimation


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


def main():
    parser = argparse.ArgumentParser(description='Animate balance robot log.')
    parser.add_argument('file', nargs='?', help='Log file (default: clipboard)')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='Playback speed multiplier (default: 1.0)')
    parser.add_argument('--save', metavar='FILE',
                        help='Save animation to GIF or MP4 instead of showing')
    args = parser.parse_args()

    if args.file:
        with open(args.file) as f:
            lines = f.readlines()
    else:
        root = tk.Tk()
        root.withdraw()
        lines = root.clipboard_get().splitlines()
        root.destroy()
        print(f"Read {len(lines)} lines from clipboard.")

    rows = [r for line in lines if (r := parse_line(line)) is not None]
    if not rows:
        sys.exit("No parseable lines found.")

    t_ms  = np.array([r['t']                  for r in rows])
    theta = np.array([r.get('th',   0.0)      for r in rows])
    u_arr = np.array([r.get('u',    0.0)      for r in rows])
    th_d  = np.array([r.get('th_d', 0.0)      for r in rows])
    spd   = np.array([r.get('spd',  0.0)      for r in rows])

    # ── Robot geometry (display units) ────────────────────────────────────────
    R  = 0.12          # wheel radius
    L  = 1.0           # body length
    cx, cy = 0.0, R    # wheel centre

    WHEEL_RADIUS_M  = 0.034
    COM_HEIGHT_M    = 0.105
    DISPLAY_PER_M   = L / COM_HEIGHT_M   # display units per real metre

    MARK_SPACING_M  = 0.10               # floor tick every 10 cm
    MARK_SPACING_D  = MARK_SPACING_M * DISPLAY_PER_M

    # Accumulate wheel rotation for the spoke, and robot x position
    wheel_angle = np.zeros(len(rows))
    x_display   = np.zeros(len(rows))   # cumulative x in display units
    for i in range(1, len(rows)):
        dt = (t_ms[i] - t_ms[i - 1]) / 1000.0
        wheel_angle[i] = wheel_angle[i - 1] + spd[i] * dt
        x_display[i]   = x_display[i - 1] + spd[i] * WHEEL_RADIUS_M * dt * DISPLAY_PER_M

    U_MAX   = 30.0   # used to normalise arrow length
    ARR_LEN = 0.8    # max arrow length in display units

    # ── Figure ────────────────────────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(5, 8))
    ax.set_aspect('equal')
    ax.set_xlim(-1.8, 1.8)
    ax.set_ylim(-0.3, 2.3)
    ax.axis('off')

    # Ground
    ax.fill_between([-1.8, 1.8], [-0.3, -0.3], [0, 0], color='#cccccc', zorder=0)
    ax.axhline(0, color='#888', lw=1.5, zorder=1)

    # Floor tick marks — pre-allocate enough to tile the view + 2 buffer
    N_MARKS = int(np.ceil(3.6 / MARK_SPACING_D)) + 2
    mark_lines = [ax.plot([], [], color='#666', lw=1.2, zorder=2)[0]
                  for _ in range(N_MARKS)]
    mark_texts = [ax.text(0, -0.13, '', fontsize=6, ha='center',
                          color='#444', zorder=2)
                  for _ in range(N_MARKS)]

    # Vertical reference
    ax.plot([cx, cx], [cy, cy + L + R + 0.1],
            color='#cccccc', lw=1, ls='--', zorder=1)

    # Wheel
    ax.add_patch(Circle((cx, cy), R, facecolor='#555555',
                         edgecolor='#222222', lw=2, zorder=3))
    spoke_line, = ax.plot([], [], color='#dddddd', lw=1.5, zorder=4)

    # Body
    body_line, = ax.plot([], [], color='#2196F3', lw=5,
                          solid_capstyle='round', zorder=5)
    com_dot,   = ax.plot([], [], 'o', color='#e53935', ms=12, zorder=6)

    # Control arrow (quiver — single arrow)
    quiv = ax.quiver([cx], [cy], [0], [0],
                     color='#FF9800',
                     scale=1, scale_units='xy', units='xy',
                     width=0.038, headwidth=4, headlength=5,
                     headaxislength=4, zorder=7)

    # u value label near arrow tip
    u_label = ax.text(cx, cy, '', fontsize=9, fontweight='bold',
                      va='center', zorder=8)

    # Info panel
    info = ax.text(-1.7, 2.22, '', fontsize=9, fontfamily='monospace',
                   va='top', color='#111111',
                   bbox=dict(facecolor='white', alpha=0.80,
                             edgecolor='none', pad=5),
                   zorder=9)

    title_obj = ax.set_title('', fontsize=10)

    # ── Playback state ────────────────────────────────────────────────────────
    paused   = [False]
    frame_i  = [0]

    def on_key(event):
        if event.key == ' ':
            paused[0] = not paused[0]

    fig.canvas.mpl_connect('key_press_event', on_key)

    def update(_):
        i = frame_i[0]
        if not paused[0] and i < len(rows) - 1:
            frame_i[0] += 1

        th = theta[i]
        u  = u_arr[i]

        # Body stick
        bx = cx + (L + R) * np.sin(th)
        by = cy + (L + R) * np.cos(th)
        body_line.set_data([cx, bx], [cy, by])
        com_dot.set_data([bx], [by])

        # Floor tick marks — scroll with robot position
        x_off = x_display[i]
        first_world = np.floor((-1.8 + x_off) / MARK_SPACING_D) * MARK_SPACING_D
        for j, (ml, mt) in enumerate(zip(mark_lines, mark_texts)):
            world = first_world + j * MARK_SPACING_D
            screen = world - x_off
            ml.set_data([screen, screen], [-0.18, 0.04])
            dist_m = world / DISPLAY_PER_M
            mt.set_position((screen, -0.22))
            mt.set_text(f'{dist_m:.1f}')

        # Rotating spoke
        ang = wheel_angle[i]
        spoke_line.set_data(
            [cx + R * np.cos(ang), cx - R * np.cos(ang)],
            [cy + R * np.sin(ang), cy - R * np.sin(ang)],
        )

        # Control arrow — length proportional to u, colour by direction
        dx = float(np.clip(u / U_MAX, -1.0, 1.0)) * ARR_LEN
        quiv.set_UVC([dx], [0])
        col = '#FF9800' if u >= 0 else '#29B6F6'
        quiv.set_color(col)

        # Label at arrow tip
        tip_x = cx + dx
        offset = 0.08 if dx >= 0 else -0.08
        u_label.set_position((tip_x + offset, cy))
        u_label.set_ha('left' if dx >= 0 else 'right')
        u_label.set_text(f'u={u:+.1f}')
        u_label.set_color(col)

        # Info text
        dist_m = x_display[i] / DISPLAY_PER_M
        info.set_text(
            f"t   = {(t_ms[i] - t_ms[0]) / 1000:.2f} s\n"
            f"θ   = {th:+.4f} rad  ({np.degrees(th):+.1f}°)\n"
            f"θ̇   = {th_d[i]:+.4f} rad/s\n"
            f"u   = {u:+.1f} rad/s\n"
            f"spd = {spd[i]:+.1f} rad/s\n"
            f"x   = {dist_m:+.3f} m"
        )
        status = '⏸ paused' if paused[0] else f'▶ ×{args.speed:.1f}'
        title_obj.set_text(f'frame {i + 1}/{len(rows)}  {status}  [space=pause]')

        return body_line, com_dot, spoke_line, quiv, u_label, info

    dt_log_ms  = float(np.median(np.diff(t_ms))) if len(t_ms) > 1 else 50.0
    interval_ms = max(16.0, dt_log_ms / args.speed)

    anim = FuncAnimation(fig, update, frames=itertools.count(),
                          interval=interval_ms, blit=False)

    if args.save:
        print(f"Saving to {args.save} …")
        anim.save(args.save, fps=round(1000 / interval_ms))
        print("Done.")
    else:
        plt.tight_layout()
        plt.show()


if __name__ == '__main__':
    main()
