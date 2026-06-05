"""""
State vector:  x = [pos, vel, theta, theta_dot]
    pos       : forward position of the body (m)   — read from stepper counters
    vel       : forward velocity (m/s)
    theta     : tilt angle from vertical (rad)
    theta_dot : tilt rate (rad/s)

Computes continuous- and discrete-time gains. Use the DISCRETE gains.

Usage:
    pip install numpy scipy
    python compute_gains.py
"""
import numpy as np
from scipy.linalg import solve_continuous_are, solve_discrete_are
from scipy.signal import cont2discrete

# ── Physical parameters — measure these on your robot ────────────────────────
m   = 0.84
l   = 0.105
M   = 0.46
I   = 0.02
r   = 0.0335                 # WHEEL RADIUS (m) — measure this! firmware-only
b   = 0.0                    # viscous friction (N·s/m) — unused
g   = 9.81                   # m/s²
Ts  = 0.01                   # sample period (s) — must match LOOP_INTERVAL_MS

# ── Linearised model coefficients ────────────────────────────────────────────
D0   = (M + m)*(I + m*l**2) - (m*l)**2
a_tt =  (M + m)*m*g*l / D0       # theta -> theta_ddot  (unstable)
a_xt = -(m*l)*(m*g*l) / D0       # theta -> x_ddot
b_th =  m*l / D0                 # u     -> theta_ddot
b_x  =  (I + m*l**2) / D0        # u     -> x_ddot

print("=" * 52)
print("Plant (4-state)")
print("=" * 52)
print(f"  D0     = {D0:.5f}")
print(f"  a_tt   = {a_tt:.3f}   (tilt pole +/-{a_tt**0.5:.2f} rad/s)")
print(f"  a_xt   = {a_xt:.3f}")
print(f"  b_th   = {b_th:.4f}")
print(f"  b_x    = {b_x:.4f}")

# ── Continuous 4-state system:  state = [x, x_dot, theta, theta_dot] ──────────
A = np.array([[0, 1, 0,    0],
              [0, 0, a_xt, 0],
              [0, 0, 0,    1],
              [0, 0, a_tt, 0]])
B = np.array([[0    ],
              [b_x  ],
              [0    ],
              [-b_th]])

# ── LQR cost matrices  (penalise [x, x_dot, theta, theta_dot]) ────────────────
#   x        : how hard to hold position (too big -> fights balancing)
#   x_dot    : damping on forward drift
#   theta    : tilt stiffness
#   theta_dot: tilt damping
Q = np.diag([0.0, 1000.0, 30000.0, 100000.0])
R = np.array([[1.0]])

# ── Continuous-time LQR (reference) ───────────────────────────────────────────
Pc = solve_continuous_are(A, B, Q, R)
Kc = (np.linalg.inv(R) @ B.T @ Pc).flatten()

# ── Discrete-time LQR (USE THESE) ─────────────────────────────────────────────
Ad, Bd, *_ = cont2discrete((A, B, np.eye(4), np.zeros((4, 1))), Ts, method='zoh')
Pd = solve_discrete_are(Ad, Bd, Q, R)
Kd = (np.linalg.inv(R + Bd.T @ Pd @ Bd) @ (Bd.T @ Pd @ Ad)).flatten()
cl = np.linalg.eigvals(Ad - Bd @ Kd.reshape(1, -1))

labels = ["K_x  (pos) ", "K_v  (vel) ", "K_th (tilt)", "K_w  (rate)"]

print("\n" + "=" * 52)
print("Continuous-time LQR  (reference)")
print("=" * 52)
for lb, k in zip(labels, Kc):
    print(f"  {lb} = {k:9.3f}")

print("\n" + "=" * 52)
print(f"Discrete-time LQR  (USE THESE)   Ts = {Ts*1000:.0f} ms")
print("=" * 52)
for lb, k in zip(labels, Kd):
    print(f"  {lb} = {k:9.3f}")
print(f"  |CL poles| = {np.round(np.abs(cl), 4)}")
stable = np.all(np.abs(cl) < 1.0)
print(f"  Stable?    = {'YES' if stable else 'NO -- raise Q or lower R'}")

print("\nPaste into firmware:")
print(f"  const float K_X          = {Kd[0]:.3f}f;")
print(f"  const float K_X_DOT      = {Kd[1]:.3f}f;")
print(f"  const float K_THETA      = {Kd[2]:.2f}f;")
print(f"  const float K_THETA_DOT  = {Kd[3]:.2f}f;")
print(f"  const float WHEEL_RADIUS = {r}f;")