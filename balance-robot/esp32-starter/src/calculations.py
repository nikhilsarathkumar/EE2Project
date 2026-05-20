"""compute_gains.py — compute LQR K gains for the balancing robot.

Computes both continuous-time and discrete-time gains and compares them.
Use the DISCRETE gains in main_lqr.cpp — they are correct for a sampled
controller. The continuous gains work in practice at 100 Hz but are
theoretically incorrect.

Usage:
    pip install numpy scipy
    python compute_gains.py
"""

import numpy as np
from scipy.linalg import solve_continuous_are, solve_discrete_are
from scipy.signal import cont2discrete

# ── Physical parameters — measure these on your robot ────────────────────────
m   = 1.2957    # body mass (kg)
l   = 0.105   # CoM height above axle (m)
M   = (1/3) * m * l**2    # effective wheel mass incl. rotational inertia (kg)
I   = 0.35**2 * m * (1/12)  # body moment of inertia about CoM (kg·m²)
b   = 0.0   # viscous friction (N·s/m)
g   = 9.81   # m/s²
Ts  = 0.01   # sample period (s) — must match LOOP_INTERVAL_MS in firmware

# ── Linearised model coefficients ────────────────────────────────────────────
D0   = (M + m)*(I + m*l**2) - (m*l)**2
a_tt = (M + m)*m*g*l / D0
b_th = m*l / D0

print("=" * 52)
print("Plant")
print("=" * 52)
print(f"  Δ₀       = {D0:.4f}")
print(f"  a_θθ     = {a_tt:.3f}  (unstable pole ±{a_tt**0.5:.3f} rad/s)")
print(f"  b_θ      = {b_th:.4f}")

# ── Continuous system ─────────────────────────────────────────────────────────
A = np.array([[0,    1   ],
              [a_tt, 0   ]])
B = np.array([[0    ],
              [-b_th]])

# ── LQR cost matrices ─────────────────────────────────────────────────────────
# Q[0,0]: angle penalty — larger = stiffer response to tilt
# Q[1,1]: rate penalty  — larger = more damping
# R:      control effort penalty — smaller = more aggressive
Q = np.diag([300.0, 2.0])
R = np.array([[0.01]])

# ── Continuous-time LQR (CARE) ────────────────────────────────────────────────
Pc = solve_continuous_are(A, B, Q, R)
Kc = (np.linalg.inv(R) @ B.T @ Pc).flatten()
cl_eigs_c = np.linalg.eigvals(A - B @ Kc.reshape(1, -1))

# ── Discrete-time LQR (DARE) ──────────────────────────────────────────────────
sysd = cont2discrete((A, B, np.eye(2), np.zeros((2, 1))), Ts, method='zoh')
Ad, Bd = sysd[0], sysd[1]
Pd = solve_discrete_are(Ad, Bd, Q, R)
Kd = (np.linalg.inv(R + Bd.T @ Pd @ Bd) @ (Bd.T @ Pd @ Ad)).flatten()
cl_eigs_d = np.linalg.eigvals(Ad - Bd @ Kd.reshape(1, -1))

# ── Results ───────────────────────────────────────────────────────────────────
print()
print("=" * 52)
print(f"Continuous-time LQR  (theoretically incorrect for sampled controller)")
print("=" * 52)
print(f"  K1 (angle) = {Kc[0]:.2f}")
print(f"  K2 (rate)  = {Kc[1]:.2f}")
print(f"  CL poles   = {cl_eigs_c}")

print()
print("=" * 52)
print(f"Discrete-time LQR  (USE THESE)  Ts = {Ts*1000:.0f} ms")
print("=" * 52)
print(f"  K1 (angle) = {Kd[0]:.2f}")
print(f"  K2 (rate)  = {Kd[1]:.2f}")
print(f"  CL poles   = {cl_eigs_d}  (must be inside unit circle)")
print(f"  |poles|    = {np.abs(cl_eigs_d)}")

stable = all(np.abs(cl_eigs_d) < 1.0)
print(f"  Stable?    = {'YES' if stable else 'NO — increase Q or decrease R'}")

print()
print("Paste into main_lqr.cpp:")
print(f"  const float K1 = {Kd[0]:.1f}f;")
print(f"  const float K2 = {Kd[1]:.1f}f;")