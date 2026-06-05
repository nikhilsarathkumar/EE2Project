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
Q = np.diag([21000, 10000])
R = np.array([[1.0]])

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
print("Paste into main.cpp  (use CONTINUOUS gains — model input is force, not speed):")
print(f"  const float K_THETA     = {Kc[0]:.1f}f;")
print(f"  const float K_THETA_DOT = {Kc[1]:.1f}f;")
print()
print("Discrete gains (theoretically correct for sampled controller,")
print("but only valid if u = wheel force, not wheel speed):")
print(f"  K1 = {Kd[0]:.2f},  K2 = {Kd[1]:.2f}")

# ── Inverse LQR — find Q that produces a desired K ───────────────────────────
# Given K = [K1_abs, K2_abs] (positive, matching |K_THETA|, |K_THETA_DOT| in firmware)
# and a chosen R, compute the unique diagonal Q such that LQR(A,B,Q,R) → K.
# Derivation: from K = R⁻¹BᵀP and requiring Q[0,1]=0 (diagonal Q).

K1_target = 160.0   # |K_THETA|   from main.cpp
K2_target = 100.0   # |K_THETA_DOT| from main.cpp
R_inv     = 1.0     # free choice — scales Q proportionally

P01 = R_inv * K1_target / b_th
P11 = R_inv * K2_target / b_th
P00 = R_inv * K2_target * (K1_target - a_tt / b_th)

Q00 = R_inv * K1_target * (K1_target - 2.0 * a_tt / b_th)
Q11 = R_inv * (K2_target**2 - 2.0 * K1_target / b_th)

P_inv = np.array([[P00, P01], [P01, P11]])
Q_inv = np.diag([Q00, Q11])

eigs_P = np.linalg.eigvals(P_inv)
valid  = np.all(eigs_P > 0) and Q00 >= 0 and Q11 >= 0

print()
print("=" * 52)
print(f"Inverse LQR  (K_target = [{K1_target}, {K2_target}])")
print("=" * 52)
print(f"  Q[0,0]  = {Q00:.1f}")
print(f"  Q[1,1]  = {Q11:.1f}")
print(f"  R       = {R_inv}")
print(f"  P valid = {'YES' if valid else 'NO — gains too low for this model'}")

if valid:
    # Verify: forward LQR should recover K_target
    Pc2 = solve_continuous_are(A, B, Q_inv, np.array([[R_inv]]))
    K_check = (np.linalg.inv(np.array([[R_inv]])) @ B.T @ Pc2).flatten()
    print(f"  Verification: Kc = [{K_check[0]:.1f}, {K_check[1]:.1f}]  (should be [{-K1_target:.0f}, {-K2_target:.0f}])")
    print()
    print(f"  Use these Q values in the LQR section above to get your target gains.")
    print(f"  Q = np.diag([{Q00:.1f}, {Q11:.1f}])")
    print(f"  R = np.array([[{R_inv}]])")