"""compute_gains.py — LQR gains for the 3-state acceleration-input model.

Model (tau_a -> 0, no actuator lag):
    state  z = [theta, theta_dot, x_dot]
    input  u = omega_c_dot   (commanded wheel ANGULAR ACCELERATION, rad/s^2)

    theta_ddot = a_tt*theta - kappa*u
    x_ddot     = rho*u
    a_tt  = m g l / (I + m l^2)        <- measured directly by the swing test
    kappa = m l rho / (I + m l^2) = a_tt * rho / g

The firmware law is  u = -(K_THETA*theta + K_THETA_DOT*theta_dot + K_XDOT*x_dot),
i.e. u = -K z, so the DARE gains paste in directly (sign included).

Usage:
    pip install numpy scipy
    python compute_gains.py
"""

import numpy as np
from scipy.linalg import solve_discrete_are
from scipy.signal import cont2discrete

# ── Measured plant coefficients ──────────────────────────────────────────────
a_tt = 47.0     # mgl/(I+ml^2)  [1/s^2]  <- from the swing test: a_tt = (2*pi/T)^2
rho  = 0.034    # wheel radius  [m]      <- caliper
g    = 9.81     # m/s^2
Ts   = 0.01     # sample period [s] — must match LOOP_INTERVAL_MS in firmware

# optional cross-check from individual params (swing test is authoritative):
#   m, l, I -> a_tt = m*g*l / (I + m*l**2)

kappa = a_tt * rho / g          # input coupling into theta_ddot

# ── Continuous 3-state system ────────────────────────────────────────────────
A = np.array([[0.0,   1.0, 0.0],
              [a_tt,  0.0, 0.0],
              [0.0,   0.0, 0.0]])
B = np.array([[0.0   ],
              [-kappa],
              [ rho  ]])

print("=" * 52)
print("Plant  (3-state, acceleration input)")
print("=" * 52)
print(f"  a_tt   = {a_tt:.3f}   (unstable pole +/-{a_tt**0.5:.3f} rad/s)")
print(f"  kappa  = {kappa:.4f}  (= a_tt*rho/g)")
print(f"  rho    = {rho:.4f}")

# ── LQR cost matrices ────────────────────────────────────────────────────────
# Q penalises state, R penalises control effort (now an ACCELERATION).
# Bryson's rule starting point: Q_ii ~ 1/x_i_max^2, R ~ 1/u_max^2, then iterate.
#   Q[0,0] theta   : larger = stiffer to tilt
#   Q[1,1] thetadot: larger = more damping
#   Q[2,2] x_dot   : larger = tighter velocity regulation (must be > 0 for DARE)
# These are STARTING points — tune against the robot.
Q = np.diag([60.0, 30.0, 1.0])
R = np.array([[100.0]])

# ── Discrete-time LQR (DARE) — USE THESE ─────────────────────────────────────
# ZOH discretisation is exact here: the firmware holds u over the sample and
# integrates it (omega_c += u*Ts), which is the ZOH on the acceleration input.
sysd = cont2discrete((A, B, np.eye(3), np.zeros((3, 1))), Ts, method='zoh')
Ad, Bd = sysd[0], sysd[1]
Pd = solve_discrete_are(Ad, Bd, Q, R)
Kd = (np.linalg.inv(R + Bd.T @ Pd @ Bd) @ (Bd.T @ Pd @ Ad)).flatten()
cl_d = np.linalg.eigvals(Ad - Bd @ Kd.reshape(1, -1))

print()
print("=" * 52)
print(f"Discrete-time LQR  (USE THESE)   Ts = {Ts*1000:.0f} ms")
print("=" * 52)
print(f"  K_theta     = {Kd[0]:.2f}")
print(f"  K_theta_dot = {Kd[1]:.2f}")
print(f"  K_x_dot     = {Kd[2]:.2f}")
print(f"  CL poles    = {np.round(cl_d, 3)}  (must be inside unit circle)")
print(f"  |poles|     = {np.round(np.abs(cl_d), 4)}")

stable = bool(np.all(np.abs(cl_d) < 1.0))
print(f"  Stable?     = {'YES' if stable else 'NO — increase Q or decrease R'}")

print()
print("Paste into main.cpp  (u = -K z, so signs are included):")
print(f"  const float K_THETA     = {Kd[0]:.2f}f;")
print(f"  const float K_THETA_DOT = {Kd[1]:.2f}f;")
print(f"  const float K_XDOT      = {Kd[2]:.2f}f;")
print()
print("Bring-up tip: set K_XDOT=0 in firmware to confirm balance on theta first;")
print("it will drift in velocity. Then re-enable K_XDOT to regulate x_dot.")