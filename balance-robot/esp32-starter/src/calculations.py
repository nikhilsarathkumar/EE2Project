"""compute_gains.py — gains for the DIRECT SPEED law (no omega_c integrator).

Firmware law:  u = -(K_THETA*theta + K_THETA_DOT*theta_dot),  u commanded as
a wheel SPEED (rad/s) straight into setTargetSpeedRad().

Model (speed enters via its derivative, stepper assumed to track instantly):
    theta_ddot = a_tt*theta - kappa*omega_dot,   kappa = a_tt*rho/g

This is a 2-pole placement, not a force LQR — there is no scalar that turns
force gains into speed gains, because speed enters the tilt equation through
omega_dot, not directly. So we place the two tilt poles directly:

    K_THETA     = 2*zeta*a_tt / (wn*kappa)
    K_THETA_DOT = (1 + a_tt/wn**2) / kappa

Usage:  python compute_gains.py
"""

import numpy as np

# ── Measured plant ───────────────────────────────────────────────────────────
a_tt = 37.0     # mgl/(I+ml^2)  [1/s^2]  <- swing test: a_tt = (2*pi/T)^2
rho  = 0.034    # wheel radius  [m]      <- caliper
g    = 9.81

# ── Desired closed-loop tilt response ────────────────────────────────────────
wn   = 4.0      # natural frequency [rad/s] — higher = stiffer/faster
zeta = 0.7      # damping ratio     — ~0.6-0.9 is a sensible range

kappa = a_tt * rho / g

K_THETA     = 2.0*zeta*a_tt / (wn*kappa)
K_THETA_DOT = (1.0 + a_tt/wn**2) / kappa

# ── Forward check: reconstruct the closed-loop poles from these gains ────────
# theta_ddot = [a_tt/(1-kappa*Kthd)]*theta + [kappa*Kth/(1-kappa*Kthd)]*theta_dot
den = 1.0 - kappa*K_THETA_DOT
c_theta = a_tt / den            # = -wn^2
c_thd   = kappa*K_THETA / den   # = -2*zeta*wn
poles = np.roots([1.0, -c_thd, -c_theta])

print("=" * 50)
print("Direct speed law  (no omega_c)")
print("=" * 50)
print(f"  a_tt   = {a_tt:.3f}   (open-loop unstable pole +/-{a_tt**0.5:.3f})")
print(f"  kappa  = {kappa:.4f}")
print(f"  target : wn = {wn:.2f} rad/s,  zeta = {zeta:.2f}")
print()
print(f"  K_THETA     = {K_THETA:.2f}")
print(f"  K_THETA_DOT = {K_THETA_DOT:.2f}")
print(f"  ratio       = {K_THETA/K_THETA_DOT:.2f} : 1")
print()
print(f"  closed-loop poles = {np.round(poles,3)}")
stable = np.all(np.real(poles) < 0)
print(f"  stable?           = {'YES' if stable else 'NO'}")
print()
print("Paste into main.cpp (firmware does u = -K_THETA*theta - K_THETA_DOT*theta_dot):")
print(f"  const float K_THETA     = -{K_THETA:.2f}f;")
print(f"  const float K_THETA_DOT = -{K_THETA_DOT:.2f}f;")