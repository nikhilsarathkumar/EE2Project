"""
Estimates the derived physical parameters M, I, and b for the balance robot.

M  — effective wheel mass (translational + rotational inertia of wheel)
I  — body moment of inertia about its own centre of mass
b  — viscous rolling friction coefficient

Edit the values in MEASURED INPUTS below to match your robot.
"""

# ══════════════════════════════════════════════════════════════════════════════
# MEASURED INPUTS  — fill these in from your robot
# ══════════════════════════════════════════════════════════════════════════════

# Wheel
m_w   = 0.100   # mass of one wheel (kg)
rho   = 0.040   # wheel radius (m)
# Wheel assumed to be a solid disc: J_w = 0.5 * m_w * rho^2
# If hollow/spoked, change the coefficient (0.5 → closer to 1.0)
wheel_inertia_factor = 0.5

# Body
m     = 0.800   # body mass (kg)  — everything above the axle
L_body = 0.300  # total length of body (m)  — used to estimate I
# Body approximated as a uniform rod rotating about one end:
#   CoM is at L_body/2 from axle  →  I_axle = m*L_body^2/3
#   parallel axis: I_com = I_axle - m*(L_body/2)^2 = m*L_body^2/12
# If your CoM is not at the midpoint, adjust l_com below.
l_com = L_body / 2   # distance from axle to body CoM (m)

# Friction
# b is best measured by letting the robot coast and fitting an exponential decay.
# As a starting estimate we use: b ≈ (total weight) × (rolling resistance coeff)
# Typical rubber-on-floor rolling resistance coefficient: 0.005–0.02
rolling_resistance_coeff = 0.01
g = 9.81

# ══════════════════════════════════════════════════════════════════════════════
# DERIVED PARAMETERS
# ══════════════════════════════════════════════════════════════════════════════

# Effective wheel mass: M = m_w + J_w / rho^2
J_w = wheel_inertia_factor * m_w * rho**2
M   = m_w + J_w / rho**2

# Body moment of inertia about CoM (uniform rod, CoM at midpoint)
I   = m * L_body**2 / 12

# Viscous friction coefficient: b ≈ mu_r * (m + 2*m_w) * g / v_ref
# where v_ref ~ 1 m/s converts rolling resistance (force) to viscous (force/velocity)
total_weight = (m + 2 * m_w) * g
b = rolling_resistance_coeff * total_weight   # N·s/m

# ══════════════════════════════════════════════════════════════════════════════
# OUTPUT
# ══════════════════════════════════════════════════════════════════════════════

print(f"M = {M:.4f}  kg      (effective wheel mass, each wheel treated as solid disc)")
print(f"I = {I:.4f}  kg·m²   (body MOI about CoM, uniform rod approximation)")
print(f"b = {b:.4f}  N·s/m   (viscous rolling friction, rolling resistance estimate)")
print()
print("Other parameters to pass to A/B calculation:")
print(f"  m   = {m}   kg")
print(f"  l   = {l_com}   m   (axle to CoM)")
print(f"  rho = {rho}   m   (wheel radius)")
print(f"  g   = {g}   m/s²")
