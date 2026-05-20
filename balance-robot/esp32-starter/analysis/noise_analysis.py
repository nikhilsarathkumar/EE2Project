"""
IMU Noise Characterisation for Kalman Filter Parameterisation
=============================================================
Reads stationary MPU6050 data logged at 100 Hz over serial.
File format: two space-separated columns per line — accel_z (m/s²), gyro_y (rad/s)

State vector assumed by downstream Kalman/LQR: [theta, theta_dot, x_pos, x_dot]
  - theta     estimated via accel:  a_z / 9.81  (small-angle approx)
  - theta_dot measured directly by gyro_y

Outputs:
  - Summary statistics and R/Q matrices printed to console
  - Plots saved to ./plots/
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal, stats
from scipy.optimize import curve_fit
import os
import sys

# ─── CONFIG ───────────────────────────────────────────────────────────────────

DATA_FILE = "imu_log.txt"   # path to logged .txt file (relative or absolute)
FS        = 100.0           # sample rate Hz — matches 10 ms loop interval
DT        = 1.0 / FS
PLOTS_DIR = "plots"
G         = 9.81            # m/s²

# ─── SETUP ────────────────────────────────────────────────────────────────────

os.makedirs(PLOTS_DIR, exist_ok=True)

plt.rcParams.update({"figure.dpi": 120, "font.size": 10})


def save(name):
    path = os.path.join(PLOTS_DIR, name)
    plt.savefig(path, bbox_inches="tight")
    print(f"  Saved: {path}")
    plt.close()


# ─── LOAD DATA ────────────────────────────────────────────────────────────────

print("=" * 60)
print("LOADING DATA")
print("=" * 60)

try:
    # Handle UTF-16 (PowerShell Tee-Object default) and skip non-numeric lines
    with open(DATA_FILE, encoding="utf-16") as f:
        lines = f.readlines()
except (FileNotFoundError, UnicodeError):
    try:
        with open(DATA_FILE, encoding="utf-8") as f:
            lines = f.readlines()
    except FileNotFoundError:
        sys.exit(f"Error: '{DATA_FILE}' not found. Run the ESP32 logger first.")

rows = []
for line in lines:
    parts = line.strip().split()
    if len(parts) == 2:
        try:
            rows.append([float(parts[0]), float(parts[1])])
        except ValueError:
            pass  # skip header/non-numeric lines

if not rows:
    sys.exit("Error: no valid data rows found in file.")

data = np.array(rows)

if data.ndim != 2 or data.shape[1] < 2:
    sys.exit("Error: expected two columns (accel_z, gyro_y) per line.")

accel_z = data[:, 0]   # raw accelerometer z  (m/s²)
gyro_y  = data[:, 1]   # raw gyroscope y       (rad/s)
N       = len(accel_z)
t       = np.arange(N) * DT

print(f"  Samples loaded : {N}")
print(f"  Duration       : {t[-1]:.1f} s  @  {FS:.0f} Hz")


# ─── SECTION 1: SUMMARY STATISTICS ───────────────────────────────────────────

print("\n" + "=" * 60)
print("SECTION 1 — SUMMARY STATISTICS")
print("=" * 60)

for label, sig, unit in [("Accel Z", accel_z, "m/s²"),
                          ("Gyro Y",  gyro_y,  "rad/s")]:
    mu  = np.mean(sig)
    std = np.std(sig, ddof=1)
    var = std ** 2
    print(f"\n  {label}:")
    print(f"    Mean     : {mu:+.6f} {unit}")
    print(f"    Std dev  : {std:.6f} {unit}")
    print(f"    Variance : {var:.2e} {unit}²")
    print(f"    Min/Max  : {sig.min():.6f} / {sig.max():.6f} {unit}")


# ─── SECTION 2: GYRO BIAS ─────────────────────────────────────────────────────

print("\n" + "=" * 60)
print("SECTION 2 — GYRO BIAS")
print("=" * 60)

gyro_bias = np.mean(gyro_y)
print(f"\n  Gyro bias (mean at rest) : {gyro_bias:+.6f} rad/s")
print(f"  Suggestion: subtract {gyro_bias:+.6f} from every gyro reading before")
print(f"  feeding into the Kalman filter.")
print(f"\n  Bias-corrected gyro_y = g.gyro.y - ({gyro_bias:.6f})")

gyro_y_corrected = gyro_y - gyro_bias


# ─── SECTION 3: HISTOGRAMS WITH GAUSSIAN FIT ─────────────────────────────────

print("\n" + "=" * 60)
print("SECTION 3 — HISTOGRAMS")
print("=" * 60)

def gaussian(x, mu, sigma):
    return stats.norm.pdf(x, mu, sigma)

fig, axes = plt.subplots(1, 2, figsize=(12, 4))
fig.suptitle("Histogram with Gaussian Fit — stationary noise")

for ax, sig, label, unit in [
        (axes[0], accel_z,          "Accel Z", "m/s²"),
        (axes[1], gyro_y_corrected, "Gyro Y (bias-removed)", "rad/s")]:

    counts, edges, _ = ax.hist(sig, bins=60, density=True,
                                alpha=0.6, color="steelblue", label="Data")
    x_fit = np.linspace(sig.min(), sig.max(), 400)
    mu_fit, std_fit = np.mean(sig), np.std(sig, ddof=1)
    ax.plot(x_fit, gaussian(x_fit, mu_fit, std_fit),
            "r-", lw=2, label=f"Gaussian\nμ={mu_fit:.4f}\nσ={std_fit:.4f}")
    ax.set_xlabel(f"{label} ({unit})")
    ax.set_ylabel("Probability density")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

plt.tight_layout()
save("histograms.png")
print("  Done.")


# ─── SECTION 4: POWER SPECTRAL DENSITY ───────────────────────────────────────

print("\n" + "=" * 60)
print("SECTION 4 — POWER SPECTRAL DENSITY")
print("=" * 60)

fig, axes = plt.subplots(1, 2, figsize=(12, 4))
fig.suptitle("Power Spectral Density")

for ax, sig, label, unit in [
        (axes[0], accel_z,          "Accel Z", "m/s²"),
        (axes[1], gyro_y_corrected, "Gyro Y (bias-removed)", "rad/s")]:

    f, pxx = signal.welch(sig, fs=FS, nperseg=min(1024, N // 4))
    ax.semilogy(f, pxx, color="steelblue")
    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel(f"PSD ({unit}²/Hz)")
    ax.set_title(label)
    ax.grid(True, alpha=0.3)

    # annotate noise floor
    noise_floor = np.median(pxx)
    ax.axhline(noise_floor, color="r", ls="--", lw=1,
               label=f"Median floor\n{noise_floor:.2e} {unit}²/Hz")
    ax.legend(fontsize=8)

plt.tight_layout()
save("psd.png")
print("  Flat PSD → white noise (good for Kalman). Peaks indicate coloured noise / vibration.")


# ─── SECTION 5: AUTOCORRELATION ──────────────────────────────────────────────

print("\n" + "=" * 60)
print("SECTION 5 — AUTOCORRELATION")
print("=" * 60)

def normalised_autocorr(x):
    x = x - np.mean(x)
    result = np.correlate(x, x, mode="full")
    result /= result[len(result) // 2]   # normalise to 1 at lag 0
    lags = np.arange(-(len(x) - 1), len(x))
    return lags, result

fig, axes = plt.subplots(1, 2, figsize=(12, 4))
fig.suptitle("Autocorrelation (normalised) — check for white noise")

for ax, sig, label in [
        (axes[0], accel_z,          "Accel Z"),
        (axes[1], gyro_y_corrected, "Gyro Y (bias-removed)")]:

    lags, acf = normalised_autocorr(sig)
    # plot only ±100 lags for clarity
    mask = np.abs(lags) <= 100
    ax.plot(lags[mask] * DT * 1000, acf[mask], color="steelblue", lw=0.8)
    confidence = 1.96 / np.sqrt(N)
    ax.axhline( confidence, color="r", ls="--", lw=1, label="95% CI")
    ax.axhline(-confidence, color="r", ls="--", lw=1)
    ax.axhline(0, color="k", lw=0.5)
    ax.set_xlabel("Lag (ms)")
    ax.set_ylabel("Autocorrelation")
    ax.set_title(label)
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

plt.tight_layout()
save("autocorrelation.png")
print("  White noise: only lag-0 spike above CI bounds.")
print("  Coloured noise: significant structure beyond lag-0 → filter bandwidth too wide,")
print("  or vibration present. Consider reducing mpu.setFilterBandwidth().")


# ─── SECTION 6: ALLAN VARIANCE (GYRO) ────────────────────────────────────────

print("\n" + "=" * 60)
print("SECTION 6 — ALLAN VARIANCE (GYRO)")
print("=" * 60)

def allan_variance(data, fs):
    """Overlapping Allan variance across decades of averaging time."""
    n = len(data)
    max_m = n // 2
    # logarithmically spaced cluster sizes
    m_vals = np.unique(np.round(
        np.logspace(0, np.log10(max_m), 200)).astype(int))
    taus, adevs = [], []
    for m in m_vals:
        # phase array (integral of rate)
        phase = np.cumsum(data) / fs
        # overlapping Allan deviation
        n_phase = len(phase)
        if n_phase < 2 * m + 1:
            continue
        diffs = phase[2*m:] - 2*phase[m:n_phase-m] + phase[:n_phase-2*m]
        avar  = np.mean(diffs**2) / (2 * (m / fs)**2)
        taus.append(m / fs)
        adevs.append(np.sqrt(avar))
    return np.array(taus), np.array(adevs)

taus, adevs = allan_variance(gyro_y_corrected, FS)

fig, ax = plt.subplots(figsize=(8, 5))
ax.loglog(taus, adevs, "steelblue", lw=1.5, label="Allan deviation")

# ── Angle Random Walk: fit slope -½ region (short tau) ──
# Use first decade of tau values
short_mask = taus < taus[len(taus) // 5]
if short_mask.sum() >= 2:
    coeffs = np.polyfit(np.log10(taus[short_mask]),
                        np.log10(adevs[short_mask]), 1)
    slope_arw = coeffs[0]
    # ARW coefficient = Allan deviation at tau = 1 s on the -½ line
    arw = adevs[short_mask][np.argmin(np.abs(taus[short_mask] - 1.0))] \
          if np.any(taus[short_mask] >= 1.0) \
          else 10**np.polyval(coeffs, 0)   # extrapolate to tau=1
    ax.loglog(taus[short_mask],
              10**np.polyval(coeffs, np.log10(taus[short_mask])),
              "r--", lw=1, label=f"ARW slope ({slope_arw:.2f})")
    print(f"\n  Angle Random Walk (ARW) ≈ {arw:.5f} rad/√s")
    print(f"    (Allan deviation at τ=1 s, slope ≈ -0.5 expected for white noise)")
else:
    arw = np.std(gyro_y_corrected, ddof=1) / np.sqrt(FS)
    print(f"\n  ARW estimated from std: {arw:.5f} rad/√s")

# ── Bias Instability: minimum of the curve ──
min_idx   = np.argmin(adevs)
tau_bi    = taus[min_idx]
bias_inst = adevs[min_idx]
ax.axvline(tau_bi, color="orange", ls=":", lw=1,
           label=f"Bias instability τ={tau_bi:.2f}s\n  σ={bias_inst:.5f} rad/s")

ax.set_xlabel("Averaging time τ (s)")
ax.set_ylabel("Allan deviation (rad/s)")
ax.set_title("Allan Variance — Gyro Y")
ax.legend(fontsize=8)
ax.grid(True, which="both", alpha=0.3)
plt.tight_layout()
save("allan_variance.png")

print(f"  Bias instability    ≈ {bias_inst:.5f} rad/s  at τ={tau_bi:.2f} s")
print(f"  Interpretation: below τ={tau_bi:.2f}s noise is ARW-dominated (white),")
print(f"                  above it bias drift dominates.")


# ─── SECTION 7: R AND Q MATRICES ─────────────────────────────────────────────

print("\n" + "=" * 60)
print("SECTION 7 — KALMAN FILTER NOISE MATRICES")
print("=" * 60)

sigma_accel = np.std(accel_z, ddof=1)
sigma_gyro  = np.std(gyro_y_corrected, ddof=1)

var_accel = sigma_accel ** 2
var_gyro  = sigma_gyro  ** 2

# Convert accel variance to theta variance: theta = a_z / g
var_theta_from_accel = var_accel / (G ** 2)

print("\n  ── Measurement Noise Covariance R ──")
print(f"  Measurements: z = [theta (from accel), theta_dot (from gyro)]")
print()
print(f"  σ_accel = {sigma_accel:.6f} m/s²   →   σ_theta = σ_accel/g = {np.sqrt(var_theta_from_accel):.6f} rad")
print(f"  σ_gyro  = {sigma_gyro:.6f} rad/s")
print()
print(f"  R = diag([{var_theta_from_accel:.2e}, {var_gyro:.2e}])")
print()
print(f"  # Python / NumPy:")
print(f"  R = np.diag([{var_theta_from_accel:.6e}, {var_gyro:.6e}])")

print("\n  ── Process Noise Covariance Q ──")
print(f"  State: x = [theta, theta_dot, x_pos, x_dot]")
print()
print(f"  Reasoning:")
print(f"    - theta is integrated from theta_dot, so its process noise")
print(f"      accumulates as ~dt² × q_theta_dot per step.")
print(f"    - theta_dot is driven by unmodelled angular accelerations")
print(f"      (motor torque noise, impacts). Starting estimate: gyro ARW² × FS.")
print(f"    - x_pos / x_dot are mechanical states — encoder/motor noise")
print(f"      is not measured here, so use conservative values and tune.")
print()

# q_theta_dot: base on gyro white-noise floor (ARW² * fs gives variance/sample)
q_theta_dot = (arw ** 2) * FS          # rad²/s per sample
q_theta     = q_theta_dot * (DT ** 2)  # integrated into angle
# x states: no direct measurement — suggest 10× looser as a starting point
q_x_dot     = q_theta_dot * 10         # rough starting value; tune empirically
q_x_pos     = q_x_dot * (DT ** 2)

print(f"  q_theta     ≈ {q_theta:.2e}  rad²")
print(f"  q_theta_dot ≈ {q_theta_dot:.2e}  (rad/s)²")
print(f"  q_x_pos     ≈ {q_x_pos:.2e}  m²        (tune — not observable from IMU alone)")
print(f"  q_x_dot     ≈ {q_x_dot:.2e}  (m/s)²    (tune — not observable from IMU alone)")
print()
print(f"  Q = np.diag([{q_theta:.6e}, {q_theta_dot:.6e}, {q_x_pos:.6e}, {q_x_dot:.6e}])")
print()
print(f"  NOTE: Q is the most model-dependent matrix. Start with these values,")
print(f"  run the closed-loop system, and increase Q entries for states that")
print(f"  the filter tracks too slowly, or decrease them if it is too noisy.")


# ─── DONE ─────────────────────────────────────────────────────────────────────

print("\n" + "=" * 60)
print(f"COMPLETE — all plots saved to ./{PLOTS_DIR}/")
print("=" * 60)
