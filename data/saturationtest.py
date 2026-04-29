import time
import sys
import board
import busio
import RPi.GPIO as GPIO
import matplotlib
matplotlib.use("Agg")  # headless — no display needed on Pi
import matplotlib.pyplot as plt
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_GYROSCOPE

# ══════════════════════════════════════════════════════════════════════════════
#  HARDWARE PINS
# ══════════════════════════════════════════════════════════════════════════════
R_EN = 26
L_EN = 16
RPWM = 5
LPWM = 6

# ══════════════════════════════════════════════════════════════════════════════
#  RAMP PARAMETERS  ← edit these
# ══════════════════════════════════════════════════════════════════════════════
PWM_START      = 5.0    # % — duty cycle at t=0
PWM_END        = 70.0  # % — duty cycle at end of ramp
RAMP_DURATION  = 30.0   # seconds — time to go from PWM_START to PWM_END
                         # e.g. 30s = slow ramp, 10s = fast ramp

# ══════════════════════════════════════════════════════════════════════════════
#  SATURATION DETECTION  ← tune if triggering too early/late
# ══════════════════════════════════════════════════════════════════════════════
SAT_WINDOW_SEC    = 3.0  # seconds — rolling look-back window
SAT_RPM_THRESHOLD = 2.0  # RPM     — max RPM gain in window before flagging sat
SAT_MIN_RUN_SEC   = 4.0  # seconds — ignore detection before this time

# ══════════════════════════════════════════════════════════════════════════════
#  HELPERS
# ══════════════════════════════════════════════════════════════════════════════

def get_ramp_pwm(elapsed: float) -> float:
    """Return the linearly interpolated duty cycle for the current time."""
    progress = min(elapsed / RAMP_DURATION, 1.0)
    return PWM_START + (PWM_END - PWM_START) * progress

def safe_hardware(pwm_r, pwm_l):
    pwm_r.stop()
    pwm_l.stop()
    for pin in [RPWM, LPWM, R_EN, L_EN]:
        GPIO.output(pin, GPIO.LOW)
    GPIO.cleanup()

def generate_graph(time_data, rpm_data, pwm_data, sat_time, sat_rpm):
    print("Rendering graph...")

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
    fig.patch.set_facecolor("#f7f8fa")

    # ── Subplot 1: RPM ────────────────────────────────────────────────────────
    ax1.set_facecolor("#ffffff")

    if sat_time is not None:
        # Split trace at saturation point
        split = next(i for i, t in enumerate(time_data) if t >= sat_time)
        ax1.plot(time_data[:split + 1], rpm_data[:split + 1],
                 color="#1a73e8", linewidth=2.0, label="Spinning up")
        ax1.plot(time_data[split:], rpm_data[split:],
                 color="#d93025", linewidth=2.0, label="Saturated")
        ax1.axvspan(sat_time, time_data[-1], color="#ff4d4d", alpha=0.08)
        ax1.axvline(sat_time, color="#d93025", linestyle="--", linewidth=1.4)

        # Annotation
        x_offset = (time_data[-1] - time_data[0]) * 0.04
        y_mid    = (max(rpm_data) - min(rpm_data)) * 0.45 + min(rpm_data)
        ax1.annotate(
            f"  Saturation\n  t = {sat_time:.2f} s\n  {sat_rpm:.1f} RPM",
            xy=(sat_time, sat_rpm if sat_rpm else 0),
            xytext=(sat_time + x_offset, y_mid),
            fontsize=9, color="#d93025",
            arrowprops=dict(arrowstyle="->", color="#d93025", lw=1.2),
            bbox=dict(boxstyle="round,pad=0.35", fc="white", ec="#d93025", lw=0.9)
        )
    else:
        ax1.plot(time_data, rpm_data, color="#1a73e8",
                 linewidth=2.0, label="Platform RPM")

    ax1.set_title(
        f"Reaction Wheel Ramp Test  |  {PWM_START:.0f}% → {PWM_END:.0f}% over {RAMP_DURATION:.0f}s",
        fontsize=13, fontweight="bold", pad=10
    )
    ax1.set_ylabel("Platform Velocity (RPM)", fontsize=11)
    ax1.legend(fontsize=9, framealpha=0.9)
    ax1.grid(True, linestyle="--", linewidth=0.5, color="#cccccc")
    ax1.set_xlim(left=0)
    for spine in ax1.spines.values():
        spine.set_edgecolor("#cccccc")

    # ── Subplot 2: PWM ramp ───────────────────────────────────────────────────
    ax2.set_facecolor("#ffffff")
    ax2.plot(time_data, pwm_data, color="#34a853",
             linewidth=2.0, label="Motor PWM (ramp)")

    if sat_time is not None:
        ax2.axvline(sat_time, color="#d93025", linestyle="--",
                    linewidth=1.4, label="Saturation point")

    ax2.set_ylim(0, 105)
    ax2.set_xlabel("Time (s)", fontsize=11)
    ax2.set_ylabel("Motor Output (% PWM)", fontsize=11)
    ax2.legend(fontsize=9, framealpha=0.9)
    ax2.grid(True, linestyle="--", linewidth=0.5, color="#cccccc")
    for spine in ax2.spines.values():
        spine.set_edgecolor("#cccccc")

    plt.tight_layout()

    filename = f"ramp_saturation_{int(PWM_START)}to{int(PWM_END)}pct_{int(RAMP_DURATION)}s.png"
    plt.savefig(filename, dpi=150, bbox_inches="tight")
    print(f"Graph saved -> {filename}")
    return filename

# ══════════════════════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════════════════════

def main():
    print("=" * 56)
    print("  REACTION WHEEL RAMP SATURATION TEST")
    print("=" * 56)
    print(f"  Ramp   : {PWM_START}% -> {PWM_END}% over {RAMP_DURATION}s")
    print(f"  Rate   : {(PWM_END - PWM_START) / RAMP_DURATION:.2f} %/s")
    print(f"  Detect : RPM gain < {SAT_RPM_THRESHOLD} over {SAT_WINDOW_SEC}s  (starts at {SAT_MIN_RUN_SEC}s)")
    print()

    # 1. IMU SETUP
    try:
        i2c    = busio.I2C(board.SCL, board.SDA, frequency=400000)
        sensor = BNO08X_I2C(i2c)
        sensor.enable_feature(BNO_REPORT_GYROSCOPE)
        print("IMU initialised.")
    except Exception as e:
        print(f"Error initialising IMU: {e}")
        sys.exit(1)

    # 2. GPIO / MOTOR SETUP
    GPIO.setmode(GPIO.BCM)
    GPIO.setup([R_EN, L_EN, RPWM, LPWM], GPIO.OUT)
    GPIO.output(RPWM, GPIO.LOW)
    GPIO.output(LPWM, GPIO.LOW)
    GPIO.output(R_EN, GPIO.HIGH)
    GPIO.output(L_EN, GPIO.HIGH)

    pwm_r = GPIO.PWM(RPWM, 1000)
    pwm_l = GPIO.PWM(LPWM, 1000)
    pwm_r.start(0)
    pwm_l.start(0)
    print("Motor initialised.")
    print(f"\nStarting ramp... (Ctrl+C to abort)\n")

    # 3. DATA BUFFERS
    time_data = []
    rpm_data  = []
    pwm_data  = []

    window: list[tuple[float, float]] = []  # (wall_time, rpm)

    sat_time = None
    sat_rpm  = None

    start_time = time.time()
    last_time  = start_time

    try:
        while True:
            now     = time.time()
            dt      = now - last_time
            elapsed = now - start_time

            # Enforce 50 Hz loop rate
            if dt < 0.02:
                time.sleep(0.001)
                continue

            last_time = now

            # 4. COMPUTE AND APPLY RAMP
            current_pwm = get_ramp_pwm(elapsed)
            pwm_l.ChangeDutyCycle(0)
            pwm_r.ChangeDutyCycle(current_pwm)

            # Stop ramping once we hit PWM_END
            if elapsed >= RAMP_DURATION:
                current_pwm = PWM_END

            # 5. READ IMU
            platform_rpm = 0.0
            try:
                gyro_data = sensor.gyro
                if gyro_data and gyro_data[2] is not None:
                    platform_rpm = gyro_data[2] * 9.549297  # rad/s -> RPM
            except Exception:
                pass

            # 6. LOG
            time_data.append(elapsed)
            rpm_data.append(platform_rpm)
            pwm_data.append(current_pwm)

            # 7. SATURATION DETECTION
            window.append((now, platform_rpm))
            window[:] = [(t, r) for t, r in window
                         if now - t <= SAT_WINDOW_SEC]

            if elapsed >= SAT_MIN_RUN_SEC and len(window) >= 10:
                window_age = now - window[0][0]
                if window_age >= SAT_WINDOW_SEC * 0.8:
                    window_rpms = [r for _, r in window]
                    rpm_gain    = max(window_rpms) - min(window_rpms)

                    if rpm_gain < SAT_RPM_THRESHOLD:
                        sat_time = elapsed
                        sat_rpm  = platform_rpm
                        print(f"\n*** SATURATION DETECTED ***")
                        print(f"    Time      : {sat_time:.2f} s")
                        print(f"    RPM       : {sat_rpm:.1f}")
                        print(f"    PWM at sat: {current_pwm:.1f}%")
                        print(f"    RPM gain over last {window_age:.1f}s: {rpm_gain:.2f}")
                        break

            print(f"  t={elapsed:>6.2f}s | RPM={platform_rpm:>7.2f} | PWM={current_pwm:>5.1f}%")

    except KeyboardInterrupt:
        print("\nAborted by user.")

    finally:
        # 8. SAFE HARDWARE
        safe_hardware(pwm_r, pwm_l)
        print("Hardware safed.")

        # 9. GRAPH
        if len(time_data) >= 2:
            fname = generate_graph(time_data, rpm_data, pwm_data, sat_time, sat_rpm)
            if sat_time is not None:
                print(f"\nResult : Saturated at {sat_time:.2f}s  |  {sat_rpm:.1f} RPM  |  {pwm_data[time_data.index(min(time_data, key=lambda t: abs(t - sat_time)))]:.1f}% PWM")
            else:
                print("\nSaturation not detected — try lowering SAT_RPM_THRESHOLD or increasing RAMP_DURATION")
        else:
            print("Not enough data to plot.")

if __name__ == "__main__":
    main()