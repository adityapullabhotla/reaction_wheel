import cv2
import time
import threading
import numpy as np
import RPi.GPIO as GPIO
from picamera2 import Picamera2
from camera_live_feed import app, set_camera, update_tracking_marker

# ══════════════════════════════════════════════════════════════════════════════
#  HARDWARE PINS
# ══════════════════════════════════════════════════════════════════════════════
R_EN, L_EN = 26, 16
RPWM, LPWM = 5,  6

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup([R_EN, L_EN, RPWM, LPWM], GPIO.OUT)
GPIO.output(R_EN, GPIO.HIGH)
GPIO.output(L_EN, GPIO.HIGH)

pwm_r = GPIO.PWM(RPWM, 1000)
pwm_l = GPIO.PWM(LPWM, 1000)
pwm_r.start(0)
pwm_l.start(0)

# ══════════════════════════════════════════════════════════════════════════════
#  TUNING — everything in one place
# ══════════════════════════════════════════════════════════════════════════════

# ── Vision ────────────────────────────────────────────────────────────────────
# HSV range for a standard yellow-green tennis ball
BALL_LOWER = (25, 120, 150)
BALL_UPPER = (55, 255, 255)

MIN_AREA = 3000    # px²  — ignore specks
CIRCULARITY_THRESHOLD = 0.45   # 0–1, ball ≈ 0.78 when whole

# ── Smoothing ─────────────────────────────────────────────────────────────────
# Rolling average over this many frames. More = smoother but laggier.
SMOOTH_FRAMES  = 5

# ── PID ───────────────────────────────────────────────────────────────────────
Kp = 0.65   # proportional — main driver
Ki = 0.0   # integral — leave at 0 unless steady-state offset persists
Kd = 0.4   # derivative — damps oscillation; raise slowly if needed

# ── Motor output (%) ──────────────────────────────────────────────────────────
# Keep these LOW so the camera stays steady
PWM_DEAD   =  0.0   # below dead-zone: off
PWM_MIN    =  15.0   # minimum power that actually moves the platform
PWM_MAX    =  9.5   # absolute ceiling — never exceed this
DEAD_ZONE_PX = 100   # ±px around centre before motor fires

# ── Search state ──────────────────────────────────────────────────────────────
# Very slow creep so the camera can actually lock on when panning
SEARCH_PWM = 5   # % — barely above stall
SEARCH_REVERSE_SEC = 2.5   # seconds before reversing sweep direction

# ── Momentum (edge tracking) ──────────────────────────────────────────────────
# When the ball leaves the frame the motor keeps running in the last-known
# direction at reduced power for this many seconds before switching to Search.
# This handles the "ball partially out of frame" case you mentioned.
MOMENTUM_SEC   = 0.2   # how long to coast after last detection
MOMENTUM_SCALE = 0.5  # fraction of last PWM output to use while coasting


# ══════════════════════════════════════════════════════════════════════════════
#  STATE MACHINE
# ══════════════════════════════════════════════════════════════════════════════
class State:
    TRACKING  = "TRACKING"   # ball visible — running PID
    MOMENTUM  = "MOMENTUM"   # ball just lost — coasting in last direction
    SEARCHING = "SEARCHING"  # ball gone — slow sweep


# ══════════════════════════════════════════════════════════════════════════════
#  MOTOR HELPERS
# ══════════════════════════════════════════════════════════════════════════════

def set_motor(pwm_cmd: float):
    """
    Positive  → turn right  (RPWM)
    Negative  → turn left   (LPWM)
    Near zero → both off
    """
    power = abs(pwm_cmd)
    if power < 0.5:
        pwm_l.ChangeDutyCycle(0)
        pwm_r.ChangeDutyCycle(0)
        return
    if pwm_cmd > 0:
        pwm_l.ChangeDutyCycle(0)
        pwm_r.ChangeDutyCycle(power)
    else:
        pwm_r.ChangeDutyCycle(0)
        pwm_l.ChangeDutyCycle(power)


def pid_to_motor(error: float, raw_pid: float) -> float:
    """
    Map PID output → safe motor command.

    Dead zone  → 0
    Otherwise  → sign preserved, magnitude clamped to [PWM_MIN, PWM_MAX]
    """
    if abs(error) < DEAD_ZONE_PX:
        return 0.0
    sign  = 1.0 if raw_pid >= 0 else -1.0
    power = max(PWM_MIN, min(abs(raw_pid), PWM_MAX))
    return sign * power


# ══════════════════════════════════════════════════════════════════════════════
#  BALL DETECTION
# ══════════════════════════════════════════════════════════════════════════════

def find_ball(frame):
    """
    Returns (cx, cy, radius) of the best tennis-ball candidate, or None.
    Works even when only part of the ball is in frame (lower circularity OK).
    """
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, BALL_LOWER, BALL_UPPER)

    # Morphology: close gaps, remove noise
    k    = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    mask = cv2.erode(mask,  k, iterations=1)
    mask = cv2.dilate(mask, k, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)

    best, best_area = None, 0
    for c in contours:
        area  = cv2.contourArea(c)
        perim = cv2.arcLength(c, True)
        circ  = (4 * np.pi * area / perim ** 2) if perim > 0 else 0
        if area < MIN_AREA or circ < CIRCULARITY_THRESHOLD:
            continue
        if area > best_area:
            best_area, best = area, c

    if best is None:
        return None

    M = cv2.moments(best)
    if M["m00"] == 0:
        return None

    cx     = int(M["m10"] / M["m00"])
    cy     = int(M["m01"] / M["m00"])
    _, rad = cv2.minEnclosingCircle(best)
    return cx, cy, int(rad)


# ══════════════════════════════════════════════════════════════════════════════
#  MAIN TRACKING LOOP
# ══════════════════════════════════════════════════════════════════════════════

def track():
    print("Initialising camera…")
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "BGR888"})
    picam2.configure(config)

    # --- THE CAMERA PHYSICS FIX ---
    picam2.set_controls({
        "FrameRate": 60,            # Force a blistering fast 60 FPS
        "AeEnable": False,          # Kill the Auto-Exposure algorithm entirely
        "ExposureTime": 15000,      # Shutter speed in microseconds (15ms). Stops motion blur!
        "AnalogueGain": 7.0         # Sensor sensitivity (ISO). Boost this if the image is too dark.
    })

    picam2.start()
    time.sleep(2.0)

    set_camera(picam2)
    threading.Thread(
        target=lambda: app.run(host="0.0.0.0", port=5000, threaded=True),
        daemon=True
    ).start()

    print("\n" + "═" * 54)
    print("  LIVE FEED → http://<YOUR_PI_IP>:5000")
    print("═" * 54)
    print(f"  Kp={Kp} Ki={Ki} Kd={Kd} | PWM {PWM_MIN}–{PWM_MAX}%")
    print(f"  Dead={DEAD_ZONE_PX}px | Smooth={SMOOTH_FRAMES}f")
    print(f"  Momentum={MOMENTUM_SEC}s | Search={SEARCH_PWM}%\n")

    # ── State ────────────────────────────────────────────────────────────────
    CENTER_X       = 320          # horizontal midpoint of 640-wide frame
    state          = State.SEARCHING

    # PID
    integral       = 0.0
    prev_error     = 0.0
    prev_time      = time.time()

    # Smoothing
    x_history: list[int] = []

    # Momentum
    last_pwm_out   = 0.0          # last motor command while tracking
    lost_at        = 0.0          # time ball disappeared

    # Search sweep
    search_dir     = 1
    sweep_start    = time.time()

    try:
        while True:
            frame = picam2.capture_array()
            frame = frame[:, :, ::-1]     # RGB → BGR
            frame = cv2.flip(frame, -1)   # 180° rotation (camera is upside-down)

            now = time.time()
            dt  = max(now - prev_time, 0.001)
            prev_time = now

            result = find_ball(frame)

            # ── TRACKING ─────────────────────────────────────────────────
            if result is not None:
                ball_cx, ball_cy, radius = result
                update_tracking_marker(ball_cx, ball_cy, radius)

                # Accumulate smoothing buffer
                x_history.append(ball_cx)
                if len(x_history) > SMOOTH_FRAMES:
                    x_history.pop(0)

                # Wait until buffer is full to avoid D-term spike on startup
                if len(x_history) < SMOOTH_FRAMES:
                    set_motor(0)
                    state = State.TRACKING
                    print(f"  WARMUP   | buffering {len(x_history)}/{SMOOTH_FRAMES}")
                    continue

                smoothed_cx = int(sum(x_history) / len(x_history))
                error       = CENTER_X - smoothed_cx   # + = ball is left of centre

                # Seed prev_error on first real reading so D-term starts near 0
                if state != State.TRACKING:
                    prev_error = error
                    integral   = 0.0

                state = State.TRACKING

                # PID
                integral  += error * dt
                derivative = (error - prev_error) / dt
                raw_out    = Kp * error + Ki * integral + Kd * derivative
                pwm_out    = pid_to_motor(error, raw_out)

                set_motor(pwm_out)
                last_pwm_out = pwm_out
                prev_error   = error

                label = "CENTRED " if abs(error) < DEAD_ZONE_PX else "TRACKING"
                print(f"  {label} | x:{smoothed_cx:3d} | "
                      f"err:{error:+5.1f}px | motor:{pwm_out:+5.1f}%")

            # ── BALL LOST ─────────────────────────────────────────────────
            else:
                update_tracking_marker(None, None, None)
                x_history.clear()

                # ── MOMENTUM — coast in last known direction ──────────────
                if state == State.TRACKING:
                    state   = State.MOMENTUM
                    lost_at = now

                if state == State.MOMENTUM:
                    elapsed = now - lost_at
                    if elapsed < MOMENTUM_SEC and abs(last_pwm_out) > 0.5:
                        coast = last_pwm_out * MOMENTUM_SCALE
                        set_motor(coast)
                        print(f"  MOMENTUM | coasting {coast:+5.1f}% "
                              f"({elapsed:.2f}/{MOMENTUM_SEC}s)")
                        continue
                    else:
                        # Momentum expired → enter Search
                        state       = State.SEARCHING
                        search_dir  = 1 if (last_pwm_out >= 0) else -1  # start in last direction
                        sweep_start = now
                        prev_error  = 0.0
                        integral    = 0.0

                # ── SEARCH — slow oscillating sweep ──────────────────────
                if state == State.SEARCHING:
                    # Reverse sweep direction every SEARCH_REVERSE_SEC
                    if now - sweep_start >= SEARCH_REVERSE_SEC:
                        search_dir  *= -1
                        sweep_start  = now

                    set_motor(search_dir * SEARCH_PWM)
                    rev_in = max(0.0, SEARCH_REVERSE_SEC - (now - sweep_start))
                    print(f"  SEARCH   | {'>>>' if search_dir > 0 else '<<<'} "
                          f"@ {SEARCH_PWM}% | rev in {rev_in:.1f}s")

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        set_motor(0)
        pwm_l.stop()
        pwm_r.stop()
        for pin in [RPWM, LPWM, R_EN, L_EN]:
            GPIO.output(pin, GPIO.LOW)
        GPIO.cleanup()
        picam2.stop()
        print("Hardware cleaned up.")


if __name__ == "__main__":
    track()