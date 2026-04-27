import cv2
import time
import threading
import numpy as np
import RPi.GPIO as GPIO
from picamera2 import Picamera2
from camera_live_feed import app, set_camera, update_tracking_marker

# ── HARDWARE PINS ─────────────────────────────────────────────────────────────
R_EN = 26
L_EN = 16
RPWM = 5
LPWM = 6

# ── GPIO SETUP ────────────────────────────────────────────────────────────────
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup([R_EN, L_EN, RPWM, LPWM], GPIO.OUT)
GPIO.output(R_EN, GPIO.HIGH)
GPIO.output(L_EN, GPIO.HIGH)

pwm_r = GPIO.PWM(RPWM, 1000)
pwm_l = GPIO.PWM(LPWM, 1000)
pwm_r.start(0)
pwm_l.start(0)

# ── PID GAINS ─────────────────────────────────────────────────────────────────
# Kd intentionally zeroed out for now. The D term was the source of the
# startup kick (derivative of a step from 0 → first_error is enormous).
# Get stable P-only tracking first, then reintroduce Kd cautiously.
Kp = 2
Ki = 0.0
Kd = 0.5  # <<< re-enable only after P-only tracking is stable

# ── MOTOR LIMITS ──────────────────────────────────────────────────────────────
PWM_MIN = 8.0
PWM_MAX = 10.0

# ── DEAD ZONE & RAMP ──────────────────────────────────────────────────────────
# Below DEAD_ZONE_PX  → motor off
# DEAD_ZONE to RAMP   → output ramps 0 → PWM_MIN (no hard jump to 8%)
# Above RAMP_ZONE_PX  → clamped PID output [PWM_MIN, PWM_MAX]
DEAD_ZONE_PX = 20
RAMP_ZONE_PX = 60

# ── CENTROID SMOOTHING ────────────────────────────────────────────────────────
# Buffer must be FULL before PID runs. This prevents the derivative kick
# from a half-filled buffer producing a spurious large error on frame 1-2.
SMOOTH_FRAMES = 6

# How many consecutive missed frames before we clear the history buffer.
# A value of 5 means brief occlusions (ball blinks out for 1-2 frames) don't
# wipe the smoothing buffer and restart from zero.
MISS_TOLERANCE = 5

# ── HSV THRESHOLDS ────────────────────────────────────────────────────────────
BALL_LOWER = (30, 80, 60)
BALL_UPPER = (50, 255, 255)

# ── CONTOUR FILTERS ───────────────────────────────────────────────────────────
MIN_AREA              = 300
CIRCULARITY_THRESHOLD = 0.50

# ── SEARCH SETTINGS ───────────────────────────────────────────────────────────
SEARCH_PWM         = 8.5
SEARCH_REVERSE_SEC = 2.0

# ── DEBUG ─────────────────────────────────────────────────────────────────────
DEBUG_VISION = False


# ── MOTOR CONTROL ─────────────────────────────────────────────────────────────

def set_motor(pwm_command):
    if abs(pwm_command) < 0.5:
        pwm_l.ChangeDutyCycle(0)
        pwm_r.ChangeDutyCycle(0)
        return
    power = abs(pwm_command)
    if pwm_command > 0:
        pwm_l.ChangeDutyCycle(0)
        pwm_r.ChangeDutyCycle(power)
    else:
        pwm_r.ChangeDutyCycle(0)
        pwm_l.ChangeDutyCycle(power)


def soft_clamp(error, raw_pid_output):
    """
    Three-region output mapping that eliminates the hard 0→PWM_MIN jump:
      |error| < DEAD_ZONE_PX               → 0
      DEAD_ZONE_PX ≤ |error| < RAMP_ZONE_PX → linear ramp 0 → PWM_MIN
      |error| ≥ RAMP_ZONE_PX               → clamped [PWM_MIN, PWM_MAX]
    """
    abs_err = abs(error)
    if abs_err < DEAD_ZONE_PX:
        return 0.0
    sign = 1 if raw_pid_output >= 0 else -1
    if abs_err < RAMP_ZONE_PX:
        ramp = (abs_err - DEAD_ZONE_PX) / (RAMP_ZONE_PX - DEAD_ZONE_PX)
        return sign * ramp * PWM_MIN
    return sign * max(PWM_MIN, min(abs(raw_pid_output), PWM_MAX))


# ── BALL DETECTION ────────────────────────────────────────────────────────────

def find_ball(frame, debug=False):
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, BALL_LOWER, BALL_UPPER)

    if debug:
        cv2.imwrite("/tmp/mask_raw.png", mask)
        h, w   = frame.shape[:2]
        cx, cy = w // 2, h // 2
        roi    = hsv[max(0, cy-40):cy+40, max(0, cx-40):cx+40]
        if roi.size > 0:
            print(f"[DEBUG] H:{roi[:,:,0].min()}-{roi[:,:,0].max()}  "
                  f"S:{roi[:,:,1].min()}-{roi[:,:,1].max()}  "
                  f"V:{roi[:,:,2].min()}-{roi[:,:,2].max()}")

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask   = cv2.erode(mask,  kernel, iterations=1)
    mask   = cv2.dilate(mask, kernel, iterations=2)

    if debug:
        cv2.imwrite("/tmp/mask_morphed.png", mask)

    contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)

    best = None
    best_area = 0
    for c in contours:
        area  = cv2.contourArea(c)
        perim = cv2.arcLength(c, True)
        circ  = (4 * np.pi * area) / (perim ** 2) if perim > 0 else 0
        if debug and area > 150:
            print(f"[DEBUG] area={area:.0f} circ={circ:.3f} "
                  f"pass={area >= MIN_AREA and circ >= CIRCULARITY_THRESHOLD}")
        if area < MIN_AREA or circ < CIRCULARITY_THRESHOLD:
            continue
        if area > best_area:
            best_area = area
            best      = c

    if best is None:
        return None
    M = cv2.moments(best)
    if M["m00"] == 0:
        return None
    cx     = int(M["m10"] / M["m00"])
    cy     = int(M["m01"] / M["m00"])
    _, rad = cv2.minEnclosingCircle(best)
    return cx, cy, int(rad)


# ── MAIN LOOP ─────────────────────────────────────────────────────────────────

def track():
    print("Initializing camera...")
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (640, 480), "format": "BGR888"}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(2.0)

    set_camera(picam2)
    threading.Thread(
        target=lambda: app.run(host='0.0.0.0', port=5000, threaded=True),
        daemon=True
    ).start()

    print("\n" + "=" * 50)
    print("LIVE FEED → http://<YOUR_PI_IP>:5000")
    print("=" * 50)
    print(f"Kp={Kp} Kd={Kd} | PWM {PWM_MIN}-{PWM_MAX}% | "
          f"Dead={DEAD_ZONE_PX}px Ramp={RAMP_ZONE_PX}px | "
          f"Smooth={SMOOTH_FRAMES}f MissTol={MISS_TOLERANCE}\n")

    center_x       = 320
    prev_error     = None   # None = uninitialised; set from first real reading
    prev_time      = time.time()

    ball_x_history = []
    miss_count     = 0

    search_dir        = 1
    search_started_at = time.time()

    try:
        while True:
            frame = picam2.capture_array()
            frame = frame[:, :, ::-1]
            frame = cv2.flip(frame, -1)

            now = time.time()
            dt  = max(now - prev_time, 0.001)
            prev_time = now

            result = find_ball(frame, debug=DEBUG_VISION)

            # ── TRACKING ──────────────────────────────────────────────────
            if result is not None:
                miss_count        = 0
                search_dir        = 1
                search_started_at = now

                ball_cx, ball_cy, radius = result
                update_tracking_marker(ball_cx, ball_cy, radius)

                ball_x_history.append(ball_cx)
                if len(ball_x_history) > SMOOTH_FRAMES:
                    ball_x_history.pop(0)

                # ── WARMUP: wait until buffer is full before running PID ──
                # This is the primary fix for the derivative startup kick.
                # When prev_error is None the buffer is still filling — we
                # hold the motor off and just accumulate position samples.
                if len(ball_x_history) < SMOOTH_FRAMES:
                    set_motor(0)
                    print(f"WARMUP    | Filling buffer "
                          f"{len(ball_x_history)}/{SMOOTH_FRAMES}...")
                    continue

                smoothed_cx = int(sum(ball_x_history) / len(ball_x_history))
                error       = center_x - smoothed_cx

                # Seed prev_error from the first real smoothed reading so
                # the very first derivative calculation is (≈0 - ≈0) not
                # (first_error - 0) which was causing the startup kick.
                if prev_error is None:
                    prev_error = error

                if abs(error) < DEAD_ZONE_PX:
                    set_motor(0)
                    prev_error = error
                    print(f"CENTRED   | X:{smoothed_cx:3d} | err:{error:+.0f}px")

                else:
                    raw_out = Kp * error  # Kd=0 for now, pure P control
                    pwm_out = soft_clamp(error, raw_out)
                    set_motor(pwm_out)
                    prev_error = error
                    print(f"TRACKING  | X:{smoothed_cx:3d} | "
                          f"err:{error:+5.1f}px | motor:{pwm_out:+5.1f}%")

            # ── SEARCHING ─────────────────────────────────────────────────
            else:
                update_tracking_marker(None, None, None)
                miss_count += 1

                # Only wipe the smoothing buffer after several consecutive
                # misses. Brief 1-2 frame blinks don't reset the history,
                # so reacquisition continues smoothly from the last position.
                if miss_count >= MISS_TOLERANCE:
                    ball_x_history.clear()
                    prev_error = None   # force re-seed on next acquisition

                # Reverse sweep direction every SEARCH_REVERSE_SEC seconds
                if now - search_started_at >= SEARCH_REVERSE_SEC:
                    search_dir        *= -1
                    search_started_at  = now

                set_motor(search_dir * SEARCH_PWM)
                print(f"SEARCHING | {'>>>' if search_dir > 0 else '<<<'} "
                      f"@ {SEARCH_PWM}% | miss={miss_count} | "
                      f"rev in {max(0, SEARCH_REVERSE_SEC-(now-search_started_at)):.1f}s")

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        set_motor(0)
        pwm_l.stop()
        pwm_r.stop()
        GPIO.output(RPWM, GPIO.LOW)
        GPIO.output(LPWM, GPIO.LOW)
        GPIO.output(R_EN, GPIO.LOW)
        GPIO.output(L_EN, GPIO.LOW)
        GPIO.cleanup()
        picam2.stop()
        print("Hardware cleaned up.")


if __name__ == '__main__':
    track()