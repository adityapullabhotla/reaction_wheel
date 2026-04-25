import cv2
import time
import threading
import numpy as np
import RPi.GPIO as GPIO
from picamera2 import Picamera2
from camera_live_feed import app, set_camera

# --- HARDWARE PINS ---
R_EN = 26
L_EN = 16
RPWM = 5
LPWM = 6

# --- GPIO SETUP ---
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup([R_EN, L_EN, RPWM, LPWM], GPIO.OUT)
GPIO.output(R_EN, GPIO.HIGH)
GPIO.output(L_EN, GPIO.HIGH)

pwm_r = GPIO.PWM(RPWM, 1000)
pwm_l = GPIO.PWM(LPWM, 1000)
pwm_r.start(0)
pwm_l.start(0)

# --- PID CONSTANTS ---
# Low Kp keeps motor commands small and smooth.
# High Kd relative to Kp is the key anti-jitter trick — it heavily suppresses
# fast pixel noise (high rate of change) while still responding to real
# slow ball movement (low rate of change).
Kp = 0.1
Ki = 0.00
Kd = 0.05

# --- MOTOR POWER LIMITS ---
PWM_MIN = 12.0   # stiction floor
PWM_MAX = 40.0   # hard cap — keeps everything slow and controlled

# --- DEAD ZONE ---
# 15px = ~2.3% of frame width. Tight enough that the ball looks centered,
# but not so tight that pixel noise triggers constant tiny corrections.
DEAD_ZONE_PX = 15

# --- BALL POSITION SMOOTHING ---
# Average the detected ball X position over this many frames before feeding
# it to the PID. Kills frame-to-frame pixel jitter at the source.
SMOOTH_FRAMES = 3

# --- TENNIS BALL HSV THRESHOLDS ---
BALL_LOWER = (25, 100, 100)
BALL_UPPER = (45, 255, 255)

MIN_AREA = 400
CIRCULARITY_THRESHOLD = 0.4

# --- MISS / SEARCH SETTINGS ---
# Ball must disappear for this many consecutive frames before search activates.
# During the grace period the motor coasts at its last value.
MISS_FRAMES_BEFORE_SEARCH = 8

# How long to hold the last-known direction before giving up and sweeping.
SEARCH_HOLD_TIME = 1.5      # seconds
SEARCH_HOLD_PWM  = 12.0     # low power directional chase

# Constant slow sweep power once hold phase expires.
SEARCH_SWEEP_PWM = 10.0     # slow enough to actually catch the ball when it appears


def clamp_motor(value):
    """Bound command to [PWM_MIN, PWM_MAX], preserving sign. Returns 0 if tiny."""
    if abs(value) < 0.5:
        return 0.0
    sign      = 1 if value > 0 else -1
    magnitude = max(PWM_MIN, min(abs(value), PWM_MAX))
    return sign * magnitude


def set_motor(pwm_command):
    """Send a signed duty-cycle to the correct H-bridge pin."""
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


def is_circular(contour):
    """True if contour circularity >= threshold. 0.4 allows partial half-circles."""
    area      = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    if perimeter == 0:
        return False
    return (4 * np.pi * area) / (perimeter ** 2) >= CIRCULARITY_THRESHOLD


def find_ball(frame):
    """
    HSV color mask + circularity filter.
    Returns (cx, cy, radius) of the largest passing blob, or None.
    """
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, BALL_LOWER, BALL_UPPER)
    mask = cv2.erode(mask,  None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best      = None
    best_area = 0
    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_AREA:
            continue
        if not is_circular(c):
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


def track():
    print("Initializing Pi 5 Camera...")
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "BGR888"})
    picam2.configure(config)
    picam2.start()
    time.sleep(2.0)

    # Hand the camera instance to camera_live_feed so it can serve the stream
    set_camera(picam2)

    # Start the Flask stream in a background thread
    stream_thread = threading.Thread(
        target=lambda: app.run(host='0.0.0.0', port=5000, threaded=True),
        daemon=True
    )
    stream_thread.start()
    print("\n" + "="*50)
    print("LIVE FEED RUNNING!")
    print("Open a browser and go to: http://<YOUR_PI_IP>:5000")
    print("="*50 + "\n")

    center_x = 320

    # PID state
    integral   = 0.0
    prev_error = 0.0
    prev_time  = time.time()
    last_pwm   = 0.0

    # Ball position smoother — rolling average over last SMOOTH_FRAMES detections
    ball_x_history = []

    # Search / miss state
    miss_count       = 0
    last_known_dir   = 0    # +1 or -1, direction ball was last seen heading
    lost_since       = None
    search_sweep_dir = 1    # alternates during sweep phase

    print("Tracking active. Press Ctrl+C to stop.")
    print(f"Kp={Kp}  Kd={Kd}  |  PWM {PWM_MIN}–{PWM_MAX}%  |  Dead zone ±{DEAD_ZONE_PX}px  |  Smooth over {SMOOTH_FRAMES} frames")

    try:
        while True:
            frame = picam2.capture_array()
            frame = frame[:, :, ::-1]   # fix R↔B channel order
            frame = cv2.flip(frame, -1)

            current_time = time.time()
            dt = current_time - prev_time
            if dt <= 0:
                dt = 0.001
            prev_time = current_time

            result = find_ball(frame)

            if result is not None:
                # ── TRACKING MODE ─────────────────────────────────────────
                ball_cx, ball_cy, radius = result

                # Reset miss/search state
                miss_count       = 0
                lost_since       = None
                search_sweep_dir = 1

                # Smooth the ball X position over the last N frames
                ball_x_history.append(ball_cx)
                if len(ball_x_history) > SMOOTH_FRAMES:
                    ball_x_history.pop(0)
                smoothed_cx = int(sum(ball_x_history) / len(ball_x_history))

                error = center_x - smoothed_cx   # +ve = ball left → spin right

                # Record direction for search memory
                if abs(error) > DEAD_ZONE_PX:
                    last_known_dir = 1 if error > 0 else -1

                if abs(error) < DEAD_ZONE_PX:
                    # Ball is centred — stop motor, bleed integrator
                    pwm_out  = 0.0
                    integral = 0.0
                else:
                    integral  += error * dt
                    derivative = (error - prev_error) / dt
                    raw_out    = (Kp * error) + (Ki * integral) + (Kd * derivative)
                    pwm_out    = clamp_motor(raw_out)

                set_motor(pwm_out)
                last_pwm   = pwm_out
                prev_error = error

                print(f"TRACKING  | Raw X: {ball_cx:3d} | Smooth X: {smoothed_cx:3d} | "
                      f"Error: {error:+6.1f}px | Motor: {pwm_out:+6.1f}%")

            else:
                # ── BALL NOT VISIBLE ──────────────────────────────────────
                miss_count += 1
                integral    = 0.0
                prev_error  = 0.0
                ball_x_history.clear()   # stale history — clear so next lock-on starts fresh

                if miss_count < MISS_FRAMES_BEFORE_SEARCH:
                    # COASTING — brief flicker, hold last motor value and wait
                    # Don't call set_motor — platform keeps moving as-is
                    print(f"COASTING  | Miss {miss_count}/{MISS_FRAMES_BEFORE_SEARCH} "
                          f"| Holding {last_pwm:+.1f}%")

                else:
                    # Genuinely lost — start searching
                    if lost_since is None:
                        lost_since = current_time
                    time_lost = current_time - lost_since

                    if last_known_dir != 0 and time_lost < SEARCH_HOLD_TIME:
                        # PHASE 1 — Chase in the last known direction at low power
                        set_motor(last_known_dir * SEARCH_HOLD_PWM)
                        last_pwm = last_known_dir * SEARCH_HOLD_PWM
                        print(f"SEARCH-CHASE | {'CW ' if last_known_dir > 0 else 'CCW'} "
                              f"@ {SEARCH_HOLD_PWM}%  |  {time_lost:.1f}s / {SEARCH_HOLD_TIME}s")

                    else:
                        # PHASE 2 — Slow constant sweep back and forth
                        set_motor(search_sweep_dir * SEARCH_SWEEP_PWM)
                        last_pwm = search_sweep_dir * SEARCH_SWEEP_PWM
                        print(f"SEARCH-SWEEP | {'CW ' if search_sweep_dir > 0 else 'CCW'} "
                              f"@ {SEARCH_SWEEP_PWM}%")

                        # Flip direction every 2 seconds so it scans both ways
                        if time_lost > 0 and int(time_lost) % 2 == 0 and \
                           abs(time_lost - round(time_lost)) < 0.05:
                            search_sweep_dir *= -1

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


# ── HSV TUNER (optional) ──────────────────────────────────────────────────────
# def hsv_tuner():
#     picam2 = Picamera2()
#     config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "BGR888"})
#     picam2.configure(config)
#     picam2.start()
#     time.sleep(2.0)
#     print("HSV Tuner active — Ctrl+C to stop.")
#     try:
#         while True:
#             frame = picam2.capture_array()
#             frame = frame[:, :, ::-1]
#             frame = cv2.flip(frame, -1)
#             hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
#             h, w  = hsv.shape[:2]
#             patch = hsv[h//2-10:h//2+10, w//2-10:w//2+10]
#             mean  = patch.mean(axis=(0, 1))
#             print(f"Centre HSV -> H: {mean[0]:.1f}  S: {mean[1]:.1f}  V: {mean[2]:.1f}")
#             time.sleep(0.25)
#     except KeyboardInterrupt:
#         picam2.stop()
#
# if __name__ == '__main__':
#     hsv_tuner()
# ─────────────────────────────────────────────────────────────────────────────


if __name__ == '__main__':
    track()