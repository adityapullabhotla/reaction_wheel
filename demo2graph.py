import cv2
import time
import threading
import numpy as np
import RPi.GPIO as GPIO
import matplotlib.pyplot as plt
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
BALL_LOWER = (25, 120, 150)
BALL_UPPER = (55, 255, 255)

MIN_AREA = 3000    
CIRCULARITY_THRESHOLD = 0.45   

# ── Smoothing ─────────────────────────────────────────────────────────────────
SMOOTH_FRAMES  = 5

# ── PID ───────────────────────────────────────────────────────────────────────
Kp = 0.65   
Ki = 0.0   
Kd = 0.4   

# ── Motor output (%) ──────────────────────────────────────────────────────────
PWM_DEAD   =  0.0   
PWM_MIN    =  15.0   
PWM_MAX    =  9.5   
DEAD_ZONE_PX = 100   

# ── Search state ──────────────────────────────────────────────────────────────
SEARCH_PWM = 5   
SEARCH_REVERSE_SEC = 2.5   

# ── Momentum (edge tracking) ──────────────────────────────────────────────────
MOMENTUM_SEC   = 0.2   
MOMENTUM_SCALE = 0.5  


# ══════════════════════════════════════════════════════════════════════════════
#  STATE MACHINE
# ══════════════════════════════════════════════════════════════════════════════
class State:
    TRACKING  = "TRACKING"   
    MOMENTUM  = "MOMENTUM"   
    SEARCHING = "SEARCHING"  


# ══════════════════════════════════════════════════════════════════════════════
#  MOTOR HELPERS
# ══════════════════════════════════════════════════════════════════════════════

def set_motor(pwm_cmd: float):
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
    if abs(error) < DEAD_ZONE_PX:
        return 0.0
    sign  = 1.0 if raw_pid >= 0 else -1.0
    power = max(PWM_MIN, min(abs(raw_pid), PWM_MAX))
    return sign * power


# ══════════════════════════════════════════════════════════════════════════════
#  BALL DETECTION
# ══════════════════════════════════════════════════════════════════════════════

def find_ball(frame):
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, BALL_LOWER, BALL_UPPER)

    k    = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    mask = cv2.erode(mask,  k, iterations=1)
    mask = cv2.dilate(mask, k, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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
#  REPORT GRAPHING UTILITY
# ══════════════════════════════════════════════════════════════════════════════

def generate_report_graphs(t_data, e_data, p_data):
    if not t_data:
        print("No data collected to graph.")
        return

    print("Rendering report graphs. This may take a moment...")
    plt.figure(figsize=(10, 8))

    # Subplot 1: Vision Tracking Error over Time
    plt.subplot(2, 1, 1)
    plt.plot(t_data, e_data, label='Pixel Error', color='blue', linewidth=2)
    plt.axhline(0, color='red', linestyle='--', label='Center (0 Error)')
    plt.axhline(DEAD_ZONE_PX, color='orange', linestyle=':', label='Deadzone Boundary')
    plt.axhline(-DEAD_ZONE_PX, color='orange', linestyle=':')
    plt.title('Vision Tracking Performance (Position Control)')
    plt.ylabel('Displacement Error (pixels)')
    plt.legend()
    plt.grid(True)

    # Subplot 2: Motor PWM Control Effort over Time
    plt.subplot(2, 1, 2)
    plt.plot(t_data, p_data, label='Motor Output', color='green', linewidth=2)
    plt.axhline(PWM_MAX, color='red', linestyle=':', label='Max CW Power')
    plt.axhline(-PWM_MAX, color='red', linestyle=':', label='Max CCW Power')
    plt.xlabel('Time (Seconds)')
    plt.ylabel('Motor Control Effort (% PWM)')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    filename = "vision_tracking_report.png"
    plt.savefig(filename)
    print(f"Graph successfully saved as: {filename}")


# ══════════════════════════════════════════════════════════════════════════════
#  MAIN TRACKING LOOP
# ══════════════════════════════════════════════════════════════════════════════

def track():
    print("Initialising camera…")
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "BGR888"})
    picam2.configure(config)

    picam2.set_controls({
        "FrameRate": 60,            
        "AeEnable": False,          
        "ExposureTime": 15000,      
        "AnalogueGain": 7.0         
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

    CENTER_X       = 320          
    state          = State.SEARCHING

    integral       = 0.0
    prev_error     = 0.0
    start_time     = time.time()
    prev_time      = start_time

    # ── Graph Data Arrays ────────────────────────────────────────────────────
    time_log = []
    error_log = []
    pwm_log = []

    x_history: list[int] = []
    last_pwm_out   = 0.0          
    lost_at        = 0.0          
    search_dir     = 1
    sweep_start    = time.time()

    try:
        while True:
            frame = picam2.capture_array()
            frame = frame[:, :, ::-1]     
            frame = cv2.flip(frame, -1)   

            now = time.time()
            dt  = max(now - prev_time, 0.001)
            prev_time = now
            elapsed_time = now - start_time

            result = find_ball(frame)

            # ── TRACKING ─────────────────────────────────────────────────
            if result is not None:
                ball_cx, ball_cy, radius = result
                update_tracking_marker(ball_cx, ball_cy, radius)

                x_history.append(ball_cx)
                if len(x_history) > SMOOTH_FRAMES:
                    x_history.pop(0)

                if len(x_history) < SMOOTH_FRAMES:
                    set_motor(0)
                    state = State.TRACKING
                    print(f"  WARMUP   | buffering {len(x_history)}/{SMOOTH_FRAMES}")
                    continue

                smoothed_cx = int(sum(x_history) / len(x_history))
                error       = CENTER_X - smoothed_cx   

                if state != State.TRACKING:
                    prev_error = error
                    integral   = 0.0

                state = State.TRACKING

                integral  += error * dt
                derivative = (error - prev_error) / dt
                raw_out    = Kp * error + Ki * integral + Kd * derivative
                pwm_out    = pid_to_motor(error, raw_out)

                set_motor(pwm_out)
                last_pwm_out = pwm_out
                prev_error   = error

                # Log tracking data
                time_log.append(elapsed_time)
                error_log.append(error)
                pwm_log.append(pwm_out)

                label = "CENTRED " if abs(error) < DEAD_ZONE_PX else "TRACKING"
                print(f"  {label} | x:{smoothed_cx:3d} | "
                      f"err:{error:+5.1f}px | motor:{pwm_out:+5.1f}%")

            # ── BALL LOST ─────────────────────────────────────────────────
            else:
                update_tracking_marker(None, None, None)
                x_history.clear()

                if state == State.TRACKING:
                    state   = State.MOMENTUM
                    lost_at = now

                if state == State.MOMENTUM:
                    elapsed = now - lost_at
                    if elapsed < MOMENTUM_SEC and abs(last_pwm_out) > 0.5:
                        coast = last_pwm_out * MOMENTUM_SCALE
                        set_motor(coast)
                        
                        # Log momentum data
                        time_log.append(elapsed_time)
                        error_log.append(np.nan) # NaN used so the error line breaks on the graph
                        pwm_log.append(coast)
                        
                        print(f"  MOMENTUM | coasting {coast:+5.1f}% "
                              f"({elapsed:.2f}/{MOMENTUM_SEC}s)")
                        continue
                    else:
                        state       = State.SEARCHING
                        search_dir  = 1 if (last_pwm_out >= 0) else -1  
                        sweep_start = now
                        prev_error  = 0.0
                        integral    = 0.0

                if state == State.SEARCHING:
                    if now - sweep_start >= SEARCH_REVERSE_SEC:
                        search_dir  *= -1
                        sweep_start  = now

                    sweep_pwm = search_dir * SEARCH_PWM
                    set_motor(sweep_pwm)
                    
                    # Log search data
                    time_log.append(elapsed_time)
                    error_log.append(np.nan)
                    pwm_log.append(sweep_pwm)
                    
                    rev_in = max(0.0, SEARCH_REVERSE_SEC - (now - sweep_start))
                    print(f"  SEARCH   | {'>>>' if search_dir > 0 else '<<<'} "
                          f"@ {SEARCH_PWM}% | rev in {rev_in:.1f}s")

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        # 1. Safely stop the hardware
        set_motor(0)
        pwm_l.stop()
        pwm_r.stop()
        for pin in [RPWM, LPWM, R_EN, L_EN]:
            GPIO.output(pin, GPIO.LOW)
        GPIO.cleanup()
        picam2.stop()
        print("Hardware cleaned up.")
        
        # 2. Render the graphs
        generate_report_graphs(time_log, error_log, pwm_log)


if __name__ == "__main__":
    track()