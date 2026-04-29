import cv2
import time
import threading
import sys
import numpy as np
import RPi.GPIO as GPIO
import board
import busio
import matplotlib.pyplot as plt
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_GYROSCOPE
from picamera2 import Picamera2
from camera_live_feed2 import app, set_camera, update_tracking_marker

# --- SHARED SYSTEM VARIABLES ---
target_rpm = 0.0
actual_rpm = 0.0    # Added for graphing
current_pwm = 0.0   # Added for graphing
system_running = True

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

def clamp(value, min_val, max_val):
    return max(min_val, min(value, max_val))

# ==========================================
# INNER LOOP: HIGH-SPEED IMU VELOCITY CONTROL
# ==========================================
def imu_control_thread():
    global target_rpm, actual_rpm, current_pwm, system_running
    
    print("Starting IMU Inner Loop...")
    try:
        i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
        sensor = BNO08X_I2C(i2c)
        sensor.enable_feature(BNO_REPORT_GYROSCOPE)
    except Exception as e:
        print(f"Error initializing IMU: {e}")
        system_running = False
        return

    # Demo 1 Gains
    Kp_imu = 3.5   
    Ki_imu = 0.175  
    Kd_imu = 0.1   
    
    integral = 0.0
    prev_error = 0.0
    last_time = time.time()
    
    while system_running:
        current_time = time.time()
        dt = current_time - last_time
        if dt <= 0: continue
            
        platform_rpm = 0.0
        try:
            gyro_data = sensor.gyro
            if gyro_data and gyro_data[2] is not None:
                platform_rpm = gyro_data[2] * 9.549297 
                actual_rpm = platform_rpm # Share with main thread for graphing
        except Exception:
            pass 
            
        error = target_rpm - platform_rpm
        
        integral += error * dt
        derivative = (error - prev_error) / dt
        control_signal = (Kp_imu * error) + (Ki_imu * integral) + (Kd_imu * derivative)
        
        duty_cycle = clamp(abs(control_signal), 0.0, 100.0)
        
        if control_signal > 0:
            pwm_l.ChangeDutyCycle(0)
            pwm_r.ChangeDutyCycle(duty_cycle)
            current_pwm = duty_cycle # Share signed PWM for graphing
        elif control_signal < 0:
            pwm_r.ChangeDutyCycle(0)
            pwm_l.ChangeDutyCycle(duty_cycle)
            current_pwm = -duty_cycle
        else:
            pwm_r.ChangeDutyCycle(0)
            pwm_l.ChangeDutyCycle(0)
            current_pwm = 0.0
            
        prev_error = error
        last_time = current_time
        time.sleep(0.02) # 50 Hz loop

# ==========================================
# OUTER LOOP: CAMERA VISION TRACKING
# ==========================================
BALL_LOWER = (30, 150, 100)
BALL_UPPER = (45, 255, 255)
MIN_AREA = 400
CIRCULARITY_THRESHOLD = 0.65

def is_circular(contour):
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    if perimeter == 0: return False
    return (4 * np.pi * area) / (perimeter ** 2) >= CIRCULARITY_THRESHOLD

def find_ball(frame):
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, BALL_LOWER, BALL_UPPER)
    mask = cv2.erode(mask,  None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best = None
    best_area = 0
    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_AREA or not is_circular(c): continue
        if area > best_area:
            best_area = area
            best = c

    if best is None: return None
    M = cv2.moments(best)
    if M["m00"] == 0: return None
    
    return int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]), int(cv2.minEnclosingCircle(best)[1])

# ==========================================
# GRAPH GENERATOR
# ==========================================
def generate_cascaded_graphs(t_log, err_log, t_rpm_log, a_rpm_log, pwm_log):
    if not t_log:
        print("No data collected to graph.")
        return

    print("Rendering Sensor Fusion report graphs. This may take a moment...")
    plt.figure(figsize=(12, 10))

    # Subplot 1: Outer Loop (Vision Pixel Error)
    plt.subplot(3, 1, 1)
    plt.plot(t_log, err_log, label='Pixel Displacement', color='purple', linewidth=2)
    plt.axhline(0, color='black', linestyle='--', alpha=0.6)
    plt.title('Outer Loop: Vision Position Tracking')
    plt.ylabel('Error (Pixels)')
    plt.legend(loc='upper right')
    plt.grid(True)

    # Subplot 2: Inner Loop (Velocity Tracking)
    plt.subplot(3, 1, 2)
    plt.plot(t_log, t_rpm_log, label='Commanded Target RPM (From Camera)', color='red', linestyle='--', linewidth=2)
    plt.plot(t_log, a_rpm_log, label='Actual Platform RPM (From IMU)', color='blue', linewidth=2, alpha=0.7)
    plt.title('Inner Loop: IMU Velocity Tracking')
    plt.ylabel('Angular Velocity (RPM)')
    plt.legend(loc='upper right')
    plt.grid(True)

    # Subplot 3: Motor Control Effort
    plt.subplot(3, 1, 3)
    plt.plot(t_log, pwm_log, label='Motor Output', color='green', linewidth=2)
    plt.axhline(100, color='black', linestyle=':', alpha=0.6, label='Saturation Limits')
    plt.axhline(-100, color='black', linestyle=':', alpha=0.6)
    plt.title('Actuator Control Effort')
    plt.xlabel('Time (Seconds)')
    plt.ylabel('Motor Output (% PWM)')
    plt.legend(loc='upper right')
    plt.grid(True)

    plt.tight_layout()
    filename = "demo2_cascaded_control_report.png"
    plt.savefig(filename)
    print(f"Graph successfully saved as: {filename}")


def track():
    global target_rpm, system_running, actual_rpm, current_pwm
    
    print("Initializing Pi 5 Camera...")
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "BGR888"})
    picam2.configure(config)

    picam2.set_controls({
        "FrameRate": 60,            
        "AeEnable": False,          
        "ExposureTime": 15000,      
        "AnalogueGain": 5.0         
    })

    picam2.start()
    time.sleep(2.0)

    set_camera(picam2)
    threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000, threaded=True), daemon=True).start()
    
    imu_thread = threading.Thread(target=imu_control_thread, daemon=True)
    imu_thread.start()

    # --- FIXED TUNING VARIABLES ---
    center_x = 320
    DEAD_ZONE_PX = 20  
    Kp_vision = 0.05   
    
    ball_x_history = []
    SMOOTH_FRAMES = 3

    miss_count = 0
    last_known_dir = 1
    last_sweep_toggle = time.time()
    
    # --- GRAPH LOGGING ARRAYS ---
    time_log = []
    error_log = []
    target_rpm_log = []
    actual_rpm_log = []
    pwm_log = []
    
    start_time = time.time()

    print("\nCascaded Control Active. Press Ctrl+C to stop and render graphs.")

    try:
        while system_running:
            frame = picam2.capture_array()
            frame = frame[:, :, ::-1]   
            frame = cv2.flip(frame, -1)

            result = find_ball(frame)
            current_time = time.time()
            elapsed = current_time - start_time

            if result is not None:
                # --- TRACKING MODE ---
                ball_cx, ball_cy, radius = result
                update_tracking_marker(ball_cx, ball_cy, radius)
                miss_count = 0

                ball_x_history.append(ball_cx)
                if len(ball_x_history) > SMOOTH_FRAMES: ball_x_history.pop(0)
                smoothed_cx = int(sum(ball_x_history) / len(ball_x_history))

                error = center_x - smoothed_cx   

                if abs(error) > DEAD_ZONE_PX:
                    last_known_dir = 1 if error > 0 else -1

                if abs(error) < DEAD_ZONE_PX:
                    target_rpm = 0.0 
                else:
                    target_rpm = Kp_vision * error 
                    target_rpm = clamp(target_rpm, -25.0, 25.0)

                # Log Active Tracking Data
                time_log.append(elapsed)
                error_log.append(error)
                target_rpm_log.append(target_rpm)
                actual_rpm_log.append(actual_rpm)
                pwm_log.append(current_pwm)

                print(f"TRACKING | Error: {error:+6.1f}px | Cmd RPM: {target_rpm:+.1f} | Actual RPM: {actual_rpm:+.1f}")

            else:
                # --- SEARCH MODE ---
                update_tracking_marker(None, None, None)
                miss_count += 1
                ball_x_history.clear()  

                if miss_count < 8:
                    target_rpm = 0.0 
                else:
                    if time.time() - last_sweep_toggle > 2.0:
                        last_known_dir *= -1
                        last_sweep_toggle = time.time()
                    
                    target_rpm = last_known_dir * 3.0 # Fixed to prevent saturation
                    
                # Log Search Data (Use NaN for error so the graph line breaks nicely when ball is lost)
                time_log.append(elapsed)
                error_log.append(np.nan)
                target_rpm_log.append(target_rpm)
                actual_rpm_log.append(actual_rpm)
                pwm_log.append(current_pwm)

                print(f"SEARCHING | Cmd RPM: {target_rpm:+.1f} | Actual RPM: {actual_rpm:+.1f}")

    except KeyboardInterrupt:
        print("\nStopping testbed...")
    finally:
        system_running = False 
        time.sleep(0.1)
        pwm_l.stop()
        pwm_r.stop()
        GPIO.output(RPWM, GPIO.LOW)
        GPIO.output(LPWM, GPIO.LOW)
        GPIO.output(R_EN, GPIO.LOW)
        GPIO.output(L_EN, GPIO.LOW)
        GPIO.cleanup()
        picam2.stop()
        print("Hardware cleaned up.")
        
        # Trigger the matplotlib function
        generate_cascaded_graphs(time_log, error_log, target_rpm_log, actual_rpm_log, pwm_log)

if __name__ == '__main__':
    track()