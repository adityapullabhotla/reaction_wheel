import cv2
import time
import threading
import sys
import numpy as np
import RPi.GPIO as GPIO
import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_GYROSCOPE
from picamera2 import Picamera2
from camera_live_feed2 import app, set_camera, update_tracking_marker

# --- SHARED SYSTEM VARIABLES ---
target_rpm = 0.0
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
    global target_rpm, system_running
    
    print("Starting IMU Inner Loop...")
    try:
        i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
        sensor = BNO08X_I2C(i2c)
        sensor.enable_feature(BNO_REPORT_GYROSCOPE)
    except Exception as e:
        print(f"Error initializing IMU: {e}")
        system_running = False
        return

    # Your working Demo 1 Gains
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
        except Exception:
            pass 
            
        # PID MATH (Matching current RPM to the Camera's requested RPM)
        error = target_rpm - platform_rpm
        
        integral += error * dt
        derivative = (error - prev_error) / dt
        control_signal = (Kp_imu * error) + (Ki_imu * integral) + (Kd_imu * derivative)
        
        duty_cycle = clamp(abs(control_signal), 0.0, 100.0)
        
        if control_signal > 0:
            pwm_l.ChangeDutyCycle(0)
            pwm_r.ChangeDutyCycle(duty_cycle)
        elif control_signal < 0:
            pwm_r.ChangeDutyCycle(0)
            pwm_l.ChangeDutyCycle(duty_cycle)
        else:
            pwm_r.ChangeDutyCycle(0)
            pwm_l.ChangeDutyCycle(0)
            
        prev_error = error
        last_time = current_time
        time.sleep(0.02) # 50 Hz loop

# ==========================================
# OUTER LOOP: CAMERA VISION TRACKING
# ==========================================
# TENNIS BALL HSV THRESHOLDS
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

def track():
    global target_rpm, system_running
    
    print("Initializing Pi 5 Camera...")
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "BGR888"})
    picam2.configure(config)

    # --- THE CAMERA PHYSICS FIX ---
    picam2.set_controls({
        "FrameRate": 60,            # Force a blistering fast 60 FPS
        "AeEnable": False,          # Kill the Auto-Exposure algorithm entirely
        "ExposureTime": 15000,      # Shutter speed in microseconds (15ms). Stops motion blur!
        "AnalogueGain": 5.0         # Sensor sensitivity (ISO). Boost this if the image is too dark.
    })

    picam2.start()
    time.sleep(2.0)

    set_camera(picam2)
    threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000, threaded=True), daemon=True).start()
    
    # Start the IMU Velocity Control Thread
    imu_thread = threading.Thread(target=imu_control_thread, daemon=True)
    imu_thread.start()

    center_x = 320
    DEAD_ZONE_PX = 100
    Kp_vision = 0.001  # How many RPMs to request per pixel of error
    
    ball_x_history = []
    SMOOTH_FRAMES = 3

    miss_count = 0
    last_known_dir = 1
    last_sweep_toggle = time.time()
    
    print("\nCascaded Control Active. Vision is commanding IMU velocity.")

    try:
        while system_running:
            frame = picam2.capture_array()
            frame = frame[:, :, ::-1]   
            frame = cv2.flip(frame, -1)

            result = find_ball(frame)

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
                    target_rpm = 0.0 # Ask IMU to stop the platform
                else:
                    # Map pixel error directly to a target velocity
                    target_rpm = Kp_vision * error 
                    
                    # Cap the maximum turning speed for safety
                    target_rpm = clamp(target_rpm, -25.0, 25.0)

                print(f"TRACKING | Error: {error:+6.1f}px | Commanding IMU to: {target_rpm:+.1f} RPM")

            else:
                # --- SEARCH MODE ---
                update_tracking_marker(None, None, None)
                miss_count += 1
                ball_x_history.clear()  

                if miss_count < 8:
                    target_rpm = 0.0 # Coast to a stop briefly
                else:
                    # Radar Sweep: Ask the IMU to maintain exactly 8 RPM, reversing every 2 seconds
                    if time.time() - last_sweep_toggle > 2.0:
                        last_known_dir *= -1
                        last_sweep_toggle = time.time()
                    
                    target_rpm = last_known_dir * 8.0 
                    print(f"SEARCHING | Commanding IMU to sweep at: {target_rpm:+.1f} RPM")

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        system_running = False # Kill the IMU thread
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

if __name__ == '__main__':
    track()