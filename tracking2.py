import cv2
import time
import RPi.GPIO as GPIO
from picamera2 import Picamera2

# ==========================================
# 1. ENTER YOUR CALIBRATED HSV VALUES HERE!
# ==========================================
TARGET_HUE = 80   # Replace with the first number from your calibration
TARGET_SAT = 120  # Replace with the second number
TARGET_VAL = 200  # Replace with the third number

# Automatically creates a generous window around your target color
color_lower = (max(0, TARGET_HUE - 10), max(50, TARGET_SAT - 50), max(50, TARGET_VAL - 50))
color_upper = (min(179, TARGET_HUE + 10), min(255, TARGET_SAT + 50), min(255, TARGET_VAL + 50))

# --- HARDWARE PINS ---
R_EN = 26
L_EN = 16
RPWM = 5
LPWM = 6

# --- MOTOR SETTINGS ---
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup([R_EN, L_EN, RPWM, LPWM], GPIO.OUT)

# Enable the heavy-duty motor driver
GPIO.output(R_EN, GPIO.HIGH)
GPIO.output(L_EN, GPIO.HIGH)

pwm_r = GPIO.PWM(RPWM, 1000)
pwm_l = GPIO.PWM(LPWM, 1000)
pwm_r.start(0)
pwm_l.start(0)

# --- PID CONSTANTS ---
Kp = 0.04  
Ki = 0.00  # Kept at zero to prevent violent shaking!
Kd = 0.03  

def clamp(value, min_val, max_val):
    # Minimum 20% power to break reaction wheel friction
    if 0.5 < value < 20.0:  
        return 20.0
    return max(min_val, min(value, max_val))

def set_motor(pwm_command):
    power = clamp(abs(pwm_command), 0.0, 100.0)
    
    if pwm_command > 0:
        pwm_l.ChangeDutyCycle(0)
        pwm_r.ChangeDutyCycle(power)
    elif pwm_command < 0:
        pwm_r.ChangeDutyCycle(0)
        pwm_l.ChangeDutyCycle(power)
    else:
        pwm_l.ChangeDutyCycle(0)
        pwm_r.ChangeDutyCycle(0)

def demo_2():
    print("Initializing Pi 5 Camera...")
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"})
    picam2.configure(config)
    picam2.start()
    time.sleep(2.0)
    
    center_x = 320 
    integral = 0
    prev_error = 0
    prev_time = time.time()
    search_pwm = 20.0 

    print("Demo 2 Active: Vision Tracking Initiated. Press 'Ctrl+C' to stop.")

    try:
        while True:
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            frame = cv2.flip(frame, -1)
            
            current_time = time.time()
            dt = current_time - prev_time
            if dt <= 0: dt = 0.001
            prev_time = current_time

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, color_lower, color_upper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if len(contours) > 0:
                # Target acquired! Reset the search ramp
                search_pwm = 20.0 
                
                c = max(contours, key=cv2.contourArea)
                M = cv2.moments(c)
                
                if M["m00"] > 0:
                    ball_cx = int(M["m10"] / M["m00"])
                    error = center_x - ball_cx 
                    
                    # 25-pixel Error Deadzone
                    if abs(error) < 25:
                        pwm_out = 0.0
                        integral = 0 
                    else:
                        integral += error * dt
                        derivative = (error - prev_error) / dt
                        pwm_out = (Kp * error) + (Ki * integral) + (Kd * derivative)
                    
                    set_motor(pwm_out)
                    prev_error = error
                    
                    actual_power = clamp(abs(pwm_out), 0.0, 100.0) if pwm_out != 0.0 else 0.0
                    print(f"Tracking | Ball X: {ball_cx:3d} | Error: {error:6.1f} | Power: {actual_power:5.1f}%")
            else:
                # Target lost. Initiate Ramp-Up Search Mode!
                print(f"Scanning | Ramping motor to {search_pwm:.1f}% to overcome friction...")
                set_motor(search_pwm)
                
                # Steadily increase torque to break static friction and pan the camera
                search_pwm += 0.5 
                if search_pwm > 70.0:
                    search_pwm = 20.0
                
                integral = 0 
                prev_error = 0

    except KeyboardInterrupt:
        print("\nDemo 2 Terminated. Shutting down safely...")
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

if __name__ == '__main__':
    demo_2()