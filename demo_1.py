import time
import sys
import board
import busio
import RPi.GPIO as GPIO
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_GYROSCOPE

# --- HARDWARE PINS ---
R_EN = 26
L_EN = 16
RPWM = 5
LPWM = 6
ENC_A = 17
ENC_B = 22

# --- PID GAINS (THESE NEED TUNING) ---
Kp = 1.5   # Proportional gain: How aggressively to fight the spin
Ki = 0.0   # Integral gain: Fixes steady-state errors (keep 0 to start)
Kd = 0.1   # Derivative gain: Dampens the response to prevent overshoot

# --- MOTOR SETTINGS ---
MAX_PWM = 100.0
MIN_PWM = 0.0

def clamp(value, min_val, max_val):
    return max(min_val, min(value, max_val))

def main():
    print("Initializing System for Demo 1...")
    
    # 1. IMU SETUP
    try:
        i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
        sensor = BNO08X_I2C(i2c)
        sensor.enable_feature(BNO_REPORT_GYROSCOPE)
        print("IMU initialized.")
    except Exception as e:
        print(f"Error initializing IMU: {e}")
        sys.exit(1)

    # 2. GPIO SETUP
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

    print("Motor initialized.")
    print("\nSystem Armed. Perturb the testbed to test the PID loop. Press Ctrl+C to stop.\n")

    # 3. PID VARIABLES
    target_rpm = 0.0
    integral = 0.0
    prev_error = 0.0
    last_time = time.time()
    
    try:
        while True:
            current_time = time.time()
            dt = current_time - last_time
            
            # Avoid divide-by-zero on the very first loop
            if dt <= 0:
                continue
                
            # 4. READ IMU (PROCESS VARIABLE)
            platform_rpm = 0.0
            try:
                gyro_data = sensor.gyro
                if gyro_data and gyro_data[2] is not None:
                    platform_rpm = gyro_data[2] * 9.549297 # Convert rad/s to RPM
            except Exception:
                pass # Skip loop if I2C drops a packet
                
            # 5. PID MATH
            error = target_rpm - platform_rpm
            
            # Anti-windup for integral (only integrate if we aren't saturating the motor)
            integral += error * dt
            derivative = (error - prev_error) / dt
            
            control_signal = (Kp * error) + (Ki * integral) + (Kd * derivative)
            
            # 6. MOTOR COMMAND
            # Determine direction based on the sign of the control signal
            # Note: You may need to swap 'pwm_r' and 'pwm_l' depending on your physical motor wiring
            duty_cycle = clamp(abs(control_signal), MIN_PWM, MAX_PWM)
            
            if control_signal > 0:
                # Spin one way
                pwm_l.ChangeDutyCycle(0)
                pwm_r.ChangeDutyCycle(duty_cycle)
                direction = "CW "
            elif control_signal < 0:
                # Spin the other way
                pwm_r.ChangeDutyCycle(0)
                pwm_l.ChangeDutyCycle(duty_cycle)
                direction = "CCW"
            else:
                pwm_r.ChangeDutyCycle(0)
                pwm_l.ChangeDutyCycle(0)
                direction = "OFF"
                
            print(f"Platform: {platform_rpm:>6.1f} RPM | Error: {error:>6.1f} | Command: {direction} at {duty_cycle:>5.1f}% PWM")
            
            # Update variables for next loop
            prev_error = error
            last_time = current_time
            
            # Run loop at approx 50Hz for smooth control
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nStopping testbed...")
    finally:
        pwm_r.stop()
        pwm_l.stop()
        GPIO.output(RPWM, GPIO.LOW)
        GPIO.output(LPWM, GPIO.LOW)
        GPIO.output(R_EN, GPIO.LOW)
        GPIO.output(L_EN, GPIO.LOW)
        GPIO.cleanup()
        print("Hardware safed. Exiting.")

if __name__ == "__main__":
    main()