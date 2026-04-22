import time
import sys
import board
import busio
import RPi.GPIO as GPIO
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_GYROSCOPE

# --- HARDWARE PINS (from motor_control.py) ---
R_EN = 26
L_EN = 16
RPWM = 5
LPWM = 6

# --- CONTROL TUNING ---
# Proportional gain: How aggressively the motor responds to the platform spinning.
# A Kp of 10.0 means 10 rad/s (~95 RPM) will trigger 100% duty cycle.
Kp = 10.0 
DEADBAND = 0.15 # rad/s - ignore platform movements slower than this (approx 1.5 RPM)

def main():
    print("--- Reaction Wheel Controller ---")
    
    # 1. SETUP IMU
    print("Initializing I2C and BNO085 IMU...")
    i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
    try:
        sensor = BNO08X_I2C(i2c)
        sensor.enable_feature(BNO_REPORT_GYROSCOPE)
        print("IMU initialized successfully.")
    except Exception as e:
        print(f"Error initializing IMU: {e}")
        print("Ensure i2c_arm_baudrate=400000 in config.txt and wiring is correct.")
        sys.exit(1)

    # 2. SETUP GPIO & H-BRIDGE
    print("Initializing H-Bridge GPIO...")
    GPIO.setmode(GPIO.BCM)
    GPIO.setup([R_EN, L_EN, RPWM, LPWM], GPIO.OUT)

    # Ensure PWM pins are strictly LOW to start
    GPIO.output(RPWM, GPIO.LOW)
    GPIO.output(LPWM, GPIO.LOW)
    
    # Enable both sides of the H-bridge
    GPIO.output(R_EN, GPIO.HIGH)
    GPIO.output(L_EN, GPIO.HIGH)

    # Create PWM instances (1000 Hz)
    pwm_cw = GPIO.PWM(RPWM, 1000)
    pwm_ccw = GPIO.PWM(LPWM, 1000)
    pwm_cw.start(0)
    pwm_ccw.start(0)

    print("System ready. Spin the testbed to test reaction torque. Press Ctrl+C to stop.\n")

    current_direction = None

    try:
        while True:
            # The Raspberry Pi + BNO085 combo can occasionally drop I2C packets.
            try:
                gyro_data = sensor.gyro
                
                if gyro_data and gyro_data[2] is not None:
                    rad_per_sec = gyro_data[2]
                    
                    # 3. CONTROL LOGIC
                    # If the spin is faster than the deadband, we apply braking torque
                    if abs(rad_per_sec) > DEADBAND:
                        # Calculate PWM proportionally to the spin speed
                        target_pwm = min(abs(rad_per_sec) * Kp, 100.0)
                        
                        if rad_per_sec > 0:
                            # Platform is spinning one way (let's call it positive/CW)
                            if current_direction != 'cw':
                                pwm_ccw.ChangeDutyCycle(0)
                                GPIO.output(LPWM, GPIO.LOW)
                                current_direction = 'cw'
                            pwm_cw.ChangeDutyCycle(target_pwm)
                            dir_str = "CW "
                        else:
                            # Platform is spinning the other way (negative/CCW)
                            if current_direction != 'ccw':
                                pwm_cw.ChangeDutyCycle(0)
                                GPIO.output(RPWM, GPIO.LOW)
                                current_direction = 'ccw'
                            pwm_ccw.ChangeDutyCycle(target_pwm)
                            dir_str = "CCW"
                            
                        print(f"Platform: {rad_per_sec:>6.2f} rad/s | Braking -> Dir: {dir_str} | PWM: {target_pwm:>5.1f}%")
                    
                    else:
                        # Within deadband, turn off motor
                        if current_direction is not None:
                            pwm_cw.ChangeDutyCycle(0)
                            pwm_ccw.ChangeDutyCycle(0)
                            GPIO.output(RPWM, GPIO.LOW)
                            GPIO.output(LPWM, GPIO.LOW)
                            current_direction = None
                        print(f"Platform: {rad_per_sec:>6.2f} rad/s | Braking -> OFF")

            except Exception:
                pass # Silently skip this loop iteration and try again (I2C error)
            
            # Loop at roughly 20Hz for snappy response without overloading
            time.sleep(0.05) 

    except KeyboardInterrupt:
        print("\nTest stopped by user. Shutting down safely...")
    finally:
        # 4. CLEAN SHUTDOWN
        pwm_cw.stop()
        pwm_ccw.stop()
        GPIO.output(RPWM, GPIO.LOW)
        GPIO.output(LPWM, GPIO.LOW)
        GPIO.output(R_EN, GPIO.LOW)
        GPIO.output(L_EN, GPIO.LOW)
        GPIO.cleanup()
        print("Hardware cleaned up. Exiting.")

if __name__ == "__main__":
    main()
