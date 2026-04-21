import RPi.GPIO as GPIO
import time
import sys
import board
import busio
import adafruit_bno055

# --- HARDWARE PINS ---
R_EN = 26
L_EN = 16
RPWM = 5
LPWM = 6
ENC_A = 17
ENC_B = 22

# --- MOTOR SETTINGS ---
CPR = 7.0 

def main():
    # --- 1. IMU SETUP ---
    print("Initializing I2C and BNO055 IMU...")
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        sensor = adafruit_bno055.BNO055_I2C(i2c)
        print("IMU initialized successfully.\n")
    except Exception as e:
        print(f"Error initializing IMU: {e}")
        print("Check your I2C wiring (SDA/SCL) and ensure I2C is enabled in raspi-config.")
        sys.exit(1)

    # --- 2. USER INPUT VALIDATION ---
    while True:
        try:
            user_input = input("Enter Duty Cycle (0 to 1) and Direction (e.g., '0.5 cw'): ").strip().lower().split()
            if len(user_input) != 2:
                print("Error: Please provide exactly two arguments (Duty Cycle and direction).")
                continue
            
            duty_cycle = float(user_input[0])
            direction = user_input[1]
            
            if not (0.0 <= duty_cycle <= 1.0):
                print("Error: Duty Cycle must be between 0.0 and 1.0.")
                continue
                
            if direction not in ['cw', 'ccw']:
                print("Error: Direction must be either 'cw' or 'ccw'.")
                continue
                
            break
        except ValueError:
            print("Error: Duty Cycle must be a valid number.")

    pwm_percent = duty_cycle * 100.0
    print(f"\nStarting motor... Duty Cycle: {duty_cycle} ({pwm_percent}%) | Direction: {direction}. Press Ctrl+C to stop.\n")

    # --- 3. GPIO SETUP ---
    GPIO.setmode(GPIO.BCM)
    GPIO.setup([R_EN, L_EN, RPWM, LPWM], GPIO.OUT)
    GPIO.setup([ENC_A, ENC_B], GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # --- 4. INITIALIZE H-BRIDGE STATES ---
    GPIO.output(RPWM, GPIO.LOW)
    GPIO.output(LPWM, GPIO.LOW)
    GPIO.output(R_EN, GPIO.HIGH)
    GPIO.output(L_EN, GPIO.HIGH)

    # --- 5. ENCODER INTERRUPTS ---
    encoder_pos = 0

    def encoder_callback_A(channel):
        nonlocal encoder_pos
        if GPIO.input(ENC_B) == GPIO.LOW:
            encoder_pos += 1
        else:
            encoder_pos -= 1

    GPIO.add_event_detect(ENC_A, GPIO.RISING, callback=encoder_callback_A)

    # --- 6. CONTROL AND TELEMETRY LOOP ---
    last_pos = 0
    last_time = time.time()
    active_pwm = None 
    
    try:
        if direction == 'cw':
            GPIO.output(LPWM, GPIO.LOW) 
            active_pwm = GPIO.PWM(RPWM, 1000)
            active_pwm.start(pwm_percent)
        elif direction == 'ccw':
            GPIO.output(RPWM, GPIO.LOW) 
            active_pwm = GPIO.PWM(LPWM, 1000)
            active_pwm.start(pwm_percent)

        # Telemetry loop
        while True:
            time.sleep(0.1) 
            
            current_time = time.time()
            dt = current_time - last_time
            
            current_pos = encoder_pos
            delta_pos = current_pos - last_pos
            
            last_pos = current_pos
            last_time = current_time
            
            # Motor RPM Math
            if dt > 0:
                motor_rpm = (delta_pos / CPR) * (60.0 / dt)
            else:
                motor_rpm = 0.0
                
            # Platform RPM Math (IMU)
            gyro_data = sensor.gyro
            platform_rpm = 0.0
            
            # The BNO055 returns gyroscope data in radians per second (rad/s).
            # We index [2] to get the Z-axis (yaw) rotation of the platform.
            if gyro_data is not None and gyro_data[2] is not None:
                # Conversion: 1 rad/s = 9.549297 RPM
                platform_rpm = gyro_data[2] * 9.549297
                
            print(f"Motor: {motor_rpm:>6.1f} RPM | Platform (IMU Z-Axis): {platform_rpm:>6.1f} RPM")

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected. Stopping motor safely...")
    finally:
        if active_pwm:
            active_pwm.stop()
        GPIO.output(RPWM, GPIO.LOW)
        GPIO.output(LPWM, GPIO.LOW)
        GPIO.output(R_EN, GPIO.LOW)
        GPIO.output(L_EN, GPIO.LOW)
        GPIO.cleanup()
        print("Hardware cleaned up. Exiting.")

if __name__ == "__main__":
    main()