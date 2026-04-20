import RPi.GPIO as GPIO
import time
import sys

# --- HARDWARE PINS ---
# Based on your wiring diagram
R_EN = 26
L_EN = 16
RPWM = 6
LPWM = 5
ENC_A = 17
ENC_B = 22

# --- MOTOR SETTINGS ---
# REV HD Hex Motor (1:1 ratio) is 28 Counts Per Revolution at the motor.
CPR = 28.0 

# --- PID CONSTANTS ---
# Tune these if the motor is overshooting or too sluggish.
KP = 0.4
KI = 0.2

def main():
    # 1. USER INPUT VALIDATION
    while True:
        try:
            user_input = input("Enter Target RPM and Direction (e.g., '100 cw'): ").strip().lower().split()
            if len(user_input) != 2:
                print("Error: Please provide exactly two arguments (RPM and direction).")
                continue
            
            target_rpm = float(user_input[0])
            direction = user_input[1]
            
            if not (0 <= target_rpm <= 200):
                print("Error: Target RPM must be between 0 and 200.")
                continue
                
            if direction not in ['cw', 'ccw']:
                print("Error: Direction must be either 'cw' or 'ccw'.")
                continue
                
            break
        except ValueError:
            print("Error: Target RPM must be a valid number.")

    # Convert to a signed setpoint for the PID controller
    setpoint = target_rpm if direction == 'cw' else -target_rpm
    print(f"\nStarting motor... Target Setpoint: {setpoint} RPM. Press Ctrl+C to safely stop.\n")

    # 2. GPIO SETUP
    GPIO.setmode(GPIO.BCM)
    GPIO.setup([R_EN, L_EN, RPWM, LPWM], GPIO.OUT)
    
    # Encoder pins need internal pull-ups
    GPIO.setup([ENC_A, ENC_B], GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # 3. H-BRIDGE / PWM SETUP
    # The driver uses two BTS7960B half-bridge ICs.
    pwm_r = GPIO.PWM(RPWM, 1000) # 1 kHz frequency
    pwm_l = GPIO.PWM(LPWM, 1000)
    pwm_r.start(0)
    pwm_l.start(0)

    # Both enable pins must be HIGH before applying PWM.
    GPIO.output(R_EN, GPIO.HIGH)
    GPIO.output(L_EN, GPIO.HIGH)

    # 4. ENCODER INTERRUPTS (Quadrature Decoding)
    encoder_pos = 0

    def encoder_callback_A(channel):
        nonlocal encoder_pos
        if GPIO.input(ENC_A) == GPIO.input(ENC_B):
            encoder_pos += 1
        else:
            encoder_pos -= 1

    def encoder_callback_B(channel):
        nonlocal encoder_pos
        if GPIO.input(ENC_A) != GPIO.input(ENC_B):
            encoder_pos += 1
        else:
            encoder_pos -= 1

    # Attach interrupts to BOTH edges of BOTH channels for full 28 CPR resolution
    GPIO.add_event_detect(ENC_A, GPIO.BOTH, callback=encoder_callback_A)
    GPIO.add_event_detect(ENC_B, GPIO.BOTH, callback=encoder_callback_B)

    # 5. CONTROL LOOP VARIABLES
    integral = 0
    last_pos = 0
    last_time = time.time()
    
    try:
        # Run loop at exactly 10 Hz (0.1 seconds)
        while True:
            # time.sleep(0.1) 
            
            current_time = time.time()
            dt = current_time - last_time
            
            # Safely grab current position to avoid race conditions
            current_pos = encoder_pos
            delta_pos = current_pos - last_pos
            
            last_pos = current_pos
            last_time = current_time
            
            # Calculate actual RPM
            # Revolutions = delta_pos / CPR. Divide by dt for revs/sec. Multiply by 60 for revs/min.
            current_rpm = (delta_pos / CPR) * (60.0 / dt)
            
            # --- PI CONTROLLER ---
            error = -(setpoint - current_rpm)
            integral += error * dt
            
            # Calculate output and clamp it to standard PWM bounds (-100 to 100)
            control_signal = (KP * error) + (KI * integral)
            control_signal = max(min(control_signal, 100.0), -100.0)
            
            # --- H-BRIDGE COMMANDS ---
            # Driving RPWM high and LPWM low results in forward motion, and vice versa.
            if control_signal > 0:
                pwm_r.ChangeDutyCycle(control_signal)
                pwm_l.ChangeDutyCycle(0)
            elif control_signal < 0:
                # To brake or reverse, we apply PWM to the opposite side 
                # This inherently creates active braking using the motor's back-EMF.
                pwm_r.ChangeDutyCycle(0)
                pwm_l.ChangeDutyCycle(-control_signal)
            else:
                # Coast / 0 power
                pwm_r.ChangeDutyCycle(0)
                pwm_l.ChangeDutyCycle(0)
                
            # Print Telemetry at 10 Hz
            print(f"Target: {setpoint:>6.1f} RPM | Current: {current_rpm:>6.1f} RPM | "
                  f"PWM Output: {control_signal:>6.1f}% | Total Encoder Ticks: {current_pos}")

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected. Stopping motor safely...")
    finally:
        # Stop PWM streams and pull enable pins LOW
        pwm_r.stop()
        pwm_l.stop()
        GPIO.output(R_EN, GPIO.LOW)
        GPIO.output(L_EN, GPIO.LOW)
        GPIO.cleanup()
        print("Hardware cleaned up. Exiting.")

if __name__ == "__main__":
    main()