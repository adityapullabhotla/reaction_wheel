import RPi.GPIO as GPIO
import time
import sys

# HARDWARE PINS
R_EN = 26
L_EN = 16
RPWM = 5
LPWM = 6
ENC_A = 17
ENC_B = 22

# MOTOR SETTINGS
# Changed from 28 ticks per revolution to 7 for .25X decoding to prevent CPU overloading
CPR = 7.0 

def main():
    # USER INPUT
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

    # GPIO SETUP
    GPIO.setmode(GPIO.BCM)
    GPIO.setup([R_EN, L_EN, RPWM, LPWM], GPIO.OUT)
    GPIO.setup([ENC_A, ENC_B], GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # INITIALIZE H-BRIDGE PIN STATES
    # PWM pins are strictly LOW to start
    GPIO.output(RPWM, GPIO.LOW)
    GPIO.output(LPWM, GPIO.LOW)
    
    # Both EN pins must be high before applying PWM for BOTH forward and reverse spin direction
    GPIO.output(R_EN, GPIO.HIGH)
    GPIO.output(L_EN, GPIO.HIGH)

    # ENCODER COUNT
    encoder_pos = 0

    def encoder_callback_A(channel):
        nonlocal encoder_pos
        # only trigger on the rising edge of A.
        # check the state of B to determine direction.
        if GPIO.input(ENC_B) == GPIO.LOW:
            encoder_pos += 1 # add tick for one direction
        else:
            encoder_pos -= 1 # subtract tick for the other direction

    # Only attach one interrupt event to one pin (Rising edge only)
    GPIO.add_event_detect(ENC_A, GPIO.RISING, callback=encoder_callback_A)

    # OPEN-LOOP CONTROL EXECUTION
    last_pos = 0
    last_time = time.time()
    active_pwm = None # Track which PWM stream is running
    
    try:
        # set the inactive pin as LOW, and only apply PWM to the active pin
        # only one pwm signal can be high for each direction. other one has to be low
        if direction == 'cw':
            GPIO.output(LPWM, GPIO.LOW) 
            active_pwm = GPIO.PWM(RPWM, 1000)
            active_pwm.start(pwm_percent)
        elif direction == 'ccw':
            GPIO.output(RPWM, GPIO.LOW) 
            active_pwm = GPIO.PWM(LPWM, 1000)
            active_pwm.start(pwm_percent)

        # run control loop at 10 Hertz
        while True:
            time.sleep(0.1) 
            
            current_time = time.time()
            dt = current_time - last_time
            
            current_pos = encoder_pos
            delta_pos = current_pos - last_pos
            
            last_pos = current_pos
            last_time = current_time
            
            if dt > 0:
                current_rpm = (delta_pos / CPR) * (60.0 / dt)
            else:
                current_rpm = 0.0
                
            print(f"Duty Cycle: {duty_cycle} | Direction: {direction} | Current: {current_rpm:>6.1f} RPM | Total Ticks: {current_pos}")

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected. Stopping motor safely...")
    finally:
        # shut down PWM streams
        if active_pwm:
            active_pwm.stop()
        
        # shutdown all hardware pins
        GPIO.output(RPWM, GPIO.LOW)
        GPIO.output(LPWM, GPIO.LOW)
        GPIO.output(R_EN, GPIO.LOW)
        GPIO.output(L_EN, GPIO.LOW)
        GPIO.cleanup()
        print("Hardware cleaned up. Exiting.")

if __name__ == "__main__":
    main()