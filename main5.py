import RPi.GPIO as GPIO
import time

# Pins based on your wiring diagram
R_EN = 26
L_EN = 16
RPWM = 6
LPWM = 5

def main():
    print("Testing motor at 50% power for 5 seconds...")
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # Set up pins as outputs
    GPIO.setup([R_EN, L_EN, RPWM, LPWM], GPIO.OUT)
    
    # Initialize PWM on both directional pins at 1 kHz
    pwm_r = GPIO.PWM(RPWM, 1000) 
    pwm_l = GPIO.PWM(LPWM, 1000)
    
    # Start PWM at 0% duty cycle
    pwm_r.start(0)
    pwm_l.start(0)
    
    # Both enable pins MUST be HIGH before applying PWM 
    GPIO.output(R_EN, GPIO.HIGH)
    GPIO.output(L_EN, GPIO.HIGH)
    
    try:
        # Command 50% power in one direction
        print("Motor should be spinning...")
        pwm_r.ChangeDutyCycle(25)
        pwm_l.ChangeDutyCycle(0)
        
        # Let it run for 5 seconds
        time.sleep(5)
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
    finally:
        print("Stopping motor and cleaning up GPIO.")
        # Safely power down
        pwm_r.stop()
        pwm_l.stop()
        GPIO.output(R_EN, GPIO.LOW)
        GPIO.output(L_EN, GPIO.LOW)
        GPIO.cleanup()

if __name__ == "__main__":
    main()