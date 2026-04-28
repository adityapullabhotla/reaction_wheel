import time
import sys
import board
import busio
import RPi.GPIO as GPIO
import matplotlib.pyplot as plt  # Added for graphing
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_GYROSCOPE

# --- HARDWARE PINS ---
R_EN = 26
L_EN = 16
RPWM = 5
LPWM = 6
ENC_A = 17
ENC_B = 22

# --- PID GAINS ---
Kp = 3.5   
Ki = 0.175  
Kd = 0.1   

# --- MOTOR OUTPUT SETTINGS ---
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
    print("\nSystem Armed. Perturb the testbed to test the PID loop. Press Ctrl+C to stop and generate the graph.\n")

    # 3. PID & GRAPHING VARIABLES
    target_rpm = 0.0
    integral = 0.0
    prev_error = 0.0
    
    # Data logging arrays
    time_data = []
    rpm_data = []
    pwm_data = []
    
    start_time = time.time()
    last_time = start_time
    
    try:
        while True:
            current_time = time.time()
            dt = current_time - last_time
            
            if dt <= 0:
                continue
                
            # 4. READ IMU
            platform_rpm = 0.0
            try:
                gyro_data = sensor.gyro
                if gyro_data and gyro_data[2] is not None:
                    platform_rpm = gyro_data[2] * 9.549297 # Convert rad/s to RPM
            except Exception:
                pass 
                
            # 5. PID CONTROL
            error = target_rpm - platform_rpm
            
            integral += error * dt
            derivative = (error - prev_error) / dt
            control_signal = (Kp * error) + (Ki * integral) + (Kd * derivative)
            
            # 6. MOTOR COMMAND
            duty_cycle = clamp(abs(control_signal), MIN_PWM, MAX_PWM)
            
            # Determine direction and store signed PWM for the graph
            if control_signal > 0:
                pwm_l.ChangeDutyCycle(0)
                pwm_r.ChangeDutyCycle(duty_cycle)
                direction = "CW "
                signed_pwm = duty_cycle
            elif control_signal < 0:
                pwm_r.ChangeDutyCycle(0)
                pwm_l.ChangeDutyCycle(duty_cycle)
                direction = "CCW"
                signed_pwm = -duty_cycle
            else:
                pwm_r.ChangeDutyCycle(0)
                pwm_l.ChangeDutyCycle(0)
                direction = "OFF"
                signed_pwm = 0.0
                
            # 7. LOG DATA
            elapsed_time = current_time - start_time
            time_data.append(elapsed_time)
            rpm_data.append(platform_rpm)
            pwm_data.append(signed_pwm)
                
            print(f"Time: {elapsed_time:>5.1f}s | Platform: {platform_rpm:>6.1f} RPM | Cmd: {direction} at {duty_cycle:>5.1f}%")
            
            prev_error = error
            last_time = current_time
            
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nStopping testbed and generating graph...")
        
    finally:
        # Hardware MUST be safed before rendering the graph to prevent the motor from running away
        pwm_r.stop()
        pwm_l.stop()
        GPIO.output(RPWM, GPIO.LOW)
        GPIO.output(LPWM, GPIO.LOW)
        GPIO.output(R_EN, GPIO.LOW)
        GPIO.output(L_EN, GPIO.LOW)
        GPIO.cleanup()
        print("Hardware safed.")
        
        # --- GENERATE AND SAVE GRAPH ---
        if len(time_data) > 0:
            print("Rendering plot. This may take a moment on the Raspberry Pi...")
            
            plt.figure(figsize=(10, 8))
            
            # Subplot 1: RPM over time
            plt.subplot(2, 1, 1)
            plt.plot(time_data, rpm_data, label='Actual RPM', color='blue', linewidth=2)
            plt.axhline(0, color='red', linestyle='--', label='Target RPM (0.0)')
            plt.title('Demo 1: Disturbance Rejection Performance')
            plt.ylabel('Platform Velocity (RPM)')
            plt.legend()
            plt.grid(True)
            
            # Subplot 2: Motor PWM over time
            plt.subplot(2, 1, 2)
            plt.plot(time_data, pwm_data, label='Motor Output', color='green', linewidth=2)
            plt.axhline(100, color='red', linestyle=':', label='Max CW Power')
            plt.axhline(-100, color='red', linestyle=':', label='Max CCW Power')
            plt.xlabel('Time (Seconds)')
            plt.ylabel('Motor Output (% PWM)')
            plt.legend()
            plt.grid(True)
            
            plt.tight_layout()
            
            # Save the file directly to your working directory
            filename = "demo1_disturbance_rejection.png"
            plt.savefig(filename)
            print(f"Graph successfully saved as: {filename}")
        else:
            print("No data collected to graph.")

if __name__ == "__main__":
    main()