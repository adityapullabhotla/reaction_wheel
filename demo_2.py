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
ENC_A = 17 # 추후 Motor Saturation 감지를 위해 인터럽트로 읽어야 함
ENC_B = 22

# --- PID GAINS ---
Kp = 1.5   # 비례 제어 (현재 값 유지 추천)
Ki = 0.1   # 적분 제어 (매우 작은 값부터 테스트하세요)
Kd = 0.1   # 미분 제어 (진동을 잡아줌)

# --- SETTINGS ---
MAX_PWM = 100.0
MIN_PWM = 0.0
MAX_INTEGRAL = 30.0  # 적분 누적 최대 허용치 (Anti-windup)

def clamp(value, min_val, max_val):
    return max(min_val, min(value, max_val))

def main():
    print("Initializing System for Demo 1...")
    
    # 1. IMU SETUP
    try:
        i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
        sensor = BNO08X_I2C(i2c)
        sensor.enable_feature(BNO_REPORT_GYROSCOPE)
        print("IMU initialized. Waiting 1 second for sensor stabilization...")
        time.sleep(1) # 센서 안정화 대기
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
            
            if dt <= 0:
                continue
                
            # 4. READ IMU
            platform_rpm = 0.0
            try:
                gyro_data = sensor.gyro
                if gyro_data and gyro_data[2] is not None:
                    platform_rpm = gyro_data[2] * 9.549297 # rad/s -> RPM 변환
            except Exception as e:
                print(f"IMU Read Error: {e}") # 에러를 숨기지 않고 출력
                time.sleep(0.01)
                continue
                
            # 5. PID MATH
            error = target_rpm - platform_rpm
            
            # [개선됨] 진정한 Anti-windup 구현: 에러를 누적하되, 지정된 범위를 넘지 않게 자름
            integral += error * dt
            integral = clamp(integral, -MAX_INTEGRAL, MAX_INTEGRAL) 
            
            derivative = (error - prev_error) / dt
            
            control_signal = (Kp * error) + (Ki * integral) + (Kd * derivative)
            
            # 6. MOTOR COMMAND
            duty_cycle = clamp(abs(control_signal), MIN_PWM, MAX_PWM)
            
            if control_signal > 0.5: # 0.5 정도의 작은 노이즈(Deadband)는 무시하게 설정 가능
                pwm_l.ChangeDutyCycle(0)
                pwm_r.ChangeDutyCycle(duty_cycle)
                direction = "CW "
            elif control_signal < -0.5:
                pwm_r.ChangeDutyCycle(0)
                pwm_l.ChangeDutyCycle(duty_cycle)
                direction = "CCW"
            else:
                pwm_r.ChangeDutyCycle(0)
                pwm_l.ChangeDutyCycle(0)
                direction = "OFF"
                
            print(f"Platform: {platform_rpm:>6.1f} RPM | Error: {error:>6.1f} | Command: {direction} at {duty_cycle:>5.1f}% PWM")
            
            prev_error = error
            last_time = current_time
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