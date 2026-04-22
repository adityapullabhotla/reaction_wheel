import time
import sys
import board
import busio
from adafruit_bno08x import BNO_REPORT_GYROSCOPE
from adafruit_bno08x.i2c import BNO08X_I2C

def main():
    print("Initializing I2C and Adafruit BNO085 IMU...")
    try:
        # Initialize the I2C bus
        i2c = busio.I2C(board.SCL, board.SDA)
        
        # Initialize the BNO085 sensor
        bno = BNO08X_I2C(i2c)
        print("IMU initialized successfully.")
        
        # Enable the gyroscope feature
        bno.enable_feature(BNO_REPORT_GYROSCOPE)
        print("Gyroscope feature enabled. Reading data...\n")
        print("Spin the testbed! Press Ctrl+C to stop.\n")
        
    except Exception as e:
        print(f"Error initializing IMU: {e}")
        print("Check your I2C wiring (SDA/SCL) and ensure I2C is enabled.")
        print("Note: You may need to install the library using:")
        print("pip3 install adafruit-circuitpython-bno08x")
        sys.exit(1)

    try:
        while True:
            # Read the gyroscope data
            gyro_x, gyro_y, gyro_z = bno.gyro
            
            # The BNO085 returns gyroscope data in radians per second (rad/s).
            # Z-axis (yaw) rotation of the platform.
            # Conversion: 1 rad/s = 9.549297 RPM
            rpm_z = gyro_z * 9.549297 if gyro_z is not None else 0.0
            
            print(f"Gyro (rad/s) -> X: {gyro_x:>6.3f} | Y: {gyro_y:>6.3f} | Z: {gyro_z:>6.3f}  ||  Z-Axis RPM: {rpm_z:>6.1f}")
            
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nExiting.")

if __name__ == "__main__":
    main()
