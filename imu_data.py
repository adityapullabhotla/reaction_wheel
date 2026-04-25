import time
import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_GYROSCOPE

def main():
    print("Initializing I2C and BNO085 IMU...")
    
    # Set up I2C bus at 400kHz (Required for BNO085 on Raspberry Pi)
    i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
    
    try:
        # Initialize the sensor at the default BNO085 address (0x4A)
        sensor = BNO08X_I2C(i2c)
        # Explicitly enable the gyroscope feature to get angular velocity
        sensor.enable_feature(BNO_REPORT_GYROSCOPE)
        print("IMU initialized successfully")
        print("Spin the platform with your hand.")
    except Exception as e:
        print(f"Error initializing IMU: {e}")
        return

    try:
        while True:
            # The Raspberry Pi + BNO085 combo can occasionally drop I2C packets.
            # We use a try/except block to keep the script from crashing if it misses a read.
            try:
                gyro_data = sensor.gyro
                
                if gyro_data and gyro_data[2] is not None:
                    # gyro_data returns (X, Y, Z) in radians per second.
                    # Z-axis [2] represents the flat rotation of the platform.
                    rad_per_sec = gyro_data[2]
                    
                    # Convert rad/s to RPM (1 rad/s = 9.549297 RPM)
                    rpm = rad_per_sec * 9.549297
                    
                    print(f"Platform Angular Velocity:  {rpm:>7.2f} RPM  |  {rad_per_sec:>7.2f} rad/s")
            
            except Exception:
                pass # Silently skip this loop iteration and try again
            
            # Pause briefly so the terminal is readable (approx 10Hz)
            time.sleep(0.1) 

    except KeyboardInterrupt:
        print("\nTest stopped by user. Exiting.")

if __name__ == "__main__":
    main()