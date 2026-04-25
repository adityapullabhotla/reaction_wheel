import cv2
import time
from picamera2 import Picamera2

def test_camera():
    print("Initializing native Pi 5 camera...")
    # 1. Initialize the native Pi 5 camera library
    picam2 = Picamera2()

    # 2. Configure it for high-speed tracking (640x480)
    config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"})
    picam2.configure(config)
    
    print("Starting camera... warming up sensor.")
    picam2.start()
    time.sleep(2.0)

    frames_grabbed = 0
    print("Attempting to grab frames. Press 'Ctrl+C' to stop.")

    try:
        while True:
            # 3. Grab the frame directly as a Numpy array (OpenCV's native format!)
            frame = picam2.capture_array()
            
            # Picamera2 grabs in RGB, but OpenCV expects BGR colors
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            frames_grabbed += 1
            if frames_grabbed % 10 == 0:
                print(f"Success! Grabbed {frames_grabbed} frames directly from ISP...")

            #Leave these commented out while on SSH
            # cv2.imshow('Arducam Feed', frame)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break

    except KeyboardInterrupt:
        print("\nStopping camera test.")
    except Exception as e:
        print(f"\nAn error occurred: {e}")
    finally:
        picam2.stop()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    test_camera()