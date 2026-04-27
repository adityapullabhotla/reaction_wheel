import cv2
import time
import numpy as np
from picamera2 import Picamera2
from flask import Flask, Response

app = Flask(__name__)
picam2 = None

# --- CHANGE: Global variables to store the ball's location ---
tracked_x = None
tracked_y = None
tracked_r = None

def init_camera():
    global picam2
    print("Initializing native Pi 5 camera...")
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "BGR888"})
    picam2.configure(config)
    picam2.start()
    time.sleep(2.0)
    print("Camera ready")

def set_camera(cam):
    global picam2
    picam2 = cam

# --- CHANGE: Function that tracking.py calls to update the target ---
def update_tracking_marker(x, y, r):
    global tracked_x, tracked_y, tracked_r
    tracked_x = x
    tracked_y = y
    tracked_r = r

def generate_frames():
    while True:
        frame = picam2.capture_array()
        frame = frame[:, :, ::-1]   
        frame = cv2.flip(frame, -1) 

        # Center Crosshair reference (Green)
        cv2.line(frame, (320, 0), (320, 480), (0, 255, 0), 1)
        cv2.line(frame, (0, 240), (640, 240), (0, 255, 0), 1)

        # --- CHANGE: Draw the dynamic tracking marker and HSV text ---
        if tracked_x is not None and tracked_y is not None and tracked_r is not None:
            
            # Ensure coordinates are safely within frame boundaries
            if 0 <= tracked_x < 640 and 0 <= tracked_y < 480:
                # Grab the raw BGR pixel exactly at the center of the ball
                pixel_bgr = np.uint8([[frame[tracked_y, tracked_x]]])
                
                # Convert only that single pixel to HSV to save Pi CPU cycles
                h, s, v = cv2.cvtColor(pixel_bgr, cv2.COLOR_BGR2HSV)[0][0]
                
                # Overlay the text slightly above and to the right of the center
                hsv_text = f"HSV: {h}, {s}, {v}"
                cv2.putText(frame, hsv_text, (tracked_x + 15, tracked_y - 15), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            # Draw a circle outlining the ball
            cv2.circle(frame, (tracked_x, tracked_y), tracked_r, (0, 0, 255), 2)
            # Draw a dot perfectly in the center of the ball
            cv2.circle(frame, (tracked_x, tracked_y), 4, (0, 0, 255), -1)

        ret, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    init_camera()
    print("\n" + "="*50)
    print("VIDEO STREAM RUNNING!")
    print("Open a web browser on your Mac and go to:")
    print("http://<YOUR_PI_IP_ADDRESS>:5000")
    print("="*50 + "\n")
    app.run(host='0.0.0.0', port=5000, threaded=True)