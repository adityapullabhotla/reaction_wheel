
import cv2
import time
from picamera2 import Picamera2
from flask import Flask, Response

# Set up Flash web abb to remotely view live feed of camera
app = Flask(__name__)
picam2 = None

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
    """
    Called by tracking.py to inject its already-running picam2 instance
    so only one script opens the camera
    """
    global picam2
    picam2 = cam

def generate_frames():
    while True:
        frame = picam2.capture_array()
        frame = frame[:, :, ::-1]   # fix RGB channel order from Pi to OpenCV (wow thank god)
        frame = cv2.flip(frame, -1) # flip frame 180 degrees so images appear right side up (camera mounted upside down)

        # Crosshair for center reference
        cv2.line(frame, (320, 0), (320, 480), (0, 255, 0), 1)
        cv2.line(frame, (0, 240), (640, 240), (0, 255, 0), 1)

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