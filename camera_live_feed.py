import cv2
import time
from picamera2 import Picamera2
from flask import Flask, Response

app = Flask(__name__)
picam2 = None

def init_camera():
    global picam2
    print("Initializing native Pi 5 camera...")
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"})
    picam2.configure(config)
    picam2.start()
    time.sleep(2.0)
    print("Camera warmed up and ready!")

def generate_frames():
    while True:
        # Grab frame and convert to OpenCV BGR format
        # Grab frame and swap the Red and Blue channels!
        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = cv2.flip(frame, -1)
        
        # Add a crosshair so you can see the exact center for Demo 2
        cv2.line(frame, (320, 0), (320, 480), (0, 255, 0), 1)
        cv2.line(frame, (0, 240), (640, 240), (0, 255, 0), 1)

        # Compress the frame to JPEG
        ret, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()

        # Yield the frame in a format the web browser understands (multipart)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/')
def video_feed():
    # This endpoint returns the continuous video stream
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    init_camera()
    print("\n" + "="*50)
    print("VIDEO STREAM RUNNING!")
    print("Open a web browser on your Mac and go to:")
    print("http://<YOUR_PI_IP_ADDRESS>:5000")
    print("="*50 + "\n")
    
    # Run the server on all network interfaces on port 5000
    app.run(host='0.0.0.0', port=5000, threaded=True)