from flask import Flask, render_template, Response
import cv2
from pupil_apriltags import Detector

app = Flask(__name__)

at_detector = Detector(
   families="tag16h5",
   nthreads=1,
   quad_decimate=2.0,
   quad_sigma=1.6,
   refine_edges=1,
   decode_sharpening=0.25,
   debug=0
)

#  for cctv camera use rtsp://username:password@ip_address:554/user=username_password='password'_channel=channel_number_stream=0.sdp' instead of camera
# for local webcam use cv2.VideoCapture(0)

def gen_frames():  # generate frame by frame from camera
    camera = cv2.VideoCapture(1)
    while True:
        # Capture frame-by-frame
        success, frame = camera.read()  # read the camera frame
        if not success:
            break
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            results = at_detector.detect(gray)
            print("Results")
            for result in results:
                print("ID")
                print(result.tag_id)
                print("Hamming")
                print(result.hamming)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')  # concat frame one by one and show result


@app.route('/video_feed')
def video_feed():
    #Video streaming route. Put this in the src attribute of an img tag
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/')
def index():
    """Video streaming home page."""
    return render_template('index.html')


if __name__ == '__main__':
    app.run(debug=False,host="0.0.0.0")
