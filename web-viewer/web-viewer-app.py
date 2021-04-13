from flask import Response
from flask import request
from flask import Flask
from flask import render_template
from flask import jsonify
import threading
import queue
import time
import cv2
import os
# from imutils.video import VideoStream

WIDTH = 1280
HEIGHT = 720
FPS = 30

# bufferless VideoCapture from https://stackoverflow.com/a/54755738/7637307
class VideoCapture:

  def __init__(self, name, cap_init=None):
    self.cap = cv2.VideoCapture(name)
    if cap_init is not None:
        cap_init(self.cap)
    self.q = queue.Queue()
    t = threading.Thread(target=self._reader)
    t.daemon = True
    t.start()

  # read frames as soon as they are available, keeping only most recent one
  def _reader(self):
    while True:
      ret, frame = self.cap.read()
      if not ret:
        break
      if not self.q.empty():
        try:
          self.q.get_nowait()   # discard previous (unprocessed) frame
        except queue.Empty:
          pass
      self.q.put(frame)

  def read(self):
    return self.q.get()

app = Flask(__name__, template_folder=os.path.dirname(os.path.realpath(__file__))) # template_folder='/home/pi'

output_frame = None
output_frame_lock = threading.Lock()

def c930e_cap_itit(cap):
    global WIDTH, HEIGHT
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)


tracker = None
tracker_box = None
tracker_box_lock = threading.Lock()

def draw_overlay(image):
    global  tracker_box
    cv2.putText(image, f':)',
            (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv2.LINE_AA)
    with tracker_box_lock:
        if tracker_box is not None:
            cv2.rectangle(image, (tracker_box['xmin'],tracker_box['ymin']),
            (tracker_box['xmax'],tracker_box['ymax']), (0,255,0), 2)

initial_tracker = None
lock_initial_tracker = threading.Lock()

def update_tracking(image):
    global tracker, tracker_box, tracker_box_lock, lock_initial_tracker, initial_tracker
    with lock_initial_tracker:
        if initial_tracker is not None:
            tracker = cv2.TrackerCSRT_create()
            xmin = initial_tracker['xmin']
            xmax = initial_tracker['xmax']
            ymin = initial_tracker['ymin']
            ymax = initial_tracker['ymax']
            success = tracker.init(image, (xmin, ymin, xmax - xmin, ymax - ymin))
            initial_tracker = None
    if tracker is not None:
        (success, box) = tracker.update(image)
        if success:
            (x, y, w, h) = [int(v) for v in box]
            with tracker_box_lock:
                tracker_box = {
                    'xmin': x,
                    'xmax': x + w,
                    'ymin': y,
                    'ymax': y + h
                }

def monitor_video():
    global output_frame, output_frame_lock

    videoCapture = VideoCapture(0, cap_init=c930e_cap_itit)

    #cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
    #cap.set(cv2.CAP_PROP_EXPOSURE, -6)
    time.sleep(2.0)
    print('camera ready')

    while True:
        frame = videoCapture.read()
        image_with_overlay = frame.copy()
        update_tracking(frame)
        draw_overlay(image_with_overlay)
        with output_frame_lock:
            output_frame = image_with_overlay

def generate():
	# grab global references to the output frame and lock variables
	global output_frame, output_frame_lock
	# loop over frames from the output stream
	while True:
		# wait until the output_frame_lock is acquired
		with output_frame_lock:
			# check if the output frame is available, otherwise skip
			# the iteration of the loop
			if output_frame is None:
				continue
			# encode the frame in JPEG format
			(flag, encodedImage) = cv2.imencode(".jpg", output_frame)
			# ensure the frame was successfully encoded
			if not flag:
				continue
		# yield the output frame in the byte format
		yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + 
			bytearray(encodedImage) + b'\r\n')

@app.route("/")
def index():
    global WIDTH, HEIGHT
    return render_template("index.html",image_width=WIDTH,image_height=HEIGHT)

@app.route("/video_feed")
def video_feed():
	# return the response generated along with the specific media
	# type (mime type)
	return Response(generate(),
		mimetype = "multipart/x-mixed-replace; boundary=frame")

@app.route("/initial_tracker",methods=['PUT'])
def set_initial_tracker():
    global initial_tracker, lock_initial_tracker, initial_tracker_updated, lock_initial_tracker_updated
    xmin = int(float(request.form['xmin']))
    xmax = int(float(request.form['xmax']))
    ymin = int(float(request.form['ymin']))
    ymax = int(float(request.form['ymax']))
    with lock_initial_tracker:
        initial_tracker = {
            'xmin': xmin,
            'xmax': xmax,
            'ymin': ymin,
            'ymax': ymax
        }
        print(initial_tracker)
    return jsonify({'success': True}), 200

if __name__ == '__main__':
    t = threading.Thread(target=monitor_video)
    t.daemon = True
    t.start()

    app.run('0.0.0.0', 8080, debug=True,
		threaded=True, use_reloader=False)


