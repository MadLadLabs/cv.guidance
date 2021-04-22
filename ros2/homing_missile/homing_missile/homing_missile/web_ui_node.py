import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_default, qos_profile_sensor_data
from homing_missile_interfaces.msg import SynchronizedImage
from cv_bridge import CvBridge

from flask import Response
from flask import Flask
from flask import render_template

import cv2
import threading
import os
import time

class WebUINode(Node):

    def __init__(self):
        super().__init__('web_ui_node')
        self.subscription = self.create_subscription(SynchronizedImage, 'raw_indexed_image', self.listener_callback, qos_profile=qos_profile_default) # , qos_profile=qos_profile_sensor_data
        self.subscription
        self.br = CvBridge()
        self.image = None
        self.image_lock = threading.Lock()

    def listener_callback(self, msg):
        self.get_logger().debug('Received image frame.')
        with self.image_lock:
            self.image = self.br.imgmsg_to_cv2(msg.image)

    def get_image(self):
        with self.image_lock:
            return self.image

def web_thread_func(subscriber):
    app = Flask(__name__, template_folder=os.environ['VIEW_TEMPLATES_PATH'])

    @app.route("/")
    def index():
        # return the rendered template
        return render_template("index.html")

    def generate():
        while True:
            image = subscriber.get_image()
            if image is None:
                continue
            (flag, encodedImage) = cv2.imencode(".jpg", image)
            if not flag:
                continue
            yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + 
                bytearray(encodedImage) + b'\r\n')
            time.sleep(0.1)

    @app.route("/video_feed")
    def video_feed():
        return Response(generate(),
            mimetype = "multipart/x-mixed-replace; boundary=frame")

    app.run('0.0.0.0', 8080, debug=True,
		threaded=True, use_reloader=False)

def main():
    rclpy.init()
    subscriber = WebUINode()
    web_thread = threading.Thread(target=web_thread_func, args=[subscriber])
    web_thread.daemon = True
    web_thread.start()
    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()