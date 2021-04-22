import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_default, qos_profile_sensor_data
from homing_missile_interfaces.msg import SynchronizedImage
from std_msgs.msg import Int64
from cv_bridge import CvBridge
import cv2
import threading
import time

class CameraPublisher(Node):

    def __init__(self):
        super().__init__('publisher')
        self.millis = 0
        self.millis_lock = threading.Lock()

        self.__init_params()

        self.publisher_ = self.create_publisher(SynchronizedImage, 'raw_indexed_image', qos_profile=qos_profile_default) # , qos_profile=qos_profile_sensor_data
        self.subscription_ = self.create_subscription(Int64, 'flight_controller_millis', self.__millis_callback, qos_profile=qos_profile_default) #, qos_profile=qos_profile_sensor_data
        self.subscription_ # suppresses unused variable warning

        self.__init_cap()
        self.br = CvBridge()

        t = threading.Thread(target=self.__video_reader)
        t.daemon = True
        t.start()


    def __init_params(self):
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 10)

        self.width = self.get_parameter('width')
        self.height = self.get_parameter('height')
        self.fps = self.get_parameter('fps')

        if self.width.value is None:
            error_msg = 'Missing parameter width (video width)'
            self.get_logger().error(error_msg)
            raise Exception(error_msg)

        if self.height.value is None:
            error_msg = 'Missing parameter height (video height)'
            self.get_logger().error(error_msg)
            raise Exception(error_msg)

        if self.fps.value is None:
            error_msg = 'Missing parameter fps (video fps)'
            self.get_logger().error(error_msg)
            raise Exception(error_msg)

    def __init_cap(self):
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width.value)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height.value)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps.value)

    def __millis_callback(self, msg):
        with self.millis_lock:
            self.millis = msg.data

    def __video_reader(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            msg = SynchronizedImage()
            msg.image = self.br.cv2_to_imgmsg(frame)
            with self.millis_lock:
                msg.millis = self.millis
            self.publisher_.publish(msg)
            self.get_logger().debug('Published synchronized video frame')
            time.sleep(0.02)

def main():
    rclpy.init()
    node = CameraPublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()