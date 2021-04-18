import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class Publisher(Node):

    def __init__(self):
        super().__init__('publisher')
        self.declare_parameter('image_path', None)
        self.publisher_ = self.create_publisher(Image, 'my_topic', 10)
        publish_period = 1
        image_path = self.get_parameter('image_path')
        while image_path.value is None:
            self.get_logger().info('"image_path" parameter not set. Waiting 1 second for the parameter to be set')
            time.sleep(1)
            image_path = self.get_parameter('image_path')
        self.get_logger().info(f'"image_path" parameter value: "{image_path.value}"')
        self.image = cv2.imread(image_path.value,0)
        self.timer = self.create_timer(publish_period, self.timer_callback)
        self.br = CvBridge()

    def timer_callback(self):
        self.publisher_.publish(self.br.cv2_to_imgmsg(self.image))
        self.get_logger().info('Publishing image')

def main():
    rclpy.init()
    publisher = Publisher()

    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()