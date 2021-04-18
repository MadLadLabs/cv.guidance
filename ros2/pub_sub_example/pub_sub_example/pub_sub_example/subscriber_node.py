import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class Subscriber(Node):

    def __init__(self):
        super().__init__('subscriber')
        self.publisher_ = self.create_subscription(String, 'my_topic', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')

def main():
    rclpy.init()
    subscriber = Subscriber()

    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()