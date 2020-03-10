import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class TelegramBridge(Node):
	def __init__(self):
		super().__init__('telegram_bridge')
		self.pub_to_ros = self.create_publisher(String, 'message_from_telegram', 10)
		self.sub_from_ros = self.create_subscription(String, 'message_to_telegram', self.handle_ros_message, 10)

	def handle_ros_message(self, msg):
		self.get_logger().info(str(msg.data))


def main(args=None):
    rclpy.init(args=args)

    bridge = TelegramBridge()
    rclpy.spin(bridge)

    bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
