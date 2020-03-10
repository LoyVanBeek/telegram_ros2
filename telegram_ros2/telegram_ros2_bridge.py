import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from telegram.ext import Updater


class TelegramBridge(Node):
    def __init__(self):
        super(TelegramBridge, self).__init__('telegram_bridge')
        self.pub_to_ros = self.create_publisher(String, 'message_to_ros', 10)
        self.sub_from_ros = self.create_subscription(String, 'message_from_ros', self.handle_ros_message, 10)

        self.declare_parameter("api_token")
        self._telegram_updater = Updater(token=self.get_parameter("api_token").value,
                                         use_context=True)

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
