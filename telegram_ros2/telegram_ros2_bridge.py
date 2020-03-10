import functools

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from telegram import Location, ReplyKeyboardMarkup, Bot
from telegram.error import TimedOut
from telegram.ext import Updater, CommandHandler, MessageHandler, Filters


class TelegramBridge(Node):
    def __init__(self):
        super(TelegramBridge, self).__init__('telegram_bridge')
        self._from_telegram_string_publisher = self.create_publisher(String, 'message_to_ros', 10)
        self.sub_from_ros = self.create_subscription(String, 'message_from_ros', self.handle_ros_message, 10)

        self._telegram_chat_id = None

        self.declare_parameter("api_token")
        self._telegram_updater = Updater(token=self.get_parameter("api_token").value, use_context=True)
        self._telegram_updater.dispatcher.add_error_handler(
            lambda _, update, error: self.get_logger().error("Update {} caused error {}".format(update, error)))

        self._telegram_updater.dispatcher.add_handler(CommandHandler("start", self._telegram_start_callback))
        self._telegram_updater.dispatcher.add_handler(CommandHandler("stop", self._telegram_stop_callback))

        self._telegram_updater.dispatcher.add_handler(MessageHandler(Filters.text, self._telegram_message_callback))

    def start(self):
        self._telegram_updater.start_polling()
        self.get_logger().debug("Started polling Telegram updater")

    def stop(self):
        self.get_logger().debug("Stopping Telegram updater")
        self._telegram_updater.stop()
        self.get_logger().debug("Stopped Telegram updater")

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc_val, exc_tb):
        return self.stop()

    def handle_ros_message(self, msg):
        self.get_logger().info(str(msg.data))

    def telegram_callback(callback_function):
        """
        Decorator to restrict telegram methods only to the active chat or tell to /start one if needed
        :param callback_function: A callback function taking a telegram.Bot and a telegram.Update
        :return: Wrapped callback function
        """

        @functools.wraps(callback_function)
        def wrapper(self, update, context):
            self.get_logger().debug("Incoming update from telegram: {}".format(update))
            if self._telegram_chat_id is None:
                self.get_logger().warn("Discarding message. No active chat_id.")
                update.message.reply_text("ROS Bridge not initialized. Type /start to set-up ROS bridge")
            elif self._telegram_chat_id != update.message.chat_id:
                self.get_logger().warn("Discarding message. Invalid chat_id")
                update.message.reply_text(
                    "ROS Bridge initialized to another chat_id. Type /start to connect to this chat_id")
            else:
                return callback_function(self, update, context)

        return wrapper

    def _telegram_start_callback(self, update, context):
        """
        Called when a telegram user sends the '/start' event to the bot, using this event, the bridge can be connected
        to a specific conversation
        :param update: Received update event that holds the chat_id and message data
        """
        if self._telegram_chat_id is not None and self._telegram_chat_id != update.message.chat_id:
            self.get_logger().warn("Changing to different chat_id!")
            self._telegram_updater.bot.send_message(self._telegram_chat_id,
                                                    "Lost ROS bridge connection to this chat_id (somebody took over)")

        self._telegram_chat_id = update.message.chat_id

        self.get_logger().info("Starting telegram ROS bridge for chat id {}".format(self._telegram_chat_id))
        update.message.reply_text(
            "Telegram ROS bridge initialized, only replying to chat_id {} (current). Type /stop to disconnect".format(self._telegram_chat_id))

    @telegram_callback
    def _telegram_stop_callback(self, update, context):
        """
        Called when a telegram user sends the '/stop' event to the bot. Then, the user is disconnected from the bot and
        will no longer receive messages.
        :param update: Received update event that holds the chat_id and message data
        """

        self.get_logger().info("Stopping telegram ROS bridge for chat id {}".format(self._telegram_chat_id))
        update.message.reply_text("Disconnecting chat_id {}. So long and thanks for all the fish!"
                                  " Type /start to reconnect".format(self._telegram_chat_id))
        self._telegram_chat_id = None


    @telegram_callback
    def _telegram_message_callback(self, update, context):
        """
        Called when a new telegram message has been received. The method will verify whether the incoming message is
        from the bridges telegram conversation by comparing the chat_id.
        :param update: Received update that holds the chat_id and message data
        """
        self._from_telegram_string_publisher.publish(String(data=update.message.text))


def main(args=None):
    rclpy.init(args=args)

    bridge = TelegramBridge()
    with bridge:
        while rclpy.ok():
            try:
                rclpy.spin_once(bridge, timeout_sec=0.1)
            except KeyboardInterrupt:
                break

    bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
