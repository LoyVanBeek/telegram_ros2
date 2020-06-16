"""
Copyright 2020 Loy van Beek.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the 'Software'), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
"""

import functools
from io import BytesIO

import cv2
from cv_bridge import CvBridge
from instant_messaging_interfaces.msg import Options
import numpy as np
from rcl_interfaces.msg import ParameterType
import rclpy
from rclpy.node import Node, ParameterDescriptor
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Header, String

from telegram import Location, ReplyKeyboardMarkup
from telegram.error import TimedOut
from telegram.ext import CommandHandler, Filters, MessageHandler, Updater


class TelegramBridge(Node):

    def __init__(self):
        super(TelegramBridge, self).__init__('telegram_bridge')

        self._cv_bridge = CvBridge()

        self._telegram_chat_id = None

        self.declare_parameter('caption_as_frame_id', False,
                               ParameterDescriptor(type=ParameterType.PARAMETER_BOOL,
                                                   description='When receiving a picture, '
                                                               'put the caption in the frame_id'))
        self._caption_as_frame_id = self.get_parameter_or('caption_as_frame_id', False).value

        self.declare_parameter('api_token', '',
                               ParameterDescriptor(type=ParameterType.PARAMETER_STRING,
                                                   description='Telegram API token. '
                                                               'Get your own via '
                                                               'https://t.me/botfather'))
        self._telegram_updater = Updater(token=self.get_parameter('api_token').value,
                                         use_context=True)
        dp = self._telegram_updater.dispatcher

        dp.add_error_handler(lambda _, update, error:
                             self.get_logger().error(
                                 'Update {} caused error {}'.format(update, error)))

        self.declare_parameter('whitelist', [],
                               ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER_ARRAY,
                                                   description='list of accepted chat IDs'))
        self.declare_parameter('blacklist', [],
                               ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER_ARRAY,
                                                   description='list of unaccepted chat IDs'))

        self.get_logger().info('Initial whitelist: {}'
                               .format(self.get_parameter('whitelist').value))
        self.get_logger().info('Initial blacklist: {}'
                               .format(self.get_parameter('blacklist').value))

        dp.add_handler(CommandHandler('start', self._telegram_start_callback))
        dp.add_handler(CommandHandler('stop', self._telegram_stop_callback))

        self._from_telegram_string_publisher = self.create_publisher(
            String, 'message_to_ros', 10)
        self._from_ros_string_subscriber = self.create_subscription(
            String, 'message_from_ros', self._ros_message_callback, 10)
        dp.add_handler(MessageHandler(Filters.text, self._telegram_message_callback))

        self._from_telegram_image_publisher = self.create_publisher(
            Image, 'image_to_ros', 10)
        self._from_ros_image_subscriber = self.create_subscription(
            Image, 'image_from_ros', self._ros_image_callback, 10)
        dp.add_handler(MessageHandler(Filters.photo, self._telegram_image_callback))

        self._from_telegram_location_publisher = self.create_publisher(
            NavSatFix, 'location_to_ros', 10)
        self._to_telegram_location_subscriber = self.create_subscription(
            NavSatFix, 'location_from_ros', self._ros_location_callback, 10)
        dp.add_handler(MessageHandler(Filters.location, self._telegram_location_callback))

        self._to_telegram_options_subscriber = self.create_subscription(
            Options, 'options_from_ros', self._ros_options_callback, 10)

    def start(self):
        self.get_logger().info('Start polling Telegram updater')
        self._telegram_updater.start_polling()
        self.get_logger().info('Started polling Telegram updater')

    def stop(self):
        self.get_logger().info('Stopping Telegram updater')
        self._telegram_updater.stop()
        self.get_logger().info('Stopped Telegram updater')

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.byebye()
        return self.stop()

    def telegram_callback(callback_function):
        """
        Decorate callbacks from the Telegram API to handle starting the session.

        :config callback_function: A callback function taking a telegram.Bot and a telegram.Update
        :return: Wrapped callback function
        """
        @functools.wraps(callback_function)
        def wrapper(self, update, context):
            self.get_logger().debug('Incoming update from telegram: {}'.format(update))

            if self.is_blacklisted(update.message.chat_id):
                self.get_logger().warn(
                    'Discarding message. User {} is blacklisted'.format(update.message.from_user))
                update.message.reply_text(
                    'You (chat id {}) are not authorized to chat with this bot'.format(
                        update.message.from_user['id']))
                return
            else:
                if self._telegram_chat_id is None:
                    self.get_logger().warn('Discarding message. No active chat_id.')
                    update.message.reply_text(
                        'ROS Bridge not initialized. Type /start to set-up ROS bridge')
                elif self._telegram_chat_id != update.message.chat_id:
                    self.get_logger().warn('Discarding message. Invalid chat_id')
                    update.message.reply_text(
                        'ROS Bridge initialized to another chat_id. '
                        'Type /start to connect to this chat_id')
                else:
                    return callback_function(self, update, context)

        return wrapper

    def ros_callback(callback_function):
        """
        Decorate callback called by ROS to verify whether we have an active chat_id.

        In case of exceptions, these will be logged.

        :config callback_function: A callback function taking a ros msg
        :return: Wrapped callback function
        """
        @functools.wraps(callback_function)
        def wrapper(self, msg):
            if not self._telegram_chat_id:
                self.get_logger().error('ROS Bridge not initialized, dropping message of type {}'
                                        .format(type(msg)))
            else:
                try:
                    callback_function(self, msg)
                except TimedOut as e:
                    self.get_logger().error('Telegram timeout: {}'.format(e))

        return wrapper

    def is_whitelisted(self, chat_id):
        """
        Check if the chat_id is whitelisted and allowed to send messages to us.

        :param chat_id:
        :return:
        """
        # If the whitelist is empty, it is disabled and anyone is allowed.
        whitelist = self.get_parameter_or('whitelist', []).value
        self.get_logger().debug('Whitelist: {}'.format(whitelist))
        if whitelist:
            whitelisted = chat_id in whitelist
            self.get_logger().debug(
                '{} in *white*list of length {}: {}'.format(chat_id, len(whitelist), whitelisted))
            return whitelisted
        else:
            return True

    def is_blacklisted(self, chat_id):
        """
        Check if the chat_id is blacklisted and blocks from sending messages to us.

        :param chat_id:
        :return:
        """
        self.get_logger().debug('Checking whether {} is blacklisted'.format(chat_id))
        # TODO: Getting default params or List params at all doesn't seem to
        # work in the way I expect at least. If the value is defined as [] in yaml,
        # still returns None here
        blacklist = self.get_parameter_or('blacklist', alternative_value=[]).value
        if blacklist:
            self.get_logger().debug('Blacklist: {}'.format(blacklist))
            blacklisted = chat_id in blacklist
            self.get_logger().debug(
                '{} in *black*list of length {}: {}'.format(chat_id, len(blacklist), blacklisted))
            return blacklisted
        return False

    def _telegram_start_callback(self, update, context):
        """
        Call when a telegram user sends the '/start' event to the bot.

        Using this event, the bridge can be connected to a specific conversation.

        :config update: Received update event that holds the chat_id and message data
        """
        if not self.is_whitelisted(update.message.chat_id) and \
                not self.is_blacklisted(update.message.chat_id):
            self.get_logger().warn('Discarding message. User {} not whitelisted'
                                   .format(update.message.from_user))
            update.message.reply_text('You (chat id {}) are not authorized to cha'
                                      't with this bot'.format(update.message.from_user['id']))
            return

        if self._telegram_chat_id is not None and self._telegram_chat_id != update.message.chat_id:
            self.get_logger().warn('Changing to different chat_id!')
            self._telegram_updater.bot.send_message(
                self._telegram_chat_id,
                'Lost ROS bridge connection to this chat_id (somebody took over)')
        self._telegram_chat_id = update.message.chat_id

        self.get_logger().info('Starting telegram ROS bridge for chat id {}'
                               .format(self._telegram_chat_id))
        update.message.reply_text(
            'Telegram ROS bridge initialized, only replying to chat_id {} (current). '
            'Type /stop to disconnect'.format(self._telegram_chat_id))

    @telegram_callback
    def _telegram_stop_callback(self, update, context):
        """
        Call when a telegram user sends the '/stop' event to the bot.

        Then, the user is disconnected from the bot and will no longer receive messages.

        :config update: Received update event that holds the chat_id and message data
        """
        self.get_logger().info('Stopping telegram ROS bridge for chat id {}'
                               .format(self._telegram_chat_id))
        update.message.reply_text('Disconnecting chat_id {}. So long and thanks for all the fish!'
                                  ' Type /start to reconnect'.format(self._telegram_chat_id))
        self._telegram_chat_id = None

    @telegram_callback
    def _telegram_message_callback(self, update, context):
        """
        Call when a new telegram message has been received.

        The message is then published into the ROS world

        :config update: Received update that holds the chat_id and message data
        """
        self.get_logger().info('Got a message: {}'.format(update))
        self._from_telegram_string_publisher.publish(String(data=update.message.text))

    @ros_callback
    def _ros_message_callback(self, msg):
        """
        Call when a new ROS String message should be sent to the Telegram session.

        :config msg: String message
        """
        self.get_logger().info(str(msg.data))
        self._telegram_updater.bot.send_message(self._telegram_chat_id, msg.data)

    @telegram_callback
    def _telegram_image_callback(self, update, context):
        """
        Call when a new telegram image has been received. This is then passed on to ROS.

        :config update: Received update that holds the chat_id and image data
        """
        self.get_logger().debug('Received image, downloading highest resolution image ...')
        byte_array = update.message.photo[-1].get_file().download_as_bytearray()
        self.get_logger().debug('Download complete, converting ...')

        img = cv2.imdecode(np.asarray(byte_array, dtype=np.uint8), cv2.IMREAD_COLOR)
        msg = self._cv_bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.get_logger().debug('Conversion complete, publishing ...')

        if self._caption_as_frame_id:
            msg.header.frame_id = update.message.caption
        self._from_telegram_image_publisher.publish(msg)

        if update.message.caption:
            self._from_telegram_string_publisher.publish(String(data=update.message.caption))

    @ros_callback
    def _ros_image_callback(self, msg):
        """
        Call when a new ROS String image should be sent to the telegram session.

        :config msg: String image
        """
        cv2_img = self._cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        caption = msg.header.frame_id if self._caption_as_frame_id else ''
        photo = BytesIO(cv2.imencode('.jpg', cv2_img)[1].tobytes())
        self._telegram_updater.bot.send_photo(self._telegram_chat_id,
                                              photo=photo,
                                              caption=caption)

    @telegram_callback
    def _telegram_location_callback(self, update, context):
        """
        Call when a new telegram Location is received.

        The Location will be published into the ROS world as a NavSatFix

        :config update: Received update that holds the chat_id and message data
        """
        self._from_telegram_location_publisher.publish(NavSatFix(
            header=Header(),
            latitude=update.message.location.latitude,
            longitude=update.message.location.longitude,
            position_covariance_type=NavSatFix.COVARIANCE_TYPE_UNKNOWN
        ))

    @ros_callback
    def _ros_location_callback(self, msg):
        """
        Call when a new ROS NavSatFix message should be sent to the telegram session.

        :config msg: NavSatFix that the robot wants to share
        """
        self._telegram_updater.bot.send_location(self._telegram_chat_id,
                                                 location=Location(msg.longitude, msg.latitude))

    @ros_callback
    def _ros_options_callback(self, msg):
        """
        Call when a new ROS Options message should be sent to the telegram session.

        :config msg: Options that the robot wants to share
        """
        def chunks(l, n):
            """Yield successive n-sized chunks from l."""
            for i in range(0, len(l), n):
                yield l[i:i + n]

        options_keyboard = ReplyKeyboardMarkup(keyboard=list(chunks(msg.options, 5)),
                                               resize_keyboard=True,
                                               one_time_keyboard=True)
        self._telegram_updater.bot.send_message(self._telegram_chat_id,
                                                text=msg.question,
                                                reply_markup=options_keyboard)

    def byebye(self):
        self.get_logger().info('Saying goodbye. Byebye')
        self._telegram_updater.bot.send_message(
            self._telegram_chat_id,
            'Byebye, I\'m shutting down. Don\'t forget to /start me again later')


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
