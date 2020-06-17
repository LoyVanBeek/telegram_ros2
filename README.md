![ROS2 Lint](https://github.com/LoyVanBeek/telegram_ros2/workflows/ROS2%20Lint/badge.svg) 

# telegram_ros2
Bridge the Telegram chat service to ROS2. Send messages, images, locations to and from ROS2

## Useage
```bash
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
git clone https://github.com/LoyVanBeek/instant_messaging_interfaces.git
git clone https://github.com/LoyVanBeek/telegram_ros2.git
```

Get a Telegram API token via https://core.telegram.org/bots#6-botfather 
The token can be put in the [`config/example_param.launch`](https://github.com/LoyVanBeek/telegram_ros2/blob/master/config/example_param.yaml):
```bash
$EDITOR config/example_param.launch
```
and save your token instead of the dummy example token (nope, the example one is not valid, you really need your own)

```bash
cd ~/dev_ws  # or cd .., whatever floats your goat. 
colcon build  # --symlink-install # TIL this is possible, for those wanting to hack on this
ros2 launch telegram_ros2 telegram_bridge.launch.py
```

Now, chat with your bot via Telegram. 
It'll report you need to first send a `/start` command (ince this bot can currently only talk to 1 person at a time)
So, enter `/start` and start chatting. 

## Topics
- Receiving messages from Telegram: `ros2 topic echo /message_to_ros`
- Receiving images from Telegram: `ros2 topic echo /image_to_ros` or rather look at images with `rqt`
- Receiving location from Telegram: `ros2 topic echo /location_to_ros`

- Sending a message to Telegram: `ros2 topic pub --once /message_from_ros std_msgs/msg/String data:\ \'Hello\'\`
- Sending a location to Telegram: `ros2 topic pub --once  /location_from_ros sensor_msgs/msg/NavSatFix` or also use `rqt`
- Images are sent to Telegram via publishing them to `/image_from_ros`, so you can remap your camera feed for example. 

