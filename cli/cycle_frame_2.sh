
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{'width': 5, 'off':True}"

ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{'x':3.5, 'y':6.0}"

ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{'r': 255, 'g':255, 'b':255, 'width': 3, 'off':False}"

ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{'x':3.0, 'y':3.0}"

ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{'r':150, 'g':100, 'b':40, 'width': 3}"

ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{'x':5.5, 'y':5.0}"

ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{'x':8.0, 'y':3.0}"
