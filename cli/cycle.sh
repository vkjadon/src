# Frame

ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{'width': 5, 'off':True}"

ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{'x':3.5, 'y':6.0}"

ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{'r': 255, 'g':255, 'b':255, 'width': 3, 'off':False}"

ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{'x':3.0, 'y':3.0}"

ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{'r':150, 'g':100, 'b':40, 'width': 3}"

ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{'x':5.5, 'y':5.0}"

ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{'x':8.0, 'y':3.0}"

#Handle

ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{'width': 5, 'off':True}"

ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{'x':3.7, 'y':6.1}"

ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{'r': 150, 'g':100, 'b':40, 'width': 3, 'off':False}"

ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{'x':3.3, 'y':5.9}"

#Seat

ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{'width': 5, 'off':True}"

ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{'x':5.0, 'y':5.0}"

ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{'r':255, 'g':0, 'b':220, 'width': 3, 'off':False}"

ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{'x':6.0, 'y':5.0}"

ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{'r':255, 'g':250, 'b': 200, 'width': 3, 'off':False}"

ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{'x':6.0, 'y':5.4}"

ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{'r':255, 'g':200, 'b': 0, 'width': 3, 'off':False}"

ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{'x':5.0, 'y':5.4}"

ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{'r':255, 'g':250, 'b': 200, 'width': 3, 'off':False}"

ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{'x':5.0, 'y':5.0}"

#Wheels

ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{'width': 3, 'off':True}"

ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{'x':3, 'y':2}"

ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{'r':250, 'g': 250, 'b':250, 'width': 3, 'off':False}"

#Publish on /turtle1/cmd_vel Topic using Twist Message Type

ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{'linear':{'x':6.28}, 'angular':{'z':6.28}}" --once

sleep 0.5

ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{'width': 3, 'off':True}"

ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{'x':8, 'y':2}"

ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{'r':250, 'g': 250, 'b':250, 'width': 3, 'off':False}"

ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{'linear':{'x':6.28}, 'angular':{'z':6.28}}" --once

sleep 0.5

ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{'off':True}"

ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{'x':5.5, 'y':5.5}"
