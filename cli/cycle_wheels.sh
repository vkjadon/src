
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{'width': 3, 'off':True}"

ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{'x':3, 'y':2}"

ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{'r':250, 'g':250, 'b':250, 'width': 3, 'off':False}"

#Publish on /turtle1/cmd_vel Topic using Twist Message Type

ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{'linear':{'x':6.28}, 'angular':{'z':6.28}}" --once

sleep 3.0

ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{'width': 3, 'off':True}"

ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{'x':8, 'y':2}"

ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{'r':250, 'g':250, 'b':250, 'width': 3, 'off':False}"

ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{'linear':{'x':6.28}, 'angular':{'z':6.28}}" --once






