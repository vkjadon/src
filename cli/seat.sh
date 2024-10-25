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
