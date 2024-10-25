
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{'width': 5, 'off':True}"

ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{'x':3.7, 'y':6.1}"

ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{'r': 150, 'g':100, 'b':40, 'width': 3, 'off':False}"

ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{'x':3.3, 'y':5.9}"

