cmake_minimum_required(VERSION 3.8)
project(custom_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(turtlesim REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "msg/Sphere.msg"
  "msg/GoToGoal.msg"
  "srv/AddThreeInts.srv"
  "action/CruiseSpeed.action"
  "action/CountAction.action"
  DEPENDENCIES 
    geometry_msgs 
    # turtlesim
    # std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
