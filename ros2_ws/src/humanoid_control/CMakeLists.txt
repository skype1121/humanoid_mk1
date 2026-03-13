cmake_minimum_required(VERSION 3.8)
project(humanoid_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/DriverCommand.msg"
  "msg/MotorCommand.msg"
  "msg/MotorCommandArray.msg"
  "msg/MotorState.msg"
  "msg/MotorStateArray.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces
)

add_executable(control_loop_node src/control_loop_node.cpp)
ament_target_dependencies(control_loop_node rclcpp std_msgs)

install(TARGETS
  control_loop_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/ak70_can_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ak70_can_node
)

install(PROGRAMS
  scripts/ak70_driver_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ak70_driver_node
)

install(PROGRAMS
  scripts/ak70_command_test_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ak70_command_test_node
)

install(PROGRAMS
  scripts/ak70_control_panel.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ak70_control_panel
)

install(DIRECTORY
  scripts/humanoid_control_py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
