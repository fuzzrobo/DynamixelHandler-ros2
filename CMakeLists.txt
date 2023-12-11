cmake_minimum_required(VERSION 3.8)
project(dynamixel_handler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED) # 自作msgの作成のために必要
find_package(std_msgs REQUIRED) # 自作msgのHeaderのために必要

# 以下の部分は自作ｍｓｇの定義のために必要
set(msg_files
#   "msg/DynamixelState.msg"
  "msg/DynamixelInfo.msg"
  "msg/DynamixelCmd.msg"
)
set(srv_files
# rebootとかはこっちかな？
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)
## ここまで

# ノードの定義
add_executable(dynamixel_handler_node 
	src/download/port_handler_linux.cpp
	src/download/port_handler.cpp
	src/dynamixel_parameters.cpp
	src/dynamixel_serialport.cpp
	src/dynamixel_handler_ROS2.cpp
	src/main.cpp
)
target_include_directories(dynamixel_handler_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dynamixel_handler_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
	dynamixel_handler_node
  "rclcpp"
  "rosidl_default_generators"
)

install(TARGETS dynamixel_handler_node
  DESTINATION lib/${PROJECT_NAME})
## ここまで

# 以下の部分は同じPkg内でmsgを定義＆使用を行う場合にい必要な追記
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(dynamixel_handler_node "${cpp_typesupport_target}")
## ここまで

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
