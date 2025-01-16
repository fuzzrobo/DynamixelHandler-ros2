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

# 以下の部分は自作msgの定義のために必要
set(msg_files
  "msg/DxlStates.msg"
  "msg/DxlCommandsX.msg"
  "msg/DxlCommandsP.msg"
  "msg/DxlCommandsAll.msg"

  "msg/DynamixelStatus.msg"
  "msg/DynamixelPresent.msg"
  "msg/DynamixelGoal.msg"
  "msg/DynamixelGain.msg"
  "msg/DynamixelLimit.msg"
  "msg/DynamixelError.msg"
  "msg/DynamixelExtra.msg"
  "msg/DynamixelExtraDrivemode.msg"
  "msg/DynamixelExtraShutdown.msg"
  "msg/DynamixelExtraRestureconfig.msg"
  "msg/DynamixelExtraMovingstatus.msg"
  "msg/DynamixelExtraLed.msg"

  "msg/DynamixelDebug.msg"
  "msg/DynamixelDebugElement.msg"
  "msg/DynamixelShortcut.msg"
  
  "msg/DynamixelControlXPwm.msg"
  "msg/DynamixelControlXCurrent.msg"
  "msg/DynamixelControlXVelocity.msg"
  "msg/DynamixelControlXPosition.msg"
  "msg/DynamixelControlXExtendedPosition.msg"
  "msg/DynamixelControlXCurrentBasePosition.msg"
  "msg/DynamixelControlPPwm.msg"
  "msg/DynamixelControlPCurrent.msg"
  "msg/DynamixelControlPVelocity.msg"
  "msg/DynamixelControlPPosition.msg"
  "msg/DynamixelControlPExtendedPosition.msg"

  "msg/DxlExternalPort.msg"
)
set(srv_files

)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# インクルードディレクトリの設定
include_directories(
  include/mylib_dynamixel/src
)

# ===============================dynamixel_unify_baudrate_node===============================

# =============================== dynamixel_unify_baudrate_node =========================
add_executable(
  dynamixel_unify_baudrate_node 
	include/mylib_dynamixel/download/port_handler_linux_fix.cpp
	include/mylib_dynamixel/download/port_handler_fix.cpp
	include/mylib_dynamixel/src/dynamixel_communicator_main.cpp
	include/mylib_dynamixel/src/dynamixel_communicator_retry.cpp
  
	src/sub_node/dynamixel_unify_baudrate.cpp
)
target_include_directories(
  dynamixel_unify_baudrate_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )
target_compile_features(
  dynamixel_unify_baudrate_node 
  PUBLIC c_std_99 cxx_std_17
)

ament_target_dependencies(
	dynamixel_unify_baudrate_node
  rclcpp
  std_msgs
  rosidl_default_runtime
)

install(
  TARGETS dynamixel_unify_baudrate_node
  DESTINATION lib/${PROJECT_NAME}
  )

# 以下の部分は同じPkg内でmsgを定義＆使用を行う場合にい必要な追記
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(dynamixel_unify_baudrate_node "${cpp_typesupport_target}")

# ============================ dynamixel_handler_node ================================

# 実行可能ファイルの名前を変更
add_executable(
  dynamixel_handler_node 
	include/mylib_dynamixel/download/port_handler_linux_fix.cpp
	include/mylib_dynamixel/download/port_handler_fix.cpp
	include/mylib_dynamixel/src/dynamixel_communicator_main.cpp
	include/mylib_dynamixel/src/dynamixel_communicator_retry.cpp
  
	src/dynamixel_handler_ros_interface_sub.cpp
	src/dynamixel_handler_ros_interface_pub.cpp
	src/dynamixel_handler_dyn_interface_loop.cpp
	src/dynamixel_handler_dyn_interface_once.cpp

  src/optional_function/external_port.cpp

	src/main.cpp
)
target_include_directories(
  dynamixel_handler_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )
target_compile_features(
  dynamixel_handler_node 
  PUBLIC c_std_99 cxx_std_17
)

ament_target_dependencies(
	dynamixel_handler_node
  rclcpp
  std_msgs
  rosidl_default_runtime
)

install(
  TARGETS dynamixel_handler_node
  DESTINATION lib/${PROJECT_NAME}
)

# 以下の部分は同じPkg内でmsgを定義＆使用を行う場合にい必要な追記
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(dynamixel_handler_node "${cpp_typesupport_target}")
# ==================================================================================


install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

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
