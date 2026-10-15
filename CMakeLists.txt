cmake_minimum_required(VERSION 3.0.2)
project(teleop_joy_ctrl)

find_package(catkin REQUIRED COMPONENTS
  dobot_v4_bringup
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/arm_bringup_guard_node.py
  scripts/light_serial_controller_node.py
  scripts/restore_system_teleop_on_shutdown.py
  scripts/teleop_joy_ctrl_node.py
  scripts/video_recording_manager_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY scripts/teleop_joy_ctrl/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts/teleop_joy_ctrl
)