cmake_minimum_required(VERSION 3.0.2)
project(tao_serial)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/tao_serial_node.py
  scripts/tao_moveit_bridge.py
  scripts/test_f407_v2_serial.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)