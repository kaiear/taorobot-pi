cmake_minimum_required(VERSION 3.0.2)
project(tao_tasks)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/line_follow_controller.py
  scripts/face_greeting_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)