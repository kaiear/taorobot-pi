cmake_minimum_required(VERSION 3.0.2)
project(vision_sorter)

add_compile_options(-std=c++17 -O2)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

set(VISION_CORE_SOURCES
  src/args.cpp
  src/vision.cpp
  src/face_detector.cpp
)

set(MISSION_SOURCES
  src/arm_kinematics.cpp
  src/sorter_controller.cpp
  src/ros1_robot_control.cpp
)

add_executable(line_node src/ros1_line_node.cpp ${VISION_CORE_SOURCES})
target_link_libraries(line_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(object_node src/ros1_object_node.cpp ${VISION_CORE_SOURCES})
target_link_libraries(object_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(face_node src/ros1_face_node.cpp ${VISION_CORE_SOURCES})
target_link_libraries(face_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(mission_node
  src/ros1_mission_node.cpp
  ${VISION_CORE_SOURCES}
  ${MISSION_SOURCES}
)
target_link_libraries(mission_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS line_node object_node face_node mission_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
