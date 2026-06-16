#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

COMMON_SOURCES=(
  "${SCRIPT_DIR}/src/args.cpp"
  "${SCRIPT_DIR}/src/robot_serial.cpp"
  "${SCRIPT_DIR}/src/arm_kinematics.cpp"
  "${SCRIPT_DIR}/src/face_detector.cpp"
  "${SCRIPT_DIR}/src/vision.cpp"
  "${SCRIPT_DIR}/src/apriltag_detector.cpp"
  "${SCRIPT_DIR}/src/sorter_controller.cpp"
)

g++ -std=c++17 -O2 -I"${SCRIPT_DIR}/include" "${SCRIPT_DIR}/src/main.cpp" "${COMMON_SOURCES[@]}" -o "${SCRIPT_DIR}/vision_sorter" \
  $(pkg-config --cflags --libs opencv4)

g++ -std=c++17 -O2 -I"${SCRIPT_DIR}/include" "${SCRIPT_DIR}/src/mission_node.cpp" "${COMMON_SOURCES[@]}" -o "${SCRIPT_DIR}/mission_node" \
  $(pkg-config --cflags --libs opencv4)

g++ -std=c++17 -O2 -I"${SCRIPT_DIR}/include" "${SCRIPT_DIR}/src/line_node.cpp" "${SCRIPT_DIR}/src/args.cpp" "${SCRIPT_DIR}/src/vision.cpp" -o "${SCRIPT_DIR}/line_node" \
  $(pkg-config --cflags --libs opencv4)

g++ -std=c++17 -O2 -I"${SCRIPT_DIR}/include" "${SCRIPT_DIR}/src/object_node.cpp" "${SCRIPT_DIR}/src/args.cpp" "${SCRIPT_DIR}/src/vision.cpp" -o "${SCRIPT_DIR}/object_node" \
  $(pkg-config --cflags --libs opencv4)

g++ -std=c++17 -O2 -I"${SCRIPT_DIR}/include" "${SCRIPT_DIR}/src/face_node.cpp" "${SCRIPT_DIR}/src/args.cpp" "${SCRIPT_DIR}/src/vision.cpp" "${SCRIPT_DIR}/src/face_detector.cpp" -o "${SCRIPT_DIR}/face_node" \
  $(pkg-config --cflags --libs opencv4)

if pkg-config --exists apriltag; then
  g++ -std=c++17 -O2 -I"${SCRIPT_DIR}/include" -DHAVE_APRILTAG "${SCRIPT_DIR}/src/main.cpp" "${COMMON_SOURCES[@]}" -o "${SCRIPT_DIR}/vision_sorter_apriltag" \
    $(pkg-config --cflags --libs opencv4 apriltag)
  g++ -std=c++17 -O2 -I"${SCRIPT_DIR}/include" -DHAVE_APRILTAG "${SCRIPT_DIR}/src/mission_node.cpp" "${COMMON_SOURCES[@]}" -o "${SCRIPT_DIR}/mission_node_apriltag" \
    $(pkg-config --cflags --libs opencv4 apriltag)
fi

g++ -std=c++17 -O2 -I"${SCRIPT_DIR}/include" "${SCRIPT_DIR}/src/serial_link_test.cpp" "${SCRIPT_DIR}/src/robot_serial.cpp" -o "${SCRIPT_DIR}/serial_link_test"

echo "Built:"
echo "  ${SCRIPT_DIR}/vision_sorter"
echo "  ${SCRIPT_DIR}/mission_node"
echo "  ${SCRIPT_DIR}/line_node"
echo "  ${SCRIPT_DIR}/object_node"
echo "  ${SCRIPT_DIR}/face_node"
echo "  ${SCRIPT_DIR}/serial_link_test"
if [[ -x "${SCRIPT_DIR}/vision_sorter_apriltag" ]]; then
  echo "  ${SCRIPT_DIR}/vision_sorter_apriltag"
fi
if [[ -x "${SCRIPT_DIR}/mission_node_apriltag" ]]; then
  echo "  ${SCRIPT_DIR}/mission_node_apriltag"
fi
