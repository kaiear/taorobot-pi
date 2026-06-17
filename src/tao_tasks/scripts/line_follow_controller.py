#!/usr/bin/env python3
"""Integrated OpenCV line-follow controller for standalone map testing.

This node keeps the existing Python/ROS base communication path: it publishes
``geometry_msgs/Twist`` to ``/cmd_vel`` and lets the already working serial node
talk to the chassis.

When ``~line_follow/use_camera`` is true, this file directly opens the camera and
ports the line-follow/intersection logic from ``vision_sorter/src``:

* HSV black-line thresholding.
* Three weighted ROIs from ``VisionConfig``.
* C++ style crossing state machine: ROI2 arms the cross, ROI1 counts it.
* Pick/place crosses such as 2/5/9 enter the C++ style color block state machine.

When ``use_camera`` is false, the previous topic-driven controller behavior is
kept for compatibility with ``vision_sorter/line_node``.
"""

import math
import sys
import time

import cv2
import numpy as np
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, Int16MultiArray, String, UInt8
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

try:
    import moveit_commander
except ImportError:
    moveit_commander = None


K_PI = math.pi
BLOCK_NONE = "none"
BLOCK_RED = "red"
BLOCK_GREEN = "green"
BLOCK_BLUE = "blue"
ARM_PHASE_IDLE = "idle"
ARM_PHASE_PICK = "pick"
ARM_PHASE_PLACE = "place"


def clamp(value, low, high):
    return max(low, min(high, value))


def private_param(name, default):
    """Read either ~name or grouped ~line_follow/name from rosparam YAML."""
    if rospy.has_param("~" + name):
        return rospy.get_param("~" + name)
    return rospy.get_param("~line_follow/" + name, default)


def as_float_list(value, default):
    if value is None:
        return list(default)
    return [float(v) for v in value]


def as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def color_name(value):
    if value in (BLOCK_RED, BLOCK_GREEN, BLOCK_BLUE):
        return value
    return BLOCK_NONE


def contour_angle_rad(contour):
    """Python port of vision_sorter::contourAngleRad."""
    if contour is None or len(contour) < 4:
        return 0.0
    rect = cv2.minAreaRect(contour)
    angle = float(rect[2])
    if angle < -45.0:
        angle += 90.0
    elif angle > 45.0:
        angle -= 90.0
    return -angle * K_PI / 180.0


class ArmKinematics:
    """Small Python port of vision_sorter/src/arm_kinematics.cpp."""

    def __init__(self):
        self.joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 500.0]

    def move(self, x, y, z, move_time_ms):
        if y < 0.0:
            return None
        best_alpha = None
        for alpha in range(0, -136, -1):
            if self.analysis(x, y, z, float(alpha)) == 0:
                best_alpha = float(alpha)
        if best_alpha is None:
            return None
        self.analysis(x, y, z, best_alpha)
        self.joints[6] = float(move_time_ms)
        return list(self.joints)

    def claw(self, spin_claw, hand, move_time_ms):
        self.joints[4] = float(spin_claw)
        self.joints[5] = float(hand)
        self.joints[6] = float(move_time_ms)
        return list(self.joints)

    def analysis(self, x, y, z, alpha):
        x *= 10.0
        y *= 10.0
        z *= 10.0
        l0 = 2100.0
        l1 = 1250.0
        l2 = 1200.0
        l3 = 1550.0

        theta6 = 0.0 if x == 0.0 else math.atan(x / y) * 270.0 / K_PI
        y = math.sqrt(x * x + y * y)
        y = y - l3 * math.cos(alpha * K_PI / 180.0)
        z = z - l0 - l3 * math.sin(alpha * K_PI / 180.0)
        if z < -l0:
            return 1
        if math.sqrt(y * y + z * z) > (l1 + l2):
            return 2
        radius = math.sqrt(y * y + z * z)
        if radius <= 0.0:
            return 8
        ccc = math.acos(clamp(y / radius, -1.0, 1.0))
        bbb = (y * y + z * z + l1 * l1 - l2 * l2) / (2.0 * l1 * radius)
        if bbb > 1.0 or bbb < -1.0:
            return 5
        zf_flag = -1.0 if z < 0.0 else 1.0
        theta5 = (ccc * zf_flag + math.acos(bbb)) * 180.0 / K_PI
        if theta5 > 180.0 or theta5 < 0.0:
            return 6
        aaa = -(y * y + z * z - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
        if aaa > 1.0 or aaa < -1.0:
            return 3
        theta4 = 180.0 - math.acos(aaa) * 180.0 / K_PI
        if theta4 > 135.0 or theta4 < -135.0:
            return 4
        theta3 = alpha - theta5 + theta4
        if theta3 > 90.0 or theta3 < -90.0:
            return 7
        self.joints[0] = -theta6 * K_PI / 180.0
        self.joints[1] = -(theta5 - 90.0) * K_PI / 180.0
        self.joints[2] = theta4 * K_PI / 180.0
        self.joints[3] = -theta3 * K_PI / 180.0
        return 0


class MoveItArmBackend:
    """Plan arm motion with MoveIt first, then fall back to the direct serial path.

    This intentionally keeps the sorter state machine unchanged: the chassis
    still uses cmd_vel, cross-count logic still decides when to pick/place, and
    the gripper is still controlled by the C++-compatible serial command path.

    The old C++ coordinate convention is first converted to a conservative
    5-joint target by ArmKinematics.  When MoveIt is available, MoveGroup plans
    and executes a legal trajectory to that joint target, so joint limits and
    dead-zone avoidance come from the MoveIt URDF/SRDF configuration.  The real
    hardware trajectory is accepted by tao_moveit_bridge, which converts it to
    ARM_JOINTS serial commands for the STM32 side.  If MoveIt planning/execution
    fails, the caller receives the original direct joints so publish_arm_command
    can keep the existing serial chain working.
    """

    def __init__(self, fallback_arm):
        self.fallback_arm = fallback_arm
        self.enabled = as_bool(private_param("use_moveit_backend", False))
        self.use_commander = as_bool(private_param("moveit_use_commander", True))
        self.group_name = private_param("moveit_group_name", "arm")
        self.action_name = private_param("moveit_action_name", "arm_controller/follow_joint_trajectory")
        self.joint_names = list(private_param("moveit_arm_joint_names", [
            "arm_0_joint", "arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint"
        ]))
        self.planning_time = float(private_param("moveit_planning_time", 3.0))
        self.num_planning_attempts = int(private_param("moveit_num_planning_attempts", 5))
        self.max_velocity_scaling = float(private_param("moveit_max_velocity_scaling", 0.35))
        self.max_acceleration_scaling = float(private_param("moveit_max_acceleration_scaling", 0.35))
        self.wait_timeout = float(private_param("moveit_wait_timeout", 2.0))
        self.result_timeout_padding = float(private_param("moveit_result_timeout_padding", 1.0))
        self.use_fallback_on_failure = as_bool(private_param("moveit_fallback_on_failure", True))
        self.last_joints = None
        self.group = None
        self.client = None

        if self.enabled:
            if self.use_commander and moveit_commander is not None:
                try:
                    moveit_commander.roscpp_initialize(sys.argv)
                    self.group = moveit_commander.MoveGroupCommander(self.group_name)
                    self.group.set_planning_time(self.planning_time)
                    self.group.set_num_planning_attempts(self.num_planning_attempts)
                    self.group.set_max_velocity_scaling_factor(clamp(self.max_velocity_scaling, 0.01, 1.0))
                    self.group.set_max_acceleration_scaling_factor(clamp(self.max_acceleration_scaling, 0.01, 1.0))
                    rospy.loginfo("MoveIt commander backend ready: group=%s", self.group_name)
                except Exception as exc:
                    rospy.logerr("MoveIt commander init failed: %s", exc)
                    self.group = None
            elif self.use_commander:
                rospy.logerr("moveit_commander is not available; cannot use MoveIt planning backend")

            # Legacy safety net: if MoveGroup is unavailable, keep the previous
            # action-bridge behavior.  It is not a planner, but it still lets the
            # existing tao_moveit_bridge serial path run when requested.
            if self.group is None:
                self.client = actionlib.SimpleActionClient(self.action_name, FollowJointTrajectoryAction)
                if not self.client.wait_for_server(rospy.Duration(self.wait_timeout)):
                    rospy.logerr("MoveIt arm action server not available: %s", self.action_name)
                    self.client = None

            if self.group is None and self.client is None:
                self.enabled = False
            else:
                rospy.loginfo("MoveIt arm backend connected: group=%s action=%s", self.group_name, self.action_name)

    def move(self, x, y, z, move_time_ms):
        fallback_joints = self.fallback_arm.move(x, y, z, move_time_ms)
        if not self.enabled or (self.group is None and self.client is None):
            return fallback_joints
        if fallback_joints is None or len(fallback_joints) < 5:
            return None
        target = list(fallback_joints[:5])
        if self.plan_and_execute(target) or self.send_trajectory(target, move_time_ms):
            self.last_joints = target
            return []
        return fallback_joints if self.use_fallback_on_failure else None

    def claw(self, spin_claw, hand, move_time_ms):
        # Keep the gripper on the direct serial chain.  The hand group in SRDF is
        # useful for visualization, but pick/place timing only needs open/close
        # commands and should not depend on arm trajectory planning success.
        return self.fallback_arm.claw(spin_claw, hand, move_time_ms)

    def plan_and_execute(self, positions):
        if self.group is None:
            return False
        if len(positions) != len(self.joint_names):
            rospy.logerr("MoveIt joint target length mismatch: positions=%d joints=%d", len(positions), len(self.joint_names))
            return False
        try:
            self.group.set_start_state_to_current_state()
            self.group.set_joint_value_target({name: float(value) for name, value in zip(self.joint_names, positions)})
            ok = bool(self.group.go(wait=True))
            self.group.stop()
            self.group.clear_pose_targets()
            if not ok:
                rospy.logwarn("MoveIt commander failed to plan/execute target=%s", [round(v, 3) for v in positions])
            return ok
        except Exception as exc:
            rospy.logwarn("MoveIt commander exception: %s", exc)
            try:
                self.group.stop()
                self.group.clear_pose_targets()
            except Exception:
                pass
            return False

    def send_trajectory(self, positions, move_time_ms):
        if self.client is None:
            return False
        if len(positions) != len(self.joint_names):
            rospy.logerr("MoveIt target length mismatch: positions=%d joints=%d", len(positions), len(self.joint_names))
            return False
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = self.joint_names
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.05)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]
        point.time_from_start = rospy.Duration(max(float(move_time_ms) / 1000.0, 0.08))
        goal.trajectory.points = [point]
        self.client.send_goal(goal)
        timeout = point.time_from_start + rospy.Duration(self.result_timeout_padding)
        if not self.client.wait_for_result(timeout):
            self.client.cancel_goal()
            rospy.logwarn("MoveIt arm trajectory timed out")
            return False
        state = self.client.get_state()
        if state != GoalStatus.SUCCEEDED:
            rospy.logwarn("MoveIt arm trajectory failed state=%s", state)
            return False
        return True


class IntegratedLineDetector:
    """Python port of vision_sorter detectLine + sorter_controller ROI logic."""

    def __init__(self):
        self.image_width = int(private_param("image_width", 640))
        self.image_height = int(private_param("image_height", 480))
        self.black_low = np.array(as_float_list(private_param("black_low", [0, 0, 0]), [0, 0, 0]), dtype=np.uint8)
        self.black_high = np.array(
            as_float_list(private_param("black_high", [179, 255, 85]), [179, 255, 85]), dtype=np.uint8
        )
        self.rois = private_param(
            "rois",
            [
                [0, 260, 640, 20, 0.25, 1],
                [0, 130, 640, 20, 0.20, 2],
                [0, 0, 640, 20, 0.10, 3],
            ],
        )
        self.min_line_area = float(private_param("min_line_area", 400))
        self.min_cross_area = float(private_param("min_cross_area", 4000))
        self.draw_debug = as_bool(private_param("show", False))

    @staticmethod
    def preprocess_hsv(bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        kernel = np.ones((5, 5), np.uint8)
        hsv = cv2.erode(hsv, kernel, iterations=1)
        hsv = cv2.dilate(hsv, kernel, iterations=1)
        return hsv

    @staticmethod
    def largest_blob(mask, min_area):
        find_result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = find_result[0] if len(find_result) == 2 else find_result[1]
        best_contour = None
        best_area = 0.0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area >= min_area and area > best_area:
                best_area = area
                best_contour = contour
        if best_contour is None:
            return None

        moments = cv2.moments(best_contour)
        if moments["m00"] <= 0.0:
            return None
        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"])
        return {"center": (cx, cy), "area": best_area, "contour": best_contour}

    def detect(self, frame):
        if frame is None or frame.size == 0:
            return {
                "visible": False,
                "error": 0.0,
                "angle_deg": 0.0,
                "roi_areas": {},
                "centers": [],
                "frame": frame,
            }

        frame = cv2.resize(frame, (self.image_width, self.image_height))
        weight_sum = 0.0
        centroid_sum = 0.0
        roi_areas = {}
        centers = []
        visible = False

        for roi_def in self.rois:
            x, y, w, h = [int(float(v)) for v in roi_def[:4]]
            weight = float(roi_def[4])
            roi_id = int(float(roi_def[5])) if len(roi_def) >= 6 else len(roi_areas) + 1

            x0 = clamp(x, 0, frame.shape[1])
            y0 = clamp(y, 0, frame.shape[0])
            x1 = clamp(x + w, 0, frame.shape[1])
            y1 = clamp(y + h, 0, frame.shape[0])
            if x1 <= x0 or y1 <= y0:
                roi_areas[roi_id] = 0.0
                continue

            roi = frame[y0:y1, x0:x1]
            hsv_roi = self.preprocess_hsv(roi)
            mask = cv2.inRange(hsv_roi, self.black_low, self.black_high)
            blob = self.largest_blob(mask, self.min_line_area)
            if blob is None:
                roi_areas[roi_id] = 0.0
                if self.draw_debug:
                    cv2.rectangle(frame, (x0, y0), (x1, y1), (80, 80, 80), 1)
                continue

            visible = True
            roi_areas[roi_id] = blob["area"]
            local_cx, local_cy = blob["center"]
            center = (local_cx + x0, local_cy + y0)
            centers.append(center)

            # Same as sorter_controller.cpp: large cross blobs are excluded from
            # the weighted line centroid so intersections do not corrupt steering.
            if blob["area"] < self.min_cross_area:
                centroid_sum += local_cx * weight
                weight_sum += weight

            if self.draw_debug:
                color = (0, 0, 255) if blob["area"] >= self.min_cross_area else (255, 0, 0)
                cv2.rectangle(frame, (x0, y0), (x1, y1), color, 1)
                cv2.circle(frame, center, 5, color, -1)
                cv2.putText(
                    frame,
                    "roi{} area={:.0f}".format(roi_id, blob["area"]),
                    (x0 + 4, max(15, y0 - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    color,
                    1,
                )

        error = 0.0
        angle_deg = 0.0
        if weight_sum > 0.0:
            center_pos = centroid_sum / weight_sum
            half_width = float(frame.shape[1]) / 2.0
            half_height = float(frame.shape[0]) / 2.0
            error = (center_pos - half_width) / half_width
            angle_deg = -math.atan((center_pos - half_width) / half_height) * 180.0 / math.pi

        if self.draw_debug:
            for i in range(1, len(centers)):
                cv2.line(frame, centers[i - 1], centers[i], (0, 255, 0), 2)

        return {
            "visible": visible,
            "error": error,
            "angle_deg": angle_deg,
            "roi_areas": roi_areas,
            "centers": centers,
            "frame": frame,
        }


class ColorBlockDetector:
    """Python port of vision_sorter::detectColorBlock for red/green/blue blocks."""

    DEFAULT_RANGES = {
        BLOCK_RED: [([0, 110, 100], [10, 255, 255]), ([160, 110, 100], [179, 255, 255])],
        BLOCK_GREEN: [([40, 70, 0], [90, 255, 255])],
        BLOCK_BLUE: [([95, 80, 60], [130, 255, 255])],
    }
    DRAW_COLORS = {BLOCK_RED: (0, 0, 255), BLOCK_GREEN: (0, 255, 0), BLOCK_BLUE: (255, 0, 0)}

    def __init__(self):
        self.image_width = int(private_param("image_width", 640))
        self.image_height = int(private_param("image_height", 480))
        self.min_color_area = float(private_param("min_color_area", 1000))
        self.color_roi = [int(v) for v in private_param("color_roi", [0, 0, 640, 300])]
        self.color_ranges = self._load_color_ranges(private_param("color_ranges", None))

    def _load_color_ranges(self, configured):
        ranges = {k: list(v) for k, v in self.DEFAULT_RANGES.items()}
        if isinstance(configured, list):
            for item in configured:
                if not isinstance(item, dict):
                    continue
                name = color_name(item.get("name", BLOCK_NONE))
                raw_ranges = item.get("ranges", item.get("hsv_ranges", []))
                parsed = []
                for pair in raw_ranges:
                    if len(pair) >= 2:
                        parsed.append((pair[0], pair[1]))
                if name != BLOCK_NONE and parsed:
                    ranges[name] = parsed
        return ranges

    @staticmethod
    def preprocess_hsv(bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        kernel = np.ones((5, 5), np.uint8)
        hsv = cv2.erode(hsv, kernel, iterations=1)
        hsv = cv2.dilate(hsv, kernel, iterations=1)
        return hsv

    def detect(self, frame, target_color=BLOCK_NONE):
        if frame is None or frame.size == 0:
            return None
        frame = cv2.resize(frame, (self.image_width, self.image_height))
        x, y, w, h = self.color_roi
        x0 = clamp(x, 0, frame.shape[1])
        y0 = clamp(y, 0, frame.shape[0])
        x1 = clamp(x + w, 0, frame.shape[1])
        y1 = clamp(y + h, 0, frame.shape[0])
        if x1 <= x0 or y1 <= y0:
            return None
        hsv = self.preprocess_hsv(frame[y0:y1, x0:x1])
        colors = [target_color] if color_name(target_color) != BLOCK_NONE else [BLOCK_RED, BLOCK_GREEN, BLOCK_BLUE]
        best = None
        for name in colors:
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for low, high in self.color_ranges.get(name, []):
                mask = cv2.bitwise_or(
                    mask,
                    cv2.inRange(hsv, np.array(low, dtype=np.uint8), np.array(high, dtype=np.uint8)),
                )
            blob = IntegratedLineDetector.largest_blob(mask, self.min_color_area)
            if blob is None:
                continue
            if best is None or blob["area"] > best["area"]:
                cx, cy = blob["center"]
                contour = blob["contour"] + np.array([[[x0, y0]]], dtype=blob["contour"].dtype)
                best = {
                    "color": name,
                    "center": (cx + x0, cy + y0),
                    "area": blob["area"],
                    "contour": contour,
                    "draw_color": self.DRAW_COLORS.get(name, (255, 255, 255)),
                }
        return best


class LineFollowController:
    def __init__(self):
        self.line_visible_topic = private_param("line_visible_topic", "/vision/line/visible")
        self.line_error_topic = private_param("line_error_topic", "/vision/line/error")
        self.intersection_topic = private_param("intersection_topic", "/vision/intersection/detected")
        self.cmd_vel_topic = private_param("cmd_vel_topic", "/cmd_vel")

        self.use_camera = as_bool(private_param("use_camera", True))
        self.camera_index = private_param("camera_index", 0)
        self.camera_backend = private_param("camera_backend", "")
        self.publish_cmd_vel = as_bool(private_param("publish_cmd_vel", False))
        self.rate_hz = float(private_param("rate_hz", 20.0))
        self.forward_speed = float(private_param("forward_speed", 0.06))
        self.intersection_forward_speed = float(private_param("intersection_forward_speed", 0.03))
        self.kp_angular = float(private_param("kp_angular", 0.45))
        self.max_angular_z = abs(float(private_param("max_angular_z", 0.35)))
        self.error_deadband = abs(float(private_param("error_deadband", 0.03)))
        self.lost_timeout = float(private_param("lost_timeout", 0.30))
        self.stop_on_intersection = as_bool(private_param("stop_on_intersection", False))
        self.slow_on_intersection = as_bool(private_param("slow_on_intersection", True))
        self.angular_sign = float(private_param("angular_sign", -1.0))

        # C++ sorter_controller style motion parameters. Defaults are safer than
        # the original 0.3/0.8 values; tune them up after confirming direction.
        self.cpp_style_motion = as_bool(private_param("cpp_style_motion", True))
        self.cpp_base_speed = float(private_param("cpp_base_speed", 0.12))
        self.cpp_min_speed = float(private_param("cpp_min_speed", 0.04))
        self.cpp_angle_speed_gain = float(private_param("cpp_angle_speed_gain", 0.01))
        self.cpp_angular_gain = float(private_param("cpp_angular_gain", 0.03))
        self.turn_linear_x = float(private_param("turn_linear_x", 0.06))
        self.turn_angular_z = float(private_param("turn_angular_z", 0.35))
        self.return_angular_z = float(private_param("return_angular_z", 0.35))
        self.pick_crosses = set(int(v) for v in private_param("pick_crosses", [2, 5, 9]))
        self.uturn_crosses = set(int(v) for v in private_param("uturn_crosses", []))
        self.stop_crosses = set(int(v) for v in private_param("stop_crosses", [13]))
        self.cross_clear_ratio = float(private_param("cross_clear_ratio", 0.6))
        self.cross_ignore_ticks_after_action = int(private_param("cross_ignore_ticks_after_action", 40))
        self.uturn_linear_x = float(private_param("uturn_linear_x", 0.0))
        self.uturn_angular_z = float(private_param("uturn_angular_z", 0.35))
        self.uturn_delay = int(private_param("uturn_delay", 120))
        self.post_pick_uturn_crosses = set(int(v) for v in private_param("post_pick_uturn_crosses", [2, 5, 9]))
        self.post_pick_uturn_linear_x = float(private_param("post_pick_uturn_linear_x", self.uturn_linear_x))
        self.post_pick_uturn_angular_z = float(private_param("post_pick_uturn_angular_z", self.uturn_angular_z))
        self.post_pick_uturn_delay = int(private_param("post_pick_uturn_delay", self.uturn_delay))
        self.over_turn_delay = int(private_param("over_turn_delay", 80))
        self.turn_delays = {
            3: int(private_param("first_turn_delay", 50)),
            4: int(private_param("second_turn_delay", 50)),
            6: int(private_param("third_turn_delay", 60)),
            7: int(private_param("return_delay", 100)),
            8: int(private_param("fourth_turn_delay", 50)),
            10: int(private_param("fifth_turn_delay", 60)),
            11: int(private_param("sixth_turn_delay", 50)),
        }
        self.turn_specs = {
            3: (self.turn_linear_x, -self.turn_angular_z),
            4: (self.turn_linear_x, -self.turn_angular_z),
            6: (self.turn_linear_x, self.turn_angular_z),
            7: (0.0, self.return_angular_z),
            8: (self.turn_linear_x, self.turn_angular_z),
            10: (self.turn_linear_x, -self.turn_angular_z),
            11: (self.turn_linear_x, -self.turn_angular_z),
        }

        # C++ sorter_controller pick/place state.
        self.enable_pick_place = as_bool(private_param("enable_pick_place", True))
        self.serial_tx_topic = private_param("serial_tx_topic", "/tao_serial/tx")
        self.arm_joints_topic = private_param("arm_joints_topic", "/tao_arm/joints_protocol_units")
        self.gripper_topic = private_param("gripper_topic", "/gripper/command")
        self.publish_arm_shadow_topic = as_bool(private_param("publish_arm_shadow_topic", True))
        self.use_tx_command = as_bool(private_param("use_tx_command", True))
        self.serial_tx_wait_timeout = float(private_param("serial_tx_wait_timeout", 1.0))
        self.protocol_offsets = as_float_list(private_param("protocol_offsets", [0, 0, 0, 0, 0, 0]), [0, 0, 0, 0, 0, 0])
        self.protocol_signs = [int(v) for v in private_param("protocol_signs", [1, 1, 1, 1, 1, 1])]
        self.protocol_scale = float(private_param("protocol_scale", 1000.0))
        self.protocol_min = int(private_param("protocol_min", -32768))
        self.protocol_max = int(private_param("protocol_max", 32767))
        self.arm_err_x = float(private_param("arm_err_x", 0.0))
        self.arm_up = float(private_param("arm_up", 95.0))
        self.grasp_height = float(private_param("grasp_height", 35.0))
        self.arm_skewing = float(private_param("arm_skewing", 10.0))
        self.open_gripper = float(private_param("open_gripper", 1.0))
        self.closed_gripper = float(private_param("closed_gripper", 0.0))
        self.init_arm_on_start = as_bool(private_param("init_arm_on_start", False))
        self.preposition_arm_before_pick = as_bool(private_param("preposition_arm_before_pick", False))
        self.pick_align_timeout_ticks = int(private_param("pick_align_timeout_ticks", 180))
        self.pick_align_max_retries = int(private_param("pick_align_max_retries", 2))
        self.pick_align_skip_on_timeout = as_bool(private_param("pick_align_skip_on_timeout", True))
        self.pick_align_stable_ticks = int(private_param("pick_align_stable_ticks", 50))
        self.color_log_interval = float(private_param("color_log_interval", 1.0))
        self.move_x = self.arm_err_x
        self.move_y = 150.0
        self.spin_claw = 0.0
        self.move_status = 0
        self.arm_task_phase = ARM_PHASE_IDLE
        self.captured_color = BLOCK_NONE
        self.pick_align_ticks = 0
        self.pick_align_retries = 0
        self.line_mode = True
        self.car_back_flag = False
        self.mid_adjust_position = False
        self.over_flag = False
        self.mid_over_flag = False
        self.mid_over_count = 0
        self.arm_seq = 0
        self.arm = MoveItArmBackend(ArmKinematics())
        self.color_detector = ColorBlockDetector() if self.use_camera else None

        self.visible = False
        self.error = 0.0
        self.angle_deg = 0.0
        self.intersection = False
        self.last_visible_time = rospy.Time(0)
        self.last_log_time = rospy.Time(0)
        self.last_roi_areas = {}
        self.latest_camera_frame = None

        # C++ state names: crossingFlag_, crossingRecordCnt_, timeCnt_.
        # crossing_flag: 0=idle, 1=armed by ROI2, 2=counted/action, 3=wait until leaving cross.
        self.crossing_flag = 0
        self.cross_count = 0
        self.time_count = 0
        self.turning_cross = None
        self.cross_ignore_countdown = 0
        self.post_pick_uturn_active = False
        self.post_pick_uturn_count = 0
        self.stopped = False

        self.detector = IntegratedLineDetector() if self.use_camera else None
        self.cap = None
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.serial_tx_pub = rospy.Publisher(self.serial_tx_topic, String, queue_size=10)
        self.arm_units_pub = rospy.Publisher(self.arm_joints_topic, Int16MultiArray, queue_size=10)
        self.gripper_pub = rospy.Publisher(self.gripper_topic, UInt8, queue_size=10)

        if self.enable_pick_place and self.init_arm_on_start:
            self.init_robot_pose()

        if self.use_camera:
            self.open_camera()
        else:
            rospy.Subscriber(self.line_visible_topic, Bool, self.handle_visible, queue_size=10)
            rospy.Subscriber(self.line_error_topic, Float32, self.handle_error, queue_size=10)
            rospy.Subscriber(self.intersection_topic, Bool, self.handle_intersection, queue_size=10)

        period = 1.0 / self.rate_hz if self.rate_hz > 0.0 else 0.05
        self.timer = rospy.Timer(rospy.Duration(period), self.update)

        rospy.loginfo(
            "line_follow_controller started mode=%s publish_cmd_vel=%s forward=%.3f kp=%.3f max_w=%.3f sign=%.1f",
            "integrated_camera" if self.use_camera else "topic",
            self.publish_cmd_vel,
            self.forward_speed,
            self.kp_angular,
            self.max_angular_z,
            self.angular_sign,
        )

    def open_camera(self):
        index = self.camera_index
        try:
            index = int(index)
        except (TypeError, ValueError):
            pass

        if self.camera_backend.lower() == "v4l2":
            self.cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        else:
            self.cap = cv2.VideoCapture(index)

        if not self.cap.isOpened():
            rospy.logerr("failed to open camera: %s", self.camera_index)
            return
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.detector.image_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.detector.image_height)
        rospy.loginfo("camera opened: %s", self.camera_index)

    def handle_visible(self, msg):
        self.visible = bool(msg.data)
        if self.visible:
            self.last_visible_time = rospy.Time.now()

    def handle_error(self, msg):
        self.error = float(msg.data)

    def handle_intersection(self, msg):
        self.intersection = bool(msg.data)

    def line_recently_visible(self, now):
        if self.last_visible_time == rospy.Time(0):
            return False
        return (now - self.last_visible_time).to_sec() <= self.lost_timeout

    def joints_to_protocol(self, joints):
        values = []
        for index in range(6):
            joint = float(joints[index]) if index < len(joints) else 0.0
            offset = self.protocol_offsets[index] if index < len(self.protocol_offsets) else 0.0
            sign = self.protocol_signs[index] if index < len(self.protocol_signs) else 1
            value = int(round((joint - offset) * self.protocol_scale * sign))
            values.append(int(clamp(value, self.protocol_min, self.protocol_max)))
        return values

    def publish_arm_command(self, joints):
        if joints is None:
            return
        # MoveItArmBackend returns an empty list when the trajectory has already
        # been accepted by arm_controller/follow_joint_trajectory.  In that mode
        # tao_moveit_bridge is responsible for emitting the ARM_JOINTS serial
        # command, so do not publish a duplicate direct command here.
        if len(joints) == 0:
            return
        if len(joints) < 6:
            return
        duration_ms = int(clamp(int(joints[6]) if len(joints) >= 7 else 500, 50, 5000))
        values = self.joints_to_protocol(joints)
        if self.publish_arm_shadow_topic:
            msg = Int16MultiArray()
            msg.data = [int(clamp(value, -32768, 32767)) for value in values]
            self.arm_units_pub.publish(msg)
        if self.use_tx_command:
            self.wait_for_publisher_connections(self.serial_tx_pub, self.serial_tx_wait_timeout, "serial tx")
            self.arm_seq = (self.arm_seq + 1) & 0xFF
            msg = String()
            msg.data = "ARM_JOINTS {} {}".format(self.arm_seq, " ".join(str(value) for value in values))
            msg.data += " {}".format(duration_ms)
            self.serial_tx_pub.publish(msg)

    @staticmethod
    def wait_for_publisher_connections(pub, timeout_sec, name):
        if timeout_sec <= 0.0 or pub.get_num_connections() > 0:
            return
        deadline = rospy.Time.now() + rospy.Duration(timeout_sec)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and pub.get_num_connections() == 0 and rospy.Time.now() < deadline:
            rate.sleep()
        if pub.get_num_connections() == 0:
            rospy.logwarn_throttle(2.0, "%s publisher has no subscribers; command may be missed", name)

    def move_arm(self, x, y, z, ms):
        joints = self.arm.move(x, y, z, ms)
        if joints is None:
            rospy.logwarn_throttle(1.0, "arm IK failed x=%.1f y=%.1f z=%.1f", x, y, z)
            return
        self.publish_arm_command(joints)

    def claw(self, spin, hand, ms):
        self.publish_arm_command(self.arm.claw(spin, hand, ms))
        grip = UInt8()
        grip.data = int(clamp(round(hand * 100.0), 0, 100))
        self.gripper_pub.publish(grip)

    def init_robot_pose(self):
        self.move_x = self.arm_err_x
        self.move_y = 150.0
        self.move_arm(self.move_x, self.move_y, self.arm_up, 1500)
        self.claw(0.0, self.open_gripper, 1000)

    def stop_serial(self):
        msg = String()
        msg.data = "STOP"
        self.serial_tx_pub.publish(msg)

    def update_camera_detection(self, now):
        if self.cap is None or not self.cap.isOpened():
            self.visible = False
            return None
        ok, frame = self.cap.read()
        if not ok or frame is None or frame.size == 0:
            rospy.logwarn_throttle(1.0, "camera frame read failed")
            self.visible = False
            return None
        self.latest_camera_frame = frame

        if self.line_mode:
            detection = self.detector.detect(frame)
            self.visible = bool(detection["visible"])
            self.error = float(detection["error"])
            self.angle_deg = float(detection["angle_deg"])
            self.last_roi_areas = detection["roi_areas"]
            if self.visible:
                self.last_visible_time = now
            # During the scripted post-pick U-turn at cross=2/5/9, keep line
            # perception alive for debug/logging but do not count intersections.
            # This preserves the original crossing sequence: the next counted
            # cross after pick+U-turn should be the following physical cross,
            # not the same one seen while rotating away from the pick point.
            if not self.post_pick_uturn_active:
                self.update_crossing_state()
            else:
                self.intersection = False
        else:
            detection = {"frame": cv2.resize(frame, (self.detector.image_width, self.detector.image_height))}
            self.visible = False
            self.intersection = False

        if self.detector.draw_debug:
            debug = detection["frame"]
            cv2.putText(
                debug,
                "line={} cross={} flag={} move={} color={}".format(
                    int(self.line_mode), self.cross_count, self.crossing_flag, self.move_status, self.captured_color
                ),
                (10, 460),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
            )
            cv2.imshow("line_follow_controller", debug)
            cv2.waitKey(1)
        return detection

    def update_crossing_state(self):
        roi1_area = float(self.last_roi_areas.get(1, 0.0))
        roi2_area = float(self.last_roi_areas.get(2, 0.0))
        min_cross = self.detector.min_cross_area if self.detector is not None else 4000.0
        clear_threshold = min_cross * self.cross_clear_ratio

        if self.cross_ignore_countdown > 0:
            self.cross_ignore_countdown -= 1
            self.intersection = False
            return

        if self.crossing_flag == 3:
            if roi1_area < clear_threshold and roi2_area < clear_threshold:
                self.crossing_flag = 0
                rospy.loginfo("cross cleared: roi1=%.0f roi2=%.0f", roi1_area, roi2_area)
            self.intersection = False
        elif self.crossing_flag == 0 and roi2_area > min_cross:
            self.crossing_flag = 1
            rospy.loginfo("cross armed: roi2_area=%.0f", roi2_area)
        elif self.crossing_flag == 1 and roi1_area > min_cross:
            self.crossing_flag = 2
            self.cross_count += 1
            self.time_count = 0
            self.turning_cross = None
            self.intersection = True
            rospy.loginfo("cross counted: cross=%d roi1=%.0f roi2=%.0f", self.cross_count, roi1_area, roi2_area)
        else:
            self.intersection = self.crossing_flag == 2

    def build_topic_cmd(self, now):
        cmd = Twist()
        if not self.visible or not self.line_recently_visible(now):
            return cmd, "lost"
        if self.intersection and self.stop_on_intersection:
            return cmd, "intersection_stop"

        error = 0.0 if abs(self.error) < self.error_deadband else self.error
        angular_z = self.angular_sign * self.kp_angular * error
        if not math.isfinite(angular_z):
            angular_z = 0.0

        speed = self.forward_speed
        reason = "line"
        if self.intersection and self.slow_on_intersection:
            speed = self.intersection_forward_speed
            reason = "intersection_slow"

        cmd.linear.x = speed
        cmd.angular.z = clamp(angular_z, -self.max_angular_z, self.max_angular_z)
        return cmd, reason

    def build_integrated_cmd(self, now):
        cmd = Twist()
        if self.stopped:
            return cmd, "stopped"

        if not self.line_mode:
            return self.handle_color_block()

        if self.post_pick_uturn_active:
            return self.handle_post_pick_uturn()

        if self.crossing_flag == 2 and self.cross_count in self.uturn_crosses:
            handled, reason, cross_cmd = self.handle_crossing_action()
            if handled:
                return cross_cmd, reason

        if not self.visible or not self.line_recently_visible(now):
            return cmd, "lost"

        if self.crossing_flag == 2:
            handled, reason, cross_cmd = self.handle_crossing_action()
            if handled:
                return cross_cmd, reason

        if self.cpp_style_motion:
            car_x = self.cpp_base_speed - abs(self.angle_deg * self.cpp_angle_speed_gain)
            car_x = max(self.cpp_min_speed, car_x)
            car_w = self.angle_deg * self.cpp_angular_gain
            car_y = 0.0
            if self.mid_adjust_position and abs(self.angle_deg) < 3.0:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.time_count += 1
                if self.time_count > 5:
                    self.move_x = 120.0 if self.cross_count == 3 else -120.0
                    self.move_y = 80.0
                    self.move_arm(self.move_x, self.move_y, self.arm_up, 1000)
                    self.time_count = 0
                    self.arm_task_phase = ARM_PHASE_PLACE
                    self.move_status = 2
                    self.line_mode = False
                    self.car_back_flag = False
                    self.mid_adjust_position = False
                return cmd, "mid_adjust"
            if self.mid_adjust_position:
                roi1 = float(self.last_roi_areas.get(1, 0.0))
                roi3 = float(self.last_roi_areas.get(3, 0.0))
                min_cross = self.detector.min_cross_area if self.detector is not None else 4000.0
                if roi3 > min_cross:
                    self.time_count += 1
                    if self.time_count > 5:
                        self.car_back_flag = True
                elif roi1 > min_cross:
                    self.car_back_flag = False
                else:
                    self.time_count = 0
            if self.car_back_flag:
                car_x = -car_x
                car_w /= 5.0
                car_y = car_w
            cmd.linear.x = car_x
            cmd.linear.y = car_y
            cmd.angular.z = clamp(car_w, -self.max_angular_z, self.max_angular_z)
            return cmd, "cpp_line"

        error = 0.0 if abs(self.error) < self.error_deadband else self.error
        angular_z = self.angular_sign * self.kp_angular * error
        cmd.linear.x = self.forward_speed
        cmd.angular.z = clamp(angular_z, -self.max_angular_z, self.max_angular_z)
        return cmd, "line"

    def handle_post_pick_uturn(self):
        cmd = Twist()
        cmd.linear.x = self.post_pick_uturn_linear_x
        cmd.angular.z = self.post_pick_uturn_angular_z
        if self.post_pick_uturn_count == 0:
            rospy.loginfo(
                "post-pick uturn start: cross=%d linear_x=%.3f angular_z=%.3f delay=%d",
                self.cross_count,
                self.post_pick_uturn_linear_x,
                self.post_pick_uturn_angular_z,
                self.post_pick_uturn_delay,
            )
        self.post_pick_uturn_count += 1
        if self.post_pick_uturn_count > self.post_pick_uturn_delay:
            rospy.loginfo("post-pick uturn done: cross=%d", self.cross_count)
            self.post_pick_uturn_active = False
            self.post_pick_uturn_count = 0
            self.reset_crossing(wait_clear=False, ignore_ticks=True)
        return cmd, "post_pick_uturn_cross_{}".format(self.cross_count)

    def handle_crossing_action(self):
        cmd = Twist()
        cross = self.cross_count

        if cross in self.pick_crosses and self.enable_pick_place:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.arm_task_phase = ARM_PHASE_PICK
            self.move_status = 0
            self.line_mode = False
            self.crossing_flag = 0
            self.time_count = 0
            self.pick_align_ticks = 0
            self.pick_align_retries = 0
            self.captured_color = BLOCK_NONE
            rospy.loginfo("pick state start: cross=%d", cross)
            return True, "pick_cross_{}".format(cross), cmd

        if cross in self.uturn_crosses:
            cmd.linear.x = self.uturn_linear_x
            cmd.angular.z = self.uturn_angular_z
            if self.turning_cross != cross:
                rospy.loginfo(
                    "uturn start: cross=%d linear_x=%.3f angular_z=%.3f delay=%d",
                    cross,
                    self.uturn_linear_x,
                    self.uturn_angular_z,
                    self.uturn_delay,
                )
                self.turning_cross = cross
                self.time_count = 0
            self.time_count += 1
            if self.time_count > self.uturn_delay:
                rospy.loginfo("uturn done: cross=%d", cross)
                self.reset_crossing(ignore_ticks=True)
            return True, "uturn_cross_{}".format(cross), cmd

        if cross in self.stop_crosses:
            rospy.loginfo("cross=%d reached stop point", cross)
            self.stopped = True
            self.reset_crossing()
            return True, "final_stop", cmd

        if cross == 12:
            self.over_flag = True
            self.car_back_flag = True
            self.crossing_flag = 1
            return True, "over_turn", cmd

        if cross in self.turn_specs:
            linear_x, angular_z = self.turn_specs[cross]
            delay = self.turn_delays.get(cross, 50)
            cmd.linear.x = linear_x
            cmd.angular.z = angular_z
            if self.turning_cross != cross:
                rospy.loginfo("turn start: cross=%d linear_x=%.3f angular_z=%.3f delay=%d", cross, linear_x, angular_z, delay)
                self.turning_cross = cross
                self.time_count = 0
            self.time_count += 1
            if self.time_count > delay:
                rospy.loginfo("turn done: cross=%d", cross)
                if cross in (3, 6, 10):
                    self.mid_adjust_position = True
                    self.crossing_flag = 0
                    self.time_count = 0
                    self.turning_cross = None
                else:
                    self.reset_crossing()
            return True, "turn_cross_{}".format(cross), cmd

        rospy.loginfo("cross=%d has no special action; continue", cross)
        self.reset_crossing()
        return False, "cross_continue", cmd

    def latest_frame(self):
        if self.latest_camera_frame is not None:
            return self.latest_camera_frame
        if self.cap is None or not self.cap.isOpened():
            return None
        ok, frame = self.cap.read()
        if ok:
            self.latest_camera_frame = frame
            return frame
        return None

    def handle_color_block(self):
        cmd = Twist()
        frame = self.latest_frame()
        target = self.captured_color if self.move_status >= 2 else BLOCK_NONE
        blob = self.color_detector.detect(frame, target) if self.color_detector is not None else None
        success = blob is not None
        block_cx, block_cy = blob["center"] if success else (320, 240)
        contour = blob["contour"] if success else None
        visible_color = blob["color"] if success else BLOCK_NONE

        if self.arm_task_phase != ARM_PHASE_IDLE and self.color_log_interval > 0.0:
            rospy.loginfo_throttle(
                self.color_log_interval,
                "color detect phase=%s move=%d target=%s success=%s color=%s center=(%d,%d)",
                self.arm_task_phase,
                self.move_status,
                self.captured_color,
                success,
                visible_color,
                block_cx,
                block_cy,
            )

        if self.arm_task_phase == ARM_PHASE_IDLE:
            self.line_mode = True
            return cmd, "arm_idle"

        if self.arm_task_phase == ARM_PHASE_PICK and self.move_status >= 2:
            self.finish_pick_phase()
            return cmd, "pick_done"

        if self.arm_task_phase == ARM_PHASE_PLACE and self.move_status < 2:
            self.move_status = 2

        if self.arm_task_phase == ARM_PHASE_PICK and self.move_status == 0:
            self.pick_align_ticks += 1
            if success:
                if abs(block_cx - 320) > 10:
                    self.move_x += -0.5 if block_cx > 320 else 0.5
                if abs(block_cy - 240) > 10:
                    self.move_y += -0.3 if block_cy > 240 and self.move_y > 1.0 else 0.3
                if abs(block_cx - 320) <= 10 and abs(block_cy - 240) <= 10:
                    self.time_count += 1
                    if self.time_count > self.pick_align_stable_ticks:
                        self.time_count = 0
                        self.pick_align_ticks = 0
                        self.move_status = 1
                        self.captured_color = visible_color
                        self.spin_claw = 0.0
                        length = math.sqrt(self.move_x * self.move_x + self.move_y * self.move_y)
                        if length > 1e-6:
                            self.move_x = (length + self.arm_skewing) * self.move_x / length
                            self.move_y = (length + self.arm_skewing) * self.move_y / length
                elif self.preposition_arm_before_pick:
                    self.time_count = 0
                    self.move_arm(self.move_x, self.move_y, self.arm_up, 0)
                else:
                    self.time_count = 0
            else:
                self.time_count = 0
            if self.pick_align_timeout_ticks > 0 and self.pick_align_ticks > self.pick_align_timeout_ticks:
                if self.pick_align_retries < self.pick_align_max_retries:
                    self.pick_align_retries += 1
                    self.pick_align_ticks = 0
                    self.move_x = self.arm_err_x
                    self.move_y = 150.0
                    rospy.logwarn(
                        "pick align timeout: cross=%d retry=%d/%d",
                        self.cross_count,
                        self.pick_align_retries,
                        self.pick_align_max_retries,
                    )
                elif self.pick_align_skip_on_timeout:
                    rospy.logwarn("pick align skipped after timeout: cross=%d", self.cross_count)
                    self.finish_pick_phase()
                    return cmd, "pick_align_timeout_skip"
            return cmd, "pick_align"

        if self.arm_task_phase == ARM_PHASE_PICK and self.move_status == 1:
            self.time_count += 1
            if self.time_count < 2:
                self.spin_claw = contour_angle_rad(contour)
                self.claw(self.spin_claw, self.open_gripper, 1000)
            elif self.time_count < 35:
                self.move_arm(self.move_x, self.move_y, self.arm_up, 1000)
            elif self.time_count < 70:
                self.move_arm(self.move_x, self.move_y + 30.0, self.grasp_height, 1000)
            elif 105 <= self.time_count < 140:
                self.claw(self.spin_claw, self.closed_gripper, 1000)
            elif 175 <= self.time_count < 210:
                self.move_arm(self.move_x, self.move_y, self.arm_up, 1000)
            elif 245 <= self.time_count < 280:
                self.move_status = 2
                self.pick_align_ticks = 0
                self.pick_align_retries = 0
                self.move_x = self.arm_err_x
                self.move_y = 150.0
                self.spin_claw = 0.0
                self.move_arm(self.move_x, self.move_y, self.arm_up, 1000)
            return cmd, "pick_sequence"

        if self.arm_task_phase == ARM_PHASE_PLACE and self.move_status == 2:
            self.time_count += 1
            if not success:
                cmd.linear.x = 0.1 if self.time_count < 50 else -0.1
            else:
                self.move_status = 3
                self.time_count = 0
            return cmd, "seek_place_color"

        if self.arm_task_phase == ARM_PHASE_PLACE and self.move_status == 3 and success:
            if block_cx - 320 > 50:
                cmd.linear.x = 0.1 if self.cross_count == 3 else -0.1
            elif block_cx - 320 < -50:
                cmd.linear.x = -0.1 if self.cross_count == 3 else 0.1
            else:
                self.time_count += 1
                if self.time_count > 40:
                    self.move_status = 4
                    self.time_count = 0
            return cmd, "place_base_align"

        if self.arm_task_phase == ARM_PHASE_PLACE and self.move_status == 4 and success:
            if abs(block_cx - 320) > 10:
                right = block_cx > 320
                self.move_y += (0.5 if right else -0.5) if self.cross_count == 3 else (-0.5 if right else 0.5)
            if abs(block_cy - 240) > 10:
                down = block_cy > 240
                self.move_x += (-0.3 if down else 0.3) if self.cross_count == 3 else (0.3 if down else -0.3)
            if abs(block_cx - 320) <= 10 and abs(block_cy - 240) <= 10:
                self.time_count += 1
                if self.time_count > 10:
                    self.time_count = 0
                    self.move_status = 5
                    length = math.sqrt(self.move_x * self.move_x + self.move_y * self.move_y)
                    if length > 1e-6:
                        self.move_x = (length + self.arm_skewing) * self.move_x / length * 1.1
                        self.move_y = (length + self.arm_skewing) * self.move_y / length * 0.7
            else:
                self.time_count = 0
                self.move_arm(self.move_x, self.move_y, self.arm_up, 0)
            return cmd, "place_arm_align"

        if self.arm_task_phase == ARM_PHASE_PLACE and self.move_status == 5:
            self.time_count += 1
            if self.time_count < 35:
                self.move_arm(self.move_x, self.move_y, self.arm_up, 1000)
            elif self.time_count < 70:
                self.move_arm(self.move_x, self.move_y + 40.0, self.grasp_height, 1000)
            elif self.time_count < 100:
                self.claw(0.0, self.open_gripper, 1000)
            elif 135 <= self.time_count < 170:
                self.move_arm(self.move_x, self.move_y, self.arm_up, 1000)
            elif 200 <= self.time_count < 235:
                self.move_x = self.arm_err_x
                self.move_y = 140.0
                self.move_arm(self.move_x, self.move_y, self.arm_up, 1000)
            elif 270 <= self.time_count < 300:
                self.line_mode = True
                self.crossing_flag = 1
                self.arm_task_phase = ARM_PHASE_IDLE
                self.move_status = 0
                self.captured_color = BLOCK_NONE
                self.time_count = 0
            return cmd, "place_sequence"

        return cmd, "color_wait"

    def finish_pick_phase(self):
        self.line_mode = True
        self.arm_task_phase = ARM_PHASE_IDLE
        self.move_status = 0
        self.time_count = 0
        self.move_x = self.arm_err_x
        self.move_y = 150.0
        self.spin_claw = 0.0
        if self.cross_count in self.post_pick_uturn_crosses:
            self.post_pick_uturn_active = True
            self.post_pick_uturn_count = 0
            self.crossing_flag = 0
        else:
            self.crossing_flag = 1

    def reset_crossing(self, wait_clear=True, ignore_ticks=False):
        self.crossing_flag = 3 if wait_clear else 0
        self.time_count = 0
        self.turning_cross = None
        self.intersection = False
        if ignore_ticks:
            self.cross_ignore_countdown = max(0, self.cross_ignore_ticks_after_action)

    def build_cmd(self, now):
        if self.use_camera:
            return self.build_integrated_cmd(now)
        return self.build_topic_cmd(now)

    def update(self, _event):
        now = rospy.Time.now()
        if self.use_camera:
            self.update_camera_detection(now)
        cmd, reason = self.build_cmd(now)

        if self.publish_cmd_vel:
            self.cmd_pub.publish(cmd)

        if (now - self.last_log_time).to_sec() > 1.0:
            rospy.loginfo(
                "line_follow reason=%s visible=%s err=%.3f angle=%.1f cross=%d flag=%d roi=%s cmd=(%.3f, %.3f) publish=%s",
                reason,
                self.visible,
                self.error,
                self.angle_deg,
                self.cross_count,
                self.crossing_flag,
                {k: int(v) for k, v in self.last_roi_areas.items()},
                cmd.linear.x,
                cmd.angular.z,
                self.publish_cmd_vel,
            )
            self.last_log_time = now

    def stop(self):
        if self.publish_cmd_vel:
            self.cmd_pub.publish(Twist())
        if self.cap is not None:
            self.cap.release()
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass
        time.sleep(0.05)


if __name__ == "__main__":
    rospy.init_node("line_follow_controller")
    node = LineFollowController()
    rospy.on_shutdown(node.stop)
    rospy.spin()