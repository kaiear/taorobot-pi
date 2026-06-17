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
* Pick/place crosses such as 2/5/9 perform a route-test U-turn.

When ``use_camera`` is false, the previous topic-driven controller behavior is
kept for compatibility with ``vision_sorter/line_node``.
"""

import math
import time

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32


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
        self.uturn_crosses = set(int(v) for v in private_param("uturn_crosses", [2, 5, 9]))
        self.stop_crosses = set(int(v) for v in private_param("stop_crosses", [13]))
        self.cross_clear_ratio = float(private_param("cross_clear_ratio", 0.6))
        self.cross_ignore_ticks_after_action = int(private_param("cross_ignore_ticks_after_action", 40))
        self.uturn_linear_x = float(private_param("uturn_linear_x", 0.0))
        self.uturn_angular_z = float(private_param("uturn_angular_z", 0.35))
        self.uturn_delay = int(private_param("uturn_delay", 120))
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

        self.visible = False
        self.error = 0.0
        self.angle_deg = 0.0
        self.intersection = False
        self.last_visible_time = rospy.Time(0)
        self.last_log_time = rospy.Time(0)
        self.last_roi_areas = {}

        # C++ state names: crossingFlag_, crossingRecordCnt_, timeCnt_.
        # crossing_flag: 0=idle, 1=armed by ROI2, 2=counted/action, 3=wait until leaving cross.
        self.crossing_flag = 0
        self.cross_count = 0
        self.time_count = 0
        self.turning_cross = None
        self.cross_ignore_countdown = 0
        self.stopped = False

        self.detector = IntegratedLineDetector() if self.use_camera else None
        self.cap = None
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)

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

    def update_camera_detection(self, now):
        if self.cap is None or not self.cap.isOpened():
            self.visible = False
            return None
        ok, frame = self.cap.read()
        if not ok or frame is None or frame.size == 0:
            rospy.logwarn_throttle(1.0, "camera frame read failed")
            self.visible = False
            return None

        detection = self.detector.detect(frame)
        self.visible = bool(detection["visible"])
        self.error = float(detection["error"])
        self.angle_deg = float(detection["angle_deg"])
        self.last_roi_areas = detection["roi_areas"]
        if self.visible:
            self.last_visible_time = now

        self.update_crossing_state()

        if self.detector.draw_debug:
            debug = detection["frame"]
            cv2.putText(
                debug,
                "cross={} flag={} angle={:.1f} err={:.2f}".format(
                    self.cross_count, self.crossing_flag, self.angle_deg, self.error
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
            cmd.linear.x = car_x
            cmd.angular.z = clamp(car_w, -self.max_angular_z, self.max_angular_z)
            return cmd, "cpp_line"

        error = 0.0 if abs(self.error) < self.error_deadband else self.error
        angular_z = self.angular_sign * self.kp_angular * error
        cmd.linear.x = self.forward_speed
        cmd.angular.z = clamp(angular_z, -self.max_angular_z, self.max_angular_z)
        return cmd, "line"

    def handle_crossing_action(self):
        cmd = Twist()
        cross = self.cross_count

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
            # Simplified overFlag_: rotate in place for a configured delay, then
            # resume. This keeps the route test moving without arm/color logic.
            delay = self.over_turn_delay
            cmd.angular.z = -self.return_angular_z
            self.time_count += 1
            if self.time_count > delay:
                self.reset_crossing()
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
                self.reset_crossing()
            return True, "turn_cross_{}".format(cross), cmd

        rospy.loginfo("cross=%d has no special action; continue", cross)
        self.reset_crossing()
        return False, "cross_continue", cmd

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