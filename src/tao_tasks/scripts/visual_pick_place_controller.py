#!/usr/bin/env python3
"""Vision-window pick/place controller with carried-color memory.

Services:
  /visual_pick_place/pick  - align to the visible object, close gripper, remember color
  /visual_pick_place/place - align to a visible object whose color matches memory, open gripper
"""

import threading

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, Int16MultiArray, String, UInt8
from std_srvs.srv import Trigger, TriggerResponse


def private_param(name, default):
    if rospy.has_param("~" + name):
        return rospy.get_param("~" + name)
    return rospy.get_param("~visual_pick_place/" + name, default)


def clamp(value, low, high):
    return max(low, min(high, value))


class VisionTarget:
    def __init__(self, detected_topic, offset_x_topic, offset_y_topic, area_topic, color_topic=None):
        self.detected = False
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.area = 0.0
        self.color = "unknown"
        self.last_seen = rospy.Time(0)
        rospy.Subscriber(detected_topic, Bool, self._detected_cb, queue_size=10)
        rospy.Subscriber(offset_x_topic, Float32, self._offset_x_cb, queue_size=10)
        rospy.Subscriber(offset_y_topic, Float32, self._offset_y_cb, queue_size=10)
        rospy.Subscriber(area_topic, Float32, self._area_cb, queue_size=10)
        if color_topic:
            rospy.Subscriber(color_topic, String, self._color_cb, queue_size=10)

    def _detected_cb(self, msg):
        self.detected = bool(msg.data)
        if self.detected:
            self.last_seen = rospy.Time.now()

    def _offset_x_cb(self, msg):
        self.offset_x = float(msg.data)

    def _offset_y_cb(self, msg):
        self.offset_y = float(msg.data)

    def _area_cb(self, msg):
        self.area = float(msg.data)

    def _color_cb(self, msg):
        self.color = str(msg.data).strip().lower()

    def recently_seen(self, timeout=0.5):
        return self.detected and self.last_seen != rospy.Time(0) and (rospy.Time.now() - self.last_seen).to_sec() <= timeout


class VisualPickPlaceController:
    def __init__(self):
        self.cmd_pub = rospy.Publisher(private_param("cmd_vel_topic", "/cmd_vel"), Twist, queue_size=10)
        self.gripper_pub = rospy.Publisher(private_param("gripper_topic", "/gripper/command"), UInt8, queue_size=10)
        self.arm_pub = rospy.Publisher(private_param("arm_joints_topic", "/tao_arm/joints_protocol_units"), Int16MultiArray, queue_size=10)

        self.rate_hz = float(private_param("rate_hz", 15.0))
        self.command_timeout_sec = float(private_param("command_timeout_sec", 12.0))
        self.stable_frames_required = int(private_param("stable_frames_required", 4))
        self.valid_colors = set(private_param("valid_colors", ["red", "green", "blue"]))
        self.servo = private_param("servo", {})
        self.pick_cfg = private_param("pick", {})
        self.place_cfg = private_param("place", {})
        self.gripper_cfg = private_param("gripper", {})

        self.object_target = VisionTarget(
            private_param("object_detected_topic", "/vision/object/detected"),
            private_param("object_offset_x_topic", "/vision/object/offset_x"),
            private_param("object_offset_y_topic", "/vision/object/offset_y"),
            private_param("object_area_topic", "/vision/object/area"),
            private_param("object_color_topic", "/vision/object/color"),
        )
        self.carried_color = ""
        self.lock = threading.Lock()
        rospy.Service("~pick", Trigger, self.handle_pick)
        rospy.Service("~place", Trigger, self.handle_place)
        rospy.loginfo("visual_pick_place_controller ready")

    def handle_pick(self, _req):
        with self.lock:
            ok, reason = self.align_until_ready(self.object_target, self.pick_cfg, require_color=True)
            if not ok:
                self.stop_base()
                return TriggerResponse(False, "pick failed: " + reason)
            color = self.object_target.color
            self.publish_pose(self.pick_cfg.get("pre_grasp_pose", []), self.pick_cfg.get("arm_step_sec", 0.7))
            self.set_gripper(self.gripper_cfg.get("close", 25))
            self.publish_pose(self.pick_cfg.get("lift_pose", []), self.pick_cfg.get("arm_step_sec", 0.7))
            self.carried_color = color
            self.stop_base()
            return TriggerResponse(True, color)

    def handle_place(self, _req):
        with self.lock:
            if self.carried_color not in self.valid_colors:
                return TriggerResponse(False, "no carried color")
            ok, reason = self.align_until_ready(self.object_target, self.place_cfg, require_color=True, expected_color=self.carried_color)
            if not ok:
                self.stop_base()
                return TriggerResponse(False, "place failed: " + reason)
            self.publish_pose(self.place_cfg.get("pre_place_pose", []), self.place_cfg.get("arm_step_sec", 0.7))
            self.set_gripper(self.gripper_cfg.get("open", 80))
            placed = self.carried_color
            self.carried_color = ""
            self.publish_pose(self.place_cfg.get("retreat_pose", []), self.place_cfg.get("arm_step_sec", 0.7))
            self.stop_base()
            return TriggerResponse(True, placed)

    def align_until_ready(self, target, cfg, require_color, expected_color=None):
        rate = rospy.Rate(self.rate_hz)
        start = rospy.Time.now()
        stable = 0
        while not rospy.is_shutdown() and (rospy.Time.now() - start).to_sec() < self.command_timeout_sec:
            if not target.recently_seen() or target.area < float(cfg.get("min_area", 0.0)):
                stable = 0
                self.publish_search_cmd()
                rate.sleep()
                continue
            if require_color and target.color not in self.valid_colors:
                stable = 0
                self.stop_base()
                rate.sleep()
                continue
            if expected_color and target.color != expected_color:
                stable = 0
                self.publish_search_cmd()
                rate.sleep()
                continue
            centered = abs(target.offset_x) <= float(cfg.get("center_x_tolerance", 0.1)) and abs(target.offset_y) <= float(cfg.get("center_y_tolerance", 0.15))
            close_enough = target.area >= float(cfg.get("ready_area", 2500.0))
            if centered and close_enough:
                stable += 1
                self.stop_base()
                if stable >= self.stable_frames_required:
                    return True, "ready"
            else:
                stable = 0
                self.publish_servo_cmd(target, cfg)
            rate.sleep()
        return False, "timeout"

    def publish_servo_cmd(self, target, cfg):
        if not bool(cfg.get("use_base_servo", True)):
            self.stop_base()
            return
        cmd = Twist()
        if abs(target.offset_x) > float(cfg.get("center_x_tolerance", 0.1)):
            angular = float(self.servo.get("angular_sign", -1.0)) * float(self.servo.get("kp_angular", 0.3)) * target.offset_x
            cmd.angular.z = clamp(angular, -abs(float(self.servo.get("max_angular_z", 0.18))), abs(float(self.servo.get("max_angular_z", 0.18))))
        elif target.area < float(cfg.get("ready_area", 2500.0)):
            cmd.linear.x = float(self.servo.get("forward_sign", 1.0)) * float(self.servo.get("approach_speed", 0.025))
        self.cmd_pub.publish(cmd)

    def publish_search_cmd(self):
        cmd = Twist()
        cmd.angular.z = 0.08
        self.cmd_pub.publish(cmd)

    def publish_pose(self, pose, step_sec):
        if not pose:
            return
        values = [int(v) for v in pose]
        if len(values) != 6:
            rospy.logwarn("skip arm pose with %d values: %s", len(values), pose)
            return
        self.arm_pub.publish(Int16MultiArray(data=values))
        rospy.sleep(float(step_sec))

    def set_gripper(self, value):
        self.gripper_pub.publish(UInt8(data=int(clamp(int(value), 0, 100))))
        rospy.sleep(float(self.gripper_cfg.get("settle_sec", 0.6)))

    def stop_base(self):
        self.cmd_pub.publish(Twist())


if __name__ == "__main__":
    rospy.init_node("visual_pick_place")
    node = VisualPickPlaceController()
    rospy.on_shutdown(node.stop_base)
    rospy.spin()