#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Direct joint-template tuner for the Yeahbot arm serial protocol.

This node intentionally does not use vision, MoveIt, or IK.  It sends the same
ARM_JOINTS protocol used by line_follow_controller.py, while keeping the current
target pose in degrees so tuned poses can be copied directly into YAML presets.

Commands are published as std_msgs/String to /arm_pose_tuner/command:
  goto <preset>              move to a named YAML preset
  jog <joint> <delta_deg>    add delta degrees to joint 1..6 (also accepts 0..5)
  set <j1> ... <j6>          set all six joints in degrees
  open                       set gripper joint to gripper_open_deg
  close                      set gripper joint to gripper_close_deg
  dump [name]                print copyable YAML for current pose
  list                       list preset names
  help                       print command help
"""

from __future__ import print_function

import math
import shlex

import rospy
from std_msgs.msg import String, UInt8
from std_msgs.msg import Int16MultiArray


def clamp(value, low, high):
    return max(low, min(high, value))


def as_float_list(value, default):
    if value is None:
        return list(default)
    try:
        return [float(v) for v in value]
    except TypeError:
        return list(default)


def private_param(name, default=None):
    # Support both parameters loaded under the node namespace and parameters
    # loaded as a top-level arm_pose_tuner dictionary.
    if rospy.has_param("~" + name):
        return rospy.get_param("~" + name, default)
    data = rospy.get_param("~arm_pose_tuner", None)
    if isinstance(data, dict) and name in data:
        return data.get(name, default)
    return rospy.get_param("arm_pose_tuner/" + name, default)


class ArmPoseTuner(object):
    def __init__(self):
        self.command_topic = private_param("command_topic", "/arm_pose_tuner/command")
        self.serial_tx_topic = private_param("serial_tx_topic", "/tao_serial/tx")
        self.arm_joints_topic = private_param("arm_joints_topic", "/tao_arm/joints_protocol_units")
        self.gripper_topic = private_param("gripper_topic", "/gripper/command")

        self.duration_ms = int(private_param("duration_ms", 800))
        self.step_deg = float(private_param("step_deg", 2.0))
        self.big_step_deg = float(private_param("big_step_deg", 10.0))
        self.gripper_joint_index = int(private_param("gripper_joint_index", 5))
        self.gripper_open_deg = float(private_param("gripper_open_deg", 0.0))
        self.gripper_close_deg = float(private_param("gripper_close_deg", 35.0))

        self.use_tx_command = bool(private_param("use_tx_command", True))
        self.publish_arm_shadow_topic = bool(private_param("publish_arm_shadow_topic", True))
        self.serial_tx_wait_timeout = float(private_param("serial_tx_wait_timeout", 1.0))
        self.protocol_offsets = as_float_list(private_param("protocol_offsets", [0, 0, 0, 0, 0, 0]), [0, 0, 0, 0, 0, 0])
        self.protocol_trim_degrees = as_float_list(private_param("protocol_trim_degrees", [0, 0, 0, 0, 0, 0]), [0, 0, 0, 0, 0, 0])
        self.protocol_signs = [int(v) for v in private_param("protocol_signs", [1, 1, 1, 1, 1, 1])]
        self.protocol_scale = float(private_param("protocol_scale", 1000.0))
        self.protocol_min = int(private_param("protocol_min", -32768))
        self.protocol_max = int(private_param("protocol_max", 32767))

        self.presets = private_param("presets", {}) or {}
        self.current_pose_name = "manual"
        start_pose = private_param("start_pose", "safe_home")
        self.current_joints_deg = self.get_preset(start_pose)
        if self.current_joints_deg is None:
            self.current_joints_deg = [0.0] * 6
            self.current_pose_name = "manual"
        else:
            self.current_pose_name = str(start_pose)

        self.arm_seq = 0
        self.serial_tx_pub = rospy.Publisher(self.serial_tx_topic, String, queue_size=10)
        self.arm_units_pub = rospy.Publisher(self.arm_joints_topic, Int16MultiArray, queue_size=10)
        self.gripper_pub = rospy.Publisher(self.gripper_topic, UInt8, queue_size=10)
        self.command_sub = rospy.Subscriber(self.command_topic, String, self.handle_command, queue_size=10)

        rospy.loginfo("arm_pose_tuner started command_topic=%s serial_tx_topic=%s", self.command_topic, self.serial_tx_topic)
        self.print_help()
        self.log_current("initial")
        if bool(private_param("auto_start", False)):
            self.send_current("auto_start")

    def get_preset(self, name):
        if not name or name not in self.presets:
            return None
        values = as_float_list(self.presets.get(name), [])
        if len(values) < 6:
            rospy.logwarn("preset %s has fewer than 6 joints: %s", name, values)
            return None
        return values[:6]

    def joints_deg_to_protocol(self, joints_deg):
        values = []
        for index in range(6):
            deg = float(joints_deg[index]) if index < len(joints_deg) else 0.0
            trim_deg = self.protocol_trim_degrees[index] if index < len(self.protocol_trim_degrees) else 0.0
            joint = (deg + trim_deg) * math.pi / 180.0
            offset = self.protocol_offsets[index] if index < len(self.protocol_offsets) else 0.0
            sign = self.protocol_signs[index] if index < len(self.protocol_signs) else 1
            value = int(round((joint - offset) * self.protocol_scale * sign))
            values.append(int(clamp(value, self.protocol_min, self.protocol_max)))
        return values

    def wait_for_publisher_connections(self, pub, timeout_sec, name):
        if timeout_sec <= 0.0 or pub.get_num_connections() > 0:
            return
        deadline = rospy.Time.now() + rospy.Duration(timeout_sec)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and pub.get_num_connections() == 0 and rospy.Time.now() < deadline:
            rate.sleep()
        if pub.get_num_connections() == 0:
            rospy.logwarn("%s publisher has no subscribers; command may be missed", name)

    def send_current(self, reason="manual"):
        values = self.joints_deg_to_protocol(self.current_joints_deg)
        duration_ms = int(clamp(self.duration_ms, 50, 5000))
        if self.publish_arm_shadow_topic:
            shadow = Int16MultiArray()
            shadow.data = [int(clamp(value, -32768, 32767)) for value in values]
            self.arm_units_pub.publish(shadow)
        serial = ""
        if self.use_tx_command:
            self.wait_for_publisher_connections(self.serial_tx_pub, self.serial_tx_wait_timeout, "serial tx")
            self.arm_seq = (self.arm_seq + 1) & 0xFF
            serial = "ARM_JOINTS {} {} {}".format(self.arm_seq, " ".join(str(v) for v in values), duration_ms)
            self.serial_tx_pub.publish(String(serial))
        self.log_current(reason, values, serial)

    def log_current(self, reason, protocol_values=None, serial=""):
        if protocol_values is None:
            protocol_values = self.joints_deg_to_protocol(self.current_joints_deg)
        rospy.loginfo(
            "arm tuner %s pose=%s joints_deg=%s trim_deg=%s protocol_values=%s serial='%s'",
            reason,
            self.current_pose_name,
            [round(v, 3) for v in self.current_joints_deg],
            [round(v, 3) for v in self.protocol_trim_degrees],
            protocol_values,
            serial,
        )

    def parse_joint_index(self, token):
        raw = int(token)
        # Prefer human-friendly 1..6, but accept 0..5.
        if 1 <= raw <= 6:
            return raw - 1
        if 0 <= raw <= 5:
            return raw
        raise ValueError("joint index must be 1..6 or 0..5")

    def set_gripper(self, closed):
        idx = int(clamp(self.gripper_joint_index, 0, 5))
        self.current_joints_deg[idx] = self.gripper_close_deg if closed else self.gripper_open_deg
        self.current_pose_name = "manual_close" if closed else "manual_open"
        grip = UInt8()
        grip.data = 0 if closed else 100
        self.gripper_pub.publish(grip)
        self.send_current("close" if closed else "open")

    def dump_current(self, name=None):
        key = name or self.current_pose_name or "manual_pose"
        line = "{}: [{}]".format(key, ", ".join("{:.3f}".format(v).rstrip("0").rstrip(".") for v in self.current_joints_deg))
        rospy.loginfo("copyable YAML preset:\n%s", line)
        rospy.loginfo("copy to arm_pick_template: %s_joints_deg: [%s]", key, ", ".join("{:.3f}".format(v).rstrip("0").rstrip(".") for v in self.current_joints_deg))

    def print_help(self):
        rospy.loginfo(
            "arm_pose_tuner commands: goto <preset> | jog <joint 1..6> <delta_deg> | set <j1> <j2> <j3> <j4> <j5> <j6> | open | close | dump [name] | list | help"
        )

    def list_presets(self):
        rospy.loginfo("available presets: %s", sorted(self.presets.keys()))

    def handle_command(self, msg):
        text = (msg.data or "").strip()
        if not text:
            return
        try:
            parts = shlex.split(text)
            if not parts:
                return
            cmd = parts[0].lower()
            if cmd == "goto":
                if len(parts) < 2:
                    raise ValueError("usage: goto <preset>")
                pose = self.get_preset(parts[1])
                if pose is None:
                    raise ValueError("unknown preset '{}'".format(parts[1]))
                self.current_joints_deg = pose
                self.current_pose_name = parts[1]
                self.send_current("goto")
            elif cmd == "jog":
                if len(parts) < 3:
                    raise ValueError("usage: jog <joint 1..6> <delta_deg>")
                idx = self.parse_joint_index(parts[1])
                delta = float(parts[2])
                self.current_joints_deg[idx] += delta
                self.current_pose_name = "manual"
                self.send_current("jog")
            elif cmd == "set":
                if len(parts) != 7:
                    raise ValueError("usage: set <j1> <j2> <j3> <j4> <j5> <j6>")
                self.current_joints_deg = [float(v) for v in parts[1:7]]
                self.current_pose_name = "manual"
                self.send_current("set")
            elif cmd == "open":
                self.set_gripper(False)
            elif cmd == "close":
                self.set_gripper(True)
            elif cmd == "dump":
                self.dump_current(parts[1] if len(parts) >= 2 else None)
            elif cmd == "list":
                self.list_presets()
            elif cmd == "help":
                self.print_help()
            else:
                raise ValueError("unknown command '{}'".format(cmd))
        except Exception as exc:
            rospy.logerr("arm_pose_tuner command failed: %s; input='%s'", exc, text)
            self.print_help()


def main():
    rospy.init_node("arm_pose_tuner")
    ArmPoseTuner()
    rospy.spin()


if __name__ == "__main__":
    main()
