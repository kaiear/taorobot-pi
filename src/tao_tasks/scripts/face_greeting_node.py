#!/usr/bin/env python3
"""Face-triggered greeting behavior with cooldown protection."""

import threading

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int16MultiArray, UInt8


def private_param(name, default):
    """Read either ~name or grouped ~face_greeting/name from rosparam YAML."""
    if rospy.has_param("~" + name):
        return rospy.get_param("~" + name)
    return rospy.get_param("~face_greeting/" + name, default)


class FaceGreetingNode:
    def __init__(self):
        self.face_detected_topic = private_param("face_detected_topic", "/vision/face/detected")
        self.cmd_vel_topic = private_param("cmd_vel_topic", "/cmd_vel")
        self.buzzer_topic = private_param("buzzer_topic", "/buzzer/play")
        self.arm_joints_topic = private_param("arm_joints_topic", "/tao_arm/joints_protocol_units")

        self.cooldown_sec = float(private_param("cooldown_sec", 20.0))
        self.trigger_on_rising_edge = bool(private_param("trigger_on_rising_edge", True))
        self.enable_buzzer = bool(private_param("enable_buzzer", True))
        self.buzzer_melody = int(private_param("buzzer_melody", 1))
        self.enable_base_wiggle = bool(private_param("enable_base_wiggle", True))
        self.base_angular_z = float(private_param("base_angular_z", 0.28))
        self.base_step_sec = float(private_param("base_step_sec", 0.35))
        self.base_cycles = int(private_param("base_cycles", 2))
        self.enable_arm_wave = bool(private_param("enable_arm_wave", False))
        self.arm_step_sec = float(private_param("arm_step_sec", 0.80))
        self.arm_wave_poses = private_param("arm_wave_poses", [])

        self.last_face = False
        self.last_trigger_time = rospy.Time(0)
        self.running = False
        self.lock = threading.Lock()

        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.buzzer_pub = rospy.Publisher(self.buzzer_topic, UInt8, queue_size=10)
        self.arm_pub = rospy.Publisher(self.arm_joints_topic, Int16MultiArray, queue_size=10)
        rospy.Subscriber(self.face_detected_topic, Bool, self.handle_face, queue_size=10)

        rospy.loginfo(
            "face_greeting_node started cooldown=%.1fs buzzer=%s base=%s arm=%s",
            self.cooldown_sec,
            self.enable_buzzer,
            self.enable_base_wiggle,
            self.enable_arm_wave,
        )

    def handle_face(self, msg):
        detected = bool(msg.data)
        rising = detected and not self.last_face
        self.last_face = detected

        if not detected:
            return
        if self.trigger_on_rising_edge and not rising:
            return

        now = rospy.Time.now()
        if self.last_trigger_time != rospy.Time(0) and (now - self.last_trigger_time).to_sec() < self.cooldown_sec:
            return

        with self.lock:
            if self.running:
                return
            self.running = True
            self.last_trigger_time = now

        threading.Thread(target=self.run_greeting, daemon=True).start()

    def run_greeting(self):
        rospy.loginfo("face greeting triggered")
        try:
            if self.enable_buzzer:
                self.buzzer_pub.publish(UInt8(data=max(0, min(255, self.buzzer_melody))))
            if self.enable_arm_wave:
                self.wave_arm()
            if self.enable_base_wiggle:
                self.wiggle_base()
        finally:
            self.stop_base()
            with self.lock:
                self.running = False

    def wave_arm(self):
        for pose in self.arm_wave_poses:
            if rospy.is_shutdown():
                return
            values = [int(v) for v in pose]
            if len(values) != 6:
                rospy.logwarn("Skipping arm wave pose with %d values: %s", len(values), pose)
                continue
            self.arm_pub.publish(Int16MultiArray(data=values))
            rospy.sleep(self.arm_step_sec)

    def wiggle_base(self):
        for _ in range(max(0, self.base_cycles)):
            if rospy.is_shutdown():
                return
            self.publish_turn(self.base_angular_z)
            rospy.sleep(self.base_step_sec)
            self.publish_turn(-self.base_angular_z)
            rospy.sleep(self.base_step_sec)

    def publish_turn(self, angular_z):
        cmd = Twist()
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)

    def stop_base(self):
        self.cmd_pub.publish(Twist())


if __name__ == "__main__":
    rospy.init_node("face_greeting_node")
    node = FaceGreetingNode()
    rospy.on_shutdown(node.stop_base)
    rospy.spin()