#!/usr/bin/env python3
"""Low-speed line-follow controller for staged map testing.

This node intentionally does only the base closed loop:
vision line error -> /cmd_vel.  Route decisions, picking and placing should be
added in later task nodes after this layer is verified on the real map.
"""

import math

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


class LineFollowController:
    def __init__(self):
        self.line_visible_topic = private_param("line_visible_topic", "/vision/line/visible")
        self.line_error_topic = private_param("line_error_topic", "/vision/line/error")
        self.intersection_topic = private_param("intersection_topic", "/vision/intersection/detected")
        self.cmd_vel_topic = private_param("cmd_vel_topic", "/cmd_vel")

        self.publish_cmd_vel = bool(private_param("publish_cmd_vel", False))
        self.rate_hz = float(private_param("rate_hz", 20.0))
        self.forward_speed = float(private_param("forward_speed", 0.06))
        self.intersection_forward_speed = float(private_param("intersection_forward_speed", 0.03))
        self.kp_angular = float(private_param("kp_angular", 0.45))
        self.max_angular_z = abs(float(private_param("max_angular_z", 0.35)))
        self.error_deadband = abs(float(private_param("error_deadband", 0.03)))
        self.lost_timeout = float(private_param("lost_timeout", 0.30))
        self.stop_on_intersection = bool(private_param("stop_on_intersection", False))
        self.slow_on_intersection = bool(private_param("slow_on_intersection", True))
        self.angular_sign = float(private_param("angular_sign", -1.0))

        self.visible = False
        self.error = 0.0
        self.intersection = False
        self.last_visible_time = rospy.Time(0)
        self.last_log_time = rospy.Time(0)

        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        rospy.Subscriber(self.line_visible_topic, Bool, self.handle_visible, queue_size=10)
        rospy.Subscriber(self.line_error_topic, Float32, self.handle_error, queue_size=10)
        rospy.Subscriber(self.intersection_topic, Bool, self.handle_intersection, queue_size=10)

        period = 1.0 / self.rate_hz if self.rate_hz > 0.0 else 0.05
        self.timer = rospy.Timer(rospy.Duration(period), self.update)

        rospy.loginfo(
            "line_follow_controller started publish_cmd_vel=%s forward=%.3f kp=%.3f max_w=%.3f sign=%.1f",
            self.publish_cmd_vel,
            self.forward_speed,
            self.kp_angular,
            self.max_angular_z,
            self.angular_sign,
        )

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

    def build_cmd(self, now):
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

    def update(self, _event):
        now = rospy.Time.now()
        cmd, reason = self.build_cmd(now)

        if self.publish_cmd_vel:
            self.cmd_pub.publish(cmd)

        if (now - self.last_log_time).to_sec() > 1.0:
            rospy.loginfo(
                "line_follow reason=%s visible=%s err=%.3f intersection=%s cmd=(%.3f, %.3f) publish=%s",
                reason,
                self.visible,
                self.error,
                self.intersection,
                cmd.linear.x,
                cmd.angular.z,
                self.publish_cmd_vel,
            )
            self.last_log_time = now

    def stop(self):
        if self.publish_cmd_vel:
            self.cmd_pub.publish(Twist())


if __name__ == "__main__":
    rospy.init_node("line_follow_controller")
    node = LineFollowController()
    rospy.on_shutdown(node.stop)
    rospy.spin()