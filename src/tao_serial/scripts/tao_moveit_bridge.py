#!/usr/bin/env python3
import threading

import actionlib
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16MultiArray, String


class TaoMoveItBridge:
    def __init__(self):
        self.arm_joint_names = rospy.get_param(
            "~arm_joint_names",
            ["arm_0_joint", "arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint"],
        )
        self.protocol_joint_count = int(rospy.get_param("~protocol_joint_count", 6))
        self.protocol_offsets = list(rospy.get_param("~protocol_offsets", [0.0, 0.0, 1.602, 1.523, 0.0, 0.0]))
        self.protocol_signs = list(rospy.get_param("~protocol_signs", [1, 1, 1, 1, 1, 1]))
        self.protocol_scale = float(rospy.get_param("~protocol_scale", 1000.0))
        self.protocol_min = int(rospy.get_param("~protocol_min", -30000))
        self.protocol_max = int(rospy.get_param("~protocol_max", 30000))
        self.default_duration_ms = int(rospy.get_param("~default_duration_ms", 500))
        self.min_duration_ms = int(rospy.get_param("~min_duration_ms", 50))
        self.max_duration_ms = int(rospy.get_param("~max_duration_ms", 5000))
        self.command_topic = rospy.get_param("~command_topic", "/tao_arm/joints_protocol_units")
        self.tx_topic = rospy.get_param("~tx_topic", "/tao_serial/tx")
        self.use_tx_command = bool(rospy.get_param("~use_tx_command", True))
        self.publish_shadow_topic = bool(rospy.get_param("~publish_shadow_topic", True))
        self.auto_set_mode = bool(rospy.get_param("~auto_set_mode", True))
        self.publish_joint_states = bool(rospy.get_param("~publish_joint_states", True))
        self.joint_state_topic = rospy.get_param("~joint_state_topic", "/joint_states")

        self._validate_params()

        self.current_positions = [self.protocol_offsets[index] for index in range(len(self.arm_joint_names))]
        self.arm_seq = 0
        self.lock = threading.Lock()

        self.command_pub = rospy.Publisher(self.command_topic, Int16MultiArray, queue_size=10)
        self.tx_pub = rospy.Publisher(self.tx_topic, String, queue_size=10, latch=True)
        self.joint_state_pub = rospy.Publisher(self.joint_state_topic, JointState, queue_size=10) if self.publish_joint_states else None

        action_name = rospy.get_param("~action_name", "arm_controller/follow_joint_trajectory")
        self.server = actionlib.SimpleActionServer(action_name, FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start=False)
        self.server.start()

        if self.auto_set_mode:
            rospy.Timer(rospy.Duration(1.0), self.publish_auto_mode, oneshot=True)
        if self.publish_joint_states:
            rospy.Timer(rospy.Duration(0.05), self.publish_current_joint_state)

        rospy.loginfo("tao_moveit_bridge action=%s command_topic=%s", action_name, self.command_topic)

    def _validate_params(self):
        if self.protocol_joint_count < len(self.arm_joint_names):
            raise rospy.ROSException("protocol_joint_count must cover all arm joints")
        if len(self.protocol_offsets) != self.protocol_joint_count:
            raise rospy.ROSException("protocol_offsets length must equal protocol_joint_count")
        if len(self.protocol_signs) != self.protocol_joint_count:
            raise rospy.ROSException("protocol_signs length must equal protocol_joint_count")

    def publish_auto_mode(self, _event):
        self.tx_pub.publish(String(data="SET_MODE ROS_AUTO"))

    def execute_cb(self, goal):
        trajectory = goal.trajectory
        if not trajectory.points:
            self.abort_goal("trajectory has no points")
            return

        joint_indices = self.resolve_joint_indices(trajectory.joint_names)
        if joint_indices is None:
            return

        feedback = FollowJointTrajectoryFeedback()
        feedback.joint_names = self.arm_joint_names

        start_time = rospy.Time.now()
        previous_time = rospy.Duration(0.0)

        for point in trajectory.points:
            if rospy.is_shutdown():
                return
            if self.server.is_preempt_requested():
                self.server.set_preempted()
                return

            wait_time = point.time_from_start - previous_time
            if wait_time.to_sec() > 0:
                rospy.sleep(wait_time)
            previous_time = point.time_from_start

            arm_positions = [point.positions[joint_indices[name]] for name in self.arm_joint_names]
            duration_ms = self.duration_to_ms(wait_time)
            protocol_values = self.positions_to_protocol(arm_positions)
            self.publish_arm_command(protocol_values, duration_ms)

            with self.lock:
                self.current_positions = list(arm_positions)

            feedback.header.stamp = rospy.Time.now()
            feedback.actual.positions = arm_positions
            feedback.desired.positions = arm_positions
            feedback.error.positions = [0.0] * len(arm_positions)
            self.server.publish_feedback(feedback)

        result = FollowJointTrajectoryResult()
        result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
        result.error_string = "sent trajectory to tao serial protocol bridge in %.3fs" % (rospy.Time.now() - start_time).to_sec()
        self.server.set_succeeded(result)

    def resolve_joint_indices(self, trajectory_joint_names):
        joint_indices = {name: index for index, name in enumerate(trajectory_joint_names)}
        missing = [name for name in self.arm_joint_names if name not in joint_indices]
        if missing:
            self.abort_goal("trajectory missing joints: %s" % ", ".join(missing))
            return None
        return joint_indices

    def positions_to_protocol(self, arm_positions):
        protocol_values = [0] * self.protocol_joint_count
        for index, position in enumerate(arm_positions):
            value = int(round((position - self.protocol_offsets[index]) * self.protocol_scale * int(self.protocol_signs[index])))
            protocol_values[index] = max(self.protocol_min, min(self.protocol_max, value))
        return protocol_values

    def duration_to_ms(self, duration):
        duration_ms = int(round(duration.to_sec() * 1000.0))
        if duration_ms <= 0:
            duration_ms = self.default_duration_ms
        return max(self.min_duration_ms, min(self.max_duration_ms, duration_ms))

    def publish_arm_command(self, protocol_values, duration_ms):
        if self.publish_shadow_topic:
            self.command_pub.publish(Int16MultiArray(data=protocol_values))
        if not self.use_tx_command:
            return

        self.arm_seq = (self.arm_seq + 1) & 0xFF
        command = "ARM_JOINTS %d %s %d" % (
            self.arm_seq,
            " ".join(str(value) for value in protocol_values),
            duration_ms,
        )
        self.tx_pub.publish(String(data=command))

    def publish_current_joint_state(self, _event):
        with self.lock:
            positions = list(self.current_positions)

        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.arm_joint_names
        msg.position = positions
        self.joint_state_pub.publish(msg)

    def abort_goal(self, message):
        rospy.logerr(message)
        result = FollowJointTrajectoryResult()
        result.error_code = FollowJointTrajectoryResult.INVALID_GOAL
        result.error_string = message
        self.server.set_aborted(result)


def main():
    rospy.init_node("tao_moveit_bridge")
    TaoMoveItBridge()
    rospy.spin()


if __name__ == "__main__":
    main()