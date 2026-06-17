#!/usr/bin/env python3
import os
import json
import struct
import sys
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16MultiArray, String, UInt8

try:
    import serial
except ImportError:
    serial = None

sys.path.insert(0, os.path.dirname(__file__))
import serial_protocol as proto


class TaoSerialNode:
    FRAME_HEADER = b"\xAA\x55"
    FRAME_TAIL = 0x7D
    STM_TO_ROS_PAYLOAD_LEN = 32
    STM_TO_ROS_FRAME_LEN = 2 + STM_TO_ROS_PAYLOAD_LEN + 1 + 1

    def __init__(self):
        self.port = rospy.get_param("~port", "/dev/ttyS0")
        self.baudrate = int(rospy.get_param("~baudrate", 115200))
        self.open_serial = bool(rospy.get_param("~open_serial", False))
        self.ping_period = float(rospy.get_param("~ping_period", 1.0))
        self.timeout = float(rospy.get_param("~timeout", 0.1))
        self.control_rate_hz = float(rospy.get_param("~control_rate_hz", 20.0))
        self.cmd_timeout = float(rospy.get_param("~cmd_timeout", 0.5))
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.buzzer_topic = rospy.get_param("~buzzer_topic", "/buzzer/play")
        self.gripper_topic = rospy.get_param("~gripper_topic", "/gripper/command")
        self.arm_joints_topic = rospy.get_param("~arm_joints_topic", "/tao_arm/joints_protocol_units")
        self.status_topic = rospy.get_param("~status_topic", "/tao_serial/status_json")
        self.ack_topic = rospy.get_param("~ack_topic", "/tao_serial/ack_json")
        self.error_topic = rospy.get_param("~error_topic", "/tao_serial/error_json")
        self.pong_topic = rospy.get_param("~pong_topic", "/tao_serial/pong_json")
        self.publish_joint_states = bool(rospy.get_param("~publish_joint_states", True))
        self.joint_state_topic = rospy.get_param("~joint_state_topic", "/joint_states")
        self.joint_state_names = list(
            rospy.get_param(
                "~joint_state_names",
                ["arm_0_joint", "arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint"],
            )
        )
        self.joint_state_scale = float(rospy.get_param("~joint_state_scale", 1000.0))
        self.auto_set_mode = bool(rospy.get_param("~auto_set_mode", True))
        self.log_tx = bool(rospy.get_param("~log_tx", False))
        self.log_rx = bool(rospy.get_param("~log_rx", True))
        self.serial_port = None
        self.rx_buffer = bytearray()
        self.lock = threading.Lock()
        self.last_cmd_vel = (0, 0, 0)
        self.last_cmd_time = rospy.Time(0)
        self.mode_sent = False
        self.rx_pub = rospy.Publisher("~rx", String, queue_size=10)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10)
        self.ack_pub = rospy.Publisher(self.ack_topic, String, queue_size=10)
        self.error_pub = rospy.Publisher(self.error_topic, String, queue_size=10)
        self.pong_pub = rospy.Publisher(self.pong_topic, String, queue_size=10)
        self.joint_state_pub = rospy.Publisher(self.joint_state_topic, JointState, queue_size=10) if self.publish_joint_states else None
        self.tx_sub = rospy.Subscriber("~tx", String, self.handle_tx, queue_size=10)
        self.cmd_vel_sub = rospy.Subscriber(self.cmd_vel_topic, Twist, self.handle_cmd_vel, queue_size=10)
        self.buzzer_sub = rospy.Subscriber(self.buzzer_topic, UInt8, self.handle_buzzer, queue_size=10)
        self.gripper_sub = rospy.Subscriber(self.gripper_topic, UInt8, self.handle_gripper, queue_size=10)
        self.arm_joints_sub = rospy.Subscriber(self.arm_joints_topic, Int16MultiArray, self.handle_arm_joints, queue_size=10)
        self.arm_seq = 0

        rospy.loginfo("tao_serial_node starting")
        rospy.loginfo("port=%s baudrate=%d open_serial=%s", self.port, self.baudrate, self.open_serial)

        if self.open_serial:
            self.open_serial_port()

    def handle_cmd_vel(self, msg):
        vx = int(round(msg.linear.x * 1000.0))
        vy = int(round(msg.linear.y * 1000.0))
        wz = int(round(msg.angular.z * 1000.0))
        with self.lock:
            self.last_cmd_vel = (vx, vy, wz)
            self.last_cmd_time = rospy.Time.now()
        rospy.loginfo("cmd_vel -> base_vel vx=%d vy=%d wz=%d", vx, vy, wz)

    def handle_buzzer(self, msg):
        melody_id = int(msg.data)
        repeat = 1
        rospy.loginfo("buzzer topic -> BUZZER melody=%d repeat=%d", melody_id, repeat)
        self.send_buzzer(melody_id, repeat)

    def handle_gripper(self, msg):
        percent = max(0, min(100, int(msg.data)))
        rospy.loginfo("gripper topic -> GRIPPER percent=%d", percent)
        self.send_gripper(percent)

    def handle_arm_joints(self, msg):
        joints = [int(value) for value in msg.data]
        if len(joints) != proto.JOINT_COUNT:
            rospy.logerr("ARM_JOINTS requires %d values, got %d", proto.JOINT_COUNT, len(joints))
            return

        self.arm_seq = (self.arm_seq + 1) & 0xFF
        duration_ms = 500
        rospy.loginfo("arm topic -> ARM_JOINTS seq=%d joints=%s duration_ms=%d", self.arm_seq, joints, duration_ms)
        self.send_arm_joints(self.arm_seq, joints, duration_ms)

    def send_v2_frame(self, frame):
        if frame:
            self.write_bytes(frame)

    def send_set_mode(self, mode):
        self.send_v2_frame(proto.encode_set_mode(mode))

    def send_heartbeat(self):
        self.send_v2_frame(proto.encode_heartbeat(1))

    def send_base_vel(self, vx, vy, wz):
        self.send_v2_frame(proto.encode_base_vel(vx, vy, wz))

    def send_buzzer(self, melody_id, repeat):
        self.send_v2_frame(proto.encode_buzzer(melody_id, repeat))

    def send_gripper(self, percent):
        self.send_v2_frame(proto.encode_gripper(percent))

    def send_arm_joints(self, seq, joints, duration_ms):
        self.send_v2_frame(proto.encode_arm_joints(seq, joints, duration_ms))

    def send_stop(self):
        self.send_v2_frame(proto.encode_stop())

    def open_serial_port(self):
        if serial is None:
            rospy.logerr("pyserial is not installed. Install python3-serial on the Raspberry Pi.")
            rospy.signal_shutdown("missing pyserial")
            return

        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=self.timeout,
            )
            rospy.loginfo("Opened serial port %s at %d baud", self.port, self.baudrate)
        except serial.SerialException as exc:
            rospy.logerr("Failed to open serial port %s: %s", self.port, exc)
            rospy.signal_shutdown("serial open failed")

    def handle_tx(self, msg):
        if self.log_tx:
            rospy.loginfo("TX request: %s", msg.data)
        frame = self.build_v2_command(msg.data)
        if frame is None:
            self.write_line(msg.data)
            return
        if frame:
            self.write_bytes(frame)

    def write_line(self, text):
        if self.serial_port is None:
            return

        line = text.rstrip("\r\n") + "\n"
        self.serial_port.write(line.encode("ascii"))
        if self.log_tx:
            rospy.loginfo("TX: %s", line.strip())

    def write_bytes(self, data):
        if self.serial_port is None:
            return

        self.serial_port.write(data)
        if self.log_tx:
            rospy.loginfo("TX v2: %s", data.hex(" ").upper())

    def build_v2_command(self, text):
        parts = text.strip().split()
        if not parts:
            return None

        command = parts[0].upper()
        try:
            if command == "STOP":
                return proto.encode_stop()
            if command == "SET_MODE" and len(parts) == 2:
                mode_name = parts[1].upper()
                mode = getattr(proto.Mode, mode_name) if hasattr(proto.Mode, mode_name) else int(parts[1], 0)
                return proto.encode_set_mode(mode)
            if command == "PING":
                time_ms = int(parts[1], 0) if len(parts) == 2 else int(time.time() * 1000) & 0xFFFFFFFF
                return proto.encode_ping(time_ms)
            if command == "BASE_VEL" and len(parts) == 4:
                return proto.encode_base_vel(int(parts[1]), int(parts[2]), int(parts[3]))
            if command == "ARM_JOINTS" and len(parts) == 9:
                seq = int(parts[1], 0)
                joints = [int(value) for value in parts[2:8]]
                duration_ms = int(parts[8], 0)
                return proto.encode_arm_joints(seq, joints, duration_ms)
            if command == "GRIPPER" and len(parts) == 2:
                return proto.encode_gripper(int(parts[1], 0))
            if command == "ARM_PRESET" and len(parts) == 2:
                return proto.encode_arm_preset(int(parts[1], 0))
            if command == "BUZZER" and len(parts) == 3:
                return proto.encode_buzzer(int(parts[1], 0), int(parts[2], 0))
            if command == "HEARTBEAT" and len(parts) == 2:
                return proto.encode_heartbeat(int(parts[1], 0))
        except (ValueError, struct.error) as exc:
            rospy.logerr("Invalid v2 command '%s': %s", text, exc)
            return b""

        return None

    def read_available_data(self):
        if self.serial_port is None:
            return

        waiting = self.serial_port.in_waiting
        if waiting <= 0:
            return

        self.rx_buffer.extend(self.serial_port.read(waiting))
        self.parse_rx_buffer()

    def parse_rx_buffer(self):
        while self.rx_buffer:
            message, consumed = proto.try_decode_frame(self.rx_buffer)
            if consumed:
                del self.rx_buffer[:consumed]
                if message is None:
                    continue
                summary = proto.describe_message(message)
                if self.log_rx:
                    rospy.loginfo("RX v2 frame: %s", summary)
                self.rx_pub.publish(summary)
                self.publish_decoded_message(message)
                continue

            if self.rx_buffer.startswith(b"PONG\n"):
                del self.rx_buffer[:5]
                if self.log_rx:
                    rospy.loginfo("RX text: PONG")
                self.rx_pub.publish("PONG")
                continue

            header_index = self.rx_buffer.find(self.FRAME_HEADER)
            pong_index = self.rx_buffer.find(b"PONG\n")

            if header_index < 0:
                if pong_index < 0:
                    self.drop_noise_bytes(keep_last=4)
                    return
                self.drop_noise_bytes(keep_last=len(self.rx_buffer) - pong_index)
                continue

            if pong_index >= 0 and pong_index < header_index:
                self.drop_noise_bytes(keep_last=len(self.rx_buffer) - pong_index)
                continue

            if header_index > 0:
                del self.rx_buffer[:header_index]

            if len(self.rx_buffer) < self.STM_TO_ROS_FRAME_LEN:
                return

            frame = bytes(self.rx_buffer[:self.STM_TO_ROS_FRAME_LEN])
            if not self.is_valid_stm_frame(frame):
                del self.rx_buffer[0]
                continue

            del self.rx_buffer[:self.STM_TO_ROS_FRAME_LEN]
            message = self.decode_stm_frame(frame)
            if self.log_rx:
                rospy.loginfo("RX frame: %s", message)
            self.rx_pub.publish(message)

    def publish_decoded_message(self, message):
        decoded = proto.decode_message(message)
        json_text = json.dumps(decoded, sort_keys=True, separators=(",", ":"))
        msg_type = decoded.get("type")

        if msg_type == proto.MsgType.STATUS:
            self.status_pub.publish(json_text)
            self.publish_status_joint_state(decoded)
            return
        if msg_type == proto.MsgType.ACK:
            self.ack_pub.publish(json_text)
            return
        if msg_type == proto.MsgType.ERROR:
            self.error_pub.publish(json_text)
            return
        if msg_type == proto.MsgType.PONG:
            self.pong_pub.publish(json_text)
            return

    def publish_status_joint_state(self, decoded):
        if self.joint_state_pub is None or not decoded.get("valid_length", False):
            return

        joints = decoded.get("joints", [])
        if not joints:
            return

        count = min(len(joints), len(self.joint_state_names))
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.joint_state_names[:count]
        msg.position = [float(value) / self.joint_state_scale for value in joints[:count]]
        self.joint_state_pub.publish(msg)

    def drop_noise_bytes(self, keep_last):
        drop_len = max(0, len(self.rx_buffer) - keep_last)
        if drop_len:
            rospy.logdebug("Dropping %d serial noise bytes", drop_len)
            del self.rx_buffer[:drop_len]

    def is_valid_stm_frame(self, frame):
        if frame[:2] != self.FRAME_HEADER:
            return False
        if frame[-1] != self.FRAME_TAIL:
            return False
        checksum = sum(frame[:-2]) & 0xFF
        return checksum == frame[-2]

    def decode_stm_frame(self, frame):
        payload = frame[2:-2]
        values = struct.unpack(">16h", payload)
        acc_x, acc_y, acc_z = values[0:3]
        gyro_x, gyro_y, gyro_z = values[3:6]
        vel_x, vel_y, vel_w = values[6:9]
        arm = values[9:15]
        battery_x100 = values[15]

        return (
            "stm_data "
            "acc=(%d,%d,%d) gyro=(%d,%d,%d) "
            "vel_mm_s=(%d,%d,%d) arm=(%d,%d,%d,%d,%d,%d) bat_x100=%d"
            % (
                acc_x,
                acc_y,
                acc_z,
                gyro_x,
                gyro_y,
                gyro_z,
                vel_x,
                vel_y,
                vel_w,
                arm[0],
                arm[1],
                arm[2],
                arm[3],
                arm[4],
                arm[5],
                battery_x100,
            )
        )

    def spin(self):
        rate_hz = self.control_rate_hz if self.control_rate_hz > 0 else 20.0
        rate = rospy.Rate(rate_hz)
        while not rospy.is_shutdown():
            if self.serial_port is None:
                self.rx_pub.publish("PING skeleton alive")
            else:
                if self.auto_set_mode and not self.mode_sent:
                    self.send_set_mode(proto.Mode.ROS_AUTO)
                    self.mode_sent = True

                self.send_heartbeat()

                with self.lock:
                    last_cmd_vel = self.last_cmd_vel
                    last_cmd_time = self.last_cmd_time

                if last_cmd_time == rospy.Time(0) or (rospy.Time.now() - last_cmd_time).to_sec() > self.cmd_timeout:
                    last_cmd_vel = (0, 0, 0)

                self.send_base_vel(*last_cmd_vel)
                self.read_available_data()
            rate.sleep()

        if self.serial_port is not None:
            self.send_stop()


if __name__ == "__main__":
    rospy.init_node("tao_serial_node")
    TaoSerialNode().spin()