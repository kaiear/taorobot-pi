#!/usr/bin/env python3
import struct

import rospy
from std_msgs.msg import String

try:
    import serial
except ImportError:
    serial = None


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
        self.serial_port = None
        self.rx_buffer = bytearray()
        self.rx_pub = rospy.Publisher("~rx", String, queue_size=10)
        self.tx_sub = rospy.Subscriber("~tx", String, self.handle_tx, queue_size=10)

        rospy.loginfo("tao_serial_node starting")
        rospy.loginfo("port=%s baudrate=%d open_serial=%s", self.port, self.baudrate, self.open_serial)

        if self.open_serial:
            self.open_serial_port()

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
        rospy.loginfo("TX request: %s", msg.data)
        self.write_line(msg.data)

    def write_line(self, text):
        if self.serial_port is None:
            return

        line = text.rstrip("\r\n") + "\n"
        self.serial_port.write(line.encode("ascii"))
        rospy.loginfo("TX: %s", line.strip())

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
            if self.rx_buffer.startswith(b"PONG\n"):
                del self.rx_buffer[:5]
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
            rospy.loginfo("RX frame: %s", message)
            self.rx_pub.publish(message)

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
        rate = rospy.Rate(1.0 / self.ping_period if self.ping_period > 0 else 1.0)
        while not rospy.is_shutdown():
            if self.serial_port is None:
                self.rx_pub.publish("PING skeleton alive")
            else:
                self.write_line("PING")
                self.read_available_data()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("tao_serial_node")
    TaoSerialNode().spin()