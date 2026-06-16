#!/usr/bin/env python3
import argparse
import os
import struct
import sys
import time


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
TAO_SERIAL_SCRIPTS = os.path.join(REPO_ROOT, "src", "tao_serial", "scripts")
if TAO_SERIAL_SCRIPTS not in sys.path:
    sys.path.insert(0, TAO_SERIAL_SCRIPTS)

try:
    import serial
except ImportError:
    serial = None

from serial_protocol import (
    Mode,
    MsgType,
    encode_arm_joints,
    describe_message,
    encode_base_vel,
    encode_buzzer,
    encode_frame,
    encode_gripper,
    encode_heartbeat,
    encode_ping,
    encode_set_mode,
    encode_stop,
    try_decode_frame,
)


MODE_NAMES = {
    "manual": Mode.MANUAL,
    "ros_auto": Mode.ROS_AUTO,
    "auto": Mode.ROS_AUTO,
    "estop": Mode.ESTOP,
    "safe_idle": Mode.SAFE_IDLE,
    "idle": Mode.SAFE_IDLE,
}


def parse_mode(value):
    key = value.lower().replace("-", "_")
    if key not in MODE_NAMES:
        raise argparse.ArgumentTypeError("mode must be one of: %s" % ", ".join(sorted(MODE_NAMES)))
    return MODE_NAMES[key]


def hexdump(data):
    return " ".join("%02X" % byte for byte in data)


def read_messages(port, timeout):
    deadline = time.time() + timeout
    buffer = bytearray()
    messages = []

    while time.time() < deadline:
        waiting = port.in_waiting if port.in_waiting else 1
        data = port.read(waiting)
        if data:
            buffer.extend(data)
            while buffer:
                message, consumed = try_decode_frame(buffer)
                if message is not None:
                    messages.append(message)
                    del buffer[:consumed]
                elif consumed > 0:
                    del buffer[:consumed]
                else:
                    break
        else:
            time.sleep(0.01)

    return messages, bytes(buffer)


def make_frame(args):
    if args.command == "drive-test":
        return None
    if args.command == "arm-joints":
        return encode_arm_joints(args.seq, args.joints, args.duration)
    if args.command == "ping":
        time_ms = int(time.time() * 1000) & 0xFFFFFFFF
        return encode_ping(time_ms)
    if args.command == "stop":
        return encode_stop()
    if args.command == "set-mode":
        return encode_set_mode(args.mode)
    if args.command == "base-vel":
        return encode_base_vel(args.vx, args.vy, args.wz)
    if args.command == "heartbeat":
        return encode_heartbeat(args.state)
    if args.command == "gripper":
        return encode_gripper(args.percent)
    if args.command == "buzzer":
        return encode_buzzer(args.melody, args.repeat)
    if args.command == "raw":
        payload = bytes.fromhex(args.payload) if args.payload else b""
        return encode_frame(args.type, payload)
    raise ValueError("unsupported command: %s" % args.command)


def add_common_args(parser):
    parser.add_argument("--port", required=True, help="serial port, for example COM3, /dev/ttyS0, /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200, help="baudrate, default: 115200")
    parser.add_argument("--timeout", type=float, default=1.0, help="read timeout seconds after write")
    parser.add_argument("--no-read", action="store_true", help="send only, do not wait for response")


def build_parser():
    parser = argparse.ArgumentParser(description="Send Tao v2 serial test frames to STM32F407")
    add_common_args(parser)
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("ping", help="send PING and expect PONG")
    subparsers.add_parser("stop", help="send STOP and expect ACK")

    set_mode = subparsers.add_parser("set-mode", help="send SET_MODE and expect ACK")
    set_mode.add_argument("mode", type=parse_mode, help="manual, ros_auto, estop, safe_idle")

    base_vel = subparsers.add_parser("base-vel", help="send BASE_VEL in protocol units")
    base_vel.add_argument("--vx", type=int, default=0, help="x velocity, m/s * 1000")
    base_vel.add_argument("--vy", type=int, default=0, help="y velocity, m/s * 1000")
    base_vel.add_argument("--wz", type=int, default=0, help="yaw velocity, rad/s * 1000")

    drive_test = subparsers.add_parser("drive-test", help="send heartbeat and BASE_VEL continuously, then stop")
    drive_test.add_argument("--vx", type=int, default=100, help="x velocity, m/s * 1000")
    drive_test.add_argument("--vy", type=int, default=0, help="y velocity, m/s * 1000")
    drive_test.add_argument("--wz", type=int, default=0, help="yaw velocity, rad/s * 1000")
    drive_test.add_argument("--duration", type=float, default=2.0, help="drive duration seconds")
    drive_test.add_argument("--rate", type=float, default=20.0, help="send rate Hz")

    heartbeat = subparsers.add_parser("heartbeat", help="send HEARTBEAT")
    heartbeat.add_argument("--state", type=int, default=0)

    gripper = subparsers.add_parser("gripper", help="send GRIPPER percent")
    gripper.add_argument("percent", type=int)

    arm_joints = subparsers.add_parser("arm-joints", help="send ARM_JOINTS in rad*1000 protocol units")
    arm_joints.add_argument("joints", type=int, nargs=6, help="six joint values, rad*1000")
    arm_joints.add_argument("--seq", type=int, default=0, help="arm command sequence, default: 0")
    arm_joints.add_argument("--duration", type=int, default=500, help="motion duration ms, default: 500")

    buzzer = subparsers.add_parser("buzzer", help="send BUZZER melody/repeat")
    buzzer.add_argument("--melody", type=int, default=1)
    buzzer.add_argument("--repeat", type=int, default=1)

    raw = subparsers.add_parser("raw", help="send raw v2 type and hex payload")
    raw.add_argument("type", type=lambda value: int(value, 0), help="message type, for example 0x02")
    raw.add_argument("payload", nargs="?", default="", help="payload hex, for example '01 02'")

    return parser


def write_frame(port, frame, label):
    print(label + ":", hexdump(frame))
    port.write(frame)
    port.flush()


def run_drive_test(args):
    interval = 1.0 / args.rate if args.rate > 0 else 0.05
    deadline = time.time() + args.duration

    with serial.Serial(args.port, args.baud, timeout=0.02) as port:
        port.reset_input_buffer()
        write_frame(port, encode_set_mode(Mode.ROS_AUTO), "TX set-mode")
        time.sleep(0.05)

        try:
            while time.time() < deadline:
                write_frame(port, encode_heartbeat(1), "TX heartbeat")
                write_frame(port, encode_base_vel(args.vx, args.vy, args.wz), "TX base-vel")
                time.sleep(interval)
        finally:
            write_frame(port, encode_stop(), "TX stop")

        messages, leftover = read_messages(port, args.timeout)

    if messages:
        for message in messages:
            print("RX:", hexdump(message["frame"]), "=>", describe_message(message))
    elif leftover:
        print("RX leftover:", hexdump(leftover))

    return 0


def main():
    if serial is None:
        print("pyserial is not installed. Install it with: python -m pip install pyserial", file=sys.stderr)
        return 2

    parser = build_parser()
    args = parser.parse_args()

    if args.command == "drive-test":
        return run_drive_test(args)

    frame = make_frame(args)

    print("TX:", hexdump(frame))
    with serial.Serial(args.port, args.baud, timeout=0.02) as port:
        port.reset_input_buffer()
        port.write(frame)
        port.flush()

        if args.no_read:
            return 0

        messages, leftover = read_messages(port, args.timeout)

    if not messages:
        print("RX: no valid v2 frame")
        if leftover:
            print("RX leftover:", hexdump(leftover))
        return 1

    for message in messages:
        print("RX:", hexdump(message["frame"]), "=>", describe_message(message))
    return 0


if __name__ == "__main__":
    sys.exit(main())