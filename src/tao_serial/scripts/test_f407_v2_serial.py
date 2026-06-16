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


def print_messages(messages, leftover=None):
    if messages:
        for message in messages:
            print("RX:", hexdump(message["frame"]), "=>", describe_message(message))
    elif leftover:
        print("RX leftover:", hexdump(leftover))
    else:
        print("RX: no valid v2 frame")


def read_and_print(port, timeout):
    messages, leftover = read_messages(port, timeout)
    print_messages(messages, leftover)
    return messages


def make_frame(args):
    if args.command in ("drive-test", "buzzer-test", "gripper-test", "joint-test", "status-watch", "all-safe"):
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

    status_watch = subparsers.add_parser("status-watch", help="listen for STATUS/ACK/ERROR/PONG frames without sending motion")
    status_watch.add_argument("--duration", type=float, default=5.0, help="watch duration seconds")

    buzzer_test = subparsers.add_parser("buzzer-test", help="safe BUZZER test, short non-motion command")
    buzzer_test.add_argument("--melody", type=int, default=1)
    buzzer_test.add_argument("--repeat", type=int, default=1)

    gripper_test = subparsers.add_parser("gripper-test", help="safe GRIPPER open/close test")
    gripper_test.add_argument("--open", type=int, default=20, help="open percent, default: 20")
    gripper_test.add_argument("--close", type=int, default=70, help="close percent, default: 70")
    gripper_test.add_argument("--hold", type=float, default=0.8, help="hold time seconds between open/close")

    joint_test = subparsers.add_parser("joint-test", help="safe small ARM_JOINTS test around zero protocol units")
    joint_test.add_argument("--joint", type=int, default=0, choices=range(0, 6), help="joint index 0..5")
    joint_test.add_argument("--delta", type=int, default=80, help="small protocol delta rad*1000, default: 80")
    joint_test.add_argument("--duration", type=int, default=800, help="motion duration ms")
    joint_test.add_argument("--hold", type=float, default=1.0, help="hold time seconds")

    all_safe = subparsers.add_parser("all-safe", help="run non-aggressive protocol smoke tests: ping, mode, buzzer, gripper, tiny joint, stop")
    all_safe.add_argument("--skip-joint", action="store_true", help="skip joint motion")
    all_safe.add_argument("--skip-gripper", action="store_true", help="skip gripper motion")
    all_safe.add_argument("--skip-buzzer", action="store_true", help="skip buzzer")

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


def write_and_optionally_read(port, frame, label, timeout=0.3):
    write_frame(port, frame, label)
    return read_and_print(port, timeout)


def run_status_watch(args):
    with serial.Serial(args.port, args.baud, timeout=0.02) as port:
        print("Watching serial feedback for %.1fs ..." % args.duration)
        messages, leftover = read_messages(port, args.duration)
    print_messages(messages, leftover)
    return 0 if messages else 1


def run_buzzer_test(args):
    with serial.Serial(args.port, args.baud, timeout=0.02) as port:
        port.reset_input_buffer()
        write_and_optionally_read(port, encode_set_mode(Mode.ROS_AUTO), "TX set-mode")
        write_and_optionally_read(port, encode_buzzer(args.melody, args.repeat), "TX buzzer", args.timeout)
    return 0


def run_gripper_test(args):
    open_percent = max(0, min(100, args.open))
    close_percent = max(0, min(100, args.close))
    with serial.Serial(args.port, args.baud, timeout=0.02) as port:
        port.reset_input_buffer()
        write_and_optionally_read(port, encode_set_mode(Mode.ROS_AUTO), "TX set-mode")
        write_and_optionally_read(port, encode_gripper(open_percent), "TX gripper-open")
        time.sleep(args.hold)
        write_and_optionally_read(port, encode_gripper(close_percent), "TX gripper-close")
        time.sleep(args.hold)
        write_and_optionally_read(port, encode_gripper(open_percent), "TX gripper-open-final", args.timeout)
    return 0


def run_joint_test(args):
    joints = [0] * 6
    joints[args.joint] = int(args.delta)
    zero = [0] * 6
    with serial.Serial(args.port, args.baud, timeout=0.02) as port:
        port.reset_input_buffer()
        write_and_optionally_read(port, encode_set_mode(Mode.ROS_AUTO), "TX set-mode")
        write_and_optionally_read(port, encode_heartbeat(1), "TX heartbeat")
        write_and_optionally_read(port, encode_arm_joints(1, joints, args.duration), "TX joint-delta")
        time.sleep(args.hold)
        write_and_optionally_read(port, encode_heartbeat(1), "TX heartbeat")
        write_and_optionally_read(port, encode_arm_joints(2, zero, args.duration), "TX joint-zero")
        time.sleep(args.hold)
        write_and_optionally_read(port, encode_stop(), "TX stop", args.timeout)
    return 0


def run_all_safe(args):
    with serial.Serial(args.port, args.baud, timeout=0.02) as port:
        port.reset_input_buffer()
        write_and_optionally_read(port, encode_ping(int(time.time() * 1000) & 0xFFFFFFFF), "TX ping")
        write_and_optionally_read(port, encode_set_mode(Mode.ROS_AUTO), "TX set-mode")
        write_and_optionally_read(port, encode_heartbeat(1), "TX heartbeat")
        write_and_optionally_read(port, encode_base_vel(0, 0, 0), "TX base-zero")
        if not args.skip_buzzer:
            write_and_optionally_read(port, encode_buzzer(1, 1), "TX buzzer")
        if not args.skip_gripper:
            write_and_optionally_read(port, encode_gripper(20), "TX gripper-open")
            time.sleep(0.5)
            write_and_optionally_read(port, encode_gripper(70), "TX gripper-close")
            time.sleep(0.5)
            write_and_optionally_read(port, encode_gripper(20), "TX gripper-open-final")
        if not args.skip_joint:
            write_and_optionally_read(port, encode_arm_joints(1, [60, 0, 0, 0, 0, 0], 800), "TX tiny-joint")
            time.sleep(0.8)
            write_and_optionally_read(port, encode_arm_joints(2, [0, 0, 0, 0, 0, 0], 800), "TX joint-zero")
        write_and_optionally_read(port, encode_stop(), "TX stop", args.timeout)
    return 0


def main():
    if serial is None:
        print("pyserial is not installed. Install it with: python -m pip install pyserial", file=sys.stderr)
        return 2

    parser = build_parser()
    args = parser.parse_args()

    if args.command == "drive-test":
        return run_drive_test(args)
    if args.command == "status-watch":
        return run_status_watch(args)
    if args.command == "buzzer-test":
        return run_buzzer_test(args)
    if args.command == "gripper-test":
        return run_gripper_test(args)
    if args.command == "joint-test":
        return run_joint_test(args)
    if args.command == "all-safe":
        return run_all_safe(args)

    frame = make_frame(args)

    print("TX:", hexdump(frame))
    with serial.Serial(args.port, args.baud, timeout=0.02) as port:
        port.reset_input_buffer()
        port.write(frame)
        port.flush()

        if args.no_read:
            return 0

        messages, leftover = read_messages(port, args.timeout)

    print_messages(messages, leftover)
    return 0 if messages else 1


if __name__ == "__main__":
    sys.exit(main())