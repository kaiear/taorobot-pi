#!/usr/bin/env python3
import struct


FRAME_HEADER = 0xAA
FRAME_TAIL = 0xBB
MAX_PAYLOAD_LEN = 64
JOINT_COUNT = 6


class MsgType:
    STOP = 0x00
    SET_MODE = 0x01
    PING = 0x02
    BASE_VEL = 0x10
    ARM_JOINTS = 0x20
    GRIPPER = 0x21
    ARM_PRESET = 0x22
    BUZZER = 0x30
    HEARTBEAT = 0x40
    STATUS = 0x80
    ACK = 0x81
    ERROR = 0x82
    PONG = 0x83
    DEBUG = 0x84


class Mode:
    MANUAL = 0x00
    ROS_AUTO = 0x01
    ESTOP = 0x02
    SAFE_IDLE = 0x03


def crc8_maxim(data):
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x01:
                crc = ((crc >> 1) ^ 0x8C) & 0xFF
            else:
                crc = (crc >> 1) & 0xFF
    return crc


def encode_frame(msg_type, payload=b""):
    payload = bytes(payload)
    if len(payload) > MAX_PAYLOAD_LEN:
        raise ValueError("payload too long: %d" % len(payload))

    body = bytes([len(payload), msg_type]) + payload
    return bytes([FRAME_HEADER]) + body + bytes([crc8_maxim(body), FRAME_TAIL])


def try_decode_frame(buffer):
    if not buffer:
        return None, 0

    header_index = buffer.find(bytes([FRAME_HEADER]))
    if header_index < 0:
        return None, max(0, len(buffer) - 1)
    if header_index > 0:
        return None, header_index
    if len(buffer) < 5:
        return None, 0

    payload_len = buffer[1]
    if payload_len > MAX_PAYLOAD_LEN:
        return None, 1

    frame_len = payload_len + 5
    if len(buffer) < frame_len:
        return None, 0

    frame = bytes(buffer[:frame_len])
    if frame[-1] != FRAME_TAIL:
        return None, 1

    body = frame[1:-2]
    if crc8_maxim(body) != frame[-2]:
        return None, 1

    return {"type": frame[2], "payload": frame[3:-2], "frame": frame}, frame_len


def encode_stop():
    return encode_frame(MsgType.STOP)


def encode_set_mode(mode):
    return encode_frame(MsgType.SET_MODE, struct.pack("<B", mode))


def encode_ping(time_ms):
    return encode_frame(MsgType.PING, struct.pack("<I", time_ms & 0xFFFFFFFF))


def encode_base_vel(vx, vy, wz):
    return encode_frame(MsgType.BASE_VEL, struct.pack("<hhh", vx, vy, wz))


def encode_arm_joints(seq, joints, duration_ms):
    if len(joints) != JOINT_COUNT:
        raise ValueError("expected %d joints" % JOINT_COUNT)
    return encode_frame(MsgType.ARM_JOINTS, struct.pack("<B6hH", seq, *joints, duration_ms))


def encode_gripper(percent):
    return encode_frame(MsgType.GRIPPER, struct.pack("<B", percent))


def encode_arm_preset(preset_id):
    return encode_frame(MsgType.ARM_PRESET, struct.pack("<B", preset_id))


def encode_buzzer(melody_id, repeat):
    return encode_frame(MsgType.BUZZER, struct.pack("<BB", melody_id, repeat))


def encode_heartbeat(state):
    return encode_frame(MsgType.HEARTBEAT, struct.pack("<B", state))


def describe_message(message):
    msg_type = message["type"]
    payload = message["payload"]

    if msg_type == MsgType.STATUS and len(payload) == 22:
        values = struct.unpack("<BBBBHHB6hB", payload)
        return (
            "status mode=%d base=%d arm=%d buzzer=%d error=0x%04X "
            "battery_mv=%d joint_count=%d joints=(%d,%d,%d,%d,%d,%d) last_arm_seq=%d"
            % values
        )
    if msg_type == MsgType.ACK and len(payload) == 2:
        ack_type, result = struct.unpack("<BB", payload)
        return "ack type=0x%02X result=%d" % (ack_type, result)
    if msg_type == MsgType.ERROR and len(payload) == 3:
        error_code, detail = struct.unpack("<HB", payload)
        return "error code=0x%04X detail=%d" % (error_code, detail)
    if msg_type == MsgType.PONG and len(payload) == 4:
        (time_ms,) = struct.unpack("<I", payload)
        return "pong time_ms=%d" % time_ms

    return "type=0x%02X payload=%s" % (msg_type, payload.hex(" "))
