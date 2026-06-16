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


MSG_TYPE_NAMES = {
    MsgType.STOP: "STOP",
    MsgType.SET_MODE: "SET_MODE",
    MsgType.PING: "PING",
    MsgType.BASE_VEL: "BASE_VEL",
    MsgType.ARM_JOINTS: "ARM_JOINTS",
    MsgType.GRIPPER: "GRIPPER",
    MsgType.ARM_PRESET: "ARM_PRESET",
    MsgType.BUZZER: "BUZZER",
    MsgType.HEARTBEAT: "HEARTBEAT",
    MsgType.STATUS: "STATUS",
    MsgType.ACK: "ACK",
    MsgType.ERROR: "ERROR",
    MsgType.PONG: "PONG",
    MsgType.DEBUG: "DEBUG",
}


MODE_NAMES = {
    Mode.MANUAL: "MANUAL",
    Mode.ROS_AUTO: "ROS_AUTO",
    Mode.ESTOP: "ESTOP",
    Mode.SAFE_IDLE: "SAFE_IDLE",
}


ACK_RESULT_NAMES = {
    0: "OK",
    1: "BUSY",
    2: "REJECTED",
    3: "BAD_MODE",
    4: "LIMIT_CLAMPED",
    5: "BAD_LENGTH",
}


ERROR_NAMES = {
    0x0000: "OK",
    0x0001: "BAD_HEADER",
    0x0002: "BAD_LENGTH",
    0x0003: "BAD_CRC",
    0x0004: "BAD_TAIL",
    0x0005: "UNKNOWN_TYPE",
    0x0006: "NOT_ROS_AUTO_MODE",
    0x0007: "BASE_TIMEOUT",
    0x0008: "ARM_LIMIT",
    0x0009: "ESTOP_ACTIVE",
    0x000A: "LOW_BATTERY",
    0x000B: "SERIAL_OVERFLOW",
}


def msg_type_name(msg_type):
    return MSG_TYPE_NAMES.get(msg_type, "0x%02X" % msg_type)


def mode_name(mode):
    return MODE_NAMES.get(mode, "0x%02X" % mode)


def ack_result_name(result):
    return ACK_RESULT_NAMES.get(result, "UNKNOWN_%d" % result)


def error_name(error_code):
    return ERROR_NAMES.get(error_code, "0x%04X" % error_code)


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


def decode_message(message):
    """Decode a valid v2 message into a JSON-friendly dictionary.

    Unknown or wrong-length frames are still returned as dictionaries so callers
    can log them safely without throwing inside the serial read loop.
    """
    msg_type = message["type"]
    payload = message["payload"]
    decoded = {
        "type": msg_type,
        "type_name": msg_type_name(msg_type),
        "payload_hex": payload.hex(" ").upper(),
        "valid_length": True,
    }

    if msg_type == MsgType.STATUS:
        if len(payload) != 22:
            decoded["valid_length"] = False
            decoded["expected_length"] = 22
            decoded["length"] = len(payload)
            return decoded
        values = struct.unpack("<BBBBHHB6hB", payload)
        mode, base_state, arm_state, buzzer_state, error_code, battery_mv, joint_count = values[:7]
        joints = list(values[7:13])
        last_arm_seq = values[13]
        decoded.update(
            {
                "mode": mode,
                "mode_name": mode_name(mode),
                "base_state": base_state,
                "arm_state": arm_state,
                "buzzer_state": buzzer_state,
                "error_code": error_code,
                "error_name": error_name(error_code),
                "battery_mv": battery_mv,
                "joint_count": joint_count,
                "joints": joints,
                "last_arm_seq": last_arm_seq,
            }
        )
        return decoded

    if msg_type == MsgType.ACK:
        if len(payload) != 2:
            decoded["valid_length"] = False
            decoded["expected_length"] = 2
            decoded["length"] = len(payload)
            return decoded
        ack_type, result = struct.unpack("<BB", payload)
        decoded.update(
            {
                "ack_type": ack_type,
                "ack_type_name": msg_type_name(ack_type),
                "result": result,
                "result_name": ack_result_name(result),
            }
        )
        return decoded

    if msg_type == MsgType.ERROR:
        if len(payload) != 3:
            decoded["valid_length"] = False
            decoded["expected_length"] = 3
            decoded["length"] = len(payload)
            return decoded
        error_code, detail = struct.unpack("<HB", payload)
        decoded.update({"error_code": error_code, "error_name": error_name(error_code), "detail": detail})
        return decoded

    if msg_type == MsgType.PONG:
        if len(payload) != 4:
            decoded["valid_length"] = False
            decoded["expected_length"] = 4
            decoded["length"] = len(payload)
            return decoded
        (time_ms,) = struct.unpack("<I", payload)
        decoded.update({"time_ms": time_ms})
        return decoded

    return decoded


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
