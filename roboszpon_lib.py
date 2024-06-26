import can
import struct
import asyncio

MSG_MOTOR_COMMAND = 0x01
MSG_ACTION_REQUEST = 0x02
MSG_STATUS_REPORT = 0x03
MSG_AXIS_REPORT = 0x04
MSG_MOTOR_REPORT = 0x05
MSG_PARAMETER_WRITE = 0x06
MSG_PARAMETER_READ = 0x07
MSG_PARAMETER_RESPONSE = 0x08

ACTION_ARM = 0x00
ACTION_DISARM = 0x01
ACTION_COMMIT_CONFIG = 0x02
ACTION_RESTORE_CONFIG = 0x03
ACTION_SET_FACTORY_CONFIG = 0x04
ACTION_SOFTWARE_RESET = 0x05

ROBOSZPON_MODE_STOPPED = 0x00
ROBOSZPON_MODE_RUNNING = 0x01
ROBOSZPON_MODE_ERROR = 0x02
ROBOSZPON_MODES = {0: "STOPPED", 1: "RUNNING", 2: "ERROR"}
ROBOSZPON_PARAMETERS = {
    "COMMAND_TIMEOUT": 0x00,
    "ENCODER_ZERO": 0x01,
    "AXIS_OFFSET": 0x02,
    "REPORT_RATE": 0x03,
    "PPID_Kp": 0x04,
    "PPID_Ki": 0x05,
    "PPID_Kd": 0x06,
    "PPID_deadzone": 0x08,
    "PPID_dUmax": 0x0A,
    "VPID_Kp": 0x0C,
    "VPID_Ki": 0x0D,
    "VPID_Kd": 0x0E,
    "VPID_deadzone": 0x10,
    "VPID_dUmax": 0x12,
    "CPID_Kp": 0x14,
    "CPID_Ki": 0x15,
    "CPID_Kd": 0x16,
    "CPID_deadzone": 0x18,
    "CPID_dUmax": 0x1A,
    "CURRENT_FeedForward": 0x1B,
    "IIR_VALUE_CURMEAS": 0x1C,
    "IIR_VALUE_VELMEAS": 0x1D,
    "IIR_VALUE_PPIDU": 0x1E,
    "IIR_VALUE_VPIDU": 0x1F,
    "IIR_VALUE_CPIDU": 0x20,
	"DUTY_DEADZONE": 0x21,
    "MIN_POSITION": 0x23,
    "MAX_POSITION": 0x24,
    "MIN_VELOCITY": 0x25,
    "MAX_VELOCITY": 0x26,
    "MIN_CURRENT": 0x27,
    "MAX_CURRENT": 0x28,
    "MIN_DUTY": 0x29,
    "MAX_DUTY": 0x2F,
    "OVERHEAT_TEMPERATURE": 0x50,
    "NO_OVERHEAT_TEMPERATURE": 0x51,
    "INVERT_AXIS": 0x52,
    "INVERT_ENCODER": 0x53,
    "ENCODER_FILTER_WINDOW": 0x54,
	"DISABLE_ENCODER_ERRORS": 0x55,
}


async def send_can_frame(canbus, frame_id, data):
    try:
        frame = can.Message(
            arbitration_id=frame_id,
            data=data.to_bytes(8, byteorder="big"),
            is_extended_id=False,
        )
        await canbus.send(frame)
    except Exception as e:
        print(f"Error sending CAN frame: {e}")


def build_frame_id(node_id, message_id):
    return ((node_id & 0b11111) << 6) + (message_id & 0b111111)


def decode_message(frame_id, data):
    node_id = (frame_id >> 6) & 0b11111
    message_id = frame_id & 0b111111
    if message_id == MSG_STATUS_REPORT:
        mode = (data >> 62) & 0b11
        flags = data & 0xFFFF
        temperature = (data >> 24) & 0xFFFFFFFF
        temperature *= 0.1
        return {
            "node_id": node_id,
            "message_id": message_id,
            "mode": mode,
            "temperature": temperature,
            "flags": flags,
        }
    if message_id == MSG_AXIS_REPORT:
        position = (data >> 32) & 0xFFFFFFFF
        velocity = data & 0xFFFFFFFF
        return {
            "node_id": node_id,
            "message_id": message_id,
            "position": bits_to_float(position),
            "velocity": bits_to_float(velocity),
        }
    if message_id == MSG_MOTOR_REPORT:
        current = (data >> 32) & 0xFFFFFFFF
        duty = data & 0xFFFFFFFF
        return {
            "node_id": node_id,
            "message_id": message_id,
            "current": bits_to_float(current),
            "duty": bits_to_float(duty),
        }
    if message_id == MSG_PARAMETER_RESPONSE:
        parameter_id = (data >> 56) & 0xFF
        value = (data >> 24) & 0xFFFFFFFF
        return {
            "node_id": node_id,
            "message_id": message_id,
            "parameter_id": parameter_id,
            "value": bits_to_float(value),
        }
    return {"node_id": node_id, "message_id": message_id, "data": data}


async def send_action_request(canbus, node_id, action_id):
    await send_can_frame(canbus, build_frame_id(node_id, MSG_ACTION_REQUEST), action_id)


async def arm(canbus, node_id):
    await send_action_request(canbus, node_id, ACTION_ARM)


async def disarm(canbus, node_id):
    await send_action_request(canbus, node_id, ACTION_DISARM)


def float_to_bits(value):
    value_bits = struct.pack("f", value)
    return struct.unpack("I", value_bits)[0]


def bits_to_float(value):
    value_bits = struct.pack("I", value)
    return struct.unpack("f", value_bits)[0]


async def send_motor_command(canbus, node_id, motor_command_type, command):
    data = ((motor_command_type & 0xFF) << 56) + (
        (float_to_bits(command) & 0xFFFFFFFF) << 24
    )
    await send_can_frame(canbus, build_frame_id(node_id, MSG_MOTOR_COMMAND), data)


async def send_duty_command(canbus, node_id, command):
    await send_motor_command(canbus, node_id, 0, command)


async def send_velocity_command(canbus, node_id, command):
    await send_motor_command(canbus, node_id, 1, command)


async def send_position_command(canbus, node_id, command):
    await send_motor_command(canbus, node_id, 2, command)


async def emergency_stop(canbus):
    await send_can_frame(canbus, 0x001, 0)


async def send_parameter_write(canbus, node_id, parameter_id, value):
    data = ((parameter_id & 0xFF) << 56) + ((float_to_bits(value) & 0xFFFFFFFF) << 24)
    await send_can_frame(canbus, build_frame_id(node_id, MSG_PARAMETER_WRITE), data)


async def send_parameter_read(canbus, node_id, parameter_id):
    await send_can_frame(canbus, build_frame_id(node_id, MSG_PARAMETER_READ), parameter_id)


import time


async def read_parameter_callback(canbus, can_notifier, node_id, parameter_id, callback):
    queue = asyncio.Queue()

    def on_message_received(message: can.Message):
        decoded_message = decode_message(
            message.arbitration_id, int.from_bytes(message.data, byteorder="big")
        )
        if (
            decoded_message["node_id"] == node_id
            and decoded_message["message_id"] == MSG_PARAMETER_RESPONSE
            and decoded_message["parameter_id"] == parameter_id
        ):
            queue.put_nowait(decoded_message["value"])
            can_notifier.remove_listener(on_message_received)

    can_notifier.add_listener(on_message_received)
    await send_parameter_read(canbus, node_id, parameter_id)
