import time
import struct
from dataclasses import dataclass
from smbus2 import SMBus
import math


def _fmt(prefix: str, endian: str) -> str:
    # prefix is "I" or "f" etc
    return ("<" if endian == "little" else ">") + prefix

class PinpointI2C:
    def __init__(self, bus: int, addr: int, endian: str):
        self.bus_num = bus
        self.addr = addr
        self.endian = endian

        # Device Control bits
        self.CTRL_RESET_IMU             = 1 << 0
        self.CTRL_RESET_IMU_AND_POS     = 1 << 1
        self.CTRL_SET_Y_REVERSED        = 1 << 2
        self.CTRL_SET_Y_FORWARD         = 1 << 3
        self.CTRL_SET_X_REVERSED        = 1 << 4
        self.CTRL_SET_X_FORWARD         = 1 << 5

    def read_bytes(self, reg: int, length: int) -> bytes:
        with SMBus(self.bus_num) as bus:
            bus.read_i2c_block_data(self.addr, reg, length)
        # time.sleep(0.01)
        with SMBus(self.bus_num) as bus:
            data = bus.read_i2c_block_data(self.addr, reg, length)
        return bytes(data)

    def write_bytes(self, reg: int, data: bytes):
        with SMBus(self.bus_num) as bus:
            bus.write_i2c_block_data(self.addr, reg, list(data))

    def read_u32(self, reg: int) -> int:
        raw = self.read_bytes(reg, 4)
        return struct.unpack(_fmt("I", self.endian), raw)[0]

    def read_f32(self, reg: int) -> float:
        raw = self.read_bytes(reg, 4)
        return struct.unpack(_fmt("f", self.endian), raw)[0]

    def write_u32(self, reg: int, val: int):
        data = struct.pack(_fmt("I", self.endian), val & 0xFFFFFFFF)
        self.write_bytes(reg, data)

    def write_f32(self, reg: int, val: float):
        data = struct.pack(_fmt("f", self.endian), float(val))
        self.write_bytes(reg, data)
    
    # Control commands via reg 4 :contentReference[oaicite:10]{index=10}
    def send_control(self, control_bits: int):
        self.write_u32(4, control_bits)

    def reset_imu(self):
        self.send_control(self.CTRL_RESET_IMU)

    def reset_pos_and_imu(self):
        self.send_control(self.CTRL_RESET_IMU_AND_POS)

    def set_encoder_directions(self, x_reversed: bool, y_reversed: bool):
        bits = 0
        bits |= self.CTRL_SET_X_REVERSED if x_reversed else self.CTRL_SET_X_FORWARD
        bits |= self.CTRL_SET_Y_REVERSED if y_reversed else self.CTRL_SET_Y_FORWARD
        self.send_control(bits)

    # Config registers: ticks/mm (14), x offset (15), y offset (16), yaw scalar (17) :contentReference[oaicite:11]{index=11}
    # Remember config is lost on power cycle :contentReference[oaicite:12]{index=12}
    def set_ticks_per_mm(self, ticks_per_mm: float):
        self.write_f32(14, ticks_per_mm)

    def set_pod_offsets_mm(self, x_pod_offset_mm: float, y_pod_offset_mm: float):
        self.write_f32(15, x_pod_offset_mm)
        self.write_f32(16, y_pod_offset_mm)

    def set_yaw_scalar(self, yaw_scalar: float):
        self.write_f32(17, yaw_scalar)

    # Position registers are read/write; writing sets position (setPosition behavior) :contentReference[oaicite:13]{index=13}
    def set_position_mm_rad(self, x_mm: float, y_mm: float, h_rad: float):
        self.write_f32(8, x_mm)
        self.write_f32(9, y_mm)
        self.write_f32(10, h_rad)