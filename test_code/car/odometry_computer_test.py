#!/usr/bin/env python3
import time
import struct
from dataclasses import dataclass
from smbus2 import SMBus
import math

I2C_BUS  = 1
I2C_ADDR = 0x31  # guide says 0x31

# The guide's register addresses are 1..18. (Keep as-is.)
# If you discover your device wants 0-based addressing, set -1.
REG_BASE_OFFSET = 0

# Endianness is not explicitly stated in the guide; default to little.
# If Device ID (reg 1) doesn't read as 2, flip to "big".
ENDIAN = "little"  # "little" or "big"


def _reg_wire(doc_reg: int) -> int:
    r = doc_reg + REG_BASE_OFFSET
    if not (0 <= r <= 0xFF):
        raise ValueError(f"Register out of 8-bit range: doc={doc_reg}, wire={r}")
    return r


def _fmt(prefix: str) -> str:
    # prefix is "I" or "f" etc
    return ("<" if ENDIAN == "little" else ">") + prefix


def decode_status_bits(status_u32: int) -> str:
    # Guide: Not Ready: 0, Ready: 1, Calibrating: 1<<1, X missing: 1<<2, Y missing: 1<<3 :contentReference[oaicite:7]{index=7}
    if status_u32 == 0:
        return "NOT_READY"

    parts = []
    if status_u32 & (1 << 0):
        parts.append("READY")
    if status_u32 & (1 << 1):
        parts.append("CALIBRATING")
    if status_u32 & (1 << 2):
        parts.append("FAULT_X_POD_NOT_DETECTED")
    if status_u32 & (1 << 3):
        parts.append("FAULT_Y_POD_NOT_DETECTED")

    return "|".join(parts) if parts else f"UNKNOWN(0x{status_u32:08X})"


# Device Control bits (reg 4) :contentReference[oaicite:8]{index=8}
CTRL_RESET_IMU             = 1 << 0
CTRL_RESET_IMU_AND_POS     = 1 << 1
CTRL_SET_Y_REVERSED        = 1 << 2
CTRL_SET_Y_FORWARD         = 1 << 3
CTRL_SET_X_REVERSED        = 1 << 4
CTRL_SET_X_FORWARD         = 1 << 5
# Bits 6-7 exist but "do not use" in guide.


@dataclass
class BulkRead:
    status: int
    loop_time_us: int
    x_enc: int
    y_enc: int
    x_mm: float
    y_mm: float
    h_rad: float
    vx_mm_s: float
    vy_mm_s: float
    vh_rad_s: float


class PinpointI2C:
    def __init__(self, bus: int = I2C_BUS, addr: int = I2C_ADDR):
        self.bus_num = bus
        self.addr = addr

    def read_bytes(self, reg: int, length: int) -> bytes:
        r = _reg_wire(reg)
        with SMBus(self.bus_num) as bus:
            bus.read_i2c_block_data(self.addr, r, length)
        time.sleep(0.01)
        with SMBus(self.bus_num) as bus:
            data = bus.read_i2c_block_data(self.addr, r, length)
        return bytes(data)

    def write_bytes(self, reg: int, data: bytes):
        r = _reg_wire(reg)
        with SMBus(self.bus_num) as bus:
            bus.write_i2c_block_data(self.addr, r, list(data))

    def read_u32(self, reg: int) -> int:
        raw = self.read_bytes(reg, 4)
        return struct.unpack(_fmt("I"), raw)[0]

    def read_f32(self, reg: int) -> float:
        raw = self.read_bytes(reg, 4)
        return struct.unpack(_fmt("f"), raw)[0]

    def write_u32(self, reg: int, val: int):
        data = struct.pack(_fmt("I"), val & 0xFFFFFFFF)
        self.write_bytes(reg, data)

    def write_f32(self, reg: int, val: float):
        data = struct.pack(_fmt("f"), float(val))
        self.write_bytes(reg, data)

    # Bulk Read register (18) is 40 bytes :contentReference[oaicite:9]{index=9}
    # Interpreted as: status u32, loop u32, xenc u32, yenc u32, then 6 floats (x,y,h,vx,vy,vh)
    def read_bulk(self) -> BulkRead:
        raw = self.read_bytes(18, 40)
        fmt = ("<" if ENDIAN == "little" else ">") + "IIIIffffff"
        vals = struct.unpack(fmt, raw)
        return BulkRead(
            status=vals[0],
            loop_time_us=vals[1],
            x_enc=vals[2],
            y_enc=vals[3],
            x_mm=vals[4],
            y_mm=vals[5],
            h_rad=vals[6],
            vx_mm_s=vals[7],
            vy_mm_s=vals[8],
            vh_rad_s=vals[9],
        )

    # Control commands via reg 4 :contentReference[oaicite:10]{index=10}
    def send_control(self, control_bits: int):
        self.write_u32(4, control_bits)

    def reset_imu(self):
        self.send_control(CTRL_RESET_IMU)

    def reset_pos_and_imu(self):
        self.send_control(CTRL_RESET_IMU_AND_POS)

    def set_encoder_directions(self, x_reversed: bool, y_reversed: bool):
        bits = 0
        bits |= CTRL_SET_X_REVERSED if x_reversed else CTRL_SET_X_FORWARD
        bits |= CTRL_SET_Y_REVERSED if y_reversed else CTRL_SET_Y_FORWARD
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


def main():
    dev = PinpointI2C()

    reg_to_name = {
        1: " Device ID",
        2: " Version",
        3: " Status",
        4: " Control",
        6: " X Encoder",
        7: " Y Encoder",
        8: " X Position",
        9: " Y Position",
        10: " Yaw",
        11: " X Velocity",
        12: " Y Velocity",
        13: " Yaw Velocity",
        14: " Ticks per mm",
        15: " X Offset",
        16: " Y Offset",
        17: " Yaw Offset",
    }

    reg_to_format = {
        1: "I",
        2: "I",
        3: "I",
        4: "I",
        6: "I",
        7: "I",
        8: "f",
        9: "f",
        10: "f",
        11: "f",
        12: "f",
        13: "f",
        14: "f",
        15: "f",
        16: "f",
        17: "f",
    }

    dev.reset_pos_and_imu()
    time.sleep(1)
    dev.set_pod_offsets_mm(160.87, -201.3)
    time.sleep(1)
    dev.set_encoder_directions(True, True)
    time.sleep(1)
    # dev.set_yaw_scalar(1.0)
    # time.sleep(1)
    # dev.set_position_mm_rad(0.0, 0.0, 0.0)
    # time.sleep(1)
    while True:
        for reg, name in reg_to_name.items():
            with SMBus(1) as bus:
                # dev.write_f32(15, 0.0)
                name = name.strip()
                bus.read_i2c_block_data(0x31, reg, 4)
                time.sleep(0.01)
                data = bus.read_i2c_block_data(0x31, reg, 4)
                value = struct.unpack(_fmt(reg_to_format[reg]), bytes(data))[0]
                if name == "X Position" or name == "Y Position":
                    value /= 25.4
                if name == "Yaw":
                    value *= (180.0 / math.pi)
                    is_negative = value < 0
                    value = abs(value) % 360
                    value = -value if is_negative else value
                print(f"{name}: {value}")
                print('='*10)
        time.sleep(1)


    # Sanity check from guide: Device ID should be 2; Version starts at 1 :contentReference[oaicite:14]{index=14}
    # device_id = dev.read_u32(1)
    # version   = dev.read_u32(2)

    # print(f"ENDIAN={ENDIAN} REG_BASE_OFFSET={REG_BASE_OFFSET}")
    # print(f"Device ID={device_id} (expected 2), Version={version} (starts at 1)")

    # If these are wrong, flip ENDIAN or REG_BASE_OFFSET.

    # print("\nPolling Bulk Read (mm + radians) ...")  # units: mm and radians :contentReference[oaicite:15]{index=15}
    # try:
    #     while True:
    #         b = dev.read_bulk()
    #         status_str = decode_status_bits(b.status)
    #         print(
    #             f"status={status_str:35s} loop_us={b.loop_time_us:4d} "
    #             f"xenc={b.x_enc:10d} yenc={b.y_enc:10d} "
    #             f"x={b.x_mm:8.2f}mm y={b.y_mm:8.2f}mm h={b.h_rad:7.4f}rad "
    #             f"vx={b.vx_mm_s:8.2f} vy={b.vy_mm_s:8.2f} vh={b.vh_rad_s:7.4f}"
    #         )
    #         time.sleep(0.02)  # ~50 Hz; device runs faster but Pi polling doesn't need 1500 Hz
    # except KeyboardInterrupt:
    #     print("\nStopped.")


if __name__ == "__main__":
    main()