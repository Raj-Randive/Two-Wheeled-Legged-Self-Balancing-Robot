import smbus
import math
from time import sleep
import sys

class MPU6050:
    def __init__(self, bus_num=1, device_address=0x68):
        self.bus = smbus.SMBus(bus_num)
        self.device_address = device_address

        self.PWR_MGMT_1 = 0x6B
        self.SMPLRT_DIV = 0x19
        self.CONFIG = 0x1A
        self.GYRO_CONFIG = 0x1B
        self.INT_ENABLE = 0x38
        self.ACCEL_XOUT_H = 0x3B
        self.ACCEL_YOUT_H = 0x3D
        self.ACCEL_ZOUT_H = 0x3F
        self.GYRO_XOUT_H = 0x43
        self.GYRO_YOUT_H = 0x45
        self.GYRO_ZOUT_H = 0x47

        self.GYRO_XOUT_OFFSET = -250
        self.GYRO_YOUT_OFFSET = -200
        self.GYRO_ZOUT_OFFSET = 30

        self.MPU_Init()

    def write_byte(self, register, value):
        self.bus.write_byte_data(self.device_address, register, value)

    def read_raw_data(self, addr):
        high = self.bus.read_byte_data(self.device_address, addr)
        low = self.bus.read_byte_data(self.device_address, addr + 1)
        value = ((high << 8) | low)
        if value > 32768:
            value = value - 65536
        return value

    def MPU_Init(self):
        self.write_byte(self.SMPLRT_DIV, 7)
        self.write_byte(self.PWR_MGMT_1, 1)
        self.write_byte(self.CONFIG, 0)
        self.write_byte(self.GYRO_CONFIG, 24)
        self.write_byte(self.INT_ENABLE, 1)

class RotationCalculator:
    @staticmethod
    def get_y_rotation(x, y, z):
        radians = math.atan2(x, z)
        return -(radians * (180.0 / math.pi))

    @staticmethod
    def get_x_rotation(x, y, z):
        radians = math.atan2(y, RotationCalculator.dist(x, z))
        return -(radians * (180.0 / math.pi))

    @staticmethod
    def dist(a, b):
        return math.sqrt(a * a + b * b)

