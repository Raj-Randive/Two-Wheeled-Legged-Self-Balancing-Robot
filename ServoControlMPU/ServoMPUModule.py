import smbus
import math
from time import sleep
import sys
import RPi.GPIO as GPIO

class ServoControlMPU6050:
    def __init__(self):
        self.bus = smbus.SMBus(1)
        self.Device_Address = 0x68

        self.SERVO_PIN = 4
        self.GYRO_XOUT_OFFSET = -250
        self.GYRO_YOUT_OFFSET = -200
        self.GYRO_ZOUT_OFFSET = 30

        self.PWR_MGMT_1 = 0x6B
        self.SMPLRT_DIV = 0x19
        self.CONFIG = 0x1A
        self.GYRO_CONFIG = 0x1B
        self.INT_ENABLE = 0x38
        self.ACCEL_XOUT_H = 0x3B
        self.ACCEL_YOUT_H = 0x3D
        self.ACCEL_ZOUT_H = 0x3F

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.SERVO_PIN, GPIO.OUT)
        self.servo_pwm = GPIO.PWM(self.SERVO_PIN, 50)

    def MPU_Init(self):
        self.bus.write_byte_data(self.Device_Address, self.SMPLRT_DIV, 7)
        self.bus.write_byte_data(self.Device_Address, self.PWR_MGMT_1, 1)
        self.bus.write_byte_data(self.Device_Address, self.CONFIG, 0)
        self.bus.write_byte_data(self.Device_Address, self.GYRO_CONFIG, 24)
        self.bus.write_byte_data(self.Device_Address, self.INT_ENABLE, 1)

    def read_raw_data(self, addr):
        high = self.bus.read_byte_data(self.Device_Address, addr)
        low = self.bus.read_byte_data(self.Device_Address, addr + 1)
        value = ((high << 8) | low)
        if value > 32768:
            value = value - 65536
        return value

    def get_scaled_accel(self, raw_data):
        return raw_data * 0.000061 * 9.80665

    def map_to_servo_angle(self, scaled_accel):
        servo_angle = ((scaled_accel + 9.8) / 19.6) * 180.0
        return max(min(servo_angle, 180), 0)

    def run(self):
        try:
            self.MPU_Init()
            print("Reading MPU6050...")

            while True:
                acc_x = self.read_raw_data(self.ACCEL_XOUT_H)
                acclX_scaled = self.get_scaled_accel(acc_x)
                servo_angle = self.map_to_servo_angle(acclX_scaled)

                duty_cycle = 2.5 + (servo_angle / 180.0) * 10.0
                self.servo_pwm.start(duty_cycle)

                print("X rotation (scaled): ", servo_angle)
                sleep(0.5)

        except KeyboardInterrupt:
            self.servo_pwm.stop()
            GPIO.cleanup()
            sys.exit(0)
        except Exception as e:
            print(e)
            self.servo_pwm.stop()
            GPIO.cleanup()
            sys.exit(0)

