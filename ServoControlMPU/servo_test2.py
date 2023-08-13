import smbus
import math
from time import sleep
import sys
import RPi.GPIO as GPIO

# Register addresses
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

# Create an SMBus instance
bus = smbus.SMBus(1)

# servo pin 
SERVO_PIN = 4

# Calibration offsets for gyroscope
GYRO_XOUT_OFFSET = -250
GYRO_YOUT_OFFSET = -200
GYRO_ZOUT_OFFSET = 30

# MPU6050 device address
Device_Address = 0x68

# Initialize MPU6050
def MPU_Init():
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
    bus.write_byte_data(Device_Address, CONFIG, 0)
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)

# Read raw data from MPU6050
def read_raw_data(addr):
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr + 1)
    value = ((high << 8) | low)
    if value > 32768:
        value = value - 65536
    return value

# Calculate rotation angle around y-axis
def get_y_rotation(x, y, z):
    radians = math.atan2(x, z)
    return -(radians * (180.0 / math.pi))

# Calculate rotation angle around x-axis
def get_x_rotation(x, y, z):
    radians = math.atan2(y, dist(x, z))
    return -(radians * (180.0 / math.pi))

# Calculate distance using Pythagorean theorem
def dist(a, b):
    return math.sqrt(a * a + b * b)

if __name__ == "__main__":
    try:
        # Initialize MPU6050
        MPU_Init()
     
        print("Reading MPU6050...")
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        servo_pwm = GPIO.PWM(SERVO_PIN, 50)
        
        while True:
            # Read accelerometer data
            acc_x = read_raw_data(ACCEL_XOUT_H)
            acc_y = read_raw_data(ACCEL_YOUT_H)
            acc_z = read_raw_data(ACCEL_ZOUT_H)
             
            # Scale accelerometer data
            acclX_scaled = acc_x * 0.000061 * 9.80665
            acclY_scaled = acc_y * 0.000061 * 9.80665
            acclZ_scaled = acc_z * 0.000061 * 9.80665
             
            # Calculate rotation angles
            x_angle = get_x_rotation(acclX_scaled, acclY_scaled, acclZ_scaled)
            
            # Print rotation angles
            print("X rotation: ", int(x_angle))
            
            if x_angle > 0:
                servo_angle = x_angle
            elif x_angle < 0:
                servo_angle = 180 + x_angle
            else:
                servo_angle = 90
            
            # Limit the servo angle to a reasonable range (0 to 180 degrees)
            servo_angle = max(min(servo_angle, 180), 0)
            
            # Calculate duty cycle based on servo angle
            duty_cycle = 2.5 + (servo_angle / 180.0) * 10.0
            
            # Set servo angle using PWM
            servo_pwm.start(duty_cycle)
            
            # Delay for a short time
            sleep(0.5)
            
    except KeyboardInterrupt:
        servo_pwm.stop()
        GPIO.cleanup()
        sys.exit(0)
    except Exception as e:
        print(e)
        servo_pwm.stop()
        GPIO.cleanup()
        sys.exit(0)

