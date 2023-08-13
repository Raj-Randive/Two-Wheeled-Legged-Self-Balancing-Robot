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

# MPU6050 device address
Device_Address = 0x68

# Define the servo pin
SERVO_PIN = 4

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

        # Set up GPIO mode
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(SERVO_PIN, GPIO.OUT)

        # Create a PWM object for servo control
        servo_pwm = GPIO.PWM(SERVO_PIN, 50)  # 50 Hz frequency for SG90 servo

        print("Reading MPU6050...")

        while True:
            # Read accelerometer data
            acc_x = read_raw_data(ACCEL_XOUT_H)
            acc_y = read_raw_data(ACCEL_YOUT_H)
            acc_z = read_raw_data(ACCEL_ZOUT_H)

            # Scale accelerometer data
            acclX_scaled = acc_x * 0.000061 * 9.80665
            # Calculate rotation angles
            
            x_angle = get_x_rotation(acclX_scaled, acclY_scaled, acclZ_scaled)
            
            # Calculate desired servo position based on accelerometer data
           #servo_angle = -acclX_scaled  # Adjust the scaling factor as needed

            # Limit the servo angle to a reasonable range (e.g., -90 to 90 degrees)
           #servo_angle = max(min(servo_angle, 90), -90)

            # Map servo angle from -90 to 90 to servo output range 0 to 180
           #servo_output = int((servo_angle + 90) * (180 / 180.0))

            # Calculate duty cycle based on servo output
           #duty_cycle = 2.5 + (servo_output / 90.0)*10.0

            # Set servo angle using PWM
           #servo_pwm.start(duty_cycle)

            # Print rotation angle and servo output
            print("X rotation (degrees): ", x_angle)
           #print("Servo output (degrees): ", servo_output)

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


