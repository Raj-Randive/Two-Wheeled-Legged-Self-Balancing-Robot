# from ServoControlMPU.ServoMPUModule import ServoControlMPU6050
from StepperMotorModule.NemaMotorModule import StepperMotorController
from MPUModule.MPU6050Module import MPU6050
from MPUModule.MPU6050Module import RotationCalculator
import sys
import time


if __name__ == "__main__":
    
    DIRECTION_PIN = 20
    STEP_PIN = 21
    MS_PIN = (14, 15, 18)
    EN_PIN = 24
    ACCEL_XOUT_H = 0x3B
    ACCEL_YOUT_H = 0x3D
    ACCEL_ZOUT_H = 0x3F

    motor_controller = StepperMotorController(DIRECTION_PIN, STEP_PIN, MS_PIN, EN_PIN)
    
    try:
        # Initialize MPU6050
        mpu = MPU6050()
        rotationCal = RotationCalculator()
        mpu.MPU_Init()
        
        print("Reading MPU6050...")
        
        while True:
            # Read accelerometer data
            acc_x = mpu.read_raw_data(ACCEL_XOUT_H)
            acc_y = mpu.read_raw_data(ACCEL_YOUT_H)
            acc_z = mpu.read_raw_data(ACCEL_ZOUT_H)
             
            # Scale accelerometer data
            acclX_scaled = acc_x * 0.000061 * 9.80665
            acclY_scaled = acc_y * 0.000061 * 9.80665
            acclZ_scaled = acc_z * 0.000061 * 9.80665
             
            # Calculate rotation angles
            x_angle = rotationCal.get_x_rotation(acclX_scaled, acclY_scaled, acclZ_scaled)
            y_angle = rotationCal.get_y_rotation(acclX_scaled, acclY_scaled, acclZ_scaled)
            
            if(x_angle < 0):
                print("NEMA CHALI CLOCKWISE")
                motor_controller.move_forward(1000, 0.001)
            elif(x_angle > 0):   
                print("NEMA CHALI ANTICLOCKWISE")
                motor_controller.move_backward(1000, 0.001)
                
            # Print rotation angles
            print("X rotation: ", x_angle)
            # print("Y rotation: ", y_angle)
            
            # Delay for a short time
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        sys.exit(0)
    except Exception as e:
        print(e)
        sys.exit(0)
    
    
    
	#servo_mpu = ServoControlMPU6050()
	#servo_mpu.run()
	

	
