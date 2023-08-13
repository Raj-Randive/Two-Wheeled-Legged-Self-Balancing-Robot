import RPi.GPIO as GPIO
from RpiMotorLib import RpiMotorLib
import time

class StepperMotorController:
    def __init__(self, direction_pin, step_pin, ms_pins, en_pin):
        self.direction_pin = direction_pin
        self.step_pin = step_pin
        self.ms_pins = ms_pins
        self.en_pin = en_pin

        self.myMotor = RpiMotorLib.A4988Nema(self.direction_pin, self.step_pin, self.ms_pins, "A4988")
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.en_pin, GPIO.OUT)
        GPIO.output(self.en_pin, GPIO.LOW)

    def move_forward(self, steps, step_delay):
        self.myMotor.motor_go(True, "Full", steps, step_delay, False, step_delay)

    def move_backward(self, steps, step_delay):
        self.myMotor.motor_go(False, "Full", steps, step_delay, False, step_delay)

    def run_motor(self):
        try:
            while True:
                print("NEMA BHAAG RAHI HAI")
                self.move_forward(1000, 0.001)
                time.sleep(1)
                self.move_backward(1000, 0.001)
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            GPIO.cleanup()

#if __name__ == "__main__":
 #   direction_pin = 20
  #  step_pin = 21
   # ms_pins = (14, 15, 18)
    #en_pin = 24

    #motor_controller = StepperMotorController(direction_pin, step_pin, ms_pins, en_pin)
    #motor_controller.run_motor()
