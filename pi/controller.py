import pigpio
import time
import math

class PiMotor(object):
    def __init__(self, controller, pin_pwm, pin_a, pin_b):
        self.pin_pwm = pin_pwm
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.controller = controller
        self.controller.set_mode(self.pin_pwm, pigpio.OUTPUT)
        self.controller.set_mode(self.pin_a, pigpio.OUTPUT)
        self.controller.set_mode(self.pin_b, pigpio.OUTPUT)

    def set_value(self, value):
        if value < 0:
            value = -value
            negate = True
        else:
            negate = False

        value = min(max(value, 0), 1)

        controller.pi.write(self.pin_a, 1 if negate else 0)
        controller.pi.write(self.pin_b, 0 if negate else 1)
        controller.pi.hardware_PWM(self.pin_pwm, 800, int(1e6 * value))

class PiController(object):
    def __init__(self):
        self.pi = pigpio.pi() 

    def set_mode(self, pin, mode):
        self.pi.set_mode(pin, mode)

if __name__ == '__main__':
    controller = PiController()
    motor_left = PiMotor(controller, 12, 6, 13)
    motor_right = PiMotor(controller, 18, 23, 24)

    try:
        while True:
            time.sleep(0.001)
            #motor_left.set_value(math.sin(time.time() / 10))
            #motor_right.set_value(-math.sin(time.time() / 10))
            motor_right.set_value(1)
            motor_left.set_value(-1)

    except:
        motor_left.set_value(0)
        motor_right.set_value(0)
