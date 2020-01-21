import pigpio
import time

PWM_PIN = 12
RIGHT_PIN = 7
LEFT_PIN = 8

pi1 = pigpio.pi()

try:
    pi1.set_mode(PWM_PIN, pigpio.OUTPUT)
    pi1.set_mode(RIGHT_PIN, pigpio.OUTPUT)
    pi1.set_mode(LEFT_PIN, pigpio.OUTPUT)

#    pi1.write(RIGHT_PIN, 1)
#    pi1.write(LEFT_PIN, 1)
    
    pi1.hardware_PWM(PWM_PIN, 800, int(1e6*1))
    while True:
        time.sleep(2)
        pi1.write(LEFT_PIN, 0)
        pi1.write(RIGHT_PIN, 1)

        time.sleep(2)
        pi1.write(RIGHT_PIN, 0)
        pi1.write(LEFT_PIN, 1)

    while True:
        time.sleep(1)
        
except:
    pi1.write(PWM_PIN, 0)
    pi1.write(RIGHT_PIN, 0)
    pi1.write(LEFT_PIN, 0)
