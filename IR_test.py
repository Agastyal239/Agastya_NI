import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

GPIO.setup(17, GPIO.IN)
GPIO.setup(22, GPIO.IN)
GPIO.setup(27, GPIO.IN)


try:
    while True:
        ir_state_1 = GPIO.input(17)
        
        ir_state_2 = GPIO.input(27)

        ir_state_3 = GPIO.input(22)
        
        print("s1", ir_state_1, "s2", ir_state_2, "s3", ir_state_3)
except:
    GPIO.cleanup()