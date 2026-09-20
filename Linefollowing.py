from gpiozero import PWMOutputDevice, DigitalOutputDevice, DigitalInputDevice
from time import sleep

# --- IR sensors (pull_up=True means: 1 = white, 0 = black) ---
ir_left = DigitalInputDevice(22, pull_up=True)
ir_center = DigitalInputDevice(25, pull_up=True)
ir_right = DigitalInputDevice(6, pull_up=True)

# --- Motor driver (Cytron MDD10A) ---
left_pwm = PWMOutputDevice(18)
left_dir = DigitalOutputDevice(23)

right_pwm = PWMOutputDevice(13)
right_dir = DigitalOutputDevice(24)

BASE_SPEED = 0.4
TURN_SPEED = 0.4
FORWARD = 0     # set to 1 if motors run backward at DIR=0


def set_left(speed, forward=True):
    left_dir.value = FORWARD if forward else (1 - FORWARD)
    left_pwm.value = max(0.0, min(1.0, speed))


def set_right(speed, forward=True):
    right_dir.value = FORWARD if forward else (1 - FORWARD)
    right_pwm.value = max(0.0, min(1.0, speed))


def stop():
    left_pwm.value = 0
    right_pwm.value = 0


def forward():
    set_left(BASE_SPEED, True)
    set_right(BASE_SPEED, False)


def backward():
    set_left(BASE_SPEED, False)
    set_right(BASE_SPEED, False)


def turn_right():
    # Left wheel forward, right wheel backward
    set_left(TURN_SPEED, True)
    set_right(TURN_SPEED, False)


def turn_left():
    # Left wheel backward, right wheel forward
    set_left(TURN_SPEED, False)
    set_right(TURN_SPEED, True)


try:
    while True:

        L = ir_left.value
        C = ir_center.value
        R = ir_right.value

        # ------------------------------------------
        # Decide motion
        # ------------------------------------------

        if L == 0 and C == 0 and R == 0:
            # All black
            stop()
            motion = "STOP - ALL BLACK"

        elif C == 0:
            # Center on black
            forward()
            motion = "FORWARD"

        elif L == 0:
            # Left sensor on black
            turn_right()
            motion = "RIGHT"

        elif R == 0:
            # Right sensor on black
            turn_left()
            motion = "LEFT"

        else:
            # All white
            stop()
            motion = "STOP - LINE LOST"

        # ------------------------------------------
        # Print sensor + motion
        # ------------------------------------------

        print(f"L:{L} C:{C} R:{R}  -->  {motion}")

        sleep(0.02)


except KeyboardInterrupt:
    stop()
    print("\nRobot stopped.")
