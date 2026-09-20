import serial
import struct
import time
import select
import sys
import termios
import tty
from gpiozero import Buzzer, DigitalOutputDevice, PWMOutputDevice


# =========================================================
# MOTOR PINS - CYTRON MDD10A
# =========================================================

left_pwm = PWMOutputDevice(18)
left_dir = DigitalOutputDevice(23)

right_pwm = PWMOutputDevice(13)
right_dir = DigitalOutputDevice(24)

MAX_SPEED = 1
COMMAND_SPEED = 0.30


# =========================================================
# LIDAR AND BUZZER SETTINGS
# =========================================================

# Raspberry Pi LiDAR serial port
PORT = "/dev/ttyUSB0"
BAUDRATE = 115200

# GPIO 26 = Physical Pin 37
BUZZER_PIN = 26

# The buzzer turns on when any LiDAR distance is in this range.
BUZZER_MIN_DISTANCE = 600
BUZZER_MAX_DISTANCE = 700

# Distances in this range are printed with their angles in the terminal.
PRINT_MIN_DISTANCE = 600
PRINT_MAX_DISTANCE = 700

UPDATE_INTERVAL = 0.30

# ---------------------------------------------------------
# FRONT-FACING DETECTION CONE
#
# The LiDAR spins a full 360 degrees, but obstacles behind or
# to the side of the robot shouldn't trigger the buzzer/stop -
# only obstacles in front of it, where it's actually driving.
#
# FRONT_ANGLE is whichever angle value (0-360) corresponds to
# "straight ahead" for however the LiDAR is physically mounted
# on the robot. Most YDLIDAR X2 units report 0 degrees at a
# marked reference point on the housing - if the buzzer isn't
# triggering on obstacles actually in front of the robot (or is
# triggering on obstacles behind it), rotate this value to match
# your mounting.
#
# FRONT_HALF_ANGLE is how far to each side of FRONT_ANGLE still
# counts as "front" - 45 gives a 90-degree-wide cone (45 degrees
# left of the nose to 45 degrees right of it).
# ---------------------------------------------------------
FRONT_ANGLE = 0
FRONT_HALF_ANGLE = 45

buzzer = Buzzer(BUZZER_PIN)
obstacle_detected = False
last_motor_command = "STOPPED"
original_terminal_settings = None


# =========================================================
# ANGLE HELPERS
# =========================================================

def is_in_front(angle, front_angle=FRONT_ANGLE, half_width=FRONT_HALF_ANGLE):
    """
    True if `angle` (0-360) falls within `half_width` degrees of
    `front_angle`, wrapping correctly across the 0/360 boundary
    (e.g. front_angle=0, half_width=45 covers 315-360 and 0-45).
    """

    diff = abs(angle - front_angle) % 360

    if diff > 180:
        diff = 360 - diff

    return diff <= half_width


# =========================================================
# MOTOR HELPERS
# =========================================================

def set_motors(left_speed, right_speed):
    """Drive both motors. Positive speed is forward."""

    # Do not allow a command to restart the motors while the buzzer is on.
    if obstacle_detected:
        left_pwm.value = 0
        right_pwm.value = 0
        return False

    if left_speed >= 0:
        left_dir.off()
        left_pwm.value = min(left_speed, MAX_SPEED)
    else:
        left_dir.on()
        left_pwm.value = min(-left_speed, MAX_SPEED)

    if right_speed >= 0:
        right_dir.off()
        right_pwm.value = min(right_speed, MAX_SPEED)
    else:
        right_dir.on()
        right_pwm.value = min(-right_speed, MAX_SPEED)

    return True


def stop_motors():
    """Stop both motors immediately."""

    left_pwm.value = 0
    right_pwm.value = 0


# =========================================================
# SERIAL CONNECTION
# =========================================================

ser = serial.Serial(
    PORT,
    BAUDRATE,
    timeout=1
)

print("========================================")
print("      LIDAR MOTOR SAFETY CONTROLLER")
print("========================================")
print("LiDAR Port:", PORT)
print("Baudrate:", BAUDRATE)
print(
    "Buzzer / motor-stop range:",
    BUZZER_MIN_DISTANCE,
    "< distance <",
    BUZZER_MAX_DISTANCE,
    "mm"
)
print(
    "Front detection cone:",
    (FRONT_ANGLE - FRONT_HALF_ANGLE) % 360,
    "deg to",
    (FRONT_ANGLE + FRONT_HALF_ANGLE) % 360,
    "deg (",
    FRONT_HALF_ANGLE * 2,
    "deg wide, centered on angle",
    FRONT_ANGLE,
    ")"
)
print("Buzzer GPIO:", BUZZER_PIN)
print("Motors are stopped at startup.")
print("Controls: Up=forward, Down=back, Left/Right=turn, Space=stop")
print("Waiting for scan data...")
print()


# =========================================================
# READ ONE LIDAR PACKET
# =========================================================

def read_bytes(number):

    data = ser.read(number)

    if len(data) != number:
        return None

    return data


def read_packet():

    # Find the YDLIDAR X2 AA 55 packet header.
    while True:

        b1 = ser.read(1)

        if not b1:
            return None

        if b1[0] != 0xAA:
            continue

        b2 = ser.read(1)

        if not b2:
            return None

        if b2[0] != 0x55:
            continue

        break

    # CT + LSN.
    data = read_bytes(2)

    if data is None:
        return None

    lsn = data[1]

    if lsn == 0:
        return None

    # FSA + LSA + CHECKSUM.
    data = read_bytes(6)

    if data is None:
        return None

    fsa = struct.unpack("<H", data[0:2])[0]
    lsa = struct.unpack("<H", data[2:4])[0]

    # Distance data.
    sample_data = read_bytes(lsn * 2)

    if sample_data is None:
        return None

    distances = []

    for i in range(lsn):

        raw = struct.unpack(
            "<H",
            sample_data[i * 2:i * 2 + 2]
        )[0]

        distances.append(raw / 4.0)

    # Angles.
    start_angle = (fsa >> 1) / 64.0
    end_angle = (lsa >> 1) / 64.0

    if lsn > 1:

        if end_angle < start_angle:
            end_angle += 360.0

        angle_step = (end_angle - start_angle) / (lsn - 1)

    else:
        angle_step = 0

    points = []

    for i, distance in enumerate(distances):

        angle = (start_angle + i * angle_step) % 360.0

        if distance <= 0 or distance > 8000:
            continue

        points.append((angle, distance))

    return points


# =========================================================
# BUZZER AND MOTOR SAFETY CONTROL
# =========================================================

def check_buzzer_and_motors(scan):
    """
    Buzz and stop both motors if any reading in the front-facing
    cone (see FRONT_ANGLE / FRONT_HALF_ANGLE) is 600-700 mm.
    Obstacles outside that cone (to the sides or behind the
    robot) are ignored.
    """

    global obstacle_detected

    obstacle_detected = False

    for angle, distance in scan:

        if not is_in_front(angle):
            continue

        if (
            distance > BUZZER_MIN_DISTANCE
            and distance < BUZZER_MAX_DISTANCE
        ):

            obstacle_detected = True

            buzzer.on()
            left_pwm.value = 0
            right_pwm.value = 0

            return

    buzzer.off()


# =========================================================
# KEYBOARD MOTOR COMMANDS
# =========================================================

def enable_keyboard():
    """Read individual key presses without requiring Enter."""

    global original_terminal_settings

    if not sys.stdin.isatty():
        return

    original_terminal_settings = termios.tcgetattr(sys.stdin.fileno())
    tty.setcbreak(sys.stdin.fileno())


def restore_terminal():

    if original_terminal_settings is not None:
        termios.tcsetattr(
            sys.stdin.fileno(),
            termios.TCSADRAIN,
            original_terminal_settings
        )


def handle_keyboard():
    """Handle arrow keys while LiDAR scanning continues."""

    global last_motor_command

    if not sys.stdin.isatty():
        return

    if not select.select([sys.stdin], [], [], 0)[0]:
        return

    key = sys.stdin.read(1)

    if key == " ":
        stop_motors()
        last_motor_command = "STOPPED"
        return

    if key != "\x1b":
        return

    # Arrow keys are sent as ESC followed by [ and A/B/C/D.
    if not select.select([sys.stdin], [], [], 0.02)[0]:
        return

    if sys.stdin.read(1) != "[":
        return

    if not select.select([sys.stdin], [], [], 0.02)[0]:
        return

    arrow_key = sys.stdin.read(1)

    if arrow_key == "A":
        moved = set_motors(COMMAND_SPEED, COMMAND_SPEED)
        command_name = "FORWARD"
    elif arrow_key == "B":
        moved = set_motors(-COMMAND_SPEED, -COMMAND_SPEED)
        command_name = "BACKWARD"
    elif arrow_key == "D":
        moved = set_motors(-COMMAND_SPEED, COMMAND_SPEED)
        command_name = "LEFT"
    elif arrow_key == "C":
        moved = set_motors(COMMAND_SPEED, -COMMAND_SPEED)
        command_name = "RIGHT"
    else:
        return

    if moved:
        last_motor_command = command_name
    else:
        last_motor_command = "BLOCKED BY LIDAR"


# =========================================================
# TERMINAL OUTPUT
# =========================================================

def print_matching_points(scan):

    # ANSI escape codes redraw the same terminal view instead of scrolling.
    print("\033[2J\033[H", end="")
    print("Distances above 600 mm and below 700 mm (front cone only):")
    print("Controls: Up=forward  Down=back  Left/Right=turn  Space=stop")
    print("Motor command:", last_motor_command)

    matching_points = 0

    for angle, distance in scan:

        if not is_in_front(angle):
            continue

        if (
            distance > PRINT_MIN_DISTANCE
            and distance < PRINT_MAX_DISTANCE
        ):

            print(f"Angle: {angle:.1f} deg | Distance: {distance:.1f} mm")
            matching_points += 1

    if matching_points == 0:
        print("No distances in this range.")
        print("BUZZER OFF")
    else:
        print("BUZZER ON - MOTORS STOPPED")


# =========================================================
# MAIN LOOP
# =========================================================

try:

    # No IR sensors or line-following logic are used in this program.
    stop_motors()
    enable_keyboard()

    full_scan = []
    previous_angle = None
    last_display = time.time()

    while True:

        handle_keyboard()

        packet = read_packet()

        if packet is None:
            continue

        for angle, distance in packet:

            # Detect the 360 -> 0 degree transition.
            if (
                previous_angle is not None
                and angle < previous_angle - 180
            ):

                if len(full_scan) > 50:

                    current_time = time.time()

                    if (
                        current_time - last_display
                        >= UPDATE_INTERVAL
                    ):

                        check_buzzer_and_motors(full_scan)
                        print_matching_points(full_scan)
                        last_display = current_time

                full_scan = []

            full_scan.append((angle, distance))
            previous_angle = angle


except KeyboardInterrupt:

    print("\nStopping robot...")


finally:

    stop_motors()
    buzzer.off()
    restore_terminal()

    left_pwm.close()
    right_pwm.close()
    left_dir.close()
    right_dir.close()
    buzzer.close()
    ser.close()

    print("Motors stopped.")
    print("LiDAR disconnected.")
    print("Buzzer turned off.")
