import tkinter as tk
from tkinter import messagebox
import cv2
from PIL import Image, ImageTk
import serial
import struct
import time
import threading
from gpiozero import (
    PWMOutputDevice,
    DigitalOutputDevice,
    DigitalInputDevice,
    Buzzer,
    OutputDevice,
)


# ============================================================
# IRIS ROBOT - INTEGRATED PROGRAM
#
# Integrated:
#   1. 3-channel IR line following
#   2. YDLIDAR X2 obstacle detection
#   3. Buzzer safety system
#   4. USB webcam QR room identification
#   5. Tkinter touchscreen UI (compact 5" / 800x480 layout)
#   6. QR-triggered ROOM ENTRY TURN
#
# FIXED IR PINOUT - SAME AS THE WORKING ORIGINAL PROGRAM:
#   Left   -> GPIO 22 (Physical 15)
#   Centre -> GPIO 25 (Physical 22)
#   Right  -> GPIO 6  (Physical 31)
#
# MOTOR DRIVER:
#   Left PWM  -> GPIO 18
#   Left DIR  -> GPIO 23
#   Right PWM -> GPIO 13
#   Right DIR -> GPIO 24
#
# BUZZER:
#   GPIO 26 (Physical 37)
#
# LIDAR:
#   /dev/ttyUSB0, 115200 baud
#
# NOTE:
#   QR codes are used ONLY for room identification. When the QR of
#   the SELECTED DESTINATION room is seen, the robot performs an
#   entry manoeuvre (turn into the doorway, then drive in) instead
#   of simply stopping on the line.
# ============================================================


# ============================================================
# CONFIGURATION
# ============================================================

CAMERA_INDEX = 1

LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUDRATE = 115200

BUZZER_PIN = 26

BUZZER_MIN_DISTANCE = 600
BUZZER_MAX_DISTANCE = 700

PRINT_MIN_DISTANCE = 600
PRINT_MAX_DISTANCE = 700

LIDAR_UPDATE_INTERVAL = 0.30

# Front-facing LiDAR detection cone.
# FRONT_ANGLE = 0 degrees is straight ahead for the current mounting.
# FRONT_HALF_ANGLE = 45 gives a 90-degree cone (315-360 and 0-45).
FRONT_ANGLE = 0
FRONT_HALF_ANGLE = 45

BASE_SPEED = 0.40
TURN_SPEED = 0.40

# Set to 1 if your motor directions need to be inverted.
FORWARD_DIRECTION = 0

MAX_LOAD_KG = 5.0

ROOM_PREFIX = "Room "
ROOM_NUMBER_MAX_DIGITS = 3

ROOMS = ["Room 1", "Room 2", "Room 3", "Room 4"]
starting_room = ROOMS[0]
selected_room = None


# ------------------------------------------------------------
# ROOM ENTRY MANOEUVRE (QR TRIGGERED)
# ------------------------------------------------------------
#
# When the destination room's QR code is detected, the robot
# stops following the line and turns into the doorway.
#
# ENTRY_TURN_TIME    : how long to pivot towards the door.
# ENTRY_FORWARD_TIME : how long to drive forward into the room
#                      after the pivot.
# Tune both on the actual floor - they set how far the robot
# turns and how deep it drives into the room.
#
# ROOM_ENTRY_TURN lets each room have its own door side.
# Any room not listed uses DEFAULT_ENTRY_TURN.
# ------------------------------------------------------------

DEFAULT_ENTRY_TURN = "RIGHT"

ENTRY_TURN_TIME = 1.20
ENTRY_FORWARD_TIME = 1.50

ROOM_ENTRY_TURN = {
    "Room 1": "RIGHT",
    "Room 2": "LEFT",
    "Room 3": "RIGHT",
    "Room 4": "LEFT",
}


# ------------------------------------------------------------
# UI SCALE - tuned for a 5 inch 800x480 touchscreen.
# Increase these numbers for a larger display.
# ------------------------------------------------------------

F_TITLE = 15          # main header title
F_SYSTEM = 10         # header status text
F_PANEL_HEAD = 11     # panel headings
F_STATUS = 10         # ROBOT STATUS monospace block
F_SENSOR = 9          # SENSOR STATUS monospace block
F_TEXT = 10           # normal labels
F_SMALL = 8           # captions
F_ACTION = 13         # CURRENT ACTION banner
F_START = 16          # big start button
F_BUTTON = 11         # control buttons


# ============================================================
# SHARED ROBOT STATE
# ============================================================

state_lock = threading.Lock()

robot_state = "IDLE"
navigation_state = "LINE FOLLOWING"
current_location = "Unknown"
current_action = "WAITING"

last_qr = None

line_detected = False

door_status = "READY"
elevator_status = "NOT REQUIRED"
uv_status = "OFF"

load_value = 0.0

obstacle_detected = False
# LiDAR audible safety is enabled only after START TRANSPORT is pressed.
lidar_active = False
last_motor_command = "STOPPED"

program_running = True

# States in which the line-following loop must not drive.
HALT_STATES = ("PAUSED", "EMERGENCY STOP", "ARRIVED")


# ============================================================
# MOTOR DRIVER
# ============================================================

left_pwm = PWMOutputDevice(18)
left_dir = DigitalOutputDevice(23)

right_pwm = PWMOutputDevice(13)
right_dir = DigitalOutputDevice(24)


def set_left(speed, forward=True):
    speed = max(0.0, min(1.0, speed))
    left_dir.value = (
        FORWARD_DIRECTION
        if forward
        else (1 - FORWARD_DIRECTION)
    )
    left_pwm.value = speed


def set_right(speed, forward=True):
    speed = max(0.0, min(1.0, speed))
    right_dir.value = (
        FORWARD_DIRECTION
        if forward
        else (1 - FORWARD_DIRECTION)
    )
    right_pwm.value = speed


def stop_motors():
    left_pwm.value = 0
    right_pwm.value = 0


def forward():
    # Preserves the original line-following motor orientation.
    set_left(BASE_SPEED, True)
    set_right(BASE_SPEED, False)


def backward():
    set_left(BASE_SPEED, False)
    set_right(BASE_SPEED, False)


def turn_right():
    set_left(TURN_SPEED, True)
    set_right(TURN_SPEED, False)


def turn_left():
    set_left(TURN_SPEED, False)
    set_right(TURN_SPEED, True)


def set_motors(left_speed, right_speed):
    """Used by optional manual keyboard control."""
    with state_lock:
        if obstacle_detected:
            stop_motors()
            return False

    if left_speed >= 0:
        left_dir.off()
        left_pwm.value = min(left_speed, 1.0)
    else:
        left_dir.on()
        left_pwm.value = min(-left_speed, 1.0)

    if right_speed >= 0:
        right_dir.off()
        right_pwm.value = min(right_speed, 1.0)
    else:
        right_dir.on()
        right_pwm.value = min(-right_speed, 1.0)

    return True


# ============================================================
# 3-CHANNEL IR SENSOR
# ============================================================

# These are the exact pins used by the original working line-following code.
ir_left = DigitalInputDevice(22, pull_up=True)
ir_center = DigitalInputDevice(25, pull_up=True)
ir_right = DigitalInputDevice(6, pull_up=True)


def line_following_loop():
    """
    3-channel IR line following using the exact logic of the
    original working line-following program.

    Sensor logic (pull_up=True):
        0 = black line
        1 = white surface

    Original motion priority:
        0 0 0 -> STOP
        C == 0 -> FORWARD
        L == 0 -> RIGHT
        R == 0 -> LEFT
        1 1 1 -> STOP / LINE LOST

    The only additions here are the integrated-program safety gates:
    LiDAR obstacle, pause/emergency/arrival, and room-entry ownership.
    """

    global line_detected
    global current_action
    global last_motor_command

    while program_running:

        # Read the same three sensors as the original program.
        L = ir_left.value
        C = ir_center.value
        R = ir_right.value

        line_detected = not (L == 1 and C == 1 and R == 1)

        with state_lock:
            obstacle = obstacle_detected
            state = robot_state

        # Room-entry manoeuvre owns the motors.
        if state == "ENTERING ROOM":
            time.sleep(0.02)
            continue

        # Integrated-program safety states.
        if obstacle:
            stop_motors()
            motion = "STOP - LIDAR OBSTACLE"

        elif state in HALT_STATES:
            stop_motors()
            motion = "ARRIVED" if state == "ARRIVED" else "PAUSED"

        elif state not in ("RUNNING", "RETURNING"):
            # Do not move until START TRANSPORT is pressed.
            stop_motors()
            motion = "STOP - IDLE"

        else:
            # ====================================================
            # EXACT ORIGINAL LINE-FOLLOWING DECISION LOGIC
            # ====================================================

            if L == 0 and C == 0 and R == 0:
                # All black
                stop_motors()
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
                # All white / line lost
                stop_motors()
                motion = "STOP - LINE LOST"

        with state_lock:
            current_action = motion
            last_motor_command = motion

        time.sleep(0.02)


# ============================================================
# ROOM ENTRY MANOEUVRE
# ============================================================

def drive_timed(duration, drive_call):
    """
    Run drive_call() for 'duration' seconds of clear driving.

    The timer is paused (not consumed) whenever the LiDAR reports
    an obstacle, so the manoeuvre still travels the intended
    distance after the path clears.

    Returns False if the manoeuvre was aborted (pause, emergency
    stop, shutdown), True if it completed.
    """

    remaining = duration

    while program_running and remaining > 0:

        with state_lock:
            blocked = obstacle_detected
            aborted = robot_state != "ENTERING ROOM"

        if aborted:
            stop_motors()
            return False

        if blocked:
            stop_motors()
            time.sleep(0.05)
            continue

        drive_call()

        time.sleep(0.02)
        remaining -= 0.02

    stop_motors()
    return program_running


def start_room_entry(room):
    """
    Triggered when the destination room's QR code is detected.

    The robot leaves the line, pivots towards the door side
    configured for that room, then drives into the room.
    """

    global robot_state
    global current_action
    global navigation_state

    direction = ROOM_ENTRY_TURN.get(
        room,
        DEFAULT_ENTRY_TURN
    ).upper()

    with state_lock:
        if robot_state in ("ENTERING ROOM", "ARRIVED"):
            return
        robot_state = "ENTERING ROOM"
        navigation_state = f"ENTERING ROOM ({direction})"
        current_action = f"TURN {direction} INTO {room.upper()}"

    stop_motors()

    def manoeuvre():

        global robot_state
        global navigation_state
        global current_action
        global current_location

        pivot = turn_left if direction == "LEFT" else turn_right

        # 1. Pivot off the line, towards the doorway.
        if not drive_timed(ENTRY_TURN_TIME, pivot):
            return

        with state_lock:
            if robot_state != "ENTERING ROOM":
                return
            current_action = f"DRIVING INTO {room.upper()}"

        # 2. Drive forward through the doorway.
        if not drive_timed(ENTRY_FORWARD_TIME, forward):
            return

        stop_motors()

        with state_lock:
            if robot_state != "ENTERING ROOM":
                return
            robot_state = "ARRIVED"
            navigation_state = "INSIDE ROOM"
            current_location = room
            current_action = f"ARRIVED IN {room.upper()}"

        root.after(0, lambda: on_room_entered(room, direction))

    threading.Thread(
        target=manoeuvre,
        daemon=True
    ).start()

    # UI feedback for the start of the manoeuvre.
    action_text.config(
        text=f"↱ ENTERING {room.upper()} ({direction})",
        fg=ORANGE
    )

    next_marker.config(
        text=f"TURNING {direction} INTO DOORWAY"
    )

    system_status.config(
        text="● ENTERING ROOM",
        fg=ORANGE
    )


def on_room_entered(room, direction):
    """UI update once the entry manoeuvre has finished."""

    action_text.config(
        text=f"■ ARRIVED: {room.upper()}",
        fg=GREEN
    )

    next_marker.config(
        text="DESTINATION REACHED"
    )

    system_status.config(
        text="● DELIVERY COMPLETE",
        fg=GREEN
    )

    qr_status.config(
        text=f"IN {room.upper()}",
        fg=GREEN
    )


# ============================================================
# BUZZER
# ============================================================

buzzer = Buzzer(BUZZER_PIN)

# Relay module
# IN1 -> GPIO 2 (Physical pin 3)
# VCC -> 5V
# GND -> GND
#
# NOTE ON active_high:
# Most low-cost single/dual relay boards (the common "HW-XXX"
# modules with an opto-isolator) are ACTIVE-LOW: pulling the
# IN pin LOW energizes the relay, and HIGH keeps it off.
# GPIO2 also doubles as the I2C1 SDA line and has a hardware
# pull-up that leaves it HIGH by default at boot - so an
# active-low relay module correctly stays OFF before Python
# even runs. If active_high=True is used with gpiozero, calling
# relay.off() actually drives the pin LOW, which immediately
# switches an active-low relay board ON the moment this line
# runs - which is exactly the "turns on when I run the code"
# symptom. Setting active_high=False fixes this: gpiozero then
# writes HIGH for .off() and LOW for .on(), matching the board.
#
# If your relay module is actually ACTIVE-HIGH (LOW = off,
# HIGH = on), flip this back to active_high=True.
relay = OutputDevice(2, active_high=False, initial_value=False)
relay.off()
RELAY_DURATION = 12.0
relay_active = False


def activate_relay_for_transport():
    """Activate the delivery relay for the transport duration.

    Only ever called from start_robot(), which itself only runs
    when the START TRANSPORT button is pressed on the touchscreen -
    so the relay stays off until the user presses Start.
    """
    global relay_active

    if relay_active:
        return

    relay_active = True
    relay.on()
    print(f"Relay ON - transport active for {RELAY_DURATION:.0f} seconds")

    def deactivate():
        global relay_active
        relay.off()
        relay_active = False
        print("Relay OFF - transport timer complete")

    timer = threading.Timer(RELAY_DURATION, deactivate)
    timer.daemon = True
    timer.start()


# ============================================================
# LIDAR SERIAL CONNECTION
# ============================================================

try:
    lidar_serial = serial.Serial(
        LIDAR_PORT,
        LIDAR_BAUDRATE,
        timeout=1
    )
    lidar_available = True
except Exception as e:
    lidar_serial = None
    lidar_available = False
    print("WARNING: LiDAR could not be opened:", e)


if lidar_available:
    print("========================================")
    print("        IRIS LIDAR SAFETY SYSTEM")
    print("========================================")
    print("LiDAR Port:", LIDAR_PORT)
    print("Baudrate:", LIDAR_BAUDRATE)
    print(
        "Front detection cone:",
        (FRONT_ANGLE - FRONT_HALF_ANGLE) % 360,
        "to",
        (FRONT_ANGLE + FRONT_HALF_ANGLE) % 360,
        "degrees"
    )
    print(
        "Obstacle range:",
        BUZZER_MIN_DISTANCE,
        "< distance <",
        BUZZER_MAX_DISTANCE,
        "mm"
    )
    print("========================================")


def read_bytes(number):
    data = lidar_serial.read(number)

    if len(data) != number:
        return None

    return data


def read_packet():
    if not lidar_available:
        return None

    while program_running:

        b1 = lidar_serial.read(1)

        if not b1:
            return None

        if b1[0] != 0xAA:
            continue

        b2 = lidar_serial.read(1)

        if not b2:
            return None

        if b2[0] != 0x55:
            continue

        break

    data = read_bytes(2)

    if data is None:
        return None

    lsn = data[1]

    if lsn == 0:
        return None

    data = read_bytes(6)

    if data is None:
        return None

    fsa = struct.unpack("<H", data[0:2])[0]
    lsa = struct.unpack("<H", data[2:4])[0]

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

    start_angle = (fsa >> 1) / 64.0
    end_angle = (lsa >> 1) / 64.0

    if lsn > 1:

        if end_angle < start_angle:
            end_angle += 360.0

        angle_step = (
            end_angle - start_angle
        ) / (lsn - 1)

    else:
        angle_step = 0

    points = []

    for i, distance in enumerate(distances):

        angle = (
            start_angle + i * angle_step
        ) % 360.0

        if distance <= 0 or distance > 8000:
            continue

        points.append((angle, distance))

    return points


# ============================================================
# LIDAR SAFETY
# ============================================================

def is_in_front(angle, front_angle=FRONT_ANGLE, half_width=FRONT_HALF_ANGLE):
    """
    Return True when a LiDAR angle lies inside the front detection cone.

    Handles the 0/360 degree wrap correctly. For example, with
    FRONT_ANGLE=0 and FRONT_HALF_ANGLE=45, the active cone is
    315-360 degrees plus 0-45 degrees.
    """
    diff = abs(angle - front_angle) % 360

    if diff > 180:
        diff = 360 - diff

    return diff <= half_width


def check_lidar_scan(scan):
    """
    Stop the robot and activate the buzzer when any LiDAR reading
    in the FRONT cone is strictly between 600 and 700 mm.

    Readings outside the front cone are ignored.
    """

    global obstacle_detected
    global current_action
    global last_motor_command
    global lidar_active

    # Do not beep or apply LiDAR obstacle stopping before START TRANSPORT.
    if not lidar_active:
        with state_lock:
            obstacle_detected = False
        buzzer.off()
        return

    detected = False

    for angle, distance in scan:

        if not is_in_front(angle):
            continue

        if (
            distance > BUZZER_MIN_DISTANCE
            and distance < BUZZER_MAX_DISTANCE
        ):
            detected = True
            break

    with state_lock:
        previous = obstacle_detected
        obstacle_detected = detected

    if detected:

        buzzer.on()
        stop_motors()

        with state_lock:
            current_action = "STOP - LIDAR OBSTACLE"
            last_motor_command = "BLOCKED BY LIDAR"

    else:

        buzzer.off()

        # Never restart motors directly from the LiDAR thread.
        # The line-following loop remains responsible for movement.
        if previous:
            with state_lock:
                if robot_state not in HALT_STATES:
                    current_action = "LINE FOLLOWING"


def lidar_loop():

    if not lidar_available:
        return

    full_scan = []
    previous_angle = None
    last_display = time.time()

    while program_running:

        packet = read_packet()

        if packet is None:
            continue

        for angle, distance in packet:

            if (
                previous_angle is not None
                and angle < previous_angle - 180
            ):

                if len(full_scan) > 50:

                    current_time = time.time()

                    if (
                        current_time - last_display
                        >= LIDAR_UPDATE_INTERVAL
                    ):

                        check_lidar_scan(full_scan)

                        last_display = current_time

                full_scan = []

            full_scan.append((angle, distance))
            previous_angle = angle


# ============================================================
# TKINTER UI
# ============================================================

BG = "#101820"
PANEL = "#18252E"
PANEL2 = "#20313B"

WHITE = "#FFFFFF"
GREY = "#AAB7BD"

GREEN = "#16A085"
RED = "#C0392B"
ORANGE = "#F39C12"
BLUE = "#2980B9"
TURQUOISE = "#35C6B3"

BLACK = "#000000"


root = tk.Tk()

root.title("Hospital Autonomous Logistics Robot")
root.configure(bg=BG)
root.geometry("800x480")
root.attributes("-fullscreen", True)


def create_panel(parent, width=None, height=None):

    frame = tk.Frame(
        parent,
        bg=PANEL,
        bd=1,
        relief="ridge"
    )

    if width and height:
        frame.config(
            width=width,
            height=height
        )
        frame.pack_propagate(False)

    return frame


def label(parent, text, size=F_TEXT, color=WHITE, bold=False):

    font = ("Arial", size)

    if bold:
        font = ("Arial", size, "bold")

    return tk.Label(
        parent,
        text=text,
        font=font,
        fg=color,
        bg=parent["bg"]
    )


# ============================================================
# ON-SCREEN TOUCH KEYBOARD
#
# The robot has no physical keyboard, so any text the operator
# has to type (currently the new room name) is entered through
# this pop-up QWERTY keypad instead of tkinter's simpledialog.
#
# touch_keyboard() returns the typed string, or None if the
# operator pressed CANCEL.
# ============================================================

KB_ROWS = [
    "1234567890",
    "QWERTYUIOP",
    "ASDFGHJKL",
    "ZXCVBNM-_",
]


def touch_keyboard(
    title="Input",
    prompt="Enter text:",
    initial="",
    max_length=24
):

    result = {"value": None}

    win = tk.Toplevel(root)
    win.title(title)
    win.configure(bg=BG)
    win.transient(root)
    win.overrideredirect(False)

    kb_w = 760
    kb_h = 400

    screen_w = root.winfo_screenwidth()
    screen_h = root.winfo_screenheight()

    pos_x = max(0, (screen_w - kb_w) // 2)
    pos_y = max(0, (screen_h - kb_h) // 2)

    win.geometry(f"{kb_w}x{kb_h}+{pos_x}+{pos_y}")
    win.attributes("-topmost", True)

    shift_on = {"state": True}
    key_buttons = []

    text_var = tk.StringVar(value=initial)

    # ---------------- prompt + display ----------------

    tk.Label(
        win,
        text=prompt,
        font=("Arial", F_PANEL_HEAD, "bold"),
        fg=TURQUOISE,
        bg=BG
    ).pack(anchor="w", padx=12, pady=(8, 2))

    entry = tk.Entry(
        win,
        textvariable=text_var,
        font=("Arial", 18, "bold"),
        fg=WHITE,
        bg=PANEL2,
        insertbackground=WHITE,
        relief="flat",
        justify="left"
    )

    entry.pack(fill="x", padx=12, ipady=6)
    entry.focus_set()

    # ---------------- key actions ----------------

    def insert(char):
        current = text_var.get()
        if len(current) >= max_length:
            return
        text_var.set(current + char)
        entry.icursor("end")

    def backspace():
        text_var.set(text_var.get()[:-1])
        entry.icursor("end")

    def clear_all():
        text_var.set("")

    def toggle_shift():
        shift_on["state"] = not shift_on["state"]
        refresh_key_faces()

    def refresh_key_faces():
        upper = shift_on["state"]
        for button, char in key_buttons:
            if char.isalpha():
                button.config(
                    text=char.upper() if upper else char.lower()
                )
        shift_button.config(
            bg=GREEN if upper else PANEL2
        )

    def accept(event=None):
        result["value"] = text_var.get().strip()
        win.grab_release()
        win.destroy()

    def cancel(event=None):
        result["value"] = None
        win.grab_release()
        win.destroy()

    def make_key_command(char):
        return lambda: insert(
            char.upper() if shift_on["state"] else char.lower()
        )

    # ---------------- key grid ----------------

    keys_frame = tk.Frame(win, bg=BG)
    keys_frame.pack(padx=8, pady=8)

    for row_text in KB_ROWS:

        row_frame = tk.Frame(keys_frame, bg=BG)
        row_frame.pack(pady=3)

        for char in row_text:

            button = tk.Button(
                row_frame,
                text=char,
                font=("Arial", 15, "bold"),
                fg=WHITE,
                bg=PANEL2,
                activebackground=TURQUOISE,
                activeforeground=BLACK,
                relief="flat",
                width=3,
                height=1,
                command=(
                    make_key_command(char)
                    if char.isalpha()
                    else (lambda c=char: insert(c))
                )
            )

            button.pack(side="left", padx=3)

            key_buttons.append((button, char))

    # ---------------- bottom action row ----------------

    action_row = tk.Frame(win, bg=BG)
    action_row.pack(fill="x", padx=12, pady=(2, 10))

    shift_button = tk.Button(
        action_row,
        text="⇧ CAPS",
        font=("Arial", F_BUTTON, "bold"),
        fg=WHITE,
        bg=GREEN,
        activebackground=GREEN,
        relief="flat",
        width=8,
        height=2,
        command=toggle_shift
    )
    shift_button.pack(side="left", padx=3)

    tk.Button(
        action_row,
        text="SPACE",
        font=("Arial", F_BUTTON, "bold"),
        fg=WHITE,
        bg=PANEL2,
        activebackground=TURQUOISE,
        relief="flat",
        width=16,
        height=2,
        command=lambda: insert(" ")
    ).pack(side="left", padx=3)

    tk.Button(
        action_row,
        text="⌫ DEL",
        font=("Arial", F_BUTTON, "bold"),
        fg=WHITE,
        bg=ORANGE,
        activebackground=ORANGE,
        relief="flat",
        width=7,
        height=2,
        command=backspace
    ).pack(side="left", padx=3)

    tk.Button(
        action_row,
        text="CLEAR",
        font=("Arial", F_BUTTON, "bold"),
        fg=WHITE,
        bg=ORANGE,
        activebackground=ORANGE,
        relief="flat",
        width=7,
        height=2,
        command=clear_all
    ).pack(side="left", padx=3)

    tk.Button(
        action_row,
        text="✕ CANCEL",
        font=("Arial", F_BUTTON, "bold"),
        fg=WHITE,
        bg=RED,
        activebackground=RED,
        relief="flat",
        width=9,
        height=2,
        command=cancel
    ).pack(side="right", padx=3)

    tk.Button(
        action_row,
        text="✓ OK",
        font=("Arial", F_BUTTON, "bold"),
        fg=WHITE,
        bg=GREEN,
        activebackground=GREEN,
        relief="flat",
        width=9,
        height=2,
        command=accept
    ).pack(side="right", padx=3)

    refresh_key_faces()

    # A USB keyboard still works if one is plugged in.
    win.bind("<Return>", accept)
    win.bind("<Escape>", cancel)

    win.protocol("WM_DELETE_WINDOW", cancel)

    win.grab_set()
    win.wait_window()

    return result["value"]


def number_keypad(
    title="Number",
    prompt="Enter number:",
    initial="",
    max_digits=3
):
    """
    Numbers-only (0-9) pop-up keypad.

    Used wherever the operator has to enter a number on the
    touchscreen - currently the room number in ADD NEW ROOM.
    It is created on demand and destroyed on OK/CANCEL, so it
    never takes up space on the main screen.

    Returns the typed digits as a string, or None if cancelled.
    """

    result = {"value": None}

    win = tk.Toplevel(root)
    win.title(title)
    win.configure(bg=BG)
    win.transient(root)

    pad_w = 340
    pad_h = 430

    screen_w = root.winfo_screenwidth()
    screen_h = root.winfo_screenheight()

    win.geometry(
        f"{pad_w}x{pad_h}+"
        f"{max(0, (screen_w - pad_w) // 2)}+"
        f"{max(0, (screen_h - pad_h) // 2)}"
    )
    win.attributes("-topmost", True)

    text_var = tk.StringVar(value=initial)

    tk.Label(
        win,
        text=prompt,
        font=("Arial", F_PANEL_HEAD, "bold"),
        fg=TURQUOISE,
        bg=BG,
        wraplength=300,
        justify="left"
    ).pack(anchor="w", padx=12, pady=(10, 3))

    display = tk.Label(
        win,
        textvariable=text_var,
        font=("Consolas", 26, "bold"),
        fg=WHITE,
        bg=PANEL2,
        anchor="e",
        padx=10
    )

    display.pack(fill="x", padx=12, ipady=6)

    def insert(digit):
        current = text_var.get()
        if len(current) >= max_digits:
            return
        text_var.set(current + digit)

    def backspace():
        text_var.set(text_var.get()[:-1])

    def clear_all():
        text_var.set("")

    def accept(event=None):
        result["value"] = text_var.get().strip()
        win.grab_release()
        win.destroy()

    def cancel(event=None):
        result["value"] = None
        win.grab_release()
        win.destroy()

    # ---------------- digit grid ----------------

    grid = tk.Frame(win, bg=BG)
    grid.pack(padx=12, pady=10)

    digit_rows = [
        ["1", "2", "3"],
        ["4", "5", "6"],
        ["7", "8", "9"],
    ]

    def digit_button(parent, text, command, bg=PANEL2, span=1):
        return tk.Button(
            parent,
            text=text,
            font=("Arial", 20, "bold"),
            fg=WHITE,
            bg=bg,
            activebackground=TURQUOISE,
            activeforeground=BLACK,
            relief="flat",
            width=4 if span == 1 else 9,
            height=1,
            command=command
        )

    for row in digit_rows:

        row_frame = tk.Frame(grid, bg=BG)
        row_frame.pack(pady=4)

        for digit in row:
            digit_button(
                row_frame,
                digit,
                lambda d=digit: insert(d)
            ).pack(side="left", padx=4)

    # Bottom digit row: CLEAR | 0 | DEL
    bottom_row = tk.Frame(grid, bg=BG)
    bottom_row.pack(pady=4)

    digit_button(
        bottom_row,
        "C",
        clear_all,
        bg=ORANGE
    ).pack(side="left", padx=4)

    digit_button(
        bottom_row,
        "0",
        lambda: insert("0")
    ).pack(side="left", padx=4)

    digit_button(
        bottom_row,
        "⌫",
        backspace,
        bg=ORANGE
    ).pack(side="left", padx=4)

    # ---------------- OK / CANCEL ----------------

    action_row = tk.Frame(win, bg=BG)
    action_row.pack(fill="x", padx=12, pady=(4, 12))

    tk.Button(
        action_row,
        text="✕ CANCEL",
        font=("Arial", F_BUTTON, "bold"),
        fg=WHITE,
        bg=RED,
        activebackground=RED,
        relief="flat",
        height=2,
        command=cancel
    ).pack(side="left", expand=True, fill="x", padx=(0, 4))

    tk.Button(
        action_row,
        text="✓ OK",
        font=("Arial", F_BUTTON, "bold"),
        fg=WHITE,
        bg=GREEN,
        activebackground=GREEN,
        relief="flat",
        height=2,
        command=accept
    ).pack(side="left", expand=True, fill="x", padx=(4, 0))

    win.bind("<Return>", accept)
    win.bind("<Escape>", cancel)

    for digit in "0123456789":
        win.bind(digit, lambda e: insert(e.char))

    win.bind("<BackSpace>", lambda e: backspace())

    win.protocol("WM_DELETE_WINDOW", cancel)

    win.grab_set()
    win.wait_window()

    return result["value"]


def touch_choice(title, prompt, options):
    """
    Large-button choice dialog for the touchscreen.

    Returns the chosen option string, or None if cancelled.
    """

    result = {"value": None}

    win = tk.Toplevel(root)
    win.title(title)
    win.configure(bg=BG)
    win.transient(root)

    box_w = 460
    box_h = 200

    screen_w = root.winfo_screenwidth()
    screen_h = root.winfo_screenheight()

    win.geometry(
        f"{box_w}x{box_h}+"
        f"{max(0, (screen_w - box_w) // 2)}+"
        f"{max(0, (screen_h - box_h) // 2)}"
    )
    win.attributes("-topmost", True)

    tk.Label(
        win,
        text=prompt,
        font=("Arial", F_PANEL_HEAD, "bold"),
        fg=WHITE,
        bg=BG,
        wraplength=420,
        justify="left"
    ).pack(padx=15, pady=(18, 10))

    button_row = tk.Frame(win, bg=BG)
    button_row.pack(expand=True, fill="both", padx=15, pady=(0, 15))

    def choose(option):
        result["value"] = option
        win.grab_release()
        win.destroy()

    for option in options:
        tk.Button(
            button_row,
            text=option,
            font=("Arial", 16, "bold"),
            fg=WHITE,
            bg=BLUE,
            activebackground=TURQUOISE,
            relief="flat",
            command=lambda o=option: choose(o)
        ).pack(
            side="left",
            expand=True,
            fill="both",
            padx=6
        )

    win.protocol("WM_DELETE_WINDOW", lambda: choose(None))
    win.bind("<Escape>", lambda e: choose(None))

    win.grab_set()
    win.wait_window()

    return result["value"]


# ============================================================
# HEADER
# ============================================================

header = tk.Frame(
    root,
    bg=BG
)

header.pack(
    fill="x",
    padx=8,
    pady=(4, 2)
)

title = tk.Label(
    header,
    text="HOSPITAL AUTONOMOUS LOGISTICS ROBOT",
    font=("Arial", F_TITLE, "bold"),
    fg=WHITE,
    bg=BG
)

title.pack(side="left")

system_status = tk.Label(
    header,
    text="● SYSTEM READY",
    font=("Arial", F_SYSTEM, "bold"),
    fg=GREEN,
    bg=BG
)

system_status.pack(
    side="right",
    padx=6
)


# ============================================================
# MAIN CONTENT
# ============================================================

main = tk.Frame(root, bg=BG)

main.pack(
    fill="both",
    expand=True,
    padx=8,
    pady=2
)


# ============================================================
# LEFT SIDE
# ============================================================

left = tk.Frame(main, bg=BG)

left.pack(
    side="left",
    fill="both",
    expand=True,
    padx=(0, 4)
)


# ============================================================
# ROOM SELECTION PANEL
# ============================================================

destination_panel = create_panel(left)

destination_panel.pack(
    fill="x",
    pady=2
)

label(
    destination_panel,
    "ROOM SELECTION",
    F_PANEL_HEAD,
    TURQUOISE,
    True
).pack(
    anchor="w",
    padx=8,
    pady=(4, 2)
)

room_select_frame = tk.Frame(
    destination_panel,
    bg=PANEL
)
room_select_frame.pack(
    fill="x",
    padx=8,
    pady=(0, 4)
)

# Starting room
start_col = tk.Frame(room_select_frame, bg=PANEL)
start_col.pack(
    side="left",
    fill="x",
    expand=True,
    padx=(0, 3)
)

label(
    start_col,
    "STARTING ROOM",
    F_SMALL,
    GREY,
    True
).pack(anchor="w")

starting_room_var = tk.StringVar(value=starting_room)

starting_room_menu = tk.OptionMenu(
    start_col,
    starting_room_var,
    *ROOMS
)
starting_room_menu.config(
    font=("Arial", F_TEXT, "bold"),
    fg=WHITE,
    bg=RED,
    activebackground=GREEN,
    activeforeground=WHITE,
    relief="flat",
    highlightthickness=0,
    width=10
)
starting_room_menu["menu"].config(
    font=("Arial", F_TEXT),
    bg=PANEL,
    fg=WHITE
)
starting_room_menu.pack(
    fill="x",
    ipady=1
)

# Destination room
dest_col = tk.Frame(room_select_frame, bg=PANEL)
dest_col.pack(
    side="left",
    fill="x",
    expand=True,
    padx=(3, 0)
)

label(
    dest_col,
    "DESTINATION ROOM",
    F_SMALL,
    GREY,
    True
).pack(anchor="w")

destination_var = tk.StringVar(value=ROOMS[0])

destination_menu = tk.OptionMenu(
    dest_col,
    destination_var,
    *ROOMS
)
destination_menu.config(
    font=("Arial", F_TEXT, "bold"),
    fg=WHITE,
    bg=RED,
    activebackground=GREEN,
    activeforeground=WHITE,
    relief="flat",
    highlightthickness=0,
    width=10
)
destination_menu["menu"].config(
    font=("Arial", F_TEXT),
    bg=PANEL,
    fg=WHITE
)
destination_menu.pack(
    fill="x",
    ipady=1
)

# Add-room button
add_room_frame = tk.Frame(
    destination_panel,
    bg=PANEL
)
add_room_frame.pack(
    fill="x",
    padx=8,
    pady=(0, 3)
)


def refresh_room_menus():
    """Refresh both dropdown menus after rooms are added."""
    start_menu = starting_room_menu["menu"]
    dest_menu = destination_menu["menu"]

    start_menu.delete(0, "end")
    dest_menu.delete(0, "end")

    for room in ROOMS:
        start_menu.add_command(
            label=room,
            command=lambda r=room: starting_room_var.set(r)
        )
        dest_menu.add_command(
            label=room,
            command=lambda r=room: (
                destination_var.set(r),
                select_room(r)
            )
        )


def add_new_room():
    """Add a new hospital room from the + button.

    The room NUMBER is entered on the pop-up 0-9 keypad and the
    room is named ROOM_PREFIX + number (e.g. "Room 7"), so the
    name always matches the QR codes on the doors. The door side
    is then picked with large buttons - no physical keyboard is
    needed anywhere in this flow.
    """
    number = number_keypad(
        title="Add New Room",
        prompt="ENTER THE ROOM NUMBER",
        initial="",
        max_digits=ROOM_NUMBER_MAX_DIGITS
    )

    if number is None:
        return

    number = number.strip()

    if not number or not number.isdigit():
        return

    # "007" and "7" are the same room.
    new_room = f"{ROOM_PREFIX}{int(number)}"

    # Prevent duplicate room names.
    if any(room.lower() == new_room.lower() for room in ROOMS):
        messagebox.showwarning(
            "Room Already Exists",
            f"{new_room} is already in the room list.",
            parent=root
        )
        return

    ROOMS.append(new_room)

    # Ask which side the door of the new room is on, so the
    # QR entry manoeuvre knows which way to turn.
    side = touch_choice(
        "Door Side",
        f"Which side is the door of {new_room} on?",
        ["LEFT", "RIGHT"]
    )

    if side in ("LEFT", "RIGHT"):
        ROOM_ENTRY_TURN[new_room] = side
    else:
        ROOM_ENTRY_TURN[new_room] = DEFAULT_ENTRY_TURN

    refresh_room_menus()

    # Automatically select the newly created room as the destination.
    destination_var.set(new_room)
    select_room(new_room)

    print(
        "New room added:",
        new_room,
        "- entry turn:",
        ROOM_ENTRY_TURN[new_room]
    )


add_room_button = tk.Button(
    add_room_frame,
    text="+  ADD NEW ROOM",
    font=("Arial", F_SMALL, "bold"),
    fg=WHITE,
    bg=GREEN,
    activebackground=GREEN,
    activeforeground=WHITE,
    relief="flat",
    command=add_new_room
)

add_room_button.pack(
    fill="x",
    ipady=1
)


def select_room(room):
    global selected_room

    if room not in ROOMS:
        return

    selected_room = room
    destination_var.set(room)

    turn = ROOM_ENTRY_TURN.get(room, DEFAULT_ENTRY_TURN)

    destination_status.config(
        text=f"Destination: {room}  (enter: {turn})"
    )


destination_status = label(
    destination_panel,
    "Destination: Not selected",
    F_SMALL,
    GREY
)

destination_status.pack(
    anchor="w",
    padx=8,
    pady=(0, 4)
)

# Initialize dropdown menus.
refresh_room_menus()


# ============================================================
# START PANEL
# ============================================================

start_panel = create_panel(left)

start_panel.pack(
    fill="both",
    expand=True,
    pady=2
)

label(
    start_panel,
    "TRANSPORT CONTROL",
    F_PANEL_HEAD,
    TURQUOISE,
    True
).pack(
    anchor="w",
    padx=8,
    pady=(4, 2)
)

start_button_frame = tk.Frame(start_panel, bg=PANEL)
start_button_frame.pack(
    fill="both",
    expand=True,
    padx=8,
    pady=(0, 8)
)


# ============================================================
# RIGHT SIDE
# ============================================================

right = tk.Frame(main, bg=BG)

right.pack(
    side="right",
    fill="both",
    expand=True,
    padx=(4, 0)
)


# ============================================================
# ROBOT STATUS PANEL
# ============================================================

status_panel = create_panel(right)

status_panel.pack(
    fill="x",
    pady=(0, 4)
)

label(
    status_panel,
    "ROBOT STATUS",
    F_PANEL_HEAD,
    TURQUOISE,
    True
).pack(
    anchor="w",
    padx=8,
    pady=(4, 1)
)

status_text = tk.Label(
    status_panel,
    text=(
        "STATE      : IDLE\n"
        "NAVIGATION : LINE FOLLOWING\n"
        "LOCATION   : Unknown\n"
        "ACTION     : WAITING\n"
        "FLOOR      : 1"
    ),
    font=("Consolas", F_STATUS),
    fg=WHITE,
    bg=PANEL,
    justify="left"
)

status_text.pack(
    anchor="w",
    padx=10,
    pady=(0, 5)
)


# ============================================================
# LOAD PANEL
# ============================================================

load_panel = create_panel(right)

load_panel.pack(
    fill="x",
    pady=4
)

label(
    load_panel,
    "PAYLOAD / LOAD",
    F_PANEL_HEAD,
    TURQUOISE,
    True
).pack(
    anchor="w",
    padx=8,
    pady=(4, 1)
)

load_text = label(
    load_panel,
    "LOAD: 0.00 / 5.00 KG",
    F_TEXT,
    WHITE,
    True
)

load_text.pack(
    anchor="w",
    padx=8
)

load_bar = tk.Canvas(
    load_panel,
    height=16,
    bg="#303C42",
    highlightthickness=0
)

load_bar.pack(
    fill="x",
    padx=8,
    pady=(3, 6)
)


# ============================================================
# SENSOR PANEL
# ============================================================

sensor_panel = create_panel(right)

sensor_panel.pack(
    fill="x",
    pady=4
)

label(
    sensor_panel,
    "SENSOR STATUS",
    F_PANEL_HEAD,
    TURQUOISE,
    True
).pack(
    anchor="w",
    padx=8,
    pady=(4, 1)
)

sensor_text = tk.Label(
    sensor_panel,
    text=(
        "LINE     ● ACTIVE\n"
        "QR       ● ACTIVE\n"
        "LIDAR    ● ACTIVE\n"
        "IMU      ● NOT CONNECTED\n"
        "ENCODER  ● NOT INTEGRATED\n"
        "CAMERA   ● ACTIVE"
    ),
    font=("Consolas", F_SENSOR),
    fg=WHITE,
    bg=PANEL,
    justify="left"
)

sensor_text.pack(
    anchor="w",
    padx=10,
    pady=(0, 5)
)


# ============================================================
# ACTION PANEL
# ============================================================

action_panel = create_panel(right)

action_panel.pack(
    fill="both",
    expand=True,
    pady=4
)

label(
    action_panel,
    "CURRENT ACTION",
    F_PANEL_HEAD,
    TURQUOISE,
    True
).pack(
    anchor="w",
    padx=8,
    pady=(4, 1)
)

action_text = tk.Label(
    action_panel,
    text="➜ FOLLOW BLACK LINE",
    font=("Arial", F_ACTION, "bold"),
    fg=WHITE,
    bg=PANEL,
    wraplength=360
)

action_text.pack(pady=4)

next_marker = tk.Label(
    action_panel,
    text="NEXT MARKER: WAITING",
    font=("Arial", F_SMALL),
    fg=GREY,
    bg=PANEL
)

next_marker.pack()

extra_status = tk.Label(
    action_panel,
    text="DOOR: READY   ELEVATOR: NOT REQUIRED   UV: OFF",
    font=("Consolas", F_SMALL),
    fg=WHITE,
    bg=PANEL
)

extra_status.pack(pady=(4, 6))


# ============================================================
# UI BUTTON FUNCTIONS
# ============================================================

def start_robot():
    """Start transport only after both rooms have been selected."""
    global robot_state
    global navigation_state
    global starting_room
    global last_qr

    start_room = starting_room_var.get().strip()
    destination_room = destination_var.get().strip()

    if not start_room or not destination_room:
        messagebox.showwarning(
            "Room Selection Required",
            "Please select both a starting room and a destination room "
            "before starting transport.",
            parent=root
        )
        return

    if start_room == destination_room:
        messagebox.showwarning(
            "Invalid Transport",
            "Starting room and destination room cannot be the same.",
            parent=root
        )
        return

    starting_room = start_room

    # Enable LiDAR audible/obstacle safety only after START TRANSPORT.
    global lidar_active
    lidar_active = True

    # Allow the destination QR to trigger a fresh entry manoeuvre.
    last_qr = None

    # Start the transport and activate the relay only now.
    activate_relay_for_transport()

    with state_lock:
        robot_state = "RUNNING"
        navigation_state = "LINE FOLLOWING"

    stop_motors()

    system_status.config(
        text="● TRANSPORT STARTED",
        fg=GREEN
    )

    action_text.config(
        text=f"▶ {start_room.upper()} → {destination_room.upper()}",
        fg=GREEN
    )

    next_marker.config(
        text=f"TO {destination_room.upper()}"
    )


def pause_robot():

    global robot_state

    with state_lock:
        robot_state = "PAUSED"

    stop_motors()

    system_status.config(
        text="● ROBOT PAUSED",
        fg=ORANGE
    )

    action_text.config(
        text="Ⅱ PAUSED",
        fg=ORANGE
    )


def emergency_stop():

    global robot_state

    with state_lock:
        robot_state = "EMERGENCY STOP"

    stop_motors()

    system_status.config(
        text="● EMERGENCY STOP",
        fg=RED
    )

    action_text.config(
        text="■ EMERGENCY STOP",
        fg=RED
    )

    messagebox.showwarning(
        "Emergency Stop",
        "Robot Emergency Stop Activated!"
    )


def return_robot():

    global robot_state
    global navigation_state
    global last_qr

    last_qr = None

    with state_lock:
        robot_state = "RETURNING"
        navigation_state = "LINE FOLLOWING"

    system_status.config(
        text="● RETURNING",
        fg=BLUE
    )

    action_text.config(
        text="← RETURN TO BASE",
        fg=BLUE
    )


# ============================================================
# BIG START BUTTON
# ============================================================

start_button = tk.Button(
    start_button_frame,
    text="▶ START TRANSPORT",
    font=("Arial", F_START, "bold"),
    bg=GREEN,
    fg=WHITE,
    relief="flat",
    command=start_robot
)

start_button.pack(
    fill="both",
    expand=True
)


# ============================================================
# CONTROL BUTTONS
# ============================================================

control_frame = tk.Frame(
    root,
    bg=BG
)

control_frame.pack(
    fill="x",
    padx=8,
    pady=(2, 6)
)


# ---- QR / ROOM IDENTIFICATION BOX ----

qr_panel = create_panel(control_frame, width=190, height=104)

qr_panel.pack(
    side="left",
    padx=4
)

label(
    qr_panel,
    "QR / ROOM ID",
    F_SMALL,
    TURQUOISE,
    True
).pack(
    anchor="w",
    padx=5,
    pady=(2, 0)
)

camera_label = tk.Label(
    qr_panel,
    bg=BLACK
)

camera_label.pack(
    padx=5,
    pady=1
)

qr_status = label(
    qr_panel,
    "QR: WAITING",
    F_SMALL,
    GREY,
    True
)

qr_status.pack(pady=(1, 3))


# ---- Remaining control buttons ----

tk.Button(
    control_frame,
    text="Ⅱ PAUSE",
    font=("Arial", F_BUTTON, "bold"),
    bg=ORANGE,
    fg=WHITE,
    relief="flat",
    width=12,
    height=2,
    command=pause_robot
).pack(side="left", padx=4)

tk.Button(
    control_frame,
    text="■ EMERGENCY STOP",
    font=("Arial", F_BUTTON, "bold"),
    bg=RED,
    fg=WHITE,
    relief="flat",
    width=16,
    height=2,
    command=emergency_stop
).pack(side="left", padx=4)

tk.Button(
    control_frame,
    text="↩ RETURN",
    font=("Arial", F_BUTTON, "bold"),
    bg=BLUE,
    fg=WHITE,
    relief="flat",
    width=12,
    height=2,
    command=return_robot
).pack(side="left", padx=4)


# ============================================================
# QR CODE PROCESSING
# ============================================================

cap = cv2.VideoCapture(CAMERA_INDEX)
detector = cv2.QRCodeDetector()


def process_qr(data):
    """
    QR codes are ONLY used for room identification.

    If the detected room is the selected DESTINATION, the robot
    leaves the line and performs the entry turn into that room.
    Any other room QR is simply reported as a passing marker.
    """

    global current_location
    global current_action
    global last_qr

    data = data.strip()

    if data == last_qr:
        return

    last_qr = data

    print("QR DETECTED:", data)

    if data in ROOMS:

        with state_lock:
            state = robot_state

        # ---- Destination room reached -> enter the room ----
        if data == destination_var.get().strip():

            if state in ("ENTERING ROOM", "ARRIVED"):
                return

            if state in ("PAUSED", "EMERGENCY STOP"):
                qr_status.config(
                    text=f"{data} (HELD)",
                    fg=ORANGE
                )
                return

            stop_motors()

            qr_status.config(
                text=f"DEST: {data}",
                fg=GREEN
            )

            start_room_entry(data)
            return

        # ---- Passing marker, keep following the line ----
        with state_lock:
            current_location = data

        qr_status.config(
            text=f"PASSING: {data}",
            fg=BLUE
        )

        next_marker.config(
            text=f"PASSED {data.upper()}"
        )

    else:
        qr_status.config(
            text=f"QR: {data}",
            fg=ORANGE
        )


# ============================================================
# UPDATE ROBOT STATUS
# ============================================================

def update_status():

    global load_value

    # This remains the demonstration load value from the
    # original UI. Replace with HX711/load-cell code when
    # that program is supplied.
    load_value += 0.0

    if load_value < 0:
        load_value = 0

    if load_value > MAX_LOAD_KG:
        load_value = MAX_LOAD_KG

    load_text.config(
        text=f"LOAD: {load_value:.2f} / {MAX_LOAD_KG:.2f} KG"
    )

    load_bar.delete("all")

    width = load_bar.winfo_width()

    if width > 0:

        load_width = int(
            width * load_value / MAX_LOAD_KG
        )

        load_bar.create_rectangle(
            0,
            0,
            load_width,
            16,
            fill=(
                GREEN
                if load_value < 4
                else ORANGE
            ),
            outline=""
        )

    with state_lock:
        current_state = robot_state
        current_nav = navigation_state
        current_loc = current_location
        current_act = current_action
        obstacle = obstacle_detected

    status_text.config(
        text=(
            f"STATE      : {current_state}\n"
            f"NAVIGATION : {current_nav}\n"
            f"LOCATION   : {current_loc}\n"
            f"ACTION     : {current_act}\n"
            f"FLOOR      : 1"
        )
    )

    if obstacle:
        sensor_lidar = "🔴 OBSTACLE"
    else:
        sensor_lidar = "🟢 ACTIVE"

    sensor_text.config(
        text=(
            f"LINE     {'🟢 ACTIVE' if line_detected else '🔴 LOST'}\n"
            "QR       🟢 ACTIVE\n"
            f"LIDAR    {sensor_lidar}\n"
            "IMU      ⚪ NOT CONNECTED\n"
            "ENCODER  ⚪ NOT INTEGRATED\n"
            "CAMERA   🟢 ACTIVE"
        )
    )

    root.after(500, update_status)


# ============================================================
# CAMERA UPDATE
# ============================================================

def update_camera():

    ret, frame = cap.read()

    if not ret:

        qr_status.config(
            text="NO CAMERA",
            fg=RED
        )

        root.after(100, update_camera)
        return

    frame = cv2.flip(frame, 1)

    data, points, _ = detector.detectAndDecode(frame)

    if data:

        process_qr(data)

        if points is not None:

            points = points.astype(int)

            for i in range(4):

                pt1 = tuple(points[0][i])
                pt2 = tuple(points[0][(i + 1) % 4])

                cv2.line(
                    frame,
                    pt1,
                    pt2,
                    (0, 255, 0),
                    4
                )

    # Camera display only; QR is not used for line following.
    frame_rgb = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2RGB
    )

    image = Image.fromarray(frame_rgb)

    # Compact thumbnail for the 5 inch layout.
    image.thumbnail((170, 62))

    photo = ImageTk.PhotoImage(image)

    camera_label.config(image=photo)
    camera_label.image = photo

    root.after(30, update_camera)


# ============================================================
# EXIT / SAFE SHUTDOWN
# ============================================================

def exit_program(event=None):

    global program_running

    program_running = False

    stop_motors()
    buzzer.off()

    try:
        cap.release()
    except Exception:
        pass

    if lidar_serial is not None:
        try:
            lidar_serial.close()
        except Exception:
            pass

    root.destroy()


root.bind("<Escape>", exit_program)


# ============================================================
# START BACKGROUND SYSTEMS
# ============================================================

line_thread = threading.Thread(
    target=line_following_loop,
    daemon=True
)

line_thread.start()

if lidar_available:

    lidar_thread = threading.Thread(
        target=lidar_loop,
        daemon=True
    )

    lidar_thread.start()


# ============================================================
# INITIAL STATE
# ============================================================

stop_motors()
buzzer.off()

update_camera()
update_status()

root.mainloop()


# ============================================================
# FINAL CLEANUP
# ============================================================

program_running = False

stop_motors()
buzzer.off()

try:
    cap.release()
except Exception:
    pass

if lidar_serial is not None:
    try:
        lidar_serial.close()
    except Exception:
        pass

left_pwm.close()
right_pwm.close()
left_dir.close()
right_dir.close()

ir_left.close()
ir_center.close()
ir_right.close()

relay.off()
relay_active = False
buzzer.close()

print("Robot stopped safely.")
