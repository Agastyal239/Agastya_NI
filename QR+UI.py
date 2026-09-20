import tkinter as tk
from tkinter import messagebox
import cv2
from PIL import Image, ImageTk


# ============================================================
# IRIS - UI + QR CODE ONLY
# 800 x 480 touchscreen
# ============================================================

CAMERA_INDEX = 1

ROOM_PREFIX = "Room "
ROOM_NUMBER_MAX_DIGITS = 3

ROOMS = [
    "Room 1",
    "Room 2",
    "Room 3",
    "Room 4"
]

starting_room = ROOMS[0]
selected_room = ROOMS[0]
last_qr = None


# ============================================================
# ROOM ENTRY DIRECTIONS
# ============================================================

DEFAULT_ENTRY_TURN = "RIGHT"

ROOM_ENTRY_TURN = {
    "Room 1": "RIGHT",
    "Room 2": "LEFT",
    "Room 3": "RIGHT",
    "Room 4": "LEFT",
}


# ============================================================
# UI SETTINGS
# ============================================================

BG = "#101820"
PANEL = "#18252E"

WHITE = "#FFFFFF"
GREY = "#AAB7BD"

GREEN = "#16A085"
RED = "#C0392B"
ORANGE = "#F39C12"
BLUE = "#2980B9"
TURQUOISE = "#35C6B3"

BLACK = "#000000"

F_TITLE = 15
F_SYSTEM = 10
F_PANEL_HEAD = 11
F_STATUS = 10
F_SENSOR = 9
F_TEXT = 10
F_SMALL = 8
F_ACTION = 13
F_START = 16
F_BUTTON = 11


# ============================================================
# ROOT WINDOW
# ============================================================

root = tk.Tk()

root.title("Hospital Autonomous Logistics Robot")
root.configure(bg=BG)
root.geometry("800x480")
root.attributes("-fullscreen", True)


# ============================================================
# UI HELPERS
# ============================================================

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


def label(
    parent,
    text,
    size=F_TEXT,
    color=WHITE,
    bold=False
):

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
# TOUCH ROOM NUMBER KEYPAD
# ============================================================

def number_keypad(
    title="Room Number",
    prompt="ENTER THE ROOM NUMBER",
    initial="",
    max_digits=3
):

    result = {
        "value": None
    }

    win = tk.Toplevel(root)

    win.title(title)
    win.configure(bg=BG)
    win.transient(root)
    win.grab_set()

    screen_w = root.winfo_screenwidth()
    screen_h = root.winfo_screenheight()

    width = 420
    height = 420

    x = max(0, (screen_w - width) // 2)
    y = max(0, (screen_h - height) // 2)

    win.geometry(
        f"{width}x{height}+{x}+{y}"
    )

    tk.Label(
        win,
        text=prompt,
        font=("Arial", 14, "bold"),
        fg=WHITE,
        bg=BG
    ).pack(pady=(15, 8))

    value_var = tk.StringVar(
        value=initial
    )

    display = tk.Entry(
        win,
        textvariable=value_var,
        font=("Arial", 22, "bold"),
        justify="center",
        bg=WHITE,
        fg=BLACK
    )

    display.pack(
        padx=25,
        pady=5,
        fill="x"
    )

    keypad = tk.Frame(
        win,
        bg=BG
    )

    keypad.pack(
        expand=True,
        fill="both",
        padx=25,
        pady=10
    )

    def press(value):

        current = value_var.get()

        if len(current) < max_digits:
            value_var.set(
                current + value
            )

    def backspace():

        value_var.set(
            value_var.get()[:-1]
        )

    def clear():

        value_var.set("")

    def confirm():

        value = value_var.get().strip()

        if not value:
            return

        result["value"] = value

        win.destroy()

    buttons = [
        "1", "2", "3",
        "4", "5", "6",
        "7", "8", "9",
        "CLR", "0", "⌫"
    ]

    for i, button in enumerate(buttons):

        row = i // 3
        col = i % 3

        if button == "CLR":
            command = clear

        elif button == "⌫":
            command = backspace

        else:
            command = lambda v=button: press(v)

        tk.Button(
            keypad,
            text=button,
            font=("Arial", 15, "bold"),
            bg=PANEL,
            fg=WHITE,
            activebackground=GREEN,
            activeforeground=WHITE,
            relief="flat",
            command=command
        ).grid(
            row=row,
            column=col,
            sticky="nsew",
            padx=3,
            pady=3
        )

    for i in range(4):
        keypad.rowconfigure(
            i,
            weight=1
        )

    for i in range(3):
        keypad.columnconfigure(
            i,
            weight=1
        )

    tk.Button(
        win,
        text="CONFIRM",
        font=("Arial", 13, "bold"),
        bg=GREEN,
        fg=WHITE,
        relief="flat",
        command=confirm
    ).pack(
        fill="x",
        padx=25,
        pady=(0, 15),
        ipady=8
    )

    win.wait_window()

    return result["value"]


# ============================================================
# TOUCH LEFT / RIGHT SELECTION
# ============================================================

def touch_choice(
    title,
    prompt,
    choices
):

    result = {
        "value": None
    }

    win = tk.Toplevel(root)

    win.title(title)
    win.configure(bg=BG)
    win.transient(root)
    win.grab_set()

    width = 450
    height = 250

    screen_w = root.winfo_screenwidth()
    screen_h = root.winfo_screenheight()

    x = max(
        0,
        (screen_w - width) // 2
    )

    y = max(
        0,
        (screen_h - height) // 2
    )

    win.geometry(
        f"{width}x{height}+{x}+{y}"
    )

    tk.Label(
        win,
        text=prompt,
        font=("Arial", 13, "bold"),
        fg=WHITE,
        bg=BG
    ).pack(
        pady=25
    )

    button_frame = tk.Frame(
        win,
        bg=BG
    )

    button_frame.pack(
        fill="both",
        expand=True,
        padx=25,
        pady=10
    )

    for i, choice in enumerate(choices):

        def select(
            value=choice
        ):

            result["value"] = value
            win.destroy()

        tk.Button(
            button_frame,
            text=choice,
            font=("Arial", 15, "bold"),
            bg=GREEN,
            fg=WHITE,
            relief="flat",
            command=select
        ).pack(
            side="left",
            fill="both",
            expand=True,
            padx=5
        )

    win.wait_window()

    return result["value"]


# ============================================================
# MAIN LAYOUT
# ============================================================

main = tk.Frame(
    root,
    bg=BG
)

main.pack(
    fill="both",
    expand=True,
    padx=8,
    pady=6
)


# ============================================================
# HEADER
# ============================================================

header = tk.Frame(
    main,
    bg=BG
)

header.pack(
    fill="x",
    pady=(0, 5)
)

tk.Label(
    header,
    text="IRIS",
    font=("Arial", 20, "bold"),
    fg=TURQUOISE,
    bg=BG
).pack(
    side="left"
)

system_status = tk.Label(
    header,
    text="● SYSTEM READY",
    font=("Arial", F_SYSTEM, "bold"),
    fg=GREEN,
    bg=BG
)

system_status.pack(
    side="right",
    pady=5
)


# ============================================================
# LEFT SIDE
# ============================================================

left = tk.Frame(
    main,
    bg=BG
)

left.pack(
    side="left",
    fill="both",
    expand=True,
    padx=(0, 4)
)


# ============================================================
# ROOM SELECTION
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


# ------------------------------------------------------------
# STARTING ROOM
# ------------------------------------------------------------

start_col = tk.Frame(
    room_select_frame,
    bg=PANEL
)

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
).pack(
    anchor="w"
)

starting_room_var = tk.StringVar(
    value=starting_room
)

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


# ------------------------------------------------------------
# DESTINATION ROOM
# ------------------------------------------------------------

dest_col = tk.Frame(
    room_select_frame,
    bg=PANEL
)

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
).pack(
    anchor="w"
)

destination_var = tk.StringVar(
    value=ROOMS[0]
)

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


# ============================================================
# ROOM MENU FUNCTIONS
# ============================================================

def select_room(room):

    global selected_room

    if room not in ROOMS:
        return

    selected_room = room

    destination_var.set(room)

    turn = ROOM_ENTRY_TURN.get(
        room,
        DEFAULT_ENTRY_TURN
    )

    destination_status.config(
        text=f"Destination: {room}  (enter: {turn})"
    )


def refresh_room_menus():

    start_menu = starting_room_menu["menu"]
    dest_menu = destination_menu["menu"]

    start_menu.delete(
        0,
        "end"
    )

    dest_menu.delete(
        0,
        "end"
    )

    for room in ROOMS:

        start_menu.add_command(
            label=room,
            command=lambda r=room:
                starting_room_var.set(r)
        )

        dest_menu.add_command(
            label=room,
            command=lambda r=room:
                select_room(r)
        )


# ============================================================
# ADD NEW ROOM
# ============================================================

def add_new_room():

    number = number_keypad(
        title="Add New Room",
        prompt="ENTER THE ROOM NUMBER",
        initial="",
        max_digits=ROOM_NUMBER_MAX_DIGITS
    )

    if number is None:
        return

    number = number.strip()

    if not number.isdigit():
        return

    new_room = (
        f"{ROOM_PREFIX}{int(number)}"
    )

    if any(
        room.lower() == new_room.lower()
        for room in ROOMS
    ):

        messagebox.showwarning(
            "Room Already Exists",
            f"{new_room} is already in the room list.",
            parent=root
        )

        return

    ROOMS.append(new_room)

    side = touch_choice(
        "Door Side",
        f"Which side is the door of {new_room} on?",
        ["LEFT", "RIGHT"]
    )

    if side in ("LEFT", "RIGHT"):

        ROOM_ENTRY_TURN[
            new_room
        ] = side

    else:

        ROOM_ENTRY_TURN[
            new_room
        ] = DEFAULT_ENTRY_TURN

    refresh_room_menus()

    destination_var.set(
        new_room
    )

    select_room(
        new_room
    )


# ============================================================
# ADD ROOM BUTTON
# ============================================================

add_room_button = tk.Button(
    destination_panel,
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
    padx=8,
    pady=(0, 3),
    ipady=2
)


destination_status = label(
    destination_panel,
    "Destination: Room 1",
    F_SMALL,
    GREY
)

destination_status.pack(
    anchor="w",
    padx=8,
    pady=(0, 4)
)


# ============================================================
# TRANSPORT CONTROL
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

start_button_frame = tk.Frame(
    start_panel,
    bg=PANEL
)

start_button_frame.pack(
    fill="both",
    expand=True,
    padx=8,
    pady=(0, 8)
)


# ============================================================
# ROBOT CONTROL CALLBACKS
# ============================================================

transport_started = False


def start_transport():

    global transport_started
    global starting_room
    global selected_room
    global last_qr

    start_room = (
        starting_room_var
        .get()
        .strip()
    )

    destination_room = (
        destination_var
        .get()
        .strip()
    )

    if not start_room or not destination_room:

        messagebox.showwarning(
            "Room Selection Required",
            "Please select both a starting room "
            "and destination room.",
            parent=root
        )

        return

    if start_room == destination_room:

        messagebox.showwarning(
            "Invalid Transport",
            "Starting room and destination room "
            "cannot be the same.",
            parent=root
        )

        return

    starting_room = start_room
    selected_room = destination_room

    transport_started = True

    last_qr = None

    system_status.config(
        text="● TRANSPORT STARTED",
        fg=GREEN
    )

    action_text.config(
        text=(
            f"▶ {start_room.upper()} "
            f"→ {destination_room.upper()}"
        ),
        fg=GREEN
    )

    next_marker.config(
        text=f"TO {destination_room.upper()}"
    )

    qr_status.config(
        text="QR: ACTIVE",
        fg=GREEN
    )


def pause_transport():

    system_status.config(
        text="● ROBOT PAUSED",
        fg=ORANGE
    )

    action_text.config(
        text="Ⅱ PAUSED",
        fg=ORANGE
    )


def emergency_stop():

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

    system_status.config(
        text="● RETURNING",
        fg=BLUE
    )

    action_text.config(
        text="← RETURN TO BASE",
        fg=BLUE
    )


# ============================================================
# START BUTTON
# ============================================================

start_button = tk.Button(
    start_button_frame,
    text="▶ START TRANSPORT",
    font=("Arial", F_START, "bold"),
    bg=GREEN,
    fg=WHITE,
    relief="flat",
    command=start_transport
)

start_button.pack(
    fill="both",
    expand=True
)


# ============================================================
# RIGHT SIDE
# ============================================================

right = tk.Frame(
    main,
    bg=BG
)

right.pack(
    side="right",
    fill="both",
    expand=True,
    padx=(4, 0)
)


# ============================================================
# ROBOT STATUS
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
# LOAD
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
# SENSOR STATUS
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
# CURRENT ACTION
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

action_text.pack(
    pady=4
)

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

extra_status.pack(
    pady=(4, 6)
)


# ============================================================
# QR / ROOM IDENTIFICATION
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


qr_panel = create_panel(
    control_frame,
    width=190,
    height=104
)

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

qr_status.pack(
    pady=(1, 3)
)


# ============================================================
# QR CODE DETECTOR
# ============================================================

cap = cv2.VideoCapture(
    CAMERA_INDEX
)

detector = cv2.QRCodeDetector()


def process_qr(data):

    global last_qr

    data = data.strip()

    if not data:
        return

    if data == last_qr:
        return

    last_qr = data

    print(
        "QR DETECTED:",
        data
    )

    # --------------------------------------------------------
    # QR CODES ARE ONLY ACCEPTED AS ROOM IDENTIFIERS
    # --------------------------------------------------------

    if data not in ROOMS:

        qr_status.config(
            text=f"QR: {data}",
            fg=ORANGE
        )

        return

    # --------------------------------------------------------
    # DESTINATION QR
    # --------------------------------------------------------

    destination = (
        destination_var
        .get()
        .strip()
    )

    if data == destination:

        if not transport_started:

            qr_status.config(
                text=f"{data} - WAITING",
                fg=ORANGE
            )

            return

        qr_status.config(
            text=f"DEST: {data}",
            fg=GREEN
        )

        action_text.config(
            text=f"ENTERING {data.upper()}",
            fg=GREEN
        )

        next_marker.config(
            text="DESTINATION REACHED"
        )

        print(
            f"Destination QR detected: {data}"
        )

        print(
            "Entry direction:",
            ROOM_ENTRY_TURN.get(
                data,
                DEFAULT_ENTRY_TURN
            )
        )

        # ----------------------------------------------------
        # THIS IS WHERE THE ACTUAL ROBOT ENTRY FUNCTION
        # SHOULD BE CALLED IN THE FULL PROGRAM.
        #
        # Example:
        #
        # start_room_entry(data)
        # ----------------------------------------------------

        return

    # --------------------------------------------------------
    # PASSING ROOM QR
    # --------------------------------------------------------

    qr_status.config(
        text=f"PASSING: {data}",
        fg=BLUE
    )

    next_marker.config(
        text=f"PASSED {data.upper()}"
    )


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

        root.after(
            100,
            update_camera
        )

        return

    # Mirror camera image.
    frame = cv2.flip(
        frame,
        1
    )

    # --------------------------------------------------------
    # QR DETECTION
    # --------------------------------------------------------

    data, points, _ = (
        detector.detectAndDecode(frame)
    )

    if data:

        process_qr(
            data
        )

        if points is not None:

            points = points.astype(
                int
            )

            for i in range(4):

                pt1 = tuple(
                    points[0][i]
                )

                pt2 = tuple(
                    points[0][
                        (i + 1) % 4
                    ]
                )

                cv2.line(
                    frame,
                    pt1,
                    pt2,
                    (0, 255, 0),
                    4
                )

    # --------------------------------------------------------
    # DISPLAY CAMERA IMAGE
    # --------------------------------------------------------

    frame_rgb = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2RGB
    )

    image = Image.fromarray(
        frame_rgb
    )

    image.thumbnail(
        (170, 62)
    )

    photo = ImageTk.PhotoImage(
        image
    )

    camera_label.config(
        image=photo
    )

    camera_label.image = photo

    root.after(
        30,
        update_camera
    )


# ============================================================
# CONTROL BUTTONS
# ============================================================

tk.Button(
    control_frame,
    text="Ⅱ PAUSE",
    font=("Arial", F_BUTTON, "bold"),
    bg=ORANGE,
    fg=WHITE,
    relief="flat",
    width=12,
    height=2,
    command=pause_transport
).pack(
    side="left",
    padx=4
)

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
).pack(
    side="left",
    padx=4
)

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
).pack(
    side="left",
    padx=4
)


# ============================================================
# INITIALIZE
# ============================================================

refresh_room_menus()

select_room(
    destination_var.get()
)

update_camera()


# ============================================================
# SAFE EXIT
# ============================================================

def exit_program(event=None):

    try:
        cap.release()
    except Exception:
        pass

    root.destroy()


root.bind(
    "<Escape>",
    exit_program
)


# ============================================================
# START UI
# ============================================================

root.mainloop()