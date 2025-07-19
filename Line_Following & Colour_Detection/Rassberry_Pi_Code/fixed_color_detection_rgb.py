import cv2
import numpy as np
import serial
import time
import threading
from PIL import Image, ImageTk, ImageDraw
import tkinter as tk
import serial.tools.list_ports
from picamera2 import Picamera2

# --- Check if full-frame color is detected ---
def is_color_full_frame(mask, threshold=1.0):
    filled = cv2.countNonZero(mask)
    total = mask.shape[0] * mask.shape[1]
    return (filled / total) >= threshold

# --- HSV Ranges ---
color_ranges = {
    "Red": [
        (np.array([0, 40, 20]), np.array([12, 255, 255])),
        (np.array([165, 40, 20]), np.array([180, 255, 255]))
    ],
    "Green": [
        (np.array([30, 30, 30]), np.array([90, 255, 255]))
    ],
    "Blue": [
        (np.array([90, 50, 30]), np.array([140, 255, 255]))
    ]
}

# --- Arduino Serial Connection ---
arduino = None
for p in serial.tools.list_ports.comports():
    if 'Arduino' in p.description or 'ttyACM' in p.device or 'ttyUSB' in p.device:
        try:
            arduino = serial.Serial(p.device, 9600, timeout=1)
            time.sleep(2)
            break
        except serial.SerialException:
            continue

if not arduino:
    print("Arduino not found.")
    exit(1)

# --- GUI Setup ---
root = tk.Tk()
root.title("Color Detection GUI")
root.attributes("-fullscreen", True)

screen_width = root.winfo_screenwidth()
screen_height = root.winfo_screenheight()

bg_image_path = "/home/pi/background.jpg"  # replace with your image path
bg_image_raw = Image.open(bg_image_path).resize((screen_width, screen_height), Image.LANCZOS)
bg_photo = ImageTk.PhotoImage(bg_image_raw)

canvas = tk.Canvas(root, width=screen_width, height=screen_height)
canvas.pack(fill="both", expand=True)
canvas.create_image(0, 0, image=bg_photo, anchor="nw")

# --- Transparent rounded box ---
box_width, box_height = 520, 460
box_x = (screen_width - box_width) // 2
box_y = 80
rounded_box = Image.new("RGBA", (box_width, box_height), (0, 0, 0, 0))
draw = ImageDraw.Draw(rounded_box)
draw.rounded_rectangle((0, 0, box_width, box_height), radius=40, fill=(50, 50, 50, 180))
rounded_box_imgtk = ImageTk.PhotoImage(rounded_box)
canvas.create_image(box_x, box_y, image=rounded_box_imgtk, anchor="nw")

# --- Color logic ---
color_hex = {
    "Red": "#FF5555",
    "Green": "#55FF55",
    "Blue": "#5555FF"
}
circle_radius = 30
gap = 100
start_x = box_x + (box_width - 2 * gap) // 2
first_row_label_y = box_y + 50
first_row_circles_y = box_y + 130
second_row_label_y = box_y + 240
second_row_circles_y = box_y + 320

selected_color1 = tk.StringVar(value="")
selected_color2 = tk.StringVar(value="")
color_circles_1 = {}
color_circles_2 = {}

def draw_circles(y, selected_var, circles_dict, other_selected_var):
    for i, color in enumerate(["Red", "Green", "Blue"]):
        x = start_x + i * gap
        fill = color_hex[color] if selected_var.get() == color else "#333333"
        outline = color_hex[color] if selected_var.get() == color else "#777777"
        circle = canvas.create_oval(x - circle_radius, y - circle_radius, x + circle_radius, y + circle_radius,
                                    fill=fill, outline=outline, width=3)
        label = canvas.create_text(x, y + circle_radius + 20, text=color, fill="white", font=("Helvetica", 12))

        def handler(event, c=color):
            if c == other_selected_var.get():
                status_label.config(text="Cannot select same color twice!", fg="red")
                return
            selected_var.set(c)
            update_circles()
            check_enable_start()
            status_label.config(text="Select First and Second Color", fg="white")

        canvas.tag_bind(circle, "<Button-1>", handler)
        canvas.tag_bind(label, "<Button-1>", handler)
        circles_dict[color] = circle

def update_circles():
    for color in ["Red", "Green", "Blue"]:
        fill1 = color_hex[color] if selected_color1.get() == color else "#333333"
        outline1 = color_hex[color] if selected_color1.get() == color else "#777777"
        canvas.itemconfig(color_circles_1[color], fill=fill1, outline=outline1)

        fill2 = color_hex[color] if selected_color2.get() == color else "#333333"
        outline2 = color_hex[color] if selected_color2.get() == color else "#777777"
        canvas.itemconfig(color_circles_2[color], fill=fill2, outline=outline2)

def check_enable_start():
    if selected_color1.get() and selected_color2.get():
        start_button["state"] = "normal"
        status_label.config(text=f"Selected: {selected_color1.get()} then {selected_color2.get()}", fg="white")
    else:
        start_button["state"] = "disabled"

canvas.create_text(screen_width // 2, first_row_label_y, text="Select First Color", font=("Helvetica", 16), fill="white")
draw_circles(first_row_circles_y, selected_color1, color_circles_1, selected_color2)

canvas.create_text(screen_width // 2, second_row_label_y, text="Select Second Color", font=("Helvetica", 16), fill="white")
draw_circles(second_row_circles_y, selected_color2, color_circles_2, selected_color1)

start_button = tk.Button(root, text="Start Detection", font=("Helvetica", 14), state="disabled", command=lambda: start_color_detection())
canvas.create_window(screen_width // 2, box_y + box_height - 60, window=start_button)

status_label = tk.Label(root, text="Select First and Second Color", fg="white", font=("Helvetica", 14), bg="#323232")
canvas.create_window(screen_width // 2, box_y + box_height - 25, window=status_label)

# --- Serial log and camera preview ---
log_text_widget = tk.Text(root, height=20, width=70, bg="#222222", fg="white", font=("Consolas", 10))
canvas.create_window(20, box_y + box_height + 10, anchor="nw", window=log_text_widget)

frame_x = screen_width - 700
frame_y = box_y + box_height + 10
cam_width = 480
cam_height = 270

black_box_img = Image.new("RGBA", (cam_width, cam_height), (0, 0, 0, 255))
black_box_photo = ImageTk.PhotoImage(black_box_img)
canvas.create_image(frame_x, frame_y, image=black_box_photo, anchor="nw")

frame_label = tk.Label(root)
canvas.create_window(frame_x, frame_y, anchor="nw", window=frame_label)

stop_flag = threading.Event()
last_sent_color = None
last_send_time = time.time()
resend_interval = 0.5

def log_message(msg):
    log_text_widget.insert(tk.END, msg + "\n")
    log_text_widget.see(tk.END)
    print(msg)

def run_camera_detection():
    global last_sent_color, last_send_time

    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (cam_width, cam_height)})
    picam2.configure(config)
    picam2.set_controls({
        "ExposureTime": 35000,
        "AnalogueGain": 11.0,
        "AwbEnable": False,
        "ColourGains": (1.8, 1.5)
    })
    picam2.start()
    time.sleep(2)

    while not stop_flag.is_set():
        frame = picam2.capture_array()
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

        detected = None
        for color in color_ranges:
            for lower, upper in color_ranges[color]:
                mask = cv2.inRange(hsv, lower, upper)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5),np.uint8))
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5),np.uint8))
                if is_color_full_frame(mask):
                    detected = color
                    break
            if detected:
                break

        now = time.time()
        if detected:
            if detected != last_sent_color or (now - last_send_time > resend_interval):
                arduino.write(f"D:{detected[0]}\n".encode())
                log_message(f"FULL SCREEN COLOR DETECTED: {detected}")
                last_sent_color = detected
                last_send_time = now
        else:
            if last_sent_color is not None:
                arduino.write(b"D:-\n")
                log_message("No color detected")
                last_sent_color = None
                last_send_time = now

        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = ImageTk.PhotoImage(Image.fromarray(img))
        def update_frame():
            frame_label.imgtk = img
            frame_label.configure(image=img)
        root.after(10, update_frame)

    picam2.stop()
    cv2.destroyAllWindows()

def start_color_detection():
    x = selected_color1.get()
    y = selected_color2.get()
    if x and y:
        log_message(f"Selected Colors: {x}, {y}")
        arduino.write(f"X:{x[0]}\n".encode())
        arduino.write(f"Y:{y[0]}\n".encode())
        time.sleep(0.5)
        arduino.write(b"S:1\n")
        status_label.config(text="Running Detection...", fg="blue")
        threading.Thread(target=run_camera_detection, daemon=True).start()

def on_close(event=None):
    stop_flag.set()
    time.sleep(1)
    if arduino and arduino.is_open:
        arduino.close()
    root.destroy()

root.bind("<Escape>", on_close)
root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()
