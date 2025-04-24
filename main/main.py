import serial
import time
from collections import deque
from pynput.mouse import Controller, Button

PORT           = "/dev/ttyACM0"  
BAUDRATE       = 115200
SENSITIVITY    = 1.0             
DEAD_ZONE      = 2.0              
SMOOTH_SIZE    = 10               
CLICK_COOLDOWN = 0.5              
MAX_STEP       = 20               

def parse_line(line: str):
    parts = line.strip().split(',')
    if len(parts) != 4:
        return None
    try:
        r = float(parts[0])
        p = float(parts[1])
        y = float(parts[2])
        c = int(parts[3])
        return r, p, y, c
    except ValueError:
        return None

def run_pointer():
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    mouse = Controller()

    buf_x = deque(maxlen=SMOOTH_SIZE)
    buf_y = deque(maxlen=SMOOTH_SIZE)

    last_click = False
    last_click_time = 0.0

    print(f"[+] Conectado em {PORT} @ {BAUDRATE}bps")
    time.sleep(0.5)

    while True:
        raw = ser.readline().decode('ascii', errors='ignore')
        data = parse_line(raw)
        if data is None:
            continue

        roll, pitch, yaw, click_flag = data

        if abs(roll) <= DEAD_ZONE:
            roll = 0.0
        if abs(pitch) <= DEAD_ZONE:
            pitch = 0.0

        buf_x.append(roll)
        buf_y.append(pitch)
        avg_x = sum(buf_x) / len(buf_x)
        avg_y = sum(buf_y) / len(buf_y)

        dx = max(min(avg_x * SENSITIVITY, MAX_STEP), -MAX_STEP)
        dy = max(min(avg_y * SENSITIVITY, MAX_STEP), -MAX_STEP)

        if dx or dy:
            try:
                mouse.move(dx, -dy)
            except Exception:
                pass

        now = time.time()
        if click_flag == 1 and not last_click and (now - last_click_time) > CLICK_COOLDOWN:
            mouse.click(Button.left, 1)
            last_click_time = now
        last_click = (click_flag == 1)

if __name__ == "__main__":
    run_pointer()
