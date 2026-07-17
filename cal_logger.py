# cal_logger.py — untethered calibration data logger (Pico + ADXL345)
#
# Save this to the Pico as **main.py** so it runs on battery power.
# Each power-up creates a new file: cal_1.csv, cal_2.csv, ...
# so the procedure is: plug in battery -> do ONE activity 60-90 s -> unplug.
# Keep a note on your phone of which session number = which activity.
#
# The first TRIM_S seconds are discarded (you fumbling with the battery).
# Onboard LED: solid during warm-up/trim, blinking once per window while
# actually recording, so you know it's alive.
#
# Each CSV row: seconds_since_start, variance, rhythm

from machine import Pin, I2C
import time
import math
import os

# ---------------- settings (identical signal chain to motion_detect.py) ----

TRIM_S    = 10           # seconds to discard at start of each session
I2C_ID    = 0            # GP0/GP1 = I2C0
PIN_SDA   = 0
PIN_SCL   = 1
ADXL_ADDR = 0x1D

ODR_HZ = 50
WIN    = 128             # 2.56 s window
HOP    = 25              # one logged row every 0.5 s
LAG_LO = ODR_HZ // 4
LAG_HI = int(ODR_HZ / 1.4) + 1

# ---------------- LED (Pico: GP25; Pico W: "LED") ----------------

try:
    led = Pin("LED", Pin.OUT)
except Exception:
    led = Pin(25, Pin.OUT)

# ---------------- ADXL345 driver (same as motion_detect.py) ----------------

REG_DEVID       = 0x00
REG_BW_RATE     = 0x2C
REG_POWER_CTL   = 0x2D
REG_DATA_FORMAT = 0x31
REG_DATAX0      = 0x32

i2c = I2C(I2C_ID, sda=Pin(PIN_SDA), scl=Pin(PIN_SCL), freq=400_000)

def wr(reg, val):
    i2c.writeto(ADXL_ADDR, bytes([reg, val]))

def rd(reg, n=1):
    i2c.writeto(ADXL_ADDR, bytes([reg]))
    return i2c.readfrom(ADXL_ADDR, n)

def adxl_init():
    wr(REG_BW_RATE, 0x0A)       # 100 Hz ODR
    wr(REG_DATA_FORMAT, 0x0B)   # FULL_RES, +/-16g
    wr(REG_POWER_CTL, 0x08)     # measurement mode
    time.sleep_ms(20)

def read_xyz():
    raw = rd(REG_DATAX0, 6)
    x = int.from_bytes(raw[0:2], "little")
    y = int.from_bytes(raw[2:4], "little")
    z = int.from_bytes(raw[4:6], "little")
    if x >= 32768: x -= 65536
    if y >= 32768: y -= 65536
    if z >= 32768: z -= 65536
    return x, y, z

# ---------------- feature computation (same math) ----------------

buf = [0.0] * WIN
idx = 0
filled = 0

def push_sample(mag):
    global idx, filled
    buf[idx] = mag
    idx = (idx + 1) % WIN
    if filled < WIN:
        filled += 1

def window_features():
    n = WIN
    d = [buf[(idx + i) % WIN] for i in range(n)]
    mean = sum(d) / n
    for i in range(n):
        d[i] -= mean
    energy = 0.0
    for v in d:
        energy += v * v
    variance = energy / n
    if energy <= 0:
        return 0.0, 0.0
    best = 0.0
    for lag in range(LAG_LO, LAG_HI + 1):
        s = 0.0
        for i in range(n - lag):
            s += d[i] * d[i + lag]
        c = s / energy
        if c > best:
            best = c
    return variance, best

# ---------------- session file management ----------------

def next_filename():
    n = 1
    existing = os.listdir()
    while ("cal_%d.csv" % n) in existing:
        n += 1
    return "cal_%d.csv" % n

# ---------------- main ----------------

def run():
    adxl_init()

    fname = next_filename()
    led.on()                        # solid = warming up / trimming

    period_ms = 1000 // ODR_HZ
    next_t = time.ticks_ms()
    t0 = time.ticks_ms()
    samples_since_hop = 0
    recording = False
    rows = 0

    f = open(fname, "w")
    f.write("t,variance,rhythm\n")

    while True:
        now = time.ticks_ms()
        if time.ticks_diff(now, next_t) < 0:
            time.sleep_ms(1)
            continue
        next_t = time.ticks_add(next_t, period_ms)

        x, y, z = read_xyz()
        push_sample(math.sqrt(x * x + y * y + z * z))
        samples_since_hop += 1

        elapsed_s = time.ticks_diff(time.ticks_ms(), t0) / 1000.0

        if not recording and elapsed_s >= TRIM_S and filled == WIN:
            recording = True
            led.off()

        if recording and samples_since_hop >= HOP:
            samples_since_hop = 0
            variance, rhythm = window_features()
            f.write("%.1f,%.1f,%.3f\n" % (elapsed_s, variance, rhythm))
            rows += 1
            if rows % 4 == 0:       # flush every ~2 s so unplugging loses little
                f.flush()
            led.toggle()            # blink = recording

run()
