# motion_detect.py — Pico + LIS3DH (raw registers, works on clones)
# Output: "still" vs "moving"
#   still  = sitting, standing, riding in a vehicle, random jostles
#   moving = walking, running, any rhythmic on-your-feet locomotion
#
# How it decides, per ~2.5 s sliding window of accel magnitude:
#   1. Energy gate: if the signal barely varies -> still (couch, smooth bus)
#   2. Rhythm test: normalized autocorrelation peak in the human step band
#      (1.4–4 Hz). Footsteps are periodic -> high peak. Potholes/braking are
#      aperiodic -> low peak, rejected as still even though energy is high.
#
# Everything runs in raw LSB units — the clone's scale factor never matters.
#
# CALIBRATION: set CAL_MODE = True, follow the printed instructions, then
# paste your two thresholds below and set CAL_MODE = False.

from machine import Pin, I2C
import time
import math

# ---------------- user settings ----------------

CAL_MODE = True          # True = print per-window numbers for threshold tuning

# Fill these in after calibration (values below are placeholders that work
# "okay-ish" for a wrist-worn ±8g LIS3DH, but tune your own):
ENERGY_GATE = 4.0e6      # raw-LSB^2 variance; below this -> still
RHYTHM_TH   = 0.5        # autocorr peak 0..1; above this (and above gate) -> moving

# I2C wiring (change pins to match yours; SDO->GND gives addr 0x18, SDO->3V3 0x19)
I2C_ID   = 0
PIN_SDA  = 0
PIN_SCL  = 1
LIS_ADDR = 0x1D

# ---------------- LIS3DH raw-register driver ----------------

REG_WHO_AM_I  = 0x0F
REG_CTRL1     = 0x20
REG_CTRL4     = 0x23
REG_OUT_X_L   = 0x28

i2c = I2C(I2C_ID, sda=Pin(PIN_SDA), scl=Pin(PIN_SCL), freq=400_000)

def wr(reg, val):
    i2c.writeto_mem(LIS_ADDR, reg, bytes([val]))

def rd(reg, n=1):
    # 0x80 = auto-increment bit for multi-byte reads (needed on LIS3DH)
    return i2c.readfrom_mem(LIS_ADDR, reg | (0x80 if n > 1 else 0), n)

def lis3dh_init():
    who = rd(REG_WHO_AM_I)[0]
    print("WHO_AM_I = 0x%02X" % who)   # genuine part says 0x33; clones vary — fine
    # CTRL1 = 0x47: ODR = 50 Hz, normal mode, X/Y/Z enabled
    wr(REG_CTRL1, 0x47)
    # CTRL4 = 0xA0: BDU on (no torn reads), FS = ±8g so running doesn't clip
    wr(REG_CTRL4, 0xA0)
    time.sleep_ms(10)

def read_xyz():
    raw = rd(REG_OUT_X_L, 6)
    x = int.from_bytes(raw[0:2], "little")
    y = int.from_bytes(raw[2:4], "little")
    z = int.from_bytes(raw[4:6], "little")
    # sign-extend 16-bit two's complement
    if x >= 32768: x -= 65536
    if y >= 32768: y -= 65536
    if z >= 32768: z -= 65536
    return x, y, z

# ---------------- classifier ----------------

ODR_HZ   = 50
WIN      = 128            # 2.56 s window
HOP      = 25             # re-classify every 0.5 s
LAG_LO   = ODR_HZ // 4    # 12 -> 4 Hz upper step frequency
LAG_HI   = int(ODR_HZ / 1.4) + 1  # 36 -> 1.4 Hz lower step frequency
SMOOTH_N = 2              # windows in a row required to switch state

buf = [0.0] * WIN         # circular buffer of magnitudes
idx = 0
filled = 0

def push_sample(mag):
    global idx, filled
    buf[idx] = mag
    idx = (idx + 1) % WIN
    if filled < WIN:
        filled += 1

def window_features():
    """Return (variance, best_autocorr_peak) over the current window."""
    n = WIN
    # unroll circular buffer into time order
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
        c = s / energy          # normalized: 1.0 = perfectly periodic at this lag
        if c > best:
            best = c
    return variance, best

def classify(variance, rhythm):
    if variance < ENERGY_GATE:
        return "still"                    # too calm: couch, smooth ride
    return "moving" if rhythm > RHYTHM_TH else "still"  # energetic but aperiodic -> vehicle/jostle

# ---------------- main loop ----------------

def run():
    lis3dh_init()

    if CAL_MODE:
        print("")
        print("=== CALIBRATION MODE ===")
        print("Record ~60 s in each condition, note the numbers printed:")
        print("  1) sitting still   2) walking   3) running   4) riding in a car/bus")
        print("Columns: variance | rhythm_peak | verdict-with-current-thresholds")
        print("Then set ENERGY_GATE between sitting's and walking's variance,")
        print("and RHYTHM_TH between the bus's rhythm peaks and walking's.")
        print("")

    state = "still"
    pending_state = "still"
    pending_count = 0

    period_ms = 1000 // ODR_HZ
    next_t = time.ticks_ms()
    samples_since_hop = 0

    while True:
        # --- paced 50 Hz sampling ---
        now = time.ticks_ms()
        if time.ticks_diff(now, next_t) < 0:
            time.sleep_ms(1)
            continue
        next_t = time.ticks_add(next_t, period_ms)

        x, y, z = read_xyz()
        push_sample(math.sqrt(x * x + y * y + z * z))
        samples_since_hop += 1

        # --- classify every HOP samples once the window is full ---
        if filled == WIN and samples_since_hop >= HOP:
            samples_since_hop = 0
            variance, rhythm = window_features()
            raw = classify(variance, rhythm)

            # hysteresis: need SMOOTH_N consecutive windows to switch
            if raw == state:
                pending_count = 0
            elif raw == pending_state:
                pending_count += 1
                if pending_count >= SMOOTH_N:
                    state = raw
                    pending_count = 0
            else:
                pending_state = raw
                pending_count = 1

            if CAL_MODE:
                print("var=%12.0f   rhythm=%.2f   -> %s (smoothed: %s)"
                      % (variance, rhythm, raw, state))
            else:
                # hook your OLED / character animation on `state` here
                print(state)

run()