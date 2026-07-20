#!/usr/bin/env python3
"""Create throwaway 32x32 placeholder frames so the sketch runs before real art.
Black shapes on white background (matches gen_sprites.py's default LIT=dark)."""
from PIL import Image, ImageDraw
from pathlib import Path

S = 32
art = Path("art"); art.mkdir(exist_ok=True)

def blank():
    img = Image.new("L", (S, S), 255)   # white bg
    return img, ImageDraw.Draw(img)

def body(d, bob=0):
    # head + torso, shifted vertically by `bob`
    d.ellipse([12, 4 + bob, 20, 12 + bob], fill=0)          # head
    d.rectangle([13, 12 + bob, 19, 24 + bob], fill=0)       # torso

def save(img, name): img.save(art / name)

# rest: gentle bob
for i, bob in enumerate((0, 1)):
    img, d = blank(); body(d, bob)
    d.line([13, 24 + bob, 13, 30], fill=0); d.line([19, 24 + bob, 19, 30], fill=0)  # legs
    save(img, f"rest_{i:02d}.png")

# walk: legs alternate
for i, (l, r) in enumerate([((10, 30), (22, 28)), ((22, 30), (10, 28))]):
    img, d = blank(); body(d)
    d.line([16, 24, l[0], l[1]], fill=0); d.line([16, 24, r[0], r[1]], fill=0)
    save(img, f"walk_{i:02d}.png")

# sleep: lying down + Z
for i in range(2):
    img, d = blank()
    d.ellipse([4, 18, 12, 26], fill=0)          # head on side
    d.rectangle([12, 20, 26, 26], fill=0)       # body lying
    z = 6 + i * 2
    d.text((22, 4 + i), "z", fill=0)            # tiny z (blinking)
    save(img, f"sleep_{i:02d}.png")

print("wrote placeholder frames:", sorted(p.name for p in art.glob("*.png")))
