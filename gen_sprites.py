#!/usr/bin/env python3
"""
gen_sprites.py  -  Batch-convert the per-rank folders of monochrome PNGs into a
                   single sprites.h for U8g2 (drawXBMP / XBM = LSB-first).

Usage:
    python gen_sprites.py art/ sprites.h

Layout of the art directory -- one folder per rank, named for the rank:
    art/chud/walk_0.png  art/chud/rest_0.png  ...
    art/pleb/...
    art/nomad/...
Folders that are not named in RANKS below are ignored, so art/original/ can sit
there as reference without ending up in the build.

Naming convention inside a rank folder:
    <state>_<frameNumber>.png     e.g.  walk_0.png, rest_10.png
Files are grouped by <state>, ordered by <frameNumber>, and emitted as:
    - one const uint8_t[] per frame
    - one const uint8_t* const <rank><State>Frames[] table
    - one Animation ANIM_<RANK>_<STATE> struct
plus a RANK_SPRITES[] table so the sketch can index the whole set by rank.

Sprite dimensions are per-animation and are allowed to differ between ranks and
between states -- the sketch places sprites by their bottom edge, so a taller
pet grows upward rather than sinking through the floor.

sleep_*.png is optional. SPRITES_HAVE_SLEEP is emitted as 1 only when EVERY
rank has sleep art; that define is what turns the idle-sleep animation back on
in the sketch.

Encoding: a source pixel is treated as LIT (draws in the OLED draw color) when
it is DARK. So design your characters as BLACK shapes on a WHITE background.
Set INVERT = True if you draw the other way round.
"""
import sys, re
from pathlib import Path
from collections import defaultdict
from PIL import Image

# Order matters: it is the value order of `enum Rank` in the sketch, and the
# sketch static_asserts that the two agree in length.
RANKS = ["chud", "pleb", "nomad"]

REQUIRED_STATES = ["rest", "walk"]      # a rank without these is a build error
OPTIONAL_STATES = ["sleep"]             # emitted as nullptr when missing
STATES = REQUIRED_STATES + OPTIONAL_STATES

INVERT = False        # flip if your character is light-on-dark
FRAME_MS = 300        # default per-frame duration; tweak per animation later
THRESHOLD = 128       # pixels darker than this are "lit"


def to_xbm(img):
    """Return (w, h, bytes) packed as XBM: LSB = leftmost pixel, rows byte-padded."""
    # RGBA art is usually black-on-transparent. convert("L") drops alpha and
    # leaves transparent pixels as black, so the whole sprite becomes lit.
    # Composite onto white first so transparent == background.
    if img.mode in ("RGBA", "LA") or (img.mode == "P" and "transparency" in img.info):
        rgba = img.convert("RGBA")
        bg = Image.new("RGBA", rgba.size, (255, 255, 255, 255))
        img = Image.alpha_composite(bg, rgba)
    img = img.convert("L")
    w, h = img.size
    row_bytes = (w + 7) // 8
    out = bytearray()
    for y in range(h):
        for xb in range(row_bytes):
            b = 0
            for bit in range(8):            # bit 0 is the LEFTMOST pixel (XBM/LSB-first)
                x = xb * 8 + bit
                if x < w:
                    lit = img.getpixel((x, y)) < THRESHOLD
                    if INVERT:
                        lit = not lit
                    if lit:
                        b |= (1 << bit)
            out.append(b)
    return w, h, out


def collect(rank_dir):
    """{state: [Path, ...]} for one rank folder, frames in numeric order."""
    groups = defaultdict(list)
    for p in sorted(rank_dir.glob("*.png")):
        m = re.match(r"([A-Za-z]+)_(\d+)$", p.stem)
        if not m:
            print(f"  skip (bad name): {p.name}")
            continue
        state = m.group(1).lower()
        if state not in STATES:
            print(f"  skip (unknown state '{state}'): {p.name}")
            continue
        groups[state].append((int(m.group(2)), p))
    return {s: [p for _, p in sorted(v)] for s, v in groups.items()}


def emit_animation(lines, rank, state, paths):
    """Append the frame blobs, the frame table and the Animation for one state."""
    names = []
    w = h = 0
    for idx, path in enumerate(paths):
        fw, fh, data = to_xbm(Image.open(path))
        if names and (fw, fh) != (w, h):
            # Frames of one animation share a single w/h in the Animation
            # struct, so a mismatch inside a state would silently misread the
            # blob. Between states and between ranks it is fine.
            sys.exit(f"{rank}/{state}: frame {path.name} is {fw}x{fh}, "
                     f"expected {w}x{h} -- all frames of one state must match")
        w, h = fw, fh
        name = f"{rank}_{state}{idx}_xbm"
        names.append(name)
        lines.append(f"static const uint8_t {name}[] = {{{','.join(str(b) for b in data)}}};")

    table = f"{rank}_{state}Frames"
    anim = f"ANIM_{rank.upper()}_{state.upper()}"
    lines.append(f"static const uint8_t* const {table}[] = {{{','.join(names)}}};")
    lines.append(f"static const Animation {anim} = "
                 f"{{{table}, {len(names)}, {w}, {h}, {FRAME_MS}, true}};")
    lines.append("")
    print(f"    {state}: {len(names)} frame(s), {w}x{h}, {((w + 7) // 8) * h} bytes/frame")
    return anim


def main():
    art_dir = Path(sys.argv[1] if len(sys.argv) > 1 else "art")
    out_path = Path(sys.argv[2] if len(sys.argv) > 2 else "sprites.h")

    per_rank = {}
    for rank in RANKS:
        d = art_dir / rank
        if not d.is_dir():
            sys.exit(f"missing rank folder: {d}")
        found = collect(d)
        for state in REQUIRED_STATES:
            if not found.get(state):
                sys.exit(f"{d}: no {state}_*.png")
        per_rank[rank] = found

    have_sleep = all(per_rank[r].get("sleep") for r in RANKS)

    lines = [
        "// Auto-generated by gen_sprites.py -- do not edit by hand.",
        "// Format: XBM (LSB-first), for U8g2 drawXBMP().",
        "#pragma once",
        "#include <Arduino.h>",
        "",
        "struct Animation {",
        "  const uint8_t* const* frames;",
        "  uint8_t  frameCount;",
        "  uint8_t  w, h;",
        "  uint16_t frameMs;",
        "  bool     loop;",
        "};",
        "",
        # Keep Activity in this header so Arduino's auto-prototypes see it.
        "enum Activity { REST, WALK, SLEEP };",
        "",
        "// Every animation a single rank owns. Sizes are per-animation, so ranks",
        "// (and the states inside one rank) may differ in w/h.",
        "struct RankSprites {",
        "  const Animation* rest;",
        "  const Animation* walk;",
        "  const Animation* sleep;   // nullptr when this rank has no sleep art",
        "};",
        "",
        "// 1 only when every rank has sleep_*.png. The sketch reads this to decide",
        "// whether the idle-sleep animation exists at all -- drop sleep frames into",
        "// each art/<rank>/ folder, re-run gen_sprites.py, and the feature is back.",
        f"#define SPRITES_HAVE_SLEEP {1 if have_sleep else 0}",
        "",
    ]

    entries = []
    for rank in RANKS:
        print(f"  {rank}:")
        lines.append(f"// ---- {rank} " + "-" * (60 - len(rank)))
        anims = {}
        for state in STATES:
            paths = per_rank[rank].get(state)
            if paths:
                anims[state] = emit_animation(lines, rank, state, paths)
        entries.append("  {{&{}, &{}, {}}},   // {}".format(
            anims["rest"], anims["walk"],
            f"&{anims['sleep']}" if "sleep" in anims else "nullptr",
            rank))

    lines.append("// Indexed by `enum Rank` -- keep this order in step with the sketch,")
    lines.append("// which static_asserts on RANK_SPRITE_COUNT. Order: "
                 + ", ".join(RANKS) + ".")
    lines.append("static const RankSprites RANK_SPRITES[] = {")
    lines.extend(entries)
    lines.append("};")
    lines.append(f"static const uint8_t RANK_SPRITE_COUNT = {len(RANKS)};")
    lines.append("")

    out_path.write_text("\n".join(lines) + "\n")
    print(f"Wrote {out_path} (sleep art: {'yes' if have_sleep else 'no'})")


if __name__ == "__main__":
    main()
