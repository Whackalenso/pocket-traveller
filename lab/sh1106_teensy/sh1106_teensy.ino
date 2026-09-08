// sh1106_teensy.ino — SH1106 128x64 OLED bring-up on a Teensy 4.1.
//
// Run this before porting tamagotchi.ino to the Teensy. It answers, in order:
// is the panel wired and powered, is the controller variant right, is the
// 132-vs-128 column offset correct, are any rows/columns dead, and how fast
// can this board actually push a full frame.
//
// WIRING (Teensy 4.1, hardware SPI0)
//   OLED VCC -> 3.3V        (Teensy 4.x is 3.3 V only; do NOT feed it 5 V)
//   OLED GND -> GND
//   OLED CLK -> pin 13      (SCK0, fixed — also the on-board LED, see note)
//   OLED MOSI-> pin 11      (MOSI0, fixed)
//   OLED CS  -> pin 10
//   OLED DC  -> pin 9
//   OLED RES -> pin 8
//
// Only CS/DC/RES are free choices; SCK and MOSI are pinned by the SPI
// peripheral. Teensy 4.1 does offer alternates (SCK on 27, MOSI on 26) —
// uncomment the SPI.setSCK/setMOSI calls in setup() if you route them there.
//
// NOTE ON PIN 13: it drives the orange on-board LED through a buffer. That is
// harmless for SPI, but the LED will flicker with every transfer and you
// cannot use it as a status light while the display is running.
//
// HOW TO READ THE RESULTS — each screen isolates one variable.
//   1 all-on      blank or garbage -> power/wiring/CS/DC/RES, not code.
//                 black stripes    -> dead row/column driver (hardware).
//   2 checker     smeared or shifted -> wrong controller variant (see bottom).
//   3 edges       missing left/right column -> the 132-column offset is wrong;
//                 SH1106 RAM is 132 wide and the panel shows columns 2..129.
//                 Using an SSD1306 constructor on an SH1106 shifts by 2 px and
//                 drops the edge.
//   4 hline/vline dotted in one direction only -> page-addressing problem.
//                 dotted in both -> it is the glass; draw 2-px strokes.
//   5 text        confirms fonts land where you expect at this rotation.
//   6 contrast    finds the usable brightness range for battery work.
//   7 fps         full-frame refresh rate, printed to Serial.
//
// ROTATION: the tamagotchi runs U8G2_R3 (portrait). This test starts in R0 so
// a first-light failure is easy to reason about, then screen 8 repeats the
// line test under R3 to prove that code path separately.

#include <U8g2lib.h>
#include <SPI.h>

// Constructor args are (rotation, CS, DC, RESET).
U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, 10, 9, 8);

static const uint16_t DWELL_MS = 4000;

static void caption(const char* s) {
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(0, 6, s);
}

// 1: every pixel lit. The crudest possible check — if this is not a solid
// bright rectangle, stop and fix the hardware before reading anything else.
// Dead rows or columns show as black stripes with nothing else to hide them.
static void screenAllOn() {
  u8g2.clearBuffer();
  u8g2.drawBox(0, 0, u8g2.getDisplayWidth(), u8g2.getDisplayHeight());
  u8g2.sendBuffer();
}

// 2: 1-px checkerboard. Every pixel toggles against its neighbours, so any
// addressing or column-offset error turns this from a fine grey haze into
// visible bands or diagonal smearing.
static void screenChecker() {
  u8g2.clearBuffer();
  for (u8g2_uint_t y = 0; y < u8g2.getDisplayHeight(); y++)
    for (u8g2_uint_t x = (y & 1); x < u8g2.getDisplayWidth(); x += 2)
      u8g2.drawPixel(x, y);
  u8g2.sendBuffer();
}

// 3: outline plus corner ticks plus both diagonals. The frame must touch all
// four physical edges. A missing left or right column is the classic
// SH1106-vs-SSD1306 offset symptom; a missing top or bottom row is a page
// count or multiplex-ratio mismatch.
static void screenEdges() {
  const u8g2_uint_t w = u8g2.getDisplayWidth();
  const u8g2_uint_t h = u8g2.getDisplayHeight();
  u8g2.clearBuffer();
  u8g2.drawFrame(0, 0, w, h);
  u8g2.drawLine(0, 0, w - 1, h - 1);
  u8g2.drawLine(w - 1, 0, 0, h - 1);
  // Corner ticks: short stubs 2 px in from each corner, so you can tell a
  // clipped edge from a frame that was never drawn.
  u8g2.drawHLine(2, 2, 10);   u8g2.drawHLine(w - 12, 2, 10);
  u8g2.drawHLine(2, h - 3, 10); u8g2.drawHLine(w - 12, h - 3, 10);
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(w / 2 - 20, h / 2 + 3, "EDGES");
  u8g2.sendBuffer();
}

// 4: 1-px strokes in both directions on one screen. Under R0 the horizontal
// lines sit inside a single 8-row page and the vertical ones cross all eight,
// so the two halves exercise opposite paths through the controller.
static void screenLines() {
  const u8g2_uint_t w = u8g2.getDisplayWidth();
  const u8g2_uint_t h = u8g2.getDisplayHeight();
  u8g2.clearBuffer();
  caption("4 hline+vline 1px");
  for (u8g2_uint_t y = 12; y < h; y += 8) u8g2.drawHLine(0, y, w / 2 - 2);
  for (u8g2_uint_t x = w / 2 + 2; x < w; x += 8) u8g2.drawVLine(x, 10, h - 10);
  u8g2.sendBuffer();
}

// 5: a thickness ladder next to real text. If the 1-px rule looks dotted but
// the 2-px one is solid, the panel is simply losing thin strokes and the fix
// is to draw thicker, not to change constructors.
static void screenText() {
  const u8g2_uint_t w = u8g2.getDisplayWidth();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(0, 6, "4x6 the quick brown fox");
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 18, "6x10 Teensy 4.1");
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(0, 32, "SH1106 OK?");
  u8g2.drawHLine(0, 40, w);      // 1 px
  u8g2.drawBox(0, 46, w, 2);     // 2 px
  u8g2.drawBox(0, 54, w, 3);     // 3 px
  u8g2.sendBuffer();
}

// 6: contrast sweep against a fixed pattern. SH1106 contrast is a current
// setting, so it maps almost directly to panel power draw — worth knowing
// before this runs on a battery. Leaves contrast back at the default.
static void screenContrast() {
  static const uint8_t LEVELS[] = {0, 32, 96, 160, 255};
  for (uint8_t i = 0; i < sizeof(LEVELS); i++) {
    u8g2.setContrast(LEVELS[i]);
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    char buf[24];
    snprintf(buf, sizeof(buf), "contrast %u", LEVELS[i]);
    u8g2.drawStr(2, 12, buf);
    u8g2.drawBox(2, 20, 60, 20);
    for (u8g2_uint_t x = 68; x < u8g2.getDisplayWidth() - 2; x += 2)
      u8g2.drawVLine(x, 20, 20);
    u8g2.sendBuffer();
    delay(1200);
  }
  u8g2.setContrast(255);
}

// 7: full-frame throughput. Draws 60 frames of a moving box and reports the
// achieved rate. This is the number that tells you whether sprite animation
// has headroom, and it is where a too-slow bus clock shows up.
static void screenFps() {
  const u8g2_uint_t w = u8g2.getDisplayWidth();
  const u8g2_uint_t h = u8g2.getDisplayHeight();
  const uint16_t FRAMES = 60;
  uint32_t t0 = millis();
  for (uint16_t f = 0; f < FRAMES; f++) {
    u8g2.clearBuffer();
    caption("7 fps");
    u8g2_uint_t x = (uint32_t)f * (w - 16) / FRAMES;
    u8g2.drawBox(x, h / 2 - 8, 16, 16);
    u8g2.drawFrame(0, 0, w, h);
    u8g2.sendBuffer();
  }
  uint32_t dt = millis() - t0;
  float fps = dt ? (1000.0f * FRAMES / dt) : 0.0f;

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  char buf[24];
  snprintf(buf, sizeof(buf), "%u frames", FRAMES);
  u8g2.drawStr(2, 14, buf);
  snprintf(buf, sizeof(buf), "%lu ms", (unsigned long)dt);
  u8g2.drawStr(2, 28, buf);
  snprintf(buf, sizeof(buf), "%d.%01d fps", (int)fps, (int)(fps * 10) % 10);
  u8g2.drawStr(2, 42, buf);
  u8g2.sendBuffer();

  Serial.print("fps: ");
  Serial.print(fps);
  Serial.print("  (");
  Serial.print(dt);
  Serial.print(" ms for ");
  Serial.print(FRAMES);
  Serial.println(" full frames)");
}

// 8: the same 1-px lines under R3, the rotation the tamagotchi actually uses.
// R3 transposes the panel, so a display-space horizontal line becomes a
// controller-space vertical one crossing every page. If screen 4 was clean
// and this is dotted, the bug is in the rotated path, not the panel.
static void screenRotated() {
  u8g2.setDisplayRotation(U8G2_R3);
  const u8g2_uint_t w = u8g2.getDisplayWidth();
  const u8g2_uint_t h = u8g2.getDisplayHeight();
  u8g2.clearBuffer();
  caption("8 R3 lines");
  for (u8g2_uint_t y = 16; y < h; y += 12) u8g2.drawHLine(0, y, w);
  u8g2.drawFrame(0, 0, w, h);
  u8g2.sendBuffer();
  delay(DWELL_MS);
  u8g2.setDisplayRotation(U8G2_R0);
}

void setup() {
  // Alternate SPI0 pins on Teensy 4.1 — uncomment only if you routed them
  // there. Must be called before u8g2.begin().
  // SPI.setSCK(27);
  // SPI.setMOSI(26);

  u8g2.begin();
  u8g2.setBusClock(8000000);   // 8 MHz is safe on dupont wire; the Teensy can
                               // go far faster on a short PCB trace. If the
                               // display shows noise, lower this first.
  u8g2.setDrawColor(1);
  u8g2.setContrast(255);

  Serial.begin(115200);
  delay(2000);                 // let the host attach the port
  // deliberately no while(!Serial): must not hang when running off a battery

  Serial.println("SH1106 test — Teensy 4.1");
  Serial.print("panel ");
  Serial.print(u8g2.getDisplayWidth());
  Serial.print("x");
  Serial.println(u8g2.getDisplayHeight());
  Serial.println("1 all-on  2 checker  3 edges  4 lines  5 text  6 contrast  7 fps  8 R3");
}

void loop() {
  static const struct { const char* name; void (*draw)(); bool self_timed; } SCREENS[] = {
    {"1 all pixels on", screenAllOn,    false},
    {"2 checkerboard",  screenChecker,  false},
    {"3 edges",         screenEdges,    false},
    {"4 1px lines",     screenLines,    false},
    {"5 text ladder",   screenText,     false},
    {"6 contrast",      screenContrast, true},   // sweeps on its own clock
    {"7 fps",           screenFps,      false},
    {"8 R3 lines",      screenRotated,  true},   // manages dwell + rotation
  };
  static const uint8_t COUNT = sizeof(SCREENS) / sizeof(SCREENS[0]);
  static uint8_t i = 0;

  Serial.print("-> ");
  Serial.println(SCREENS[i].name);
  SCREENS[i].draw();
  if (!SCREENS[i].self_timed) delay(DWELL_MS);

  i = (i + 1) % COUNT;
}

// ---------------------------------------------------------------------------
// NOTHING ON SCREEN AT ALL — work down this list before touching the code:
//   * 3.3 V and GND actually reaching the module (measure at the module, not
//     at the Teensy header).
//   * RES held low permanently, or left floating on a module with no pull-up.
//   * DC and CS swapped — the two most commonly transposed wires here.
//   * A module wired for I2C. Many SH1106 boards ship jumpered for I2C and
//     need resistors moved to enable 4-wire SPI. If yours is I2C, use:
//       U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);
//     with SDA->18, SCL->19 on the Teensy 4.1, and drop the SPI include.
//
// GARBLED OR SHIFTED IMAGE — wrong controller variant. SH1106 RAM is 132
// columns wide against a 128-column panel, and modules disagree about which
// two columns to hide and how VCOMH is set. Swap the constructor and re-run:
//
//   U8G2_SH1106_128X64_VCOMH0_F_4W_HW_SPI  u8g2(U8G2_R0, 10, 9, 8);
//   U8G2_SH1106_128X64_WINSTAR_F_4W_HW_SPI u8g2(U8G2_R0, 10, 9, 8);
//   U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, 10, 9, 8);
//
// That last one is not a typo: modules sold as SH1106 are sometimes SSD1306.
// If the SSD1306 constructor is the one that looks right, believe it.
// ---------------------------------------------------------------------------
