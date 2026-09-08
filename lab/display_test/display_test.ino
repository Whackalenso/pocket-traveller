// display_test.ino — is the OLED dropping pixels, or is it just the glass?
//
// SYMPTOM being chased: the 1-px border of the XP progress bar in
// tamagotchi.ino looks broken up, while 4x6 text on the same screen reads
// solid. Text is a poor control (letterforms hide dropouts that a long
// straight line makes obvious), so these screens isolate one variable each.
//
// WHY ROTATION MATTERS HERE
//   The main sketch runs U8G2_R3, which transposes the panel: display-x maps
//   to panel-y, display-y maps to panel-x. So a horizontal line in display
//   space is, in controller memory, a vertical line crossing all eight 8-row
//   pages — exactly where a page-addressing bug would surface. A vertical
//   line in display space stays inside one page. That asymmetry is the whole
//   diagnostic: screen A and screen C draw the same 1-px stroke through
//   completely different parts of the controller.
//
// WIRING: identical to tamagotchi.ino.
//   CLK->GP2  MOSI->GP3  RES->GP4  DC->GP5  CS->GP6
//   (constructor args are rotation, CS, DC, RESET)
//
// HOW TO READ THE RESULT
//   A has gaps, C is clean            -> page addressing. Try the alternate
//                                        constructors at the bottom of this
//                                        file; one of them likely matches
//                                        your module.
//   A and C both dotted, B's 2-px and
//   3-px strokes look solid           -> the glass. Nothing to fix in code;
//                                        draw the bar border 2 px thick.
//   Gaps stay at the same screen
//   position as the line moves (D)    -> dead row/column driver. Hardware.
//   F shows missing lines/columns     -> confirms dead drivers independently.
//   A is broken but E is clean        -> the R3 code path specifically.

#include <U8g2lib.h>
#include <SPI.h>

U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R3, 8, 7, 6);

static const uint16_t DWELL_MS = 5000;

static void caption(const char* s) {
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(0, 6, s);
}

// One bar drawn exactly like the real one in tamagotchi.ino: 1-px frame,
// half-filled, full display width.
static void testBar(u8g2_uint_t y) {
  const u8g2_uint_t w = u8g2.getDisplayWidth();
  u8g2.drawFrame(0, y, w, 5);
  u8g2.drawBox(1, y + 1, (w - 2) / 2, 3);
}

// A: 1-px horizontal lines, full width, spread down the screen.
// In R3 these are the page-crossing direction — the suspect case.
static void screenA() {
  u8g2.clearBuffer();
  caption("A hline 1px");
  const u8g2_uint_t w = u8g2.getDisplayWidth();
  for (u8g2_uint_t y = 16; y < u8g2.getDisplayHeight(); y += 12)
    u8g2.drawHLine(0, y, w);
  u8g2.sendBuffer();
}

// B: stroke-thickness ladder — 1, 2, 3 px, then a solid block.
// If 1 px is dotted and 2 px is solid, the fix is thickness, not addressing.
static void screenB() {
  u8g2.clearBuffer();
  caption("B 1/2/3/solid");
  const u8g2_uint_t w = u8g2.getDisplayWidth();
  u8g2.drawHLine(0, 20, w);        // 1 px
  u8g2.drawBox(0, 32, w, 2);       // 2 px
  u8g2.drawBox(0, 44, w, 3);       // 3 px
  u8g2.drawBox(0, 56, w, 8);       // solid
  u8g2.sendBuffer();
}

// C: 1-px vertical lines. In R3 these live inside a single page, so they
// exercise the opposite path from screen A. Same stroke width, though — so
// if the glass is the problem, these look just as dotted as A.
static void screenC() {
  u8g2.clearBuffer();
  caption("C vline 1px");
  const u8g2_uint_t h = u8g2.getDisplayHeight();
  for (u8g2_uint_t x = 4; x < u8g2.getDisplayWidth(); x += 8)
    u8g2.drawVLine(x, 16, h - 16);
  u8g2.sendBuffer();
}

// D: the actual XP bar at three heights. If the gaps sit at the same
// left-to-right positions in all three, the defect is tied to the screen,
// not to the drawing.
static void screenD() {
  u8g2.clearBuffer();
  caption("D bar x3");
  testBar(16);
  testBar(60);
  testBar(104);
  u8g2.sendBuffer();
}

// E: the same physical lines, reached through R0 instead of R3. Because the
// panel is transposed under R3, the R0 equivalent of screen A is a set of
// VERTICAL lines. If the same physical dots go missing in both, the rotation
// code is innocent.
static void screenE() {
  u8g2.setDisplayRotation(U8G2_R0);
  u8g2.clearBuffer();
  caption("E R0 vline 1px");
  const u8g2_uint_t h = u8g2.getDisplayHeight();
  for (u8g2_uint_t x = 8; x < u8g2.getDisplayWidth(); x += 12)
    u8g2.drawVLine(x, 12, h - 12);
  u8g2.sendBuffer();
  delay(DWELL_MS);
  u8g2.setDisplayRotation(U8G2_R3);
}

// F: every pixel on. Any dead row, column, or page shows up as a black
// stripe with nothing else on screen to distract from it.
static void screenF() {
  u8g2.clearBuffer();
  u8g2.drawBox(0, 0, u8g2.getDisplayWidth(), u8g2.getDisplayHeight());
  u8g2.sendBuffer();
}

void setup() {
  SPI.setSCK(2);
  SPI.setTX(3);
  u8g2.begin();
  u8g2.setDrawColor(1);

  Serial.begin(115200);
  delay(2000);                      // give the host time to attach the port
  // no while(!Serial): must not hang on battery power

  Serial.print("display ");
  Serial.print(u8g2.getDisplayWidth());
  Serial.print("x");
  Serial.println(u8g2.getDisplayHeight());
  Serial.println("A hline  B thickness  C vline  D bar  E rot0  F all-on");
}

void loop() {
  static const struct { const char* name; void (*draw)(); } SCREENS[] = {
    {"A hline 1px",   screenA},
    {"B thickness",   screenB},
    {"C vline 1px",   screenC},
    {"D bar x3",      screenD},
    {"E R0 vline",    screenE},     // manages its own dwell + rotation reset
    {"F all pixels",  screenF},
  };
  static const uint8_t COUNT = sizeof(SCREENS) / sizeof(SCREENS[0]);
  static uint8_t i = 0;

  Serial.print("-> ");
  Serial.println(SCREENS[i].name);
  SCREENS[i].draw();
  if (SCREENS[i].draw != screenE) delay(DWELL_MS);

  i = (i + 1) % COUNT;
}

// ---------------------------------------------------------------------------
// IF SCREEN A IS BROKEN AND C IS CLEAN, the controller variant is wrong.
// Swap the constructor above for one of these and re-run — SH1106 panels are
// 132 columns wide internally and the variants differ in how those extra two
// columns and the VCOMH setting are handled. A module sold as "SH1106" is
// also sometimes an SSD1306, which is why the last one is listed.
//
//   U8G2_SH1106_128X64_VCOMH0_F_4W_HW_SPI  u8g2(U8G2_R3, 8, 7, 6);
//   U8G2_SH1106_128X64_WINSTAR_F_4W_HW_SPI u8g2(U8G2_R3, 8, 7, 6);
//   U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R3, 8, 7, 6);
// ---------------------------------------------------------------------------
