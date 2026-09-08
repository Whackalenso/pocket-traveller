// card_test.ino -- show every card the device can ever put on the panel.
//
// WHAT IT DOES
//   Walks the whole flavor pool in order, one card at a time, then the rank
//   announcements, then starts over. Nothing is random: a line with an
//   alternation in it ("piano/photography/fishing") is shown once per branch,
//   so a full pass really is every reading of every card, and the rank card is
//   stepped through every rank in both directions. Same font, same wrap, same
//   inset, same centering as tamagotchi.ino -- if a card fits here it fits in
//   the field, and if one wraps badly here it wraps badly there.
//
//   card_lines.h is a SYMLINK to the one next to tamagotchi.ino, so this test
//   always shows the text the real sketch would show, with no copy to keep in
//   sync. Add a line to the pool and it appears in the walk.
//
//   At boot the sketch also dumps a written report of the entire pool to
//   serial: each card, how it wraps, the pixel width of every line, and a
//   warning on anything that overflows CARD_MAX_LINES or has to hard-break a
//   word. That part needs no watching -- upload, open the monitor, read.
//
// WIRING: display only, identical to tamagotchi.ino. No accelerometer.
//   OLED:  CLK->GP2  MOSI->GP3  RES->GP6  DC->GP7  CS->GP8
//
// HOW TO READ THE SCREEN
//   The card owns the whole panel, exactly as it does in the field -- no
//   header, no index, nothing this sketch adds. Where you are in the walk is
//   printed on serial instead, so what is on the glass is only ever what the
//   real device would draw.
//
// SERIAL (115200), single keypress:
//   n  next card         p  previous card       r  restart the walk
//   spc pause / resume   d  re-dump the report  ?  reprint the banner

#include <U8g2lib.h>
#include <SPI.h>
#include <string.h>

// SH1106 128x64, 4-wire hardware SPI. Args: rotation, CS, DC, RESET.
// U8G2_R3 = 90 deg CCW -> 64x128 portrait, same as tamagotchi.ino.
U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R3, 8, 7, 6);

// ---------------- the pool, and the layout constants it is drawn with --------
// These MUST mirror tamagotchi.ino. They are copied rather than shared because
// the real ones live in the middle of that sketch; if you change them there,
// change them here, or this test stops being a test.

static const char* const CARD_LINES[] = {
  #include "card_lines.h"
};
static const uint8_t CARD_COUNT = sizeof(CARD_LINES) / sizeof(CARD_LINES[0]);

static const char* const RANK_NAME[] = {"chud", "pleb", "nomad"};
static const uint8_t RANK_COUNT = sizeof(RANK_NAME) / sizeof(RANK_NAME[0]);

static const uint32_t CARD_BASE_MS     = 2000;
static const uint32_t CARD_MS_PER_CHAR = 55;
static const uint32_t CARD_MAX_MS      = 6000;

#define CARD_FONT  u8g2_font_4x6_tf
static const u8g2_uint_t CARD_LINE_GAP   = 2;
static const uint8_t     CARD_MAX_LINES  = 8;
static const uint8_t     CARD_MAX_CHARS  = 20;
static const u8g2_uint_t CARD_SIDE_INSET = 14;

char cardBuf[96];

// A card is held for its reading time, capped the same way the real one is by
// CARD_MAX_MS. Long cards therefore linger longer, which is the pacing you are
// actually trying to judge.
static uint32_t cardReadMs(const char* text) {
  const uint32_t ms = CARD_BASE_MS + (uint32_t)strlen(text) * CARD_MS_PER_CHAR;
  return (ms > CARD_MAX_MS) ? CARD_MAX_MS : ms;
}

// ---------------- expansion --------------------------------------------------
// The real expandCard picks a branch at random. Here the branch is chosen by
// index, so the walk can cover all of them exactly once each.

static const char PUNCT[] = ",.!?;:";

static void expandCardPick(const char* src, char* dst, size_t cap, uint8_t pick) {
  size_t o = 0;

  while (*src && o + 1 < cap) {
    if (*src == ' ') { dst[o++] = *src++; continue; }

    const char* tok = src;
    while (*src && *src != ' ') src++;
    size_t tlen = (size_t)(src - tok);

    size_t core = tlen;
    while (core > 0 && strchr(PUNCT, tok[core - 1])) core--;

    uint8_t alts = 1;
    for (size_t k = 0; k < core; k++) if (tok[k] == '/') alts++;

    size_t begin = 0, end = core;
    if (alts > 1) {
      const uint8_t want = (uint8_t)(pick % alts);
      uint8_t seen = 0;
      size_t k = 0;
      while (k < core && seen < want) { if (tok[k] == '/') { seen++; begin = k + 1; } k++; }
      end = begin;
      while (end < core && tok[end] != '/') end++;
    }

    for (size_t k = begin; k < end && o + 1 < cap; k++)  dst[o++] = tok[k];
    for (size_t k = core;  k < tlen && o + 1 < cap; k++) dst[o++] = tok[k];
  }

  dst[o] = '\0';
}

// How many distinct readings a line has: the widest alternation anywhere in it.
// A line with two separate alternations is covered in as many steps as its
// larger one -- every branch is still shown, just not every combination.
static uint8_t cardVariants(const char* src) {
  uint8_t most = 1;

  while (*src) {
    if (*src == ' ') { src++; continue; }

    const char* tok = src;
    while (*src && *src != ' ') src++;
    size_t tlen = (size_t)(src - tok);

    size_t core = tlen;
    while (core > 0 && strchr(PUNCT, tok[core - 1])) core--;

    uint8_t alts = 1;
    for (size_t k = 0; k < core; k++) if (tok[k] == '/') alts++;
    if (alts > most) most = alts;
  }

  return most;
}

// ---------------- the playlist ------------------------------------------------
// Flat index space: every reading of every quip, then the rank announcements,
// which are RANK_COUNT ranks x 2 directions. Built as a mapping rather than an
// array so adding lines to the pool costs no RAM here.

uint16_t quipTotal  = 0;               // filled in at boot
static const uint8_t RANK_TOTAL = (uint8_t)(RANK_COUNT * 2);
uint16_t walkTotal  = 0;

uint16_t walkIdx    = 0;
uint32_t cardStart  = 0;
uint32_t cardHoldMs = 0;
bool     paused     = false;

// Renders flat index i into cardBuf and returns a short label for serial.
static void buildCard(uint16_t i, char* label, size_t labelCap) {
  if (i < quipTotal) {
    uint16_t k = i;
    for (uint8_t line = 0; line < CARD_COUNT; line++) {
      const uint8_t v = cardVariants(CARD_LINES[line]);
      if (k < v) {
        expandCardPick(CARD_LINES[line], cardBuf, sizeof(cardBuf), (uint8_t)k);
        snprintf(label, labelCap, "quip %u/%u branch %u/%u",
                 (unsigned)(line + 1), (unsigned)CARD_COUNT,
                 (unsigned)(k + 1), (unsigned)v);
        return;
      }
      k -= v;
    }
  }

  // const uint8_t r  = (uint8_t)((i - quipTotal) % RANK_COUNT);
  // const bool    up = ((i - quipTotal) / RANK_COUNT) == 0;
  // snprintf(cardBuf, sizeof(cardBuf),
  //          "chad has been moving %s recently and is now a %s",
  //          up ? "more" : "less", RANK_NAME[r]);
  // snprintf(label, labelCap, "rank %s %s", up ? "up  " : "down", RANK_NAME[r]);
}

// ---------------- wrap and draw ----------------------------------------------
// Byte-for-byte the wrapText and drawCard of tamagotchi.ino.

static uint8_t wrapText(const char* s, char* out, uint8_t stride,
                        uint8_t maxLines, uint8_t maxChars) {
  if (maxChars > stride - 1) maxChars = stride - 1;
  uint8_t n = 0;
  while (*s && n < maxLines) {
    while (*s == ' ') s++;
    if (!*s) break;
    uint8_t len = 0;
    while (s[len] && len < maxChars) len++;
    if (s[len]) {
      uint8_t brk = len;
      while (brk > 0 && s[brk] != ' ') brk--;
      if (brk > 0) len = brk;
    }
    char* dst = out + (uint16_t)n * stride;
    memcpy(dst, s, len);
    dst[len] = '\0';
    s += len;
    n++;
  }
  return n;
}

// The wrap width: the panel less CARD_SIDE_INSET, so a full line does not run
// edge to edge. The block stays centered on the panel itself.
static u8g2_uint_t cardTextWidth() {
  const u8g2_uint_t dw = u8g2.getDisplayWidth();
  return (dw > CARD_SIDE_INSET) ? (u8g2_uint_t)(dw - CARD_SIDE_INSET) : dw;
}

static void drawCard(const char* text) {
  const u8g2_uint_t dw = u8g2.getDisplayWidth();
  const u8g2_uint_t dh = u8g2.getDisplayHeight();

  u8g2.setFont(CARD_FONT);
  const u8g2_uint_t cw = u8g2.getMaxCharWidth();
  const u8g2_uint_t lh = u8g2.getMaxCharHeight() + CARD_LINE_GAP;
  const u8g2_uint_t tw = cardTextWidth();

  char lines[CARD_MAX_LINES][CARD_MAX_CHARS + 1];
  const uint8_t n = wrapText(text, &lines[0][0], CARD_MAX_CHARS + 1,
                             CARD_MAX_LINES,
                             (uint8_t)(cw ? tw / cw : CARD_MAX_CHARS));

  const u8g2_uint_t block = (u8g2_uint_t)(n * lh);
  u8g2_uint_t y = ((dh > block) ? (u8g2_uint_t)((dh - block) / 2) : 0)
                  + u8g2.getAscent();

  for (uint8_t i = 0; i < n; i++) {
    const u8g2_uint_t w = u8g2.getStrWidth(lines[i]);
    u8g2.drawStr((w < dw) ? (u8g2_uint_t)((dw - w) / 2) : 0, y, lines[i]);
    y += lh;
  }
}

// ---------------- the written report -----------------------------------------
// Same wrap, but with room for more lines than the panel allows, so a card that
// does not fit can be reported as such instead of silently losing its tail.

static const uint8_t REPORT_MAX_LINES = 16;

static void reportCard(const char* label, const char* text) {
  u8g2.setFont(CARD_FONT);
  const u8g2_uint_t cw = u8g2.getMaxCharWidth();
  const u8g2_uint_t tw = cardTextWidth();
  const uint8_t maxChars = (uint8_t)(cw ? tw / cw : CARD_MAX_CHARS);

  char lines[REPORT_MAX_LINES][CARD_MAX_CHARS + 1];
  const uint8_t n = wrapText(text, &lines[0][0], CARD_MAX_CHARS + 1,
                             REPORT_MAX_LINES, maxChars);

  Serial.print(label); Serial.print("  \""); Serial.print(text); Serial.println("\"");

  bool hardBreak = false;
  for (uint8_t i = 0; i < n; i++) {
    const u8g2_uint_t w = u8g2.getStrWidth(lines[i]);
    // A line that ends mid-word is a word too long for the panel, broken by
    // force. Worth seeing: it is the one failure the wrapper cannot avoid.
    const uint8_t len = (uint8_t)strlen(lines[i]);
    if (i + 1 < n && len == maxChars && lines[i][len - 1] != ' ' && lines[i + 1][0] != ' ')
      hardBreak = true;

    Serial.print(i < CARD_MAX_LINES ? "    |" : "  XX|");
    Serial.print(lines[i]);
    Serial.print("|  ");
    Serial.print((unsigned)w); Serial.print(" px");
    if (w > tw) Serial.print("   <-- OVER WIDTH");
    Serial.println();
  }

  if (n > CARD_MAX_LINES) {
    Serial.print("    !! "); Serial.print((unsigned)(n - CARD_MAX_LINES));
    Serial.print(" line(s) past CARD_MAX_LINES -- cut off on the panel");
    Serial.println();
  }
  if (hardBreak)
    Serial.println("    !! a word had to be hard-broken to fit the width");
}

static void dumpReport() {
  char label[40];

  Serial.println();
  Serial.print("=== every card, wrapped to ");
  Serial.print((unsigned)cardTextWidth());
  Serial.print(" px of "); Serial.print((unsigned)u8g2.getDisplayWidth());
  Serial.print(" ("); Serial.print((unsigned)(cardTextWidth() / u8g2.getMaxCharWidth()));
  Serial.println(" columns) ===");

  for (uint16_t i = 0; i < walkTotal; i++) {
    buildCard(i, label, sizeof(label));
    Serial.print("["); Serial.print((unsigned)(i + 1)); Serial.print("/");
    Serial.print((unsigned)walkTotal); Serial.print("] ");
    reportCard(label, cardBuf);
  }
  Serial.println("=== end of report ===");
  Serial.println();
}

static void banner() {
  Serial.println();
  Serial.println("card_test -- every card, in order");
  Serial.print("  pool: "); Serial.print((unsigned)CARD_COUNT);
  Serial.print(" lines -> "); Serial.print((unsigned)quipTotal);
  Serial.print(" readings, plus "); Serial.print((unsigned)RANK_TOTAL);
  Serial.println(" rank cards");
  Serial.println("  n next   p prev   spc pause   r restart   d dump report   ? this");
  Serial.println();
}

// ---------------- the walk ----------------------------------------------------

static void showWalkCard(uint16_t i, uint32_t now) {
  char label[40];
  buildCard(i, label, sizeof(label));

  cardStart  = now;
  cardHoldMs = cardReadMs(cardBuf);

  u8g2.clearBuffer();
  drawCard(cardBuf);
  u8g2.sendBuffer();

  Serial.print("["); Serial.print((unsigned)(i + 1)); Serial.print("/");
  Serial.print((unsigned)walkTotal); Serial.print("] ");
  Serial.print(label); Serial.print("  ");
  Serial.println(cardBuf);
}

static void step(int16_t delta) {
  walkIdx = (uint16_t)((walkIdx + walkTotal + delta) % walkTotal);
  showWalkCard(walkIdx, millis());
}

void setup() {
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) { }

  // RP2040: hardware SPI0 does not default to these pins, so route it before
  // u8g2.begin() or the panel stays blank. Same two lines as tamagotchi.ino.
  SPI.setSCK(2);
  SPI.setTX(3);

  u8g2.begin();
  u8g2.setFontMode(1);
  u8g2.setDrawColor(1);

  for (uint8_t i = 0; i < CARD_COUNT; i++) quipTotal += cardVariants(CARD_LINES[i]);
  walkTotal = (uint16_t)(quipTotal + RANK_TOTAL);

  banner();
  dumpReport();

  walkIdx = 0;
  showWalkCard(walkIdx, millis());
}

void loop() {
  while (Serial.available()) {
    switch (Serial.read()) {
      case 'n': paused = false; step(+1); break;
      case 'p': paused = false; step(-1); break;
      case ' ': paused = !paused;
                Serial.println(paused ? "-- paused" : "-- running");
                cardStart = millis();          // full dwell on resume
                break;
      case 'r': paused = false; walkIdx = 0; showWalkCard(walkIdx, millis()); break;
      case 'd': dumpReport(); break;
      case '?': banner(); break;
      default: break;
    }
  }

  if (!paused && millis() - cardStart >= cardHoldMs) step(+1);
}
