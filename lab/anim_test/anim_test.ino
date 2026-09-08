// anim_test.ino -- walk the panel through every animation the art set contains.
//
// WHAT IT DOES
//   Builds a playlist of every (rank, activity) pair that actually has frames
//   in sprites.h -- chud/pleb/nomad x rest/walk/sleep -- and plays each one in
//   turn, long enough to see the whole cycle at least twice. Shaking the device
//   interrupts with a flavor card, the same way a pickup does on the real
//   thing. So one upload exercises: every sprite, every frame, the header text
//   and XP bar at their layout extremes, and both kinds of card.
//
//   The playlist is built from RANK_SPRITES at boot, not hard-coded. Drop sleep
//   art into every art/<rank>/ folder, re-run gen_sprites.py, and the sleep
//   states appear here with no edit to this file. Until then the boot banner
//   lists them as missing, so a blank state reads as "no art yet" rather than
//   "the test is broken".
//
//   sprites.h and card_lines.h are SYMLINKS to the ones next to tamagotchi.ino.
//   That is deliberate: this test always shows the art and the text the real
//   sketch would show, with no copy to keep in sync.
//
// WIRING: identical to tamagotchi.ino.
//   ADXL345:  VCC->3V3  GND->GND  SDA->GP12  SCL->GP13
//             CS ->3V3  (REQUIRED: CS low puts it in SPI mode)
//             SDO->3V3  (address 0x1D)
//   OLED:     CLK->GP2  MOSI->GP3  RES->GP6  DC->GP7  CS->GP8
//
//   The accelerometer is optional. With none attached the playlist still runs;
//   only the motion cards go missing, and 'c' on the serial monitor fires one
//   by hand.
//
// HOW TO READ THE SCREEN
//   Top:     the real header -- "level N rank" over the XP bar.
//   Middle:  the animation under test.
//   Bottom:  "3/6 pleb walk" -- position in the playlist, rank, state.
//   Shake:   a card takes the whole panel, exactly as it does in the field.
//
//   The header numbers are fake on purpose and cycle through the cases that
//   break layouts: an empty bar, a 1-px bar, a full bar, and a level of 9999.
//   That last one against a five-letter rank is "level 9999 nomad" -- 16
//   characters at 4 px, precisely the 64 px panel width, which is the point
//   where drawHeader gives up on centering and left-aligns instead.
//
// SERIAL (115200), single keypress:
//   n  next state        p  previous state      c  fire a card now
//   spc pause / resume   r  restart playlist    ?  reprint the banner

#include <Wire.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <math.h>
#include <string.h>
#include "sprites.h"

#define ADDR 0x1D

// ======================= TUNE THESE =========================================
// Each state is held for whichever is longer: this floor, or DWELL_LOOPS full
// passes of its own cycle. The floor keeps a 2-frame walk on screen long enough
// to judge; the loop count keeps an 11-frame idle from getting cut off mid-blink.
static const uint32_t MIN_DWELL_MS = 4000;
static const uint8_t  DWELL_LOOPS  = 2;

// Shake energy that counts as a pickup. Same units and same value as the real
// sketch's HANDLE_GATE, so what triggers a card here triggers one in the field.
static float HANDLE_GATE = 4000.0f;

static const uint8_t SPRITE_SCALE = 3;
// ============================================================================

// SH1106 128x64, 4-wire hardware SPI. Args: rotation, CS, DC, RESET.
// U8G2_R3 = 90 deg CCW -> 64x128 portrait, same as tamagotchi.ino.
U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R3, 8, 7, 6);

static const char* const RANK_NAME[] = {"chud", "pleb", "nomad"};
static const char* const ACT_NAME[]  = {"rest", "walk", "sleep"};   // enum Activity order

// ---------------- sensor -----------------------------------------------------

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(ADDR);
  Wire.write(reg);
  Wire.endTransmission();          // full stop, not repeated start
  Wire.requestFrom(ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

// ADXL345 full-resolution scale: 3.9 mg per LSB -> magnitude in milli-g.
static const float LSB_TO_MG = 3.9f;

void readXYZ(int16_t &x, int16_t &y, int16_t &z) {
  Wire.beginTransmission(ADDR);
  Wire.write(0x32);                // DATAX0
  Wire.endTransmission();
  Wire.requestFrom(ADDR, (uint8_t)6);
  uint8_t b[6];
  for (int i = 0; i < 6; i++) b[i] = Wire.available() ? Wire.read() : 0;
  x = (int16_t)(b[0] | (b[1] << 8));
  y = (int16_t)(b[2] | (b[3] << 8));
  z = (int16_t)(b[4] | (b[5] << 8));
}

// Magnitude of the current sample, in milli-g (~1000 at rest).
float readMagMg() {
  int16_t x, y, z;
  readXYZ(x, y, z);
  return sqrtf((float)x * x + (float)y * y + (float)z * z) * LSB_TO_MG;
}

void enableNormalMode() {
  writeReg(0x2D, 0x00);   // standby while reconfiguring
  writeReg(0x31, 0x08);   // DATA_FORMAT: full res, +/-2g, INT active HIGH
  writeReg(0x2C, 0x0A);   // BW_RATE: 100 Hz output data rate
  writeReg(0x2E, 0x00);   // INT_ENABLE: no interrupts
  writeReg(0x2D, 0x08);   // POWER_CTL: measurement mode (REQUIRED)
}

bool haveSensor = false;   // false -> motion cards are simply never triggered

// ---------------- pickup detection -------------------------------------------
// The full walk/idle classifier is not needed here: nothing in this sketch
// depends on knowing the difference. All we want is the short-window energy
// that the real sketch uses for "a hand is on this thing", so the card fires on
// the same gesture and at the same threshold.

static const int      ODR_HZ            = 50;
static const int      SHORT_WIN         = 10;    // ~200 ms at 50 Hz
static const uint32_t HANDLE_RELEASE_MS = 800;   // quiet time before "put down"
static const uint32_t STILL_BEFORE_MS   = 1500;  // stillness that makes it a pickup

float    shortBuf[SHORT_WIN];
int      shortHead = 0, shortFilled = 0;
bool     handled        = false;
uint32_t handledLastMs  = 0;
uint32_t unhandledSince = 0;

void pushSample(float mag) {
  shortBuf[shortHead] = mag;
  shortHead = (shortHead + 1) % SHORT_WIN;
  if (shortFilled < SHORT_WIN) shortFilled++;
}

float shortVar() {
  if (shortFilled < SHORT_WIN) return 0;
  float mean = 0;
  for (int i = 0; i < SHORT_WIN; i++) mean += shortBuf[i];
  mean /= SHORT_WIN;
  float energy = 0;
  for (int i = 0; i < SHORT_WIN; i++) {
    const float d = shortBuf[i] - mean;
    energy += d * d;
  }
  return energy / SHORT_WIN;
}

// ---------------- sprite drawing ---------------------------------------------
// Verbatim from tamagotchi.ino. If a sprite looks wrong here it looks wrong
// there; that is the entire point of not reimplementing it.

static void drawXBMPScaled(u8g2_uint_t x, u8g2_uint_t y,
                           uint8_t w, uint8_t h, const uint8_t* bits) {
  if (SPRITE_SCALE <= 1) {
    u8g2.drawXBMP(x, y, w, h, bits);
    return;
  }
  const uint8_t bytesPerRow = (w + 7) / 8;
  for (uint8_t row = 0; row < h; row++) {
    for (uint8_t col = 0; col < w; col++) {
      const uint8_t b = bits[row * bytesPerRow + (col >> 3)];
      if (b & (1 << (col & 7))) {
        u8g2.drawBox(x + col * SPRITE_SCALE, y + row * SPRITE_SCALE,
                     SPRITE_SCALE, SPRITE_SCALE);
      }
    }
  }
}

// Sprites are placed by their feet, not their box: horizontally centered, with
// the bottom row always SPRITE_BOTTOM_PAD above the end of the panel. Ranks
// differ in height (nomad is 32 px to pleb's 16), and this is what stops the
// tall one sinking through the floor when the playlist steps onto it.
static const u8g2_uint_t SPRITE_BOTTOM_PAD = 8;

static void drawAnimFrame(const Animation* a, uint8_t frame) {
  const u8g2_uint_t dw = u8g2.getDisplayWidth();
  const u8g2_uint_t dh = u8g2.getDisplayHeight();
  const u8g2_uint_t sw = (u8g2_uint_t)a->w * SPRITE_SCALE;
  const u8g2_uint_t sh = (u8g2_uint_t)a->h * SPRITE_SCALE;
  const u8g2_uint_t base = dh - SPRITE_BOTTOM_PAD;

  const u8g2_uint_t x = (dw > sw)   ? (u8g2_uint_t)((dw - sw) / 2) : 0;
  const u8g2_uint_t y = (base > sh) ? (u8g2_uint_t)(base - sh)     : 0;
  drawXBMPScaled(x, y, a->w, a->h, a->frames[frame]);
}

struct AnimPlayer {
  const Animation* anim = nullptr;
  uint8_t  frame = 0;
  uint32_t lastTick = 0;

  void play(const Animation* a, uint32_t now) {
    if (a == anim) return;
    anim = a;
    frame = 0;
    lastTick = now;
  }

  void update(uint32_t now) {
    if (!anim || now - lastTick < anim->frameMs) return;
    lastTick = now;
    if (++frame >= anim->frameCount)
      frame = anim->loop ? 0 : anim->frameCount - 1;
  }

  void draw() const {
    if (anim) drawAnimFrame(anim, frame);
  }
};

AnimPlayer player;

// ---------------- the playlist -----------------------------------------------
// One entry per animation that exists. Ranks come from RANK_SPRITES, so the
// count follows the art rather than a constant in this file.

struct Cell { uint8_t rank; uint8_t act; };
static Cell    playlist[RANK_SPRITE_COUNT * 3];
static uint8_t playlistLen = 0;
static uint8_t cellIdx     = 0;
static uint32_t cellStart  = 0;
static bool     paused     = false;

// No fallback: a missing animation must come back as nullptr so the playlist
// can leave it out and the banner can name it. (animForActivity() in the real
// sketch substitutes rest for a missing sleep, which is right there and wrong
// here -- it would show the same art twice under two different labels.)
static const Animation* animFor(uint8_t rank, uint8_t act) {
  if (rank >= RANK_SPRITE_COUNT) return nullptr;
  const RankSprites& s = RANK_SPRITES[rank];
  switch (act) {
    case REST:  return s.rest;
    case WALK:  return s.walk;
    case SLEEP: return s.sleep;
    default:    return nullptr;
  }
}

static void buildPlaylist() {
  playlistLen = 0;
  for (uint8_t r = 0; r < RANK_SPRITE_COUNT; r++) {
    for (uint8_t a = REST; a <= SLEEP; a++) {
      if (animFor(r, a)) playlist[playlistLen++] = { r, (uint8_t)a };
    }
  }
}

// How long the current state stays up: enough for DWELL_LOOPS complete passes,
// but never less than the floor.
static uint32_t dwellMs(const Animation* a) {
  if (!a) return MIN_DWELL_MS;
  const uint32_t cycle = (uint32_t)a->frameCount * a->frameMs * DWELL_LOOPS;
  return (cycle > MIN_DWELL_MS) ? cycle : MIN_DWELL_MS;
}

// ---------------- header ------------------------------------------------------
// drawHeader() copied from tamagotchi.ino, with the level and progress passed in
// rather than read from flash -- this sketch has no save file, and fake numbers
// are what let it hit the layout extremes on purpose.

static const u8g2_uint_t PAD_TOP = 8;
static const u8g2_uint_t PAD_X   = 4;
static const u8g2_uint_t TEXT_BASE = PAD_TOP + 6;   // 4x6 baseline
static const u8g2_uint_t BAR_Y = PAD_TOP + 8;
static const u8g2_uint_t BAR_H = 5;

static void drawHeader(uint16_t level, uint8_t progress, uint8_t rank) {
  const u8g2_uint_t dw = u8g2.getDisplayWidth();
  const u8g2_uint_t bw = dw - 2 * PAD_X;

  char line[24];
  snprintf(line, sizeof(line), "lvl %u %s", (unsigned)level, RANK_NAME[rank]);
  u8g2.setFont(u8g2_font_4x6_tf);
  const u8g2_uint_t w = u8g2.getStrWidth(line);
  u8g2.drawStr((w < bw) ? PAD_X + (bw - w) / 2 : 0, TEXT_BASE, line);

  u8g2.drawFrame(PAD_X, BAR_Y, bw, BAR_H);
  const u8g2_uint_t inner = bw - 2;
  u8g2_uint_t fill = (u8g2_uint_t)(((uint32_t)inner * progress) / 255);
  if (fill == 0 && progress > 0) fill = 1;
  if (fill) u8g2.drawBox(PAD_X + 1, BAR_Y + 1, fill, BAR_H - 2);
}

// The header cases, one per playlist step, so every case is seen once per pass.
// They are chosen to break things: progress 0 is an empty bar, 1 exercises the
// "force at least one pixel" rule, 255 is a full one, and level 9999 is the
// widest number the field can hold. Which rank that lands next to depends on
// the playlist, so the 16-character worst case ("level 9999 nomad", exactly the
// 64 px panel width) is only guaranteed while the case count and the playlist
// length stay coprime -- 5 and 6 today. Check the panel, not this comment.
struct HeaderCase { uint16_t level; uint8_t progress; };
static const HeaderCase HEADER_CASES[] = {
  {   1,   0},   // fresh device, empty bar
  {   3,   1},   // the 1-px minimum fill
  {  12, 128},   // ordinary half-full
  { 137, 255},   // full bar
  {9999,  96},   // longest possible header string
};
static const uint8_t HEADER_CASE_COUNT =
    sizeof(HEADER_CASES) / sizeof(HEADER_CASES[0]);

// ---------------- cards -------------------------------------------------------
// The real card pool, the real expansion, the real wrapping. Only the trigger
// differs: on the device a card waits for a genuine pickup after stillness,
// here any shake will do.

static const char* const CARD_LINES[] = {
  #include "card_lines.h"
};
static const uint8_t CARD_COUNT = sizeof(CARD_LINES) / sizeof(CARD_LINES[0]);

static const uint32_t CARD_BASE_MS     = 2000;
static const uint32_t CARD_MS_PER_CHAR = 55;
static const uint32_t CARD_HOLD_MS     = 1500;
static const uint32_t CARD_MAX_MS      = 6000;
// Far shorter than the real 5 minutes: the point of this sketch is to be able
// to shake it repeatedly and keep getting cards.
static const uint32_t CARD_COOLDOWN_MS = 1000;

#define CARD_FONT  u8g2_font_4x6_tf
static const u8g2_uint_t CARD_LINE_GAP = 2;
static const uint8_t CARD_MAX_LINES = 8;
static const uint8_t CARD_MAX_CHARS = 20;

const char* cardText   = nullptr;
uint32_t    cardStart  = 0;
uint32_t    cardMinMs  = 0;
uint32_t    cardNextOk = 0;
uint8_t     cardLast   = 0xFF;
char        cardBuf[96];

// Rank announcements are the other kind of card, and they are the longer and
// harder one to lay out. They are rare in the field, so the test forces one
// every RANK_CARD_EVERY cards and steps it through the ranks and both
// directions -- otherwise the promotion text would go untested for hours.
static const uint8_t RANK_CARD_EVERY = 4;
uint8_t cardsShown = 0;

inline bool cardActive() { return cardText != nullptr; }

uint32_t cardReadMs(const char* text) {
  return CARD_BASE_MS + (uint32_t)strlen(text) * CARD_MS_PER_CHAR;
}

// "a/b/c" in a line picks one of a, b, c, with trailing punctuation riding
// along with whatever gets chosen.
static void expandCard(const char* src, char* dst, size_t cap) {
  static const char PUNCT[] = ",.!?;:";
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
      uint8_t want = (uint8_t)random(alts);
      uint8_t seen = 0;
      size_t k = 0;
      begin = 0;
      while (k < core && seen < want) { if (tok[k] == '/') { seen++; begin = k + 1; } k++; }
      end = begin;
      while (end < core && tok[end] != '/') end++;
    }

    for (size_t k = begin; k < end && o + 1 < cap; k++)  dst[o++] = tok[k];
    for (size_t k = core;  k < tlen && o + 1 < cap; k++) dst[o++] = tok[k];
  }

  dst[o] = '\0';
}

void showCard(const char* text, uint32_t now) {
  cardText  = text;
  cardStart = now;
  cardMinMs = cardReadMs(text);
  Serial.print("card: "); Serial.println(text);
}

void showRandomCard(uint32_t now) {
  uint8_t i = (uint8_t)random(CARD_COUNT);
  if (CARD_COUNT > 1 && i == cardLast) i = (uint8_t)((i + 1) % CARD_COUNT);
  cardLast = i;
  expandCard(CARD_LINES[i], cardBuf, sizeof(cardBuf));
  showCard(cardBuf, now);
}

// Walks the ranks and both directions on successive calls, so every wording of
// the announcement gets shown eventually.
void showRankCard(uint32_t now) {
  static uint8_t n = 0;
  const uint8_t to  = (uint8_t)(n % RANK_SPRITE_COUNT);
  const bool    up  = ((n / RANK_SPRITE_COUNT) % 2) == 0;
  n++;
  snprintf(cardBuf, sizeof(cardBuf),
           "chad has been moving %s recently and is now a %s",
           up ? "more" : "less", RANK_NAME[to]);
  showCard(cardBuf, now);
}

void showAttentionCard(uint32_t now) {
  if (++cardsShown % RANK_CARD_EVERY == 0) showRankCard(now);
  else                                     showRandomCard(now);
}

// Greedy word wrap; a word longer than the line is hard-broken rather than lost.
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

static void drawCard(const char* text) {
  const u8g2_uint_t dw = u8g2.getDisplayWidth();
  const u8g2_uint_t dh = u8g2.getDisplayHeight();

  u8g2.setFont(CARD_FONT);
  const u8g2_uint_t cw = u8g2.getMaxCharWidth();
  const u8g2_uint_t lh = u8g2.getMaxCharHeight() + CARD_LINE_GAP;

  char lines[CARD_MAX_LINES][CARD_MAX_CHARS + 1];
  const uint8_t n = wrapText(text, &lines[0][0], CARD_MAX_CHARS + 1,
                             CARD_MAX_LINES,
                             (uint8_t)(cw ? dw / cw : CARD_MAX_CHARS));

  const u8g2_uint_t block = (u8g2_uint_t)(n * lh);
  u8g2_uint_t y = ((dh > block) ? (u8g2_uint_t)((dh - block) / 2) : 0)
                  + u8g2.getAscent();

  for (uint8_t i = 0; i < n; i++) {
    const u8g2_uint_t w = u8g2.getStrWidth(lines[i]);
    u8g2.drawStr((w < dw) ? (u8g2_uint_t)((dw - w) / 2) : 0, y, lines[i]);
    y += lh;
  }
}

// ---------------- the test screen ---------------------------------------------

// Bottom strip: which cell of the matrix you are looking at. The sprite ends at
// SPRITE_BOTTOM_PAD above the panel edge, so this sits in that gap and overlaps
// nothing. It is the one thing on screen the real device does not draw.
static void drawCellLabel() {
  const u8g2_uint_t dw = u8g2.getDisplayWidth();
  const u8g2_uint_t dh = u8g2.getDisplayHeight();
  const Cell& c = playlist[cellIdx];

  char label[24];
  snprintf(label, sizeof(label), "%u/%u %s %s",
           (unsigned)(cellIdx + 1), (unsigned)playlistLen,
           RANK_NAME[c.rank], ACT_NAME[c.act]);

  u8g2.setFont(u8g2_font_4x6_tf);
  const u8g2_uint_t w = u8g2.getStrWidth(label);
  u8g2.drawStr((w < dw) ? (u8g2_uint_t)((dw - w) / 2) : 0, dh - 1, label);
}

void drawFrame() {
  u8g2.clearBuffer();

  // A card owns the whole panel -- no header, no sprite, no label -- exactly as
  // it does on the device.
  if (cardActive()) {
    drawCard(cardText);
    u8g2.sendBuffer();
    return;
  }

  const HeaderCase& hc = HEADER_CASES[cellIdx % HEADER_CASE_COUNT];
  player.draw();
  drawHeader(hc.level, hc.progress, playlist[cellIdx].rank);
  drawCellLabel();
  u8g2.sendBuffer();
}

// ---------------- playlist control --------------------------------------------

static void announceCell() {
  const Cell& c = playlist[cellIdx];
  const Animation* a = animFor(c.rank, c.act);
  Serial.print("["); Serial.print(cellIdx + 1);
  Serial.print("/");  Serial.print(playlistLen);
  Serial.print("] "); Serial.print(RANK_NAME[c.rank]);
  Serial.print(" ");  Serial.print(ACT_NAME[c.act]);
  Serial.print("  frames="); Serial.print(a->frameCount);
  Serial.print(" "); Serial.print(a->w); Serial.print("x"); Serial.print(a->h);
  Serial.print(" @"); Serial.print(a->frameMs); Serial.print("ms");
  Serial.print("  dwell="); Serial.print(dwellMs(a)); Serial.println("ms");
}

static void gotoCell(uint8_t idx, uint32_t now) {
  cellIdx   = idx % playlistLen;
  cellStart = now;
  player.play(animFor(playlist[cellIdx].rank, playlist[cellIdx].act), now);
  announceCell();
}

static void banner() {
  Serial.println();
  Serial.println("=== anim_test: every animation, every rank ===");
  Serial.print("ranks in sprites.h: "); Serial.println(RANK_SPRITE_COUNT);
  Serial.print("animations found:   "); Serial.println(playlistLen);

  // Name the gaps. A rank with no sleep art is the expected case today, and
  // saying so here is what keeps it from looking like a bug in the test.
  bool anyMissing = false;
  for (uint8_t r = 0; r < RANK_SPRITE_COUNT; r++) {
    for (uint8_t a = REST; a <= SLEEP; a++) {
      if (!animFor(r, a)) {
        if (!anyMissing) { Serial.println("missing art (skipped):"); anyMissing = true; }
        Serial.print("  "); Serial.print(RANK_NAME[r]);
        Serial.print(" "); Serial.println(ACT_NAME[a]);
      }
    }
  }
  if (!anyMissing) Serial.println("missing art: none -- full coverage");
  Serial.print("SPRITES_HAVE_SLEEP = "); Serial.println(SPRITES_HAVE_SLEEP);
  Serial.print("card pool: "); Serial.print(CARD_COUNT); Serial.println(" lines");
  Serial.println("keys: n next  p prev  c card  spc pause  r restart  ? banner");
  Serial.println();
}

// ---------------- setup / loop -------------------------------------------------

void setup() {
  SPI.setSCK(2);
  SPI.setTX(3);
  u8g2.begin();
  u8g2.setDrawColor(1);

  Serial.begin(115200);
  delay(2000);                      // give the host time to attach the port

  randomSeed(micros());

  Wire.setSDA(12); Wire.setSCL(13);
  Wire.begin();
  Wire.setClock(100000);

  const uint8_t devid = readReg(0x00);
  haveSensor = (devid == 0xE5);
  Serial.print("DEVID (want 0xE5) = 0x"); Serial.println(devid, HEX);
  if (haveSensor) enableNormalMode();
  else Serial.println("  no ADXL345 -- animations still run; press 'c' for a card");

  buildPlaylist();
  banner();

  if (playlistLen == 0) {           // nothing to play: say so on the panel too
    Serial.println("!! sprites.h contains no animations at all");
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_4x6_tf);
    u8g2.drawStr(2, 20, "no sprites");
    u8g2.drawStr(2, 30, "in sprites.h");
    u8g2.sendBuffer();
    return;
  }

  const uint32_t now = millis();
  unhandledSince = now;
  handledLastMs  = now;
  gotoCell(0, now);
  drawFrame();
}

void loop() {
  if (playlistLen == 0) return;

  static unsigned long nextSample = 0;
  const unsigned long now = millis();
  if (nextSample == 0) nextSample = now;

  // --- serial control ---
  while (Serial.available()) {
    switch (Serial.read()) {
      case 'n': gotoCell(cellIdx + 1, now); break;
      case 'p': gotoCell((uint8_t)(cellIdx + playlistLen - 1), now); break;
      case 'r': cardsShown = 0; gotoCell(0, now); break;
      case 'c': showAttentionCard(now); break;
      case ' ':
        paused = !paused;
        cellStart = now;            // resuming gives the state its full dwell
        Serial.println(paused ? "paused" : "running");
        break;
      case '?': banner(); break;
      default: break;               // newlines from the monitor, ignore
    }
  }

  // --- accelerometer: the only job is spotting a hand on the device ---
  if (haveSensor && (long)(now - nextSample) >= 0) {
    nextSample += 1000 / ODR_HZ;
    pushSample(readMagMg());

    if (shortVar() > HANDLE_GATE) {
      handledLastMs = now;
      if (!handled) {
        // A pickup is motion that follows stillness. Without that test, one
        // continuous shake would re-fire on every sample.
        const bool wasStill =
            unhandledSince != 0 && (now - unhandledSince) >= STILL_BEFORE_MS;
        handled = true;
        if (wasStill && !cardActive() && (long)(now - cardNextOk) >= 0) {
          showAttentionCard(now);
        }
      }
    } else if (handled && (now - handledLastMs) >= HANDLE_RELEASE_MS) {
      handled = false;
      unhandledSince = now;
    }
  }

  // --- card lifetime ---
  // Same rule as the device: stays up for its own reading time, clears once the
  // hand has been off it for CARD_HOLD_MS, and is capped so holding the thing
  // forever does not wedge the playlist.
  if (cardActive()) {
    const uint32_t age = now - cardStart;
    const uint32_t cap = (CARD_MAX_MS > cardMinMs) ? CARD_MAX_MS : cardMinMs;
    if (age >= cap ||
        (age >= cardMinMs && !handled && (now - handledLastMs) >= CARD_HOLD_MS)) {
      cardText   = nullptr;
      cardNextOk = now + CARD_COOLDOWN_MS;
      cellStart  = now;      // the interrupted state gets its full dwell back
      Serial.println("card cleared");
    }
  }

  // --- advance the playlist ---
  // Frozen while a card is up or while paused, so no state can scroll past
  // behind a card and go unseen. That is the whole reason to hold the clock
  // rather than just letting it run.
  if (!paused && !cardActive() &&
      (now - cellStart) >= dwellMs(player.anim)) {
    gotoCell(cellIdx + 1, now);
  }

  // --- redraw only on an actual change ---
  player.update(now);

  static uint8_t lastFrame = 255;
  static const Animation* lastAnim = nullptr;
  static const char* lastCard = nullptr;
  static uint8_t lastCell = 255;
  if (cardText != lastCard || cellIdx != lastCell ||
      (!cardActive() && (player.anim != lastAnim || player.frame != lastFrame))) {
    lastAnim  = player.anim;
    lastFrame = player.frame;
    lastCard  = cardText;
    lastCell  = cellIdx;
    drawFrame();
  }
}
