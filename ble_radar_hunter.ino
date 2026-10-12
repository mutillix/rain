//
// Libraries: ESP32_Display_Panel. Everything else is the ESP32 core.
//

#include <Arduino.h>
#include <esp_display_panel.hpp>
#include <BLEDevice.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_wifi.h>
#include <esp_heap_caps.h>
#include <SD_MMC.h>
#include <math.h>
#include <ctype.h>
#include <atomic>
#include <Preferences.h>

using namespace esp_panel::drivers;

#define PIN_BOOT 0
#define PIN_BL   41
#define PIN_BATT 8

#define SCP_X 20
#define SCP_Y 56
#define SCP_S 280
#define SCP_C 140
#define SCP_R 130

#define RSSI_1M (-59)
#define PATH_N  2.0f          // free-space. Raise toward 2.7 if indoor reads run long.
#define FRESH_MS 20000
#define STALE_MS 45000

const esp_panel_lcd_vendor_init_cmd_t vendor_init[] = {
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0xF1, {0x00}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0x60, {0x00, 0x00, 0x00}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0x65, {0x80}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0x79, {0x06}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0x7B, {0x00, 0x08, 0x08}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0x80, {0x55, 0x62, 0x2F, 0x17, 0xF0, 0x52, 0x70, 0xD2, 0x52, 0x62, 0xEA}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0x81, {0x26, 0x52, 0x72, 0x27}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0x84, {0x92, 0x25}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0x87, {0x10, 0x10, 0x58, 0x00, 0x02, 0x3A}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0x88, {0x00, 0x00, 0x2C, 0x10, 0x04, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x06}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0x89, {0x00, 0x00, 0x00}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0x8A, {0x13, 0x00, 0x2C, 0x00, 0x00, 0x2C, 0x10, 0x10, 0x00, 0x3E, 0x19}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0x8B, {0x15, 0xB1, 0xB1, 0x44, 0x96, 0x2C, 0x10, 0x97, 0x8E}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0x8C, {0x1D, 0xB1, 0xB1, 0x44, 0x96, 0x2C, 0x10, 0x50, 0x0F, 0x01, 0xC5, 0x12, 0x09}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0x8D, {0x0C}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0x8E, {0x33, 0x01, 0x0C, 0x13, 0x01, 0x01}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0xB3, {0x00, 0x30}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0xF1, {0x00}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0x71, {0xC0}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0x66, {0x02, 0x3F}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0xBE, {0x24, 0x00, 0x9D}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0x70, {0x01, 0xA0, 0x11, 0x40, 0xE0, 0x00, 0x11, 0x69, 0x11, 0x00, 0x00, 0x1A}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0x90, {0x04, 0x04, 0x55, 0x74, 0x00, 0x40, 0x43, 0x27, 0x27}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0x91, {0x04, 0x04, 0x55, 0x75, 0x00, 0x40, 0x42, 0x27, 0x27}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0x92, {0x04, 0x44, 0x55, 0xC0, 0x06, 0x00, 0x07, 0x05, 0x90, 0x27}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0x93, {0x04, 0x43, 0x11, 0x00, 0x00, 0x00, 0x00, 0x05, 0x90, 0x27}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0x94, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0x95, {0x96, 0x16, 0x00, 0x00, 0xFF}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0x96, {0x44, 0x53, 0x03, 0x12, 0x23, 0x24, 0x06, 0x05, 0x94, 0x27, 0x00, 0x44}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0x97, {0x44, 0x53, 0x47, 0x56, 0x20, 0x20, 0x02, 0x01, 0x94, 0x27, 0x00, 0x44}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0xBA, {0x55, 0x94, 0x2D, 0x94, 0x27}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0x9A, {0x40, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0x9B, {0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0x9C, {0x5C, 0x12, 0x00, 0x00, 0x10, 0x12, 0x00, 0x00, 0x10, 0x02, 0x00, 0x00, 0x00}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0x9D, {0x8A, 0x51, 0x00, 0x00, 0x00, 0x80, 0x1E, 0x01}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0x9E, {0x51, 0x00, 0x00, 0x00, 0x80, 0x1E, 0x01}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0xB4, {0x1D, 0x1C, 0x1E, 0x0B, 0x14, 0x02, 0x13, 0x09, 0x1E, 0x00, 0x1E, 0x10}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0xB5, {0x1D, 0x1C, 0x1E, 0x0A, 0x15, 0x03, 0x11, 0x08, 0x1E, 0x01, 0x1E, 0x12}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0xB6, {0x77, 0x77, 0x00, 0x0A, 0xFF, 0x0A, 0xFF}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0x86, {0xCD, 0x04, 0xB1, 0x02, 0x58, 0x12, 0x58, 0x0C, 0x13, 0x01, 0xA5, 0x00, 0xA5, 0xA5}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0xB7, {0x07, 0x0A, 0x0E, 0x06, 0x05, 0x03, 0x2B, 0x03, 0x03, 0x42, 0x07, 0x10, 0x10, 0x2E, 0x3F, 0x0D}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0xB8, {0x07, 0x0A, 0x0D, 0x05, 0x05, 0x02, 0x2B, 0x02, 0x03, 0x42, 0x06, 0x10, 0x0F, 0x2E, 0x3F, 0x0D}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0xB9, {0x23, 0x23}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0xBF, {0x10, 0x14, 0x14, 0x0B, 0x0B, 0x0B}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0xF2, {0x00}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0x73, {0x04, 0xDA, 0x12, 0x54, 0x47}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0x77, {0x6B, 0x5B, 0xFD, 0xC3, 0xC5}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0x7A, {0x15, 0x27}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0x7B, {0x04, 0x57}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0x7E, {0x01, 0x0E}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0xBF, {0x36}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0xE3, {0x40, 0x40}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0xF0, {0x00}),
ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(0, 0xD0, {0x00}),
ESP_PANEL_LCD_CMD_WITH_NONE_PARAM(120, 0x11), // SLPOUT
ESP_PANEL_LCD_CMD_WITH_NONE_PARAM(0, 0x29), // DISPON
ESP_PANEL_LCD_CMD_WITH_NONE_PARAM(0, 0x21), // INVON — this panel paints black as white and orange as cyan without it
};


BusQSPI *bus = new BusQSPI(10, 12, 11, 13, 14, 9); // CS CLK D0 D1 D2 D3
LCD *lcd = new LCD_ST77922(bus, 320, 480, 16, -1);

#define A_BLACK  0x0000
#define A_FAINT  0x4208
#define A_DIM    0xA520
#define A_MID    0xFD20
#define A_BRIGHT 0xFFE0
#define A_WHITE  0xFFFF

// ---- Vivid heat-map: RSSI -100..-40 dBm -> deep blue -> cyan -> green -> yellow -> red ----
static uint16_t heatColor(int rssi) {
  if (rssi < -100) rssi = -100;
  if (rssi > -40) rssi = -40;
  int v = rssi + 100;  // 0..60
  int r, g, b;
  if (v < 15) {        // blue -> cyan
    int k = v * 255 / 15;
    r = 0; g = k; b = 255;
  } else if (v < 30) { // cyan -> green
    int k = (v - 15) * 255 / 15;
    r = 0; g = 255; b = 255 - k;
  } else if (v < 45) { // green -> yellow
    int k = (v - 30) * 255 / 15;
    r = k; g = 255; b = 0;
  } else {             // yellow -> red
    int k = (v - 45) * 255 / 15;
    r = 255; g = 255 - k; b = 0;
  }
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

static uint8_t glyphBuf[32 * 24 * 2];
static uint8_t rectBuf[320 * 32 * 2];

static inline void putPx(uint8_t *p, uint16_t color) {
  p[0] = color >> 8;
  p[1] = color & 0xFF;
}

void bringUpPanel() {
  pinMode(PIN_BL, OUTPUT);
  digitalWrite(PIN_BL, HIGH);
  pinMode(48, OUTPUT);            // touch RST, pulsed before panel init
  digitalWrite(48, LOW);
  delay(120);
  digitalWrite(48, HIGH);
  delay(200);
  bus->configQSPI_FreqHz(80 * 1000 * 1000);
  lcd->configVendorCommands(vendor_init, sizeof(vendor_init) / sizeof(vendor_init[0]));
  if (!lcd->begin()) Serial.println("panel dark");
  lcd->displayOn();
  pinMode(PIN_BOOT, INPUT_PULLUP);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_BATT, ADC_11db);
}

static const uint8_t FONT96[64][5] = {
{0x00,0x00,0x00,0x00,0x00},
{0x00,0x00,0x5F,0x00,0x00},
{0x00,0x07,0x00,0x07,0x00},
{0x14,0x7F,0x14,0x7F,0x14},
{0x24,0x2A,0x7F,0x2A,0x12},
{0x23,0x13,0x08,0x64,0x62},
{0x36,0x49,0x55,0x22,0x50},
{0x00,0x05,0x03,0x00,0x00},
{0x00,0x1C,0x22,0x41,0x00},
{0x00,0x41,0x22,0x1C,0x00},
{0x14,0x08,0x3E,0x08,0x14},
{0x08,0x08,0x3E,0x08,0x08},
{0x00,0x50,0x30,0x00,0x00},
{0x08,0x08,0x08,0x08,0x08},
{0x00,0x60,0x60,0x00,0x00},
{0x20,0x10,0x08,0x04,0x02},
{0x3E,0x51,0x49,0x45,0x3E},
{0x00,0x42,0x7F,0x40,0x00},
{0x42,0x61,0x51,0x49,0x46},
{0x21,0x41,0x45,0x4B,0x31},
{0x18,0x14,0x12,0x7F,0x10},
{0x27,0x45,0x45,0x45,0x39},
{0x3C,0x4A,0x49,0x49,0x30},
{0x01,0x71,0x09,0x05,0x03},
{0x36,0x49,0x49,0x49,0x36},
{0x06,0x49,0x49,0x29,0x1E},
{0x00,0x66,0x66,0x00,0x00},
{0x00,0x56,0x36,0x00,0x00},
{0x08,0x14,0x22,0x41,0x00},
{0x14,0x14,0x14,0x14,0x14},
{0x00,0x41,0x22,0x14,0x08},
{0x02,0x01,0x51,0x09,0x06},
{0x32,0x49,0x79,0x41,0x3E},
{0x7E,0x11,0x11,0x11,0x7E},
{0x7F,0x49,0x49,0x49,0x36},
{0x3E,0x41,0x41,0x41,0x22},
{0x7F,0x41,0x41,0x22,0x1C},
{0x7F,0x49,0x49,0x49,0x41},
{0x7F,0x09,0x09,0x09,0x01},
{0x3E,0x41,0x49,0x49,0x7A},
{0x7F,0x08,0x08,0x08,0x7F},
{0x00,0x41,0x7F,0x41,0x00},
{0x20,0x40,0x41,0x3F,0x01},
{0x7F,0x08,0x14,0x22,0x41},
{0x7F,0x40,0x40,0x40,0x40},
{0x7F,0x02,0x0C,0x02,0x7F},
{0x7F,0x04,0x08,0x10,0x7F},
{0x3E,0x41,0x41,0x41,0x3E},
{0x7F,0x09,0x09,0x09,0x06},
{0x3E,0x41,0x51,0x21,0x5E},
{0x7F,0x09,0x19,0x29,0x46},
{0x46,0x49,0x49,0x49,0x31},
{0x01,0x01,0x7F,0x01,0x01},
{0x7F,0x40,0x40,0x40,0x7F},
{0x0F,0x30,0x40,0x30,0x0F},
{0x7F,0x20,0x18,0x20,0x7F},
{0x63,0x14,0x08,0x14,0x63},
{0x07,0x08,0x70,0x08,0x07},
{0x61,0x51,0x49,0x45,0x43},
{0x00,0x7F,0x41,0x41,0x00},
{0x02,0x04,0x08,0x10,0x20},
{0x00,0x41,0x41,0x7F,0x00},
{0x04,0x02,0x01,0x02,0x04},
{0x40,0x40,0x40,0x40,0x40},
};

static int alignDown(int x) { return x & ~3; }
static int alignUp(int x) { return (x + 3) & ~3; }

void lcdRect(int x, int y, int w, int h, uint16_t color) {
  if (x < 0) { w += x; x = 0; }
  if (y < 0) { h += y; y = 0; }
  if (x + w > 320) w = 320 - x;
  if (y + h > 480) h = 480 - y;
  if (w <= 0 || h <= 0) return;
  int x0 = alignDown(x);
  int x1 = alignUp(x + w);
  if (x1 > 320) x1 = 320;
  x = x0;
  w = x1 - x0;
  if (w <= 0) return;
  for (int i = 0; i < w; i++) putPx(&rectBuf[i * 2], color);
  int done = 0;
  while (done < h) {
    int chunk = h - done;
    if (chunk > 16) chunk = 16;
    for (int r = 1; r < chunk; r++) memcpy(&rectBuf[r * w * 2], rectBuf, w * 2);
    lcd->drawBitmap(x, y + done, w, chunk, rectBuf);
    done += chunk;
  }
}

static const uint8_t *glyphFor(char c) {
  if (c >= 'a' && c <= 'z') c -= 32;
  if (c < 32 || c > 95) return FONT96[0];
  return FONT96[c - 32];
}

void lcdChar(int x, int y, char c, int scale, uint16_t fg, uint16_t bg) {
  if (scale < 1) scale = 1;
  if (scale > 3) scale = 3;
  const uint8_t *g = glyphFor(c);
  int gw = 5 * scale;
  int gh = 7 * scale;
  int x0 = alignDown(x);
  int pad = x - x0;
  int wp = alignUp(pad + gw);
  if (wp > 32) wp = 32;
  if (x0 < 0 || x0 + wp > 320 || y < 0 || y + gh > 480) return;
  for (int row = 0; row < gh; row++) {
    for (int col = 0; col < wp; col++) putPx(&glyphBuf[(row * wp + col) * 2], bg);
    int bit = row / scale;
    for (int col = 0; col < 5; col++) {
      if (!(g[col] & (1 << bit))) continue;
      for (int sx = 0; sx < scale; sx++) {
        int dx = pad + col * scale + sx;
        if (dx >= 0 && dx < wp) putPx(&glyphBuf[(row * wp + dx) * 2], fg);
      }
    }
  }
  lcd->drawBitmap(x0, y, wp, gh, glyphBuf);
}

int textW(const char *s, int scale) {
  if (scale < 1) scale = 1;
  if (scale > 3) scale = 3;
  return (int)strlen(s) * 6 * scale;
}

void lcdText(int x, int y, const char *s, int scale, uint16_t fg, uint16_t bg) {
  while (*s) {
    lcdChar(x, y, *s, scale, fg, bg);
    x += 6 * scale;
    s++;
  }
}

void lcdTextC(int y, const char *s, int scale, uint16_t fg, uint16_t bg) {
  int x = (320 - textW(s, scale)) / 2;
  if (x < 4) x = 4;
  lcdText(x, y, s, scale, fg, bg);
}

static uint8_t *scpFb = NULL;

static inline void fbPx(int x, int y, uint16_t color) {
  if ((unsigned)x >= SCP_S || (unsigned)y >= SCP_S) return;
  uint8_t *p = &scpFb[(y * SCP_S + x) * 2];
  p[0] = color >> 8;
  p[1] = color & 0xFF;
}

void fbChar(int x, int y, char c, uint16_t fg) {
  const uint8_t *g = glyphFor(c);
  for (int row = 0; row < 7; row++)
    for (int col = 0; col < 5; col++)
      if (g[col] & (1 << row))
        fbPx(x + col, y + row, fg);
}
void fbText(int x, int y, const char *s, uint16_t fg) {
  while (*s) { fbChar(x, y, *s, fg); x += 6; s++; }
}

static uint16_t dimColor(uint16_t c, int b) {
  if (b >= 255) return c;
  if (b <= 0) return A_BLACK;
  int r = (c >> 11) & 0x1F, g = (c >> 5) & 0x3F, bl = c & 0x1F;
  r = (r * b) >> 8;
  g = (g * b) >> 8;
  bl = (bl * b) >> 8;
  return (r << 11) | (g << 5) | bl;
}

void fbClear() {
  memset(scpFb, 0, SCP_S * SCP_S * 2); // A_BLACK is 0x0000
}

void fbLine(int x0, int y0, int x1, int y1, uint16_t color) {
  int dx = abs(x1 - x0), dy = -abs(y1 - y0);
  int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  for (;;) {
    fbPx(x0, y0, color);
    if (x0 == x1 && y0 == y1) break;
    int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}

void fbCircle(int cx, int cy, int r, uint16_t color) {
  int x = r, y = 0, err = 0;
  while (x >= y) {
    fbPx(cx + x, cy + y, color); fbPx(cx + y, cy + x, color);
    fbPx(cx - y, cy + x, color); fbPx(cx - x, cy + y, color);
    fbPx(cx - x, cy - y, color); fbPx(cx - y, cy - x, color);
    fbPx(cx + y, cy - x, color); fbPx(cx + x, cy - y, color);
    y++;
    err += 1 + 2 * y;
    if (2 * (err - x) + 1 > 0) { x--; err += 1 - 2 * x; }
  }
}

void fbDisc(int cx, int cy, int r, uint16_t color) {
  for (int y = -r; y <= r; y++) {
    int w = (int)sqrtf((float)(r * r - y * y));
    for (int x = -w; x <= w; x++) fbPx(cx + x, cy + y, color);
  }
}

// 0 deg = north, clockwise. Screen y grows downward.
static void bearingXY(int deg, int radius, int *x, int *y) {
  float a = deg * (PI / 180.0f);
  *x = SCP_C + (int)(sinf(a) * radius);
  *y = SCP_C - (int)(cosf(a) * radius);
}

void lcdFill(uint16_t color) {
  static uint8_t chunk[320 * 16 * 2];
  for (int i = 0; i < 320; i++) putPx(&chunk[i * 2], color);
  for (int r = 1; r < 16; r++) memcpy(&chunk[r * 320 * 2], chunk, 320 * 2);
  for (int y = 0; y < 480; y += 16) {
    int c = 480 - y;
    if (c > 16) c = 16;
    lcd->drawBitmap(0, y, 320, c, chunk);
  }
}

#define MAXTGT 96
#define TYPE_BLE  1
#define TYPE_ESP  2
#define TYPE_WIFI 3
#define BAND_BLE  0
#define BAND_ESP  1
#define BAND_WIFI 2
static uint8_t bandMode = BAND_BLE;
static inline uint8_t wantType() {
  return bandMode == BAND_ESP ? TYPE_ESP : bandMode == BAND_WIFI ? TYPE_WIFI : TYPE_BLE;
}
static inline char bandChar() { return bandMode == BAND_ESP ? 'E' : bandMode == BAND_WIFI ? 'W' : 'B'; }
static inline const char *bandName() {
  return bandMode == BAND_ESP ? "ESP HUNTER" : bandMode == BAND_WIFI ? "WIFI RADAR" : "BLE RADAR";
}

struct Tgt {
  char mac[18];
  char name[34];
  uint8_t type;
  int8_t ch;
  int rssi;
  float smoothRssi;
  int minRssi;
  int maxRssi;
  uint32_t last;
  uint16_t hits;
  uint32_t born;
  bool watched;
  uint32_t hotSince;    // game: when RSSI first crossed threshold (0 = not hot)
  int8_t peakRssi;      // game: peak RSSI during hot period
  bool gameDone;        // game: already evaluated this hot streak
  uint8_t tracker;      // 0=none 1=AirTag 2=Tile 3=SmartTag 4=Chipolo
  bool alertPresent;    // watchlist: was present last check
  int trendBase;        // find: RSSI snapshot for trend
  uint32_t trendAt;     // find: when snapshot taken
};
static Tgt tgts[MAXTGT];
static int tgtCount = 0;

// ---- Tracker detection ----
#define TRK_NONE 0
#define TRK_AIRTAG 1
#define TRK_TILE 2
#define TRK_SMARTTAG 3
#define TRK_CHIPOLO 4
static const char *TRK_NAMES[] = {"", "AirTag", "Tile", "SmartTag", "Chipolo"};
static const uint16_t TRK_COLORS[] = {0, 0xFFFF, 0x07FF, 0xF81F, 0x07E0};

// Hunter game mode (game prefix — distinct from bandMode which selects the radio band)
#define GAME_CATCH_RSSI -60
#define GAME_DWELL_MS 3000
#define GAME_MAX_CATCHES 500
static Preferences gamePrefs;
static uint32_t gameScore = 0;
static uint32_t gameCatches = 0;
static int gameStreak = 0;
static uint32_t gameLastDay = 0;
static uint32_t gameBaseUptime = 0;   // seconds accumulated across boots (NVS)
static uint32_t gameLastUptimeSave = 0;
static uint32_t gameCaughtHashes[GAME_MAX_CATCHES];
static int gameCaughtN = 0;
// per-target hot state lives in Tgt (hotSince, peakRssi, gameDone below)


static uint32_t macHash(const char *mac) {
  uint32_t h = 2166136261u;
  while (*mac) { h = (h ^ (uint8_t)*mac) * 16777619u; mac++; }
  return h;
}
static int tgtAngle(int i) {
  uint32_t h = macHash(tgts[i].mac);
  if (tgts[i].type == TYPE_WIFI) {
    // spread by WiFi channel first (1-13 -> sectors), then by MAC within sector
    int ch = tgts[i].ch;
    if (ch < 1) ch = 1;
    if (ch > 13) ch = 13;
    return (ch - 1) * 27 + (int)(h % 27);
  }
  return (int)(h % 360);
}

static float bleDistM(int rssi) {
  return powf(10.0f, (RSSI_1M - rssi) / (10.0f * PATH_N));
}
static int distToR(float d) {
  if (d < 1) d = 1;
  float r = 24 + 60 * log10f(d);
  if (r > 144) r = 144;
  return (int)r;
}
static void distText(char *dst, size_t n, int rssi) {
  float d = bleDistM(rssi);
  if (d >= 99.5f) snprintf(dst, n, ">99m %ddBm", rssi);
  else snprintf(dst, n, "~%dm %ddBm", (int)(d + 0.5f), rssi);
}

static void copyClean(char *dst, int n, const char *src) {
  int j = 0;
  if (!src) src = "";
  for (int i = 0; src[i] && j < n - 1; i++) {
    unsigned char c = (unsigned char)src[i];
    if (c < 32 || c > 126 || c == '"' || c == ',') c = ' ';
    dst[j++] = (char)c;
  }
  dst[j] = '\0';
  while (j > 0 && dst[j - 1] == ' ') dst[--j] = '\0';
}

// ---- Credits (final view)  ----
#define CREDIT_NAME   "Mikael Henry"
#define CREDIT_WEB    "mutillix.com"
#define CREDIT_GITHUB "github.com/mutillix"
#define CREDIT_SLOGAN "DO WHAT YOU CAN"

static int viewMode = 0; // 0=rain 1=scope 2=list 3=hunter 4=waterfall 5=strip 6=density 7=detail 8=gps 9=tracker 10=find 11=credits
static float battV = 0;
static bool battPresent = false;
static bool sdFail = false;          // set by sdBegin / logDev
static bool scanStartFailed = false; // set by bleArm
void logDev(int i);

#define MRU 8
static int mru[MRU];
static int mruN = 0;

void mruTouch(int idx) {
  for (int i = 0; i < mruN; i++)
    if (mru[i] == idx) {
      memmove(&mru[1], &mru[0], i * sizeof(int));
      mru[0] = idx;
      return;
    }
  if (mruN < MRU) mruN++;
  memmove(&mru[1], &mru[0], (mruN - 1) * sizeof(int));
  mru[0] = idx;
}


// ---- Watchlist: flag these MACs (uppercase, colon-separated) ----
static const char *WATCH_MACS[] = {
  // "AA:BB:CC:DD:EE:FF",  // add your devices here
};
#define WATCH_COUNT (sizeof(WATCH_MACS)/sizeof(WATCH_MACS[0]))

static bool isWatched(const char *mac) {
  for (unsigned int i = 0; i < WATCH_COUNT; i++)
    if (strcasecmp(WATCH_MACS[i], mac) == 0) return true;
  return false;
}

int upsertTgt(const char *mac, const char *name, uint8_t type, int rssi, int ch, uint8_t tracker = TRK_NONE) {
  uint32_t now = millis();
  int idx = -1;
  bool isNew = false;
  for (int i = 0; i < tgtCount; i++) {
    if (strcmp(tgts[i].mac, mac) == 0) { idx = i; break; }
  }
  if (idx < 0) {
    if (tgtCount < MAXTGT) idx = tgtCount++;
    else {
      uint32_t oldest = now;
      idx = 0;
      for (int i = 0; i < tgtCount; i++)
        if (tgts[i].last < oldest) { oldest = tgts[i].last; idx = i; }
    }
    memset(&tgts[idx], 0, sizeof(Tgt));
    strncpy(tgts[idx].mac, mac, 17);
    tgts[idx].mac[17] = '\0';
    tgts[idx].born = now;
    tgts[idx].smoothRssi = rssi;
    tgts[idx].minRssi = rssi;
    tgts[idx].maxRssi = rssi;
    tgts[idx].watched = isWatched(mac);
    tgts[idx].tracker = tracker;
    isNew = true;
  }
  Tgt *t = &tgts[idx];
  t->type = type;
  t->rssi = rssi;
  t->smoothRssi = t->smoothRssi * 0.7f + rssi * 0.3f;
  if (rssi < t->minRssi) t->minRssi = rssi;
  if (rssi > t->maxRssi) t->maxRssi = rssi;
  t->ch = (int8_t)ch;
  t->last = now;
  if (tracker != TRK_NONE) t->tracker = tracker;
  if (t->hits < 65535) t->hits++;
  if (name && name[0]) copyClean(t->name, sizeof(t->name), name);
  else if (!t->name[0]) {
    // last two octets, short enough to live in a data block
    snprintf(t->name, sizeof(t->name), "%.8s", t->mac + 9);
  }
  mruTouch(idx);
  if (isNew) logDev(idx);
  return idx;
}

static int freshCount(uint32_t now) {
  int n = 0;
  for (int i = 0; i < tgtCount; i++)
    if (tgts[i].type == wantType() && now - tgts[i].last <= FRESH_MS) n++;
  return n;
}

static uint8_t faceBuf[76 * 128 * 2];

#define EYE_Y0 48
#define EYE_Y1 66

static bool sdUp = false;
static File logFile;
static uint32_t lastFlush = 0;
static int logLines = 0;

void sdBegin() {
  // Hosyond ES3C28P/ES3N28P onboard TF slot (SDMMC, not SPI).
  // CLK=38 CMD=40 D0=39. 1-bit only — D1 (41) collides with PIN_BL.
  SD_MMC.setPins(38, 40, 39);
  sdUp = SD_MMC.begin("/sdcard", true);
  sdFail = !sdUp;
  if (!sdUp) Serial.println("sd dark");
  else       Serial.println("sd up");
  rgbOff();  // GPIO40 = SD CMD, re-kill LED after SD touches it
}

static double gpsLat = 0, gpsLon = 0;
static float gpsSpeedKmh = 0, gpsAltM = 0;
static int gpsSats = 0;
static bool gpsFix = false;

void logDev(int i) {
  if (!sdUp) return;
  if (!logFile) {
    logFile = SD_MMC.open("/ble_log.csv", FILE_APPEND);
    if (logFile && logFile.size() == 0)
      logFile.println("t,name,mac,rssi,dist_m,ch,lat,lon,sats");
    if (!logFile) { sdFail = true; return; }
  }
  Tgt *t = &tgts[i];
  logFile.printf("+%lu,\"%s\",%s,%d,%.1f,%d,%.6f,%.6f,%d\n",
                 millis() / 1000, t->name, t->mac, t->rssi,
                 bleDistM(t->rssi), (int)t->ch,
                 gpsFix ? gpsLat : 0, gpsFix ? gpsLon : 0, gpsSats);
  if (++logLines % 20 == 0) {
    logFile.flush();
    lastFlush = millis();
  }
}

void logFlushTick(uint32_t now) {
  if (logFile && now - lastFlush > 30000) {
    logFile.flush();
    lastFlush = now;
  }
}

#define BLERING 32
struct BleHit { char mac[18]; char name[34]; int rssi; uint8_t tracker; };
static BleHit bleRing[BLERING];
static std::atomic<int> bleW{0};
static std::atomic<int> bleR{0};
static uint32_t scanRestartAt = 0;

// Same order as BLEAddress::toString(). Bluedroid stores print order.
// NimBLE stores the bytes reversed, and only that path flips them.
static void formatBleAddr(char *dst, const uint8_t *native) {
#if defined(CONFIG_NIMBLE_ENABLED)
  snprintf(dst, 18, "%02x:%02x:%02x:%02x:%02x:%02x",
           native[5], native[4], native[3], native[2], native[1], native[0]);
#else
  snprintf(dst, 18, "%02x:%02x:%02x:%02x:%02x:%02x",
           native[0], native[1], native[2], native[3], native[4], native[5]);
#endif
}

class ScanCB : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice d) {
    int w = bleW.load(std::memory_order_relaxed);
    int r = bleR.load(std::memory_order_acquire);
    int nx = (w + 1) % BLERING;
    if (nx == r) return;
    formatBleAddr(bleRing[w].mac, d.getAddress().getNative());
    bleRing[w].name[0] = '\0';
    if (d.haveName()) {
      String nm = d.getName();
      nm.toCharArray(bleRing[w].name, sizeof(bleRing[w].name));
    }
    bleRing[w].rssi = d.getRSSI();
    // tracker detection
    uint8_t trk = TRK_NONE;
    if (d.haveManufacturerData()) {
      String md = d.getManufacturerData();
      if (md.length() >= 3 && (uint8_t)md[0] == 0x4C && (uint8_t)md[1] == 0x00) {
        if ((uint8_t)md[2] == 0x12) trk = TRK_AIRTAG;  // Apple Find My
      }
    }
    if (trk == TRK_NONE && d.haveServiceUUID()) {
      BLEUUID uuid = d.getServiceUUID();
      uint16_t u16 = uuid.getNative()->u16.value;
      if (u16 == 0xFEED) trk = TRK_TILE;
      else if (u16 == 0xFD69) trk = TRK_SMARTTAG;
    }
    bleRing[w].tracker = trk;
    bleW.store(nx, std::memory_order_release);
  }
};

void bleArm() {
  BLEScan *s = BLEDevice::getScan();
  if (s->isScanning()) return;
  s->clearResults();
  scanStartFailed = !s->start(3, nullptr, false);
  scanRestartAt = millis() + 3200;
}

void bleBegin() {
  BLEDevice::init("air-scope");
  BLEScan *s = BLEDevice::getScan();
  s->setAdvertisedDeviceCallbacks(new ScanCB(), true);
  s->setActiveScan(true);
  s->setInterval(160);
  s->setWindow(120);
  bleArm();
}

void blePoll() {
  int r = bleR.load(std::memory_order_relaxed);
  int w = bleW.load(std::memory_order_acquire);
  while (r != w) {
    BleHit *h = &bleRing[r];
    upsertTgt(h->mac, h->name, TYPE_BLE, h->rssi, 0, h->tracker);
    r = (r + 1) % BLERING;
    bleR.store(r, std::memory_order_release);
  }
}

void blePump(uint32_t now) {
  if (bandMode != BAND_BLE) return;
  if (BLEDevice::getScan()->isScanning()) return;
  if ((int32_t)(now - scanRestartAt) < 0) return;
  bleArm();
}

#define HUNTRING 32
struct HuntHit { char mac[18]; int rssi; uint8_t ch; };
static HuntHit huntRing[HUNTRING];
static std::atomic<int> huntW{0};
static std::atomic<int> huntR{0};
static uint8_t hunterCh = 1;
static uint32_t lastHop = 0;

static const uint32_t ESP_OUIS[] = {
  0x246F28, 0x240AC4, 0x30AEA4, 0x3C71BF, 0x782184, 0x840D8E, 0x84F3EB, 0x94B97E,
  0xA47B9D, 0xAC67B2, 0xC44F33, 0xD8A01D, 0xDC4F22, 0xE868E7, 0xEC64C9, 0x083AF2,
  0x08B61F, 0x10521C, 0x14335C, 0x34865D, 0x382B78, 0x3C6105, 0x40F520, 0x4827E2,
  0x4C7525, 0x543204, 0x58BF25, 0x600194, 0x686725, 0x7CDFA1, 0x807D3A, 0x90380C,
  0x9C9C1F, 0xB4E62D, 0xBCDDC2, 0xC82E18, 0xC8C9A3, 0xCC50E3, 0xD4D4DA, 0xE05A1B,
  0xE8DB84, 0xF008D2, 0xF412FA
};
static bool isEspressifOui(uint32_t oui) {
  for (unsigned i = 0; i < sizeof(ESP_OUIS) / sizeof(ESP_OUIS[0]); i++)
    if (ESP_OUIS[i] == oui) return true;
  return false;
}

static uint32_t dbgMgmt = 0, dbgAction = 0, dbgMatch = 0;
void hunterRx(void *buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT) return;
  dbgMgmt++;
  wifi_promiscuous_pkt_t *p = (wifi_promiscuous_pkt_t *)buf;
  if (p->rx_ctrl.sig_len < 30) return;
  uint8_t *pl = p->payload;
  if ((pl[0] & 0x0C) != 0x00) return;
  uint8_t subtype = pl[0] & 0xF0;
  if (subtype == 0xD0) dbgAction++;
  if (subtype == 0xD0) {
    if (p->rx_ctrl.sig_len < 32) return;
    if (pl[24] != 127) return;
    if (pl[25] != 0x18 || pl[26] != 0xFE || pl[27] != 0x34) return;
  } else if (subtype == 0x40) {
    uint32_t oui = ((uint32_t)pl[10] << 16) | ((uint32_t)pl[11] << 8) | pl[12];
    if (!isEspressifOui(oui)) return;
  } else {
    return;
  }
  dbgMatch++;
  int w = huntW.load(std::memory_order_relaxed);
  int r = huntR.load(std::memory_order_acquire);
  int nx = (w + 1) % HUNTRING;
  if (nx == r) return;
  snprintf(huntRing[w].mac, 18, "%02x:%02x:%02x:%02x:%02x:%02x",
           pl[10], pl[11], pl[12], pl[13], pl[14], pl[15]);
  huntRing[w].rssi = p->rx_ctrl.rssi;
  huntRing[w].ch = hunterCh;
  huntW.store(nx, std::memory_order_release);
}

// ---- WiFi band: async scan ----
static uint32_t wifiScanAt = 0;
static bool wifiScanning = false;

void wifiStart() {
  // stop BLE
  BLEScan *s = BLEDevice::getScan();
  s->stop();
  uint32_t t0 = millis();
  while (s->isScanning() && millis() - t0 < 400) delay(10);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(40);
  wifiScanning = false;
  wifiScanAt = 0;
  Serial.println("wifi on");
}

void wifiStop() {
  if (wifiScanning) {
    WiFi.scanDelete();
    wifiScanning = false;
  }
  WiFi.mode(WIFI_OFF);
  scanRestartAt = millis();
  Serial.println("wifi off");
}

void wifiPoll(uint32_t now) {
  if (!wifiScanning && (int32_t)(now - wifiScanAt) >= 0) {
    WiFi.scanNetworks(true);  // async
    wifiScanning = true;
    return;
  }
  if (wifiScanning) {
    int n = WiFi.scanComplete();
    if (n >= 0) {
      for (int i = 0; i < n; i++) {
        String ssid = WiFi.SSID(i);
        if (ssid.length() == 0) ssid = "<hidden>";
        char mac[18];
        uint8_t *b = WiFi.BSSID(i);
        snprintf(mac, sizeof(mac), "%02X:%02X:%02X:%02X:%02X:%02X",
                 b[0], b[1], b[2], b[3], b[4], b[5]);
        char nm[34];
        ssid.toCharArray(nm, sizeof(nm));
        upsertTgt(mac, nm, TYPE_WIFI, WiFi.RSSI(i), WiFi.channel(i));
      }
      WiFi.scanDelete();
      wifiScanning = false;
      wifiScanAt = now + 2000;  // rescan every 2s
    } else if (n == -2) {
      // scan failed, retry
      wifiScanning = false;
      wifiScanAt = now + 1000;
    }
  }
}

void hunterPoll() {
  int r = huntR.load(std::memory_order_relaxed);
  int w = huntW.load(std::memory_order_acquire);
  while (r != w) {
    HuntHit *h = &huntRing[r];
    char nm[34];
    snprintf(nm, sizeof(nm), "ESP-%c%c%c%c%c%c",
             toupper((unsigned char)h->mac[9]), toupper((unsigned char)h->mac[10]),
             toupper((unsigned char)h->mac[12]), toupper((unsigned char)h->mac[13]),
             toupper((unsigned char)h->mac[15]), toupper((unsigned char)h->mac[16]));
    upsertTgt(h->mac, nm, TYPE_ESP, h->rssi, h->ch);
    r = (r + 1) % HUNTRING;
    huntR.store(r, std::memory_order_release);
  }
}

void hunterStart() {
  bandMode = BAND_ESP;
  BLEScan *s = BLEDevice::getScan();
  s->stop();
  uint32_t t0 = millis();
  while (s->isScanning() && millis() - t0 < 400) delay(10);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(40);
  wifi_promiscuous_filter_t filt = {};
  filt.filter_mask = WIFI_PROMIS_FILTER_MASK_ALL;
  // Ensure WiFi driver is init'd and started (band switching can leave it dead)
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_start();
  esp_err_t e1 = esp_wifi_set_promiscuous_filter(&filt);
  esp_err_t e2 = esp_wifi_set_promiscuous(true);
  esp_err_t e3 = esp_wifi_set_promiscuous_rx_cb(&hunterRx);
  Serial.printf("promisc: filt=%d en=%d cb=%d\n", e1, e2, e3);
  hunterCh = 1;
  esp_wifi_set_channel(hunterCh, WIFI_SECOND_CHAN_NONE);
  lastHop = millis();
  Serial.println("hunter on");
}

void hunterStop() {
  esp_wifi_set_promiscuous(false);
  esp_wifi_set_promiscuous_rx_cb(nullptr);
  WiFi.mode(WIFI_OFF);
  scanRestartAt = millis();
  Serial.println("hunter off");
}

// ---- Hunter game ----
static uint32_t gameDay(uint32_t now) {
  return (gameBaseUptime + now / 1000) / 86400;
}

static int gpsUtcHour = -1;

// ==================== HUNTER 2.0 (Tier 1) ====================
// Badges: bitmask in NVS "hunter" namespace, key "badges"
//  0: First BLE catch      1: First ESP-NOW catch    2: First WiFi catch
//  3: 25 uniques           4: 100 uniques            5: 250 uniques
//  6: 500 uniques           7: 1000 uniques
//  8: First AirTag          9: First Tile            10: First SmartTag
//  11: Night Owl (22:00-06:00 UTC)  12: Marathon (1hr session)
//  13: Globetrotter (5 GPS grid cells)
static uint32_t badgeBits = 0;
static const char *badgeNames[14] = {
  "BLE PIONEER", "ESP PIONEER", "WIFI PIONEER",
  "COLLECTOR 25", "COLLECTOR 100", "COLLECTOR 250",
  "COLLECTOR 500", "COLLECTOR 1000",
  "TAG SPOTTER", "TILE SPOTTER", "SMARTTAG SPOTTER",
  "NIGHT OWL", "MARATHON", "GLOBETROTTER"
};
static uint32_t badgeQueue = 0;  // pending banner display

static void badgeUnlock(int b, uint32_t now) {
  if (b < 0 || b > 13) return;
  if (badgeBits & (1UL << b)) return;
  badgeBits |= (1UL << b);
  gamePrefs.putUInt("badges", badgeBits);
  badgeQueue |= (1UL << b);
  Serial.printf("BADGE: %s\n", badgeNames[b]);
}

// OUI rarity: first 3 MAC bytes -> tier (0=common x1, 1=uncommon x2, 2=rare x3, 3=legendary x5)
// Legendary = OUI not in table (never seen before)
struct RarityEntry { uint8_t b[3]; uint8_t tier; };
static const RarityEntry ouiTable[] = {
  {{0x24,0x0A,0xC4},0}, {{0x30,0xAE,0xA4},0}, {{0x7C,0xDF,0xA1},0},  // Espressif
  {{0xDC,0xA6,0x32},0}, {{0xE8,0xDB,0x84},0},                        // Espressif
  {{0x4C,0x32,0x75},0}, {{0x88,0x1B,0x99},0}, {{0xF0,0x18,0x98},0},  // Apple
  {{0x8C,0x7A,0xAA},0}, {{0x40,0xCB,0xC0},0},                        // Apple
  {{0x8C,0xF5,0xA3},0}, {{0x9C,0x8E,0xCD},0},                        // Samsung
  {{0x00,0x17,0xAB},1}, {{0x00,0x1A,0x11},1},                        // Google
  {{0x18,0xB4,0x30},1}, {{0x54,0x60,0x09},1},                        // Amazon
  {{0x50,0xDC,0xE7},1}, {{0x74,0xC6,0x3B},1},                        // Tile
  {{0xD8,0xA0,0x1D},1}, {{0xF4,0xF5,0xD8},1},                        // Intel
  {{0xB8,0x27,0xEB},1}, {{0xDC,0xA6,0x32},1},                        // Raspberry Pi
  {{0x00,0x0C,0x29},2}, {{0x00,0x50,0x56},2},                        // VMware
  {{0x00,0x1B,0x21},2}, {{0x00,0x21,0x6A},2},                        // Intel corp
  {{0x3C,0x22,0xFB},2}, {{0xA4,0xCF,0x12},2},                        // Espressif rare
  {{0x08,0x3A,0xF2},2}, {{0x10,0x52,0x1C},2},                        // Nordic Semi
  {{0x00,0xE0,0x4C},2}, {{0x00,0x80,0x41},2},                        // Realtek
  {{0xC4,0x4E,0xAC},2}, {{0xEC,0xFA,0xBC},2},                        // Espressif
};
#define OUI_N (sizeof(ouiTable)/sizeof(ouiTable[0]))
static const uint8_t rarityMult[4] = {1, 2, 3, 5};
static const char *rarityName[4] = {"COMMON", "UNCOMMON", "RARE", "LEGENDARY"};

static int ouiRarity(const char *mac) {
  unsigned b0, b1, b2;
  if (sscanf(mac, "%2x:%2x:%2x", &b0, &b1, &b2) != 3) return 3;
  for (int i = 0; i < OUI_N; i++) {
    if (ouiTable[i].b[0]==b0 && ouiTable[i].b[1]==b1 && ouiTable[i].b[2]==b2)
      return ouiTable[i].tier;
  }
  return 3;  // not in table = legendary
}

// Combo: catches within 60s chain
static uint32_t comboLast = 0;
static int comboN = 0;
static int comboMult(uint32_t now) {
  if (now - comboLast < 60000) comboN++;
  else comboN = 1;
  comboLast = now;
  if (comboN >= 8) return 5;
  if (comboN >= 5) return 4;
  if (comboN >= 3) return 3;
  if (comboN >= 2) return 2;
  return 1;
}

// Category bonuses
static bool isDefaultSsid(const char *name) {
  const char *defaults[] = {"linksys","netgear","default","dlink","tplink",
    "xfinity","attwifi","verizon","spectrum","coxwifi",NULL};
  char lower[34];
  for (int i = 0; name[i] && i < 33; i++) lower[i] = tolower(name[i]);
  lower[33] = 0;
  for (int i = 0; defaults[i]; i++)
    if (strstr(lower, defaults[i])) return true;
  return false;
}

// Grid cells for Globetrotter (100m approx = 0.0009 deg lat)
#define GRID_CELLS 32
static int32_t gridCells[GRID_CELLS];
static int gridN = 0;
static int gridCellId(float lat, float lon) {
  int32_t la = (int32_t)(lat * 1111);   // ~90m
  int32_t lo = (int32_t)(lon * 1111);
  return (la * 73856093) ^ (lo * 19349663);
}
static bool gridAdd(float lat, float lon) {
  int id = gridCellId(lat, lon);
  for (int i = 0; i < gridN; i++) if (gridCells[i] == id) return false;
  if (gridN < GRID_CELLS) gridCells[gridN++] = id;
  return true;
}

// Session tracking for Marathon
static uint32_t sessionStart = 0;
static bool marathonDone = false;
// ==================== /HUNTER 2.0 ====================

void gameInit() {
  gamePrefs.begin("hunter", false);
  gameScore = gamePrefs.getUInt("score", 0);
  gameCatches = gamePrefs.getUInt("catches", 0);
  gameStreak = gamePrefs.getInt("streak", 0);
  gameLastDay = gamePrefs.getUInt("lastday", 0);
  gameBaseUptime = gamePrefs.getUInt("uptime", 0);
  badgeBits = gamePrefs.getUInt("badges", 0);
  sessionStart = millis();
  gameCaughtN = gamePrefs.getInt("n", 0);
  if (gameCaughtN > GAME_MAX_CATCHES) gameCaughtN = GAME_MAX_CATCHES;
  for (int i = 0; i < gameCaughtN; i++) {
    char key[8];
    snprintf(key, sizeof(key), "c%d", i);
    String mac = gamePrefs.getString(key, "");
    gameCaughtHashes[i] = mac.length() ? macHash(mac.c_str()) : 0;
  }
  Serial.printf("game: score %u catches %u streak %d\n", gameScore, gameCatches, gameStreak);
}

static bool gameIsCaught(const char *mac) {
  uint32_t h = macHash(mac);
  for (int i = 0; i < gameCaughtN; i++)
    if (gameCaughtHashes[i] == h) return true;
  return false;
}





void gameCatch(int i, uint32_t now) {
  Tgt *t = &tgts[i];
  if (gameIsCaught(t->mac)) return;  // first catches only

  // Base points
  int pts = (t->peakRssi + 100) * 10;
  if (pts < 0) pts = 0;

  // Rarity multiplier
  int rar = ouiRarity(t->mac);
  pts *= rarityMult[rar];

  // Combo multiplier
  int cmb = comboMult(now);
  pts *= cmb;

  // Category bonuses (flat)
  int bonus = 0;
  if (t->type == TYPE_WIFI && isDefaultSsid(t->name)) bonus += 50;
  int bestRssi = gamePrefs.getInt("bestrssi", -100);
  if (t->peakRssi > bestRssi) {
    bonus += 30;
    gamePrefs.putInt("bestrssi", t->peakRssi);
  }
  // multi-band: check if this MAC was seen on another band
  // (tracked via caught MACs with type suffix - simplified: skip for now)
  pts += bonus;

  gameScore += pts;
  gameCatches++;

  // ---- Badges ----
  if (t->type == TYPE_BLE) badgeUnlock(0, now);
  if (t->type == TYPE_ESP) badgeUnlock(1, now);
  if (t->type == TYPE_WIFI) badgeUnlock(2, now);
  if (gameCatches >= 25) badgeUnlock(3, now);
  if (gameCatches >= 100) badgeUnlock(4, now);
  if (gameCatches >= 250) badgeUnlock(5, now);
  if (gameCatches >= 500) badgeUnlock(6, now);
  if (gameCatches >= 1000) badgeUnlock(7, now);
  if (t->tracker == 1) badgeUnlock(8, now);
  if (t->tracker == 2) badgeUnlock(9, now);
  if (t->tracker == 3) badgeUnlock(10, now);
  if (gpsUtcHour >= 22 || gpsUtcHour < 6) badgeUnlock(11, now);
  if (!marathonDone && sessionStart > 0 && now - sessionStart >= 3600000) {
    marathonDone = true;
    badgeUnlock(12, now);
  }
  if (gpsFix && gridAdd(gpsLat, gpsLon) && gridN >= 5) badgeUnlock(13, now);
  if (gameCaughtN < GAME_MAX_CATCHES) {
    gameCaughtHashes[gameCaughtN] = macHash(t->mac);
    char key[8];
    snprintf(key, sizeof(key), "c%d", gameCaughtN);
    gamePrefs.putString(key, t->mac);
    gameCaughtN++;
    gamePrefs.putInt("n", gameCaughtN);
  }
  gamePrefs.putUInt("score", gameScore);
  gamePrefs.putUInt("catches", gameCatches);
  uint32_t day = gameDay(now);
  if (day != gameLastDay) {
    gameStreak = (day == gameLastDay + 1) ? gameStreak + 1 : 1;
    gameLastDay = day;
    gamePrefs.putInt("streak", gameStreak);
    gamePrefs.putUInt("lastday", gameLastDay);
  }
  Serial.printf("HUNT: +%d %s [%s x%d cmb x%d] (score %u)\n", pts, t->name, rarityName[rar], rarityMult[rar], cmb, gameScore);
}

void gameTick(uint32_t now) {
  for (int i = 0; i < tgtCount; i++) {
    Tgt *t = &tgts[i];
    if (t->type != wantType()) continue;
    if (now - t->last > FRESH_MS) { t->hotSince = 0; t->gameDone = false; continue; }
    if (t->gameDone) continue;
    if (t->rssi >= GAME_CATCH_RSSI) {
      if (t->hotSince == 0) {
        t->hotSince = now;
        t->peakRssi = (int8_t)t->rssi;
      } else {
        if (t->rssi > t->peakRssi) t->peakRssi = (int8_t)t->rssi;
        if (now - t->hotSince >= GAME_DWELL_MS) {
          gameCatch(i, now);
          t->gameDone = true;  // evaluated; waits for cold reset
          t->hotSince = 0;
        }
      }
    } else {
      t->hotSince = 0;  // dropped below threshold, dwell resets
    }
  }
  // persist uptime for day tracking (every 60s)
  if (now - gameLastUptimeSave > 60000) {
    gameLastUptimeSave = now;
    gamePrefs.putUInt("uptime", gameBaseUptime + now / 1000);
  }
}

void drawGameView() {
  static uint32_t lastDraw = 0;
  uint32_t now = millis();
  if (now - lastDraw < 500) return;
  lastDraw = now;
  lcdRect(0, 36, 320, 444, A_BLACK);
  lcdTextC(60, "HUNTER", 3, A_BRIGHT, A_BLACK);
  char buf[32];
  snprintf(buf, sizeof(buf), "SCORE %u", gameScore);
  lcdTextC(110, buf, 2, A_BRIGHT, A_BLACK);
  snprintf(buf, sizeof(buf), "CAUGHT %u", gameCatches);
  lcdTextC(140, buf, 2, A_MID, A_BLACK);
  snprintf(buf, sizeof(buf), "STREAK %d DAY%s", gameStreak, gameStreak == 1 ? "" : "S");
  lcdTextC(170, buf, 2, A_MID, A_BLACK);
  // combo display
  if (comboN >= 2) {
    snprintf(buf, sizeof(buf), "COMBO x%d", comboN >= 8 ? 5 : comboN >= 5 ? 4 : comboN >= 3 ? 3 : 2);
    lcdTextC(195, buf, 2, 0xFFE0, A_BLACK);
  }
  // badge unlock banner (shows for 3s)
  static uint32_t badgeBannerUntil = 0;
  static int badgeBannerIdx = -1;
  if (badgeQueue) {
    for (int b = 0; b < 14; b++) if (badgeQueue & (1UL << b)) {
      badgeBannerIdx = b;
      badgeQueue &= ~(1UL << b);
      badgeBannerUntil = now + 3000;
      break;
    }
  }
  if (badgeBannerIdx >= 0 && now < badgeBannerUntil) {
    lcdRect(20, 230, 280, 40, 0x07E0);
    lcdRect(22, 232, 276, 36, A_BLACK);
    snprintf(buf, sizeof(buf), "BADGE: %s", badgeNames[badgeBannerIdx]);
    lcdTextC(240, buf, 1, 0x07E0, A_BLACK);
  } else if (now >= badgeBannerUntil) {
    badgeBannerIdx = -1;
  }
  // badge count
  int bc = 0;
  for (int b = 0; b < 14; b++) if (badgeBits & (1UL << b)) bc++;
  snprintf(buf, sizeof(buf), "BADGES %d/14", bc);
  lcdTextC(420, buf, 1, A_DIM, A_BLACK);
  // recent catches: strongest fresh targets
  lcdTextC(210, "IN RANGE", 1, A_DIM, A_BLACK);
  static Tgt tmp[MAXTGT];
  int n = 0;
  for (int i = 0; i < tgtCount && n < MAXTGT; i++)
    if (tgts[i].type == wantType() && now - tgts[i].last <= FRESH_MS
        && tgts[i].rssi >= GAME_CATCH_RSSI - 15) tmp[n++] = tgts[i];
  // simple insertion sort by rssi desc
  for (int a = 1; a < n; a++) {
    Tgt key = tmp[a];
    int b = a - 1;
    while (b >= 0 && tmp[b].rssi < key.rssi) { tmp[b + 1] = tmp[b]; b--; }
    tmp[b + 1] = key;
  }
  int rows = n > 8 ? 8 : n;
  for (int r = 0; r < rows; r++) {
    int y = 230 + r * 23;
    bool caught = gameIsCaught(tmp[r].mac);
    lcdRect(8, y + 6, 8, 8, caught ? A_BRIGHT : A_DIM);
    char nm[26];
    snprintf(nm, sizeof(nm), "%s%.20s", caught ? "" : "+ ", tmp[r].name);
    lcdText(24, y + 4, nm, 1, caught ? A_MID : A_BRIGHT, A_BLACK);
    char rb[20];
    if (caught) snprintf(rb, sizeof(rb), "CAUGHT");
    else distText(rb, sizeof(rb), tmp[r].rssi);
    lcdText(320 - 8 - textW(rb, 1), y + 4, rb, 1, A_MID, A_BLACK);
  }
  if (n == 0) lcdTextC(250, "NOTHING CLOSE", 1, A_DIM, A_BLACK);
}

// ---- Waterfall: time (down) x device (across), brightness = RSSI ----
#define WF_COLS 40
#define WF_ROWS 150
#define WF_INTERVAL 2000
static int8_t wfBuf[WF_COLS * WF_ROWS];
static int wfHead = 0;
static uint32_t wfLastRow = 0;

void wfTick(uint32_t now) {
  if (now - wfLastRow < WF_INTERVAL) return;
  wfLastRow = now;
  wfHead = (wfHead + 1) % WF_ROWS;
  int8_t *row = &wfBuf[wfHead * WF_COLS];
  memset(row, 0x80, WF_COLS);  // -128 = no data
  // stable column per device (MAC hash), so a following contact holds its
  // column instead of shifting whenever the target list changes
  for (int i = 0; i < tgtCount; i++) {
    Tgt *t = &tgts[i];
    if (t->type != wantType() || now - t->last >= WF_INTERVAL * 2) continue;
    int c = (int)(macHash(t->mac) % WF_COLS);
    int8_t v = (int8_t)t->rssi;
    if (v > row[c]) row[c] = v;  // strongest wins on hash collision
  }
}

void drawWaterfall() {
  static uint32_t lastDraw = 0;
  uint32_t now = millis();
  if (now - lastDraw < 500) return;
  lastDraw = now;
  static uint8_t sline[320 * 2 * 2];  // 320px x 2 rows
  lcdRect(0, 36, 320, 310, A_BLACK);
  for (int r = 0; r < WF_ROWS; r++) {
    int srcRow = (wfHead - r + WF_ROWS * 2) % WF_ROWS;
    int y = 36 + r * 2;
    if (y + 2 > 346) break;
    for (int c = 0; c < WF_COLS; c++) {
      int8_t v = wfBuf[srcRow * WF_COLS + c];
      uint16_t col = A_BLACK;
      if (v != -128) {
        col = heatColor(v);
      }
      for (int px = 0; px < 8; px++) {
        sline[(px + c * 8) * 2] = col >> 8;
        sline[(px + c * 8) * 2 + 1] = col & 0xFF;
      }
    }
    memcpy(&sline[320 * 2], sline, 320 * 2);
    lcd->drawBitmap(0, y, 320, 2, sline);
  }
  lcdText(8, 348, "WATERFALL 2s/row", 1, A_DIM, A_BLACK);
}
// ---- Strip chart: RSSI over time for 6 strongest ----
#define SC_N 6
#define SC_SAMPLES 160
#define SC_INTERVAL 1000
static int8_t scBuf[SC_N][SC_SAMPLES];
static int scHead = 0;
static uint32_t scLast = 0;
static char scNames[SC_N][14];

void scTick(uint32_t now) {
  if (now - scLast < SC_INTERVAL) return;
  scLast = now;
  scHead = (scHead + 1) % SC_SAMPLES;
  // top 6 by RSSI
  int idx[SC_N];
  for (int k = 0; k < SC_N; k++) idx[k] = -1;
  for (int i = 0; i < tgtCount; i++) {
    if (tgts[i].type != wantType()) continue;
    if (now - tgts[i].last > FRESH_MS) continue;
    for (int k = 0; k < SC_N; k++) {
      if (idx[k] < 0 || tgts[i].rssi > tgts[idx[k]].rssi) {
        for (int j = SC_N - 1; j > k; j--) idx[j] = idx[j - 1];
        idx[k] = i;
        break;
      }
    }
  }
  for (int k = 0; k < SC_N; k++) {
    if (idx[k] >= 0) {
      scBuf[k][scHead] = (int8_t)tgts[idx[k]].rssi;
      strncpy(scNames[k], tgts[idx[k]].name, 13);
      scNames[k][13] = '\0';
    } else {
      scBuf[k][scHead] = -128;
      scNames[k][0] = '\0';
    }
  }
}

// Strip chart framebuffer: 320x310
static uint8_t *scFb = NULL;
static inline void scPx(int x, int y, uint16_t c) {
  if (!scFb || (unsigned)x >= 320 || y < 36 || y >= 346) return;
  int i = ((y - 36) * 320 + x) * 2;
  scFb[i] = c >> 8; scFb[i + 1] = c & 0xFF;
}

void drawStripChart() {
  static uint32_t lastDraw = 0;
  uint32_t now = millis();
  if (now - lastDraw < 500) return;
  lastDraw = now;
  if (!scFb) {
    scFb = (uint8_t *)heap_caps_malloc(320 * 310 * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!scFb) return;
  }
  memset(scFb, 0, 320 * 310 * 2);
  const uint16_t lineCol[SC_N] = {0xFFE0, 0x07FF, 0xF81F, 0xF800, 0x07E0, 0x001F};
  // gridlines
  for (int g = 0; g <= 3; g++) {
    int y = 330 - g * 280 / 3;
    for (int x = 30; x < 320; x += 4) { scPx(x, y, A_FAINT); scPx(x+1, y, A_FAINT); }
  }
  // data lines via framebuffer
  for (int k = 0; k < SC_N; k++) {
    int px = -1, py = -1;
    for (int s = 0; s < SC_SAMPLES; s++) {
      int src = (scHead - SC_SAMPLES + 1 + s + SC_SAMPLES * 2) % SC_SAMPLES;
      int8_t v = scBuf[k][src];
      int x = 30 + s * 290 / SC_SAMPLES;
      if (v == -128) { px = -1; continue; }
      int y = 330 - (v + 100) * 280 / 60;
      if (y < 50) y = 50;
      if (y > 330) y = 330;
      if (px >= 0) {
        int dx = x - px, dy = y - py;
        int steps = abs(dx) > abs(dy) ? abs(dx) : abs(dy);
        if (steps == 0) steps = 1;
        if (steps > 50) steps = 50;  // cap runaway segments
        for (int st = 0; st <= steps; st++) {
          int lx = px + dx * st / steps;
          int ly = py + dy * st / steps;
          scPx(lx, ly, lineCol[k]);
          scPx(lx+1, ly, lineCol[k]);
        }
      }
      px = x; py = y;
    }
  }
  // push framebuffer in bands
  for (int y = 0; y < 310; y += 16) {
    int h = 310 - y;
    if (h > 16) h = 16;
    lcd->drawBitmap(0, 36 + y, 320, h, &scFb[(y * 320) * 2]);
  }
  // labels (drawn on screen, not in fb)
  for (int g = 0; g <= 3; g++) {
    int y = 330 - g * 280 / 3;
    char lb[8];
    snprintf(lb, sizeof(lb), "%d", -100 + g * 20);
    lcdText(2, y - 4, lb, 1, A_DIM, A_BLACK);
  }
  for (int k = 0; k < SC_N; k++) {
    if (scNames[k][0]) lcdText(32 + k * 48, 336, scNames[k], 1, lineCol[k], A_BLACK);
  }
}

// ---- Density grid: bearing sectors x distance rings ----
#define DG_SEC 12
#define DG_RING 4
static int dgGrid[DG_SEC][DG_RING];

void dgTick(uint32_t now) {
  static uint32_t lastDg = 0;
  if (now - lastDg < 3000) return;
  lastDg = now;
  memset(dgGrid, 0, sizeof(dgGrid));
  for (int i = 0; i < tgtCount; i++) {
    if (tgts[i].type != wantType()) continue;
    if (now - tgts[i].last > FRESH_MS) continue;
    int sec = tgtAngle(i) / 30;
    if (sec >= DG_SEC) sec = DG_SEC - 1;
    float d = bleDistM(tgts[i].rssi);
    int ring = d < 3 ? 0 : d < 10 ? 1 : d < 30 ? 2 : 3;
    dgGrid[sec][ring]++;
  }
}

void drawDensity() {
  static uint32_t lastDraw = 0;
  uint32_t now = millis();
  if (now - lastDraw < 1000) return;
  lastDraw = now;
  lcdRect(0, 36, 320, 310, A_BLACK);
  const char *ringLb[DG_RING] = {"<3m", "<10m", "<30m", "30m+"};
  const char *secLb[DG_SEC] = {"N","30","60","E","120","150","S","210","240","W","300","330"};
  int cw = 320 / DG_SEC;  // 26
  int ch = 60;
  int y0 = 70;
  int maxc = 1;
  for (int s = 0; s < DG_SEC; s++)
    for (int r = 0; r < DG_RING; r++)
      if (dgGrid[s][r] > maxc) maxc = dgGrid[s][r];
  for (int r = 0; r < DG_RING; r++) {
    lcdText(4, y0 + r * ch + 24, ringLb[r], 1, A_DIM, A_BLACK);
    for (int s = 0; s < DG_SEC; s++) {
      int x = s * cw;
      int y = y0 + r * ch;
      int n = dgGrid[s][r];
      uint16_t col = n == 0 ? A_BLACK : heatColor(-100 + n * 60 / maxc);
      lcdRect(x + 1, y + 1, cw - 2, ch - 2, col);
      if (n > 0) {
        char nb[8];
        snprintf(nb, sizeof(nb), "%d", n);
        lcdText(x + cw/2 - 3, y + ch/2 - 4, nb, 1, A_BLACK, col);
      }
      if (r == 0) lcdText(x + cw/2 - 6, y0 - 14, secLb[s], 1, A_DIM, A_BLACK);
    }
  }
  lcdText(8, 330, "RF DENSITY", 1, A_DIM, A_BLACK);
}

static float sweepDeg = 0;
static char lastTop[64] = "";

static int dbIdx[6];
static int dbN = 0;

void pickDataBlocks(uint32_t now) {
  dbN = 0;
  for (int i = 0; i < tgtCount; i++) {
    if (tgts[i].type != wantType()) continue;
    if (now - tgts[i].last > FRESH_MS) continue;
    int pos = dbN;
    for (int k = 0; k < dbN; k++)
      if (tgts[i].rssi > tgts[dbIdx[k]].rssi) { pos = k; break; }
    if (dbN < 6) {
      for (int k = dbN; k > pos; k--) dbIdx[k] = dbIdx[k - 1];
      dbIdx[pos] = i;
      dbN++;
    } else if (pos < 6) {
      for (int k = 5; k > pos; k--) dbIdx[k] = dbIdx[k - 1];
      dbIdx[pos] = i;
    }
  }
}

void renderScope(uint32_t now) {
  fbClear();
  const int rings[3] = {22, 72, 122};
  for (int k = 0; k < 3; k++) fbCircle(SCP_C, SCP_C, rings[k], A_DIM);
  fbLine(SCP_C - SCP_R, SCP_C, SCP_C + SCP_R, SCP_C, A_FAINT);
  fbLine(SCP_C, SCP_C - SCP_R, SCP_C, SCP_C + SCP_R, A_FAINT);
  for (int d = 0; d < 360; d += 30) {
    int x0, y0, x1, y1;
    bearingXY(d, 138, &x0, &y0);
    bearingXY(d, 146, &x1, &y1);
    fbLine(x0, y0, x1, y1, A_DIM);
  }
  for (int k = 3; k >= 1; k--) {
    int x1, y1;
    bearingXY((int)sweepDeg - k * 7, SCP_R, &x1, &y1);
    fbLine(SCP_C, SCP_C, x1, y1, dimColor(A_BRIGHT, 30 + (3 - k) * 40));
  }
  {
    int x1, y1;
    bearingXY((int)sweepDeg, SCP_R, &x1, &y1);
    fbLine(SCP_C, SCP_C, x1, y1, A_BRIGHT);
  }
  for (int i = 0; i < tgtCount; i++) {
    if (tgts[i].type != wantType()) continue;
    uint32_t age = now - tgts[i].last;
    if (age > STALE_MS) continue;
    int bright = age < 3000 ? 255 : (int)(255 - (age - 3000) * 195 / 42000);
    if (bright < 60) bright = 60;
    int ang = tgtAngle(i);
    float diff = fabsf(sweepDeg - ang);
    if (diff > 180) diff = 360 - diff;
    bool ping = diff < 10;
    if (ping) bright = 255;
    int r = distToR(bleDistM((int)tgts[i].smoothRssi));
    int bx, by;
    bearingXY(ang, r, &bx, &by);
    int sz = ping ? 4 : 3;
    uint16_t dotCol = tgts[i].watched ? 0xFDA0 : heatColor((int)tgts[i].smoothRssi);
    fbDisc(bx, by, sz, dimColor(dotCol, bright));
    if (ping) fbCircle(bx, by, sz + 3, dimColor(A_WHITE, bright));
  }
  pickDataBlocks(now);
  static int tagX[6], tagY[6];
  static bool tagOk[6];
  for (int k = 0; k < dbN; k++) {
    int i = dbIdx[k];
    int ang = tgtAngle(i);
    int r = distToR(bleDistM((int)tgts[i].smoothRssi));
    int bx, by, tx, ty;
    bearingXY(ang, r, &bx, &by);
    bearingXY(ang, r + 30, &tx, &ty);
    if (tx < 16) tx = 16;
    if (tx > SCP_S - 86) tx = SCP_S - 86;
    if (ty < 8) ty = 8;
    if (ty > SCP_S - 30) ty = SCP_S - 30;
    bool clear = false;
    for (int tries = 0; tries < 24; tries++) {
      clear = true;
      for (int j = 0; j < k; j++)
        if (tagOk[j] && abs(tx - tagX[j]) < 88 && abs(ty - tagY[j]) < 26) { clear = false; break; }
      if (clear) break;
      ty += 26;
      if (ty > SCP_S - 30) ty = 8;
    }
    tagOk[k] = clear;
    if (!clear) continue;
    tagX[k] = tx;
    tagY[k] = ty;
    fbLine(bx, by, tx + 4, ty + 8, A_DIM);
    char nm[14];
    strncpy(nm, tgts[i].name, 13);
    nm[13] = '\0';
    fbText(tx, ty, nm, A_BRIGHT);
    char ln2[20];
    distText(ln2, sizeof(ln2), tgts[i].rssi);
    fbText(tx, ty + 10, ln2, A_MID);
  }
  fbDisc(SCP_C, SCP_C, 3, A_BRIGHT);
  for (int k = 0; k < 3; k++) {
    int lx, ly;
    bearingXY(315, rings[k], &lx, &ly);
    lx -= 8;
    ly -= 4;
    fbText(lx, ly, k == 0 ? "1m" : k == 1 ? "10m" : "100m", A_DIM);
  }
  fbText(SCP_C - 3, 4, "N", A_DIM);
  fbText(SCP_C - 3, SCP_S - 8, "S", A_DIM);
  fbText(4, SCP_C - 4, "W", A_DIM);
  fbText(SCP_S - 7, SCP_C - 4, "E", A_DIM);
#define SCP_BANDH 20
  for (int b = 0; b < SCP_S / SCP_BANDH; b++)
    lcd->drawBitmap(SCP_X, SCP_Y + b * SCP_BANDH, SCP_S, SCP_BANDH,
                    &scpFb[(b * SCP_BANDH * SCP_S) * 2]);
}

void battSample() {
  int mv = analogReadMilliVolts(PIN_BATT);
  float v = mv / 1000.0f * 2.0f; // R14/R15 100k/100k
  if (battV == 0) battV = v;
  else battV = battV * 0.7f + v * 0.3f;
  battPresent = battV > 2.80f && battV < 4.40f;
}

// ---- Watchlist alerts ----
#define ALERT_MS 8000  // how long banner shows
static char alertMsg[48] = "";
static uint32_t alertUntil = 0;
static uint16_t alertColor = 0xFFFF;

static void raiseAlert(const char *msg, uint16_t color) {
  strncpy(alertMsg, msg, sizeof(alertMsg) - 1);
  alertMsg[sizeof(alertMsg) - 1] = '\0';
  alertColor = color;
  alertUntil = millis() + ALERT_MS;
  Serial.printf("ALERT: %s\n", msg);
}

void watchTick(uint32_t now) {
  static uint32_t lastCheck = 0;
  if (now - lastCheck < 2000) return;
  lastCheck = now;
  for (int i = 0; i < tgtCount; i++) {
    if (!tgts[i].watched) continue;
    bool present = (now - tgts[i].last) < 15000;
    bool wasPresent = tgts[i].alertPresent;
    if (present && !wasPresent) {
      char msg[48];
      snprintf(msg, sizeof(msg), "+ %s HERE", tgts[i].name);
      raiseAlert(msg, 0x07E0);  // green = arrived
      tgts[i].alertPresent = true;
    } else if (!present && wasPresent && (now - tgts[i].last) > 30000) {
      char msg[48];
      snprintf(msg, sizeof(msg), "- %s GONE", tgts[i].name);
      raiseAlert(msg, 0xF800);  // red = left
      tgts[i].alertPresent = false;
    }
  }
}

void drawAlertBanner() {
  uint32_t now = millis();
  if ((int32_t)(alertUntil - now) <= 0 || !alertMsg[0]) return;
  // banner below top bar
  int w = textW(alertMsg, 2) + 16;
  if (w < 120) w = 120;
  int x = (320 - w) / 2;
  lcdRect(x, 36, w, 28, A_BLACK);
  // border
  lcdRect(x, 36, w, 1, alertColor);
  lcdRect(x, 63, w, 1, alertColor);
  lcdRect(x, 36, 1, 28, alertColor);
  lcdRect(x + w - 1, 36, 1, 28, alertColor);
  lcdText(x + 8, 42, alertMsg, 2, alertColor, A_BLACK);
}

void drawTopBar() {
  char buf[64];
  uint32_t s = millis() / 3000;
  const char *title = viewMode == 0 ? "RAIN" :
                      viewMode == 2 ? "LIST" : viewMode == 3 ? "HUNTER" :
                      viewMode == 4 ? "WATERFALL" :
                      viewMode == 5 ? "STRIP" : viewMode == 6 ? "DENSITY" :
                      viewMode == 7 ? "DETAIL" :
                      viewMode == 8 ? "GPS" :
                      viewMode == 9 ? "TRACK" :
                      viewMode == 10 ? "FIND" :
                      viewMode == 11 ? "CREDITS" :
                      bandName();
  int cnt = freshCount(millis());
  if (battPresent)
    snprintf(buf, sizeof(buf), "%s %c%d %.1fV %02d:%02d", title, bandChar(),
             cnt, battV, (int)(s / 60), (int)(s % 60));
  else
    snprintf(buf, sizeof(buf), "%s %c%d USB %02d:%02d", title, bandChar(),
             cnt, (int)(s / 60), (int)(s % 60));
  if (strcmp(buf, lastTop) == 0) return;
  strcpy(lastTop, buf);
  lcdRect(0, 0, 320, 34, A_BLACK);
  lcdText(8, 8, title, 2, A_BRIGHT, A_BLACK);
  char rb[32];
  if (battPresent)
    snprintf(rb, sizeof(rb), "%c%d %.1fV %02d:%02d", bandChar(), cnt,
             battV, (int)(s / 60), (int)(s % 60));
  else
    snprintf(rb, sizeof(rb), "%c%d USB %02d:%02d", bandChar(), cnt,
             (int)(s / 60), (int)(s % 60));
  lcdText(320 - 8 - textW(rb, 1), 12, rb, 1, A_MID, A_BLACK);
}

void drawStrip() {
  for (int row = 0; row < 4; row++)
    lcdRect(0, 340 + row * 32, 236, 30, A_BLACK);
  int shown = 0;
  uint32_t now = millis();
  for (int k = 0; k < mruN && shown < 4; k++) {
    Tgt *t = &tgts[mru[k]];
    if (t->type != wantType()) continue;
    if (now - t->last > STALE_MS) continue;
    int y = 340 + shown * 32;
    lcdRect(8, y + 10, 8, 8, A_MID);
    char nm[22];
    bool fresh = (now - t->born) < 60000;
    snprintf(nm, sizeof(nm), "%s%.18s", fresh ? "+" : "", t->name);
    lcdText(24, y + 10, nm, 1, A_BRIGHT, A_BLACK);
    char rb[20];
    distText(rb, sizeof(rb), t->rssi);
    lcdText(236 - 8 - textW(rb, 1), y + 10, rb, 1, A_MID, A_BLACK);
    shown++;
  }
}

static int tgtCmp(const void *a, const void *b) {
  return ((Tgt *)b)->rssi - ((Tgt *)a)->rssi;
}


// ---- OUI vendor lookup (first 3 MAC bytes -> vendor) ----
typedef struct { uint32_t oui; const char *name; } OuiEntry;
static const OuiEntry OUI_TABLE[] = {
  {0x246F28, "Espressif"}, {0x240AC4, "Espressif"}, {0x30C6F7, "Espressif"},
  {0x7CDFA1, "Espressif"}, {0xA4CF12, "Espressif"}, {0x8CAA1B, "Espressif"},
  {0xB827EB, "RaspberryPi"}, {0xDCA632, "RaspberryPi"}, {0xE45F01, "RaspberryPi"},
  {0x28ED6A, "Apple"}, {0x3C0630, "Apple"}, {0x8C8590, "Apple"},
  {0xF0D1A9, "Apple"}, {0xA4B197, "Apple"}, {0xD0C5F3, "Apple"},
  {0x8CF5A3, "Samsung"}, {0xB46CE9, "Samsung"}, {0xE8B4A8, "Samsung"},
  {0xD43639, "Nordic"}, {0xE7DB66, "Nordic"}, {0xF4CE73, "Nordic"},
  {0x001237, "TI"}, {0x78A504, "TI"}, {0xB0B448, "TI"},
  {0x000272, "Dell"}, {0xF8B156, "Dell"}, {0x847BEB, "Dell"},
  {0x001E67, "Intel"}, {0x3CA9F4, "Intel"}, {0xDC7196, "Intel"},
  {0x001A7D, "Cisco"}, {0x00235E, "Cisco"}, {0x6C416A, "Cisco"},
  {0x001B77, "HTC"}, {0xCC7E39, "LG"}, {0xA0E9DB, "Huawei"},
  {0x0C8BFD, "Google"}, {0xF4F5DB, "Google"}, {0x3C5A37, "Google"},
  {0x50C7BF, "Amazon"}, {0x44EADA, "Amazon"},
  {0xD8A0CB, "Fitbit"}, {0xE9B0CE, "Tile"},
  {0x5C6B4F, "Xiaomi"}, {0x64CC2E, "Xiaomi"}, {0x7CC2C6, "Xiaomi"},
  {0xAC67B2, "Sonos"}, {0xB8E62E, "Sonos"},
  {0x00142A, "Garmin"}, {0x000E9D, "Polar"},
};
#define OUI_COUNT (sizeof(OUI_TABLE)/sizeof(OUI_TABLE[0]))

static const char *ouiLookup(const char *mac) {
  // mac format "AA:BB:CC:DD:EE:FF"
  if (!mac || strlen(mac) < 8) return NULL;
  unsigned int b0, b1, b2;
  if (sscanf(mac, "%02x:%02x:%02x", &b0, &b1, &b2) != 3) return NULL;
  uint32_t oui = (b0 << 16) | (b1 << 8) | b2;
  for (unsigned int i = 0; i < OUI_COUNT; i++)
    if (OUI_TABLE[i].oui == oui) return OUI_TABLE[i].name;
  return NULL;
}


void drawDetailView() {
  static uint32_t lastDraw = 0;
  uint32_t now = millis();
  if (now - lastDraw < 500) return;
  lastDraw = now;
  lcdRect(0, 36, 320, 444, A_BLACK);
  // strongest fresh contact
  Tgt *best = NULL;
  for (int i = 0; i < tgtCount; i++) {
    if (tgts[i].type != wantType()) continue;
    if (now - tgts[i].last > STALE_MS) continue;
    if (!best || tgts[i].rssi > best->rssi) best = &tgts[i];
  }
  if (!best) {
    lcdTextC(220, "NO CONTACT", 2, A_DIM, A_BLACK);
    return;
  }
  int y = 50;
  lcdText(16, y, "CONTACT DETAIL", 3, A_BRIGHT, A_BLACK); y += 36;
  const char *vend = ouiLookup(best->mac);
  char line[48];
  snprintf(line, sizeof(line), "MAC %s", best->mac);
  lcdText(16, y, line, 2, A_MID, A_BLACK); y += 20;
  if (vend) {
    snprintf(line, sizeof(line), "Vendor: %s", vend);
    lcdText(16, y, line, 2, A_MID, A_BLACK); y += 20;
  }
  if (best->name[0]) {
    snprintf(line, sizeof(line), "Name: %.30s", best->name);
    lcdText(16, y, line, 2, A_MID, A_BLACK); y += 20;
  }
  y += 8;
  snprintf(line, sizeof(line), "RSSI now %d dBm", best->rssi);
  lcdText(16, y, line, 2, A_BRIGHT, A_BLACK); y += 20;
  snprintf(line, sizeof(line), "Min %d  Max %d dBm", best->minRssi, best->maxRssi);
  lcdText(16, y, line, 2, A_MID, A_BLACK); y += 20;
  snprintf(line, sizeof(line), "Packets: %u", best->hits);
  lcdText(16, y, line, 2, A_MID, A_BLACK); y += 20;
  uint32_t ageS = (now - best->born) / 1000;
  snprintf(line, sizeof(line), "First seen %us ago", ageS);
  lcdText(16, y, line, 2, A_MID, A_BLACK); y += 20;
  char db[20];
  distText(db, sizeof(db), best->rssi);
  snprintf(line, sizeof(line), "Est: %s", db);
  lcdText(16, y, line, 2, A_MID, A_BLACK); y += 28;
  if (best->watched)
    lcdText(16, y, "WATCHLISTED", 2, 0xFDA0, A_BLACK);
}


// ---- GPS: NEO-6M on IO45 (RX) / IO46 (TX), 9600 baud ----
#define GPS_RX 45
#define GPS_TX 46
static HardwareSerial GPSSerial(2);

static char gpsTimeStr[10] = "--:--:--";
static uint32_t gpsLastSentence = 0;

static double nmeaToDeg(const char *s) {
  // ddmm.mmmm -> decimal degrees
  double v = atof(s);
  int deg = (int)(v / 100);
  double min = v - deg * 100;
  return deg + min / 60.0;
}

static void gpsParseLine(char *line) {
  // checksum check
  char *star = strchr(line, '*');
  if (!star) return;
  uint8_t cs = 0;
  for (char *p = line + 1; p < star; p++) cs ^= *p;
  unsigned int sent = 0;
  if (sscanf(star + 1, "%02x", &sent) != 1 || (uint8_t)sent != cs) return;
  *star = '\0';

  if (strncmp(line, "$GPGGA", 6) == 0 || strncmp(line, "$GNGGA", 6) == 0) {
    // $GPGGA,time,lat,NS,lon,EW,fix,sats,hdop,alt,M,...
    char *tok = strtok(line, ",");
    int field = 0;
    char *timeS = NULL, *latS = NULL, *ns = NULL, *lonS = NULL, *ew = NULL;
    char *fixS = NULL, *satS = NULL, *altS = NULL;
    while (tok) {
      field++;
      if (field == 2) timeS = tok;
      else if (field == 3) latS = tok;
      else if (field == 4) ns = tok;
      else if (field == 5) lonS = tok;
      else if (field == 6) ew = tok;
      else if (field == 7) fixS = tok;
      else if (field == 8) satS = tok;
      else if (field == 10) altS = tok;
      tok = strtok(NULL, ",");
    }
    int fix = fixS ? atoi(fixS) : 0;
    gpsFix = fix > 0;
    if (gpsFix) {
      gpsLat = nmeaToDeg(latS ? latS : "0");
      if (ns && *ns == 'S') gpsLat = -gpsLat;
      gpsLon = nmeaToDeg(lonS ? lonS : "0");
      if (ew && *ew == 'W') gpsLon = -gpsLon;
      gpsSats = satS ? atoi(satS) : 0;
      gpsAltM = altS ? atof(altS) : 0;
      if (timeS && strlen(timeS) >= 6) {
        gpsTimeStr[0] = timeS[0]; gpsTimeStr[1] = timeS[1]; gpsTimeStr[2] = ':';
        gpsTimeStr[3] = timeS[2]; gpsTimeStr[4] = timeS[3]; gpsTimeStr[5] = ':';
        gpsTimeStr[6] = timeS[4]; gpsTimeStr[7] = timeS[5]; gpsTimeStr[8] = '\0';
      }
    }
    gpsLastSentence = millis();
  } else if (strncmp(line, "$GPRMC", 6) == 0 || strncmp(line, "$GNRMC", 6) == 0) {
    // $GPRMC,time,status,lat,NS,lon,EW,speed_knots,course,date,...
    char *tok = strtok(line, ",");
    int field = 0;
    char *spd = NULL;
    while (tok) {
      field++;
      if (field == 2 && strlen(tok) >= 2) {
        char hh[3] = {tok[0], tok[1], 0};
        gpsUtcHour = atoi(hh);
      }
      if (field == 8) spd = tok;
      tok = strtok(NULL, ",");
    }
    if (spd) gpsSpeedKmh = atof(spd) * 1.852f;
  }
}

static void gpsTick() {
  static char lineBuf[100];
  static int linePos = 0;
  while (GPSSerial.available()) {
    char c = GPSSerial.read();
    if (c == '\n' || c == '\r') {
      if (linePos > 10) {
        lineBuf[linePos] = '\0';
        gpsParseLine(lineBuf);
      }
      linePos = 0;
    } else if (linePos < (int)sizeof(lineBuf) - 1) {
      lineBuf[linePos++] = c;
    } else {
      linePos = 0;  // overflow, reset
    }
  }
}

// ---- Direction finding: hot/cold ----
static int findIdx = -1;  // currently tracked target

void findTick(uint32_t now) {
  // update trend snapshots every 5s
  for (int i = 0; i < tgtCount; i++) {
    if (now - tgts[i].trendAt > 5000) {
      tgts[i].trendBase = (int)tgts[i].smoothRssi;
      tgts[i].trendAt = now;
    }
  }
  // pick target: strongest tracker > strongest watched > strongest overall
  int best = -1;
  for (int pass = 0; pass < 3 && best < 0; pass++) {
    for (int i = 0; i < tgtCount; i++) {
      if (tgts[i].type != wantType()) continue;
      if (now - tgts[i].last > 15000) continue;
      bool ok = (pass == 0 && tgts[i].tracker != TRK_NONE) ||
                (pass == 1 && tgts[i].watched) ||
                (pass == 2);
      if (!ok) continue;
      if (best < 0 || tgts[i].rssi > tgts[best].rssi) best = i;
    }
  }
  findIdx = best;
}

void drawFindView() {
  static uint32_t lastDraw = 0;
  uint32_t now = millis();
  if (now - lastDraw < 250) return;
  lastDraw = now;
  lcdRect(0, 36, 320, 310, A_BLACK);
  if (findIdx < 0 || now - tgts[findIdx].last > 15000) {
    lcdText(16, 60, "FIND MODE", 2, A_BRIGHT, A_BLACK);
    lcdText(16, 110, "No target. Walk", 1, A_MID, A_BLACK);
    lcdText(16, 130, "around to pick up", 1, A_MID, A_BLACK);
    lcdText(16, 150, "a signal.", 1, A_MID, A_BLACK);
    return;
  }
  Tgt *t = &tgts[findIdx];
  int y = 44;
  // target name
  const char *kind = t->tracker != TRK_NONE ? TRK_NAMES[t->tracker] :
                     t->watched ? "WATCHED" : "STRONGEST";
  lcdText(16, y, kind, 1, t->tracker != TRK_NONE ? TRK_COLORS[t->tracker] : 0xFDA0, A_BLACK);
  y += 22;
  char nm[36];
  snprintf(nm, sizeof(nm), "%.17s", t->mac);
  lcdText(16, y, nm, 1, A_MID, A_BLACK);
  y += 28;

  // big RSSI number, heat colored
  char rssi[16];
  snprintf(rssi, sizeof(rssi), "%d", t->rssi);
  lcdText(16, y, rssi, 4, heatColor(t->rssi), A_BLACK);
  lcdText(150, y + 28, "dBm", 1, A_DIM, A_BLACK);
  y += 70;

  // trend arrow
  int delta = (int)t->smoothRssi - t->trendBase;
  const char *arrow;
  uint16_t arrowCol;
  const char *word;
  if (delta > 3) { arrow = "^^^"; arrowCol = 0xF800; word = "HOTTER - keep going"; }
  else if (delta < -3) { arrow = "vvv"; arrowCol = 0x001F; word = "COLDER - turn around"; }
  else { arrow = "---"; arrowCol = A_MID; word = "HOLD - steady signal"; }
  lcdText(16, y, arrow, 3, arrowCol, A_BLACK);
  y += 48;
  lcdText(16, y, word, 1, arrowCol, A_BLACK);
  y += 28;

  // signal bar
  int barW = (t->rssi + 100) * 288 / 60;
  if (barW < 0) barW = 0;
  if (barW > 288) barW = 288;
  lcdRect(16, y, barW, 16, heatColor(t->rssi));
  lcdRect(16 + barW, y, 288 - barW, 16, A_FAINT);
  y += 28;

  // distance
  char dist[32];
  snprintf(dist, sizeof(dist), "~%.1f meters", bleDistM(t->rssi));
  lcdText(16, y, dist, 2, A_BRIGHT, A_BLACK);
  y += 40;
  lcdText(16, y, "Rotate slowly.", 2, A_DIM, A_BLACK);
  lcdText(16, y + 18, "Signal peaks when", 2, A_DIM, A_BLACK);
  lcdText(16, y + 36, "facing the target.", 2, A_DIM, A_BLACK);
}

void drawTrackerView() {
  static uint32_t lastDraw = 0;
  uint32_t now = millis();
  if (now - lastDraw < 1000) return;
  lastDraw = now;
  lcdRect(0, 36, 320, 310, A_BLACK);
  int y = 44;
  lcdText(16, y, "TRACKER SCAN", 2, 0xFFFF, A_BLACK); y += 34;
  if (bandMode != BAND_BLE) {
    lcdText(16, y + 20, "Switch to BLE band", 1, A_MID, A_BLACK);
    lcdText(16, y + 40, "for tracker scan.", 1, A_MID, A_BLACK);
    return;
  }

  int found = 0;
  for (int i = 0; i < tgtCount && y < 320; i++) {
    if (tgts[i].tracker == TRK_NONE) continue;
    if (tgts[i].type != TYPE_BLE) continue;
    if (now - tgts[i].last > 30000) continue;  // only recent
    found++;
    uint16_t col = TRK_COLORS[tgts[i].tracker];
    // tracker type badge
    lcdText(16, y, TRK_NAMES[tgts[i].tracker], 1, col, A_BLACK);
    // MAC
    lcdText(110, y, tgts[i].mac, 1, A_MID, A_BLACK);
    y += 16;
    // RSSI bar + distance
    int barW = (tgts[i].rssi + 100) * 180 / 60;
    if (barW < 0) barW = 0;
    if (barW > 180) barW = 180;
    lcdRect(16, y, barW, 8, col);
    lcdRect(16 + barW, y, 180 - barW, 8, A_FAINT);
    char info[32];
    snprintf(info, sizeof(info), "%d dBm ~%.0fm", tgts[i].rssi, bleDistM(tgts[i].rssi));
    lcdText(202, y - 2, info, 1, A_BRIGHT, A_BLACK);
    y += 20;
  }
  if (found == 0) {
    lcdText(16, y + 20, "No trackers detected.", 1, A_MID, A_BLACK);
    y += 44;
    lcdText(16, y, "AirTag/Tile/SmartTag", 1, A_DIM, A_BLACK);
    y += 20;
    lcdText(16, y, "will appear here.", 1, A_DIM, A_BLACK);
  } else {
    char cnt[24];
    snprintf(cnt, sizeof(cnt), "%d tracker%s nearby", found, found == 1 ? "" : "s");
    lcdText(16, 326, cnt, 1, 0xF800, A_BLACK);
  }
}

void drawGpsView() {
  static uint32_t lastDraw = 0;
  uint32_t now = millis();
  if (now - lastDraw < 500) return;
  lastDraw = now;
  lcdRect(0, 36, 320, 444, A_BLACK);
  int y = 50;
  lcdText(16, y, "GPS", 3, A_BRIGHT, A_BLACK); y += 36;
  char line[48];
  if (!gpsFix) {
    bool heard = (now - gpsLastSentence) < 5000;
    lcdText(16, y, heard ? "SEARCHING..." : "NO GPS DATA", 2,
            heard ? A_MID : 0xF800, A_BLACK);
    y += 24;
    snprintf(line, sizeof(line), "Check wiring: TX->45 RX->46");
    lcdText(16, y, line, 2, A_DIM, A_BLACK);
    return;
  }
  snprintf(line, sizeof(line), "Lat %.6f", gpsLat);
  lcdText(16, y, line, 2, A_BRIGHT, A_BLACK); y += 24;
  snprintf(line, sizeof(line), "Lon %.6f", gpsLon);
  lcdText(16, y, line, 2, A_BRIGHT, A_BLACK); y += 24;
  y += 8;
  snprintf(line, sizeof(line), "Sats %d  Alt %.0fm", gpsSats, gpsAltM);
  lcdText(16, y, line, 2, A_MID, A_BLACK); y += 24;
  snprintf(line, sizeof(line), "Speed %.1f km/h", gpsSpeedKmh);
  lcdText(16, y, line, 2, A_MID, A_BLACK); y += 24;
  snprintf(line, sizeof(line), "UTC %s", gpsTimeStr);
  lcdText(16, y, line, 2, A_MID, A_BLACK);
}

void drawListView() {
  static uint32_t lastDraw = 0;
  uint32_t now = millis();
  if (now - lastDraw < 500) return;
  lastDraw = now;
  static Tgt tmp[MAXTGT];
  int n = 0;
  for (int i = 0; i < tgtCount && n < MAXTGT; i++)
    if (tgts[i].type == wantType() && now - tgts[i].last <= STALE_MS) tmp[n++] = tgts[i];
  qsort(tmp, n, sizeof(Tgt), tgtCmp);
  lcdRect(0, 36, 320, 444, A_BLACK);
  if (n == 0) {
    lcdTextC(220, "NO CONTACTS", 3, A_DIM, A_BLACK);
    return;
  }
  int rows = n > 19 ? 19 : n;
  for (int r = 0; r < rows; r++) {
    int y = 40 + r * 23;
    lcdRect(8, y + 6, 8, 8, tmp[r].watched ? 0xFDA0 : A_MID);
    char nm[40];
    bool fresh = (now - tmp[r].born) < 60000;
    const char *vend = ouiLookup(tmp[r].mac);
    if (vend)
      snprintf(nm, sizeof(nm), "%s%s (%s)", fresh ? "+" : "", tmp[r].mac, vend);
    else
      snprintf(nm, sizeof(nm), "%s%.22s", fresh ? "+" : "", tmp[r].name);
    lcdText(24, y + 4, nm, 1, A_BRIGHT, A_BLACK);
    char rb[20];
    distText(rb, sizeof(rb), tmp[r].rssi);
    lcdText(320 - 8 - textW(rb, 1), y + 4, rb, 1, A_MID, A_BLACK);
  }
}

static bool bootWasDown = false;
static uint32_t bootDownT = 0;

void bootPoll() {
  bool down = digitalRead(PIN_BOOT) == LOW;
  uint32_t now = millis();
  if (down && !bootWasDown) bootDownT = now;
  if (!down && bootWasDown) {
    uint32_t held = now - bootDownT;
    if (held >= 600) {
      // cycle BLE -> ESP -> WIFI -> BLE
      if (bandMode == BAND_BLE) hunterStart();
      else if (bandMode == BAND_ESP) { hunterStop(); wifiStart(); bandMode = BAND_WIFI; }
      else if (bandMode == BAND_WIFI) { wifiStop(); bandMode = BAND_BLE; }
    } else {
      viewMode = (viewMode + 1) % 12;
    }
    lcdFill(A_BLACK);
    lastTop[0] = '\0';
  }
  bootWasDown = down;
}

// ---- Position broadcast: UDP to base station ----
// Set your WiFi credentials to enable. Leave blank to disable.
#define TRACK_SSID " "
#define TRACK_PASS " "
#define TRACK_PORT 4210
static WiFiUDP trackUdp;
static bool trackUp = false;
static uint32_t trackLast = 0;

void trackBegin() {
  if (!TRACK_SSID[0] || !TRACK_PASS[0]) return;
  WiFi.begin(TRACK_SSID, TRACK_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) delay(200);
  if (WiFi.status() == WL_CONNECTED) {
    trackUp = true;
    Serial.printf("track: connected %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("track: wifi failed");
  }
}

void trackTick(uint32_t now) {
  if (!TRACK_SSID[0] || !TRACK_PASS[0]) return;
  // self-healing: reconnect if band switch dropped us
  static uint32_t retryAt = 0;
  if (WiFi.status() != WL_CONNECTED) {
    trackUp = false;
    if ((int32_t)(now - retryAt) >= 0) {
      retryAt = now + 30000;
      WiFi.mode(WIFI_STA);
      WiFi.begin(TRACK_SSID, TRACK_PASS);
    }
    return;
  }
  trackUp = true;
  if (!gpsFix) return;
  if (now - trackLast < 5000) return;
  trackLast = now;
  char msg[96];
  snprintf(msg, sizeof(msg), "POS,%.6f,%.6f,%d,%.1f,%s",
           gpsLat, gpsLon, gpsSats, gpsSpeedKmh, WiFi.macAddress().c_str());
  trackUdp.beginPacket("255.255.255.255", TRACK_PORT);
  trackUdp.write((uint8_t *)msg, strlen(msg));
  trackUdp.endPacket();
}


// ---- Matrix Rain view (default, view 0) ----
// One falling column per tracked target: MAC hex + RSSI, green head fading tail.
#define RAIN_COLS 40
#define RAIN_TOP 40
struct RainCol {
  float y;
  float speed;
  int tgtIdx;
};
static RainCol rainCols[RAIN_COLS];
static bool rainInit = false;
static uint32_t rainLast = 0;

static void drawRainView(uint32_t now) {
  if (!rainInit) {
    for (int i = 0; i < RAIN_COLS; i++) {
      rainCols[i].y = (float)(RAIN_TOP + (i * 53) % 400);
      rainCols[i].speed = 2.0f + (i % 5) * 0.8f;
      rainCols[i].tgtIdx = -1;
    }
    rainInit = true;
  }
  if (now - rainLast < 80) return;  // ~12fps
  rainLast = now;

  lcdRect(0, RAIN_TOP, 320, 480 - RAIN_TOP, A_BLACK);

  // assign live targets to columns
  for (int c = 0; c < RAIN_COLS; c++) rainCols[c].tgtIdx = -1;
  int col = 0;
  for (int i = 0; i < tgtCount && col < RAIN_COLS; i++) {
    if (tgts[i].mac[0] == 0) continue;
    if (now - tgts[i].last > 15000) continue;
    int c = (i * 7) % RAIN_COLS;
    if (rainCols[c].tgtIdx < 0) { rainCols[c].tgtIdx = i; col++; }
  }

  for (int c = 0; c < RAIN_COLS; c++) {
    RainCol *rc = &rainCols[c];
    int x = 4 + c * 8;
    if (rc->tgtIdx < 0) {
      // idle: slow drifting dim dot
      rc->y += 0.8f;
      if (rc->y > 480) rc->y = RAIN_TOP;
      lcdText(x, (int)rc->y, ".", 1, 0x0400, A_BLACK);
      continue;
    }
    Tgt *tg = &tgts[rc->tgtIdx];
    float spd = rc->speed * (1.0f + (-tg->smoothRssi - 40) / 60.0f);
    if (spd < 1.0f) spd = 1.0f;
    rc->y += spd;
    if (rc->y > 480 + 60) rc->y = RAIN_TOP;
    int headY = (int)rc->y;

    char hex[8];
    int rssi = (int)tg->smoothRssi;
    snprintf(hex, sizeof(hex), "%c%c%c%c%02X",
             tg->mac[0] ? tg->mac[0] : '0', tg->mac[1] ? tg->mac[1] : '0',
             tg->mac[3] ? tg->mac[3] : '0', tg->mac[4] ? tg->mac[4] : '0',
             (uint8_t)(rssi & 0xFF));
    for (int k = 0; k < 8; k++) {
      int cy = headY - k * 14;
      if (cy < RAIN_TOP || cy >= 470) continue;
      char s[2] = {hex[k % 6], 0};
      uint16_t fg = (k == 0) ? 0x07E0 : (k < 3) ? 0x03E0 : 0x01A0;
      lcdText(x, cy, s, 1, fg, A_BLACK);
    }
  }
}

// ---- Credits view (final, view 11) ----
static void drawCreditsView() {
  lcdRect(0, 36, 320, 444, A_BLACK);
  int y = 120;
  lcdTextC(y, CREDIT_NAME, 3, 0x07E0, 0x0000);       y += 36;
  lcdTextC(y, CREDIT_WEB, 2, 0xFFE0, 0x0000);        y += 24;
  lcdTextC(y, CREDIT_GITHUB, 2, 0x8410, 0x0000);     y += 40;
  // blank line then slogan
  lcdTextC(y, CREDIT_SLOGAN, 2, 0x07FF, 0x0000);     y += 30;
  lcdTextC(400, "TAP BOOT = VIEWS", 1, 0x8410, 0x0000);
}



// Turn off onboard RGB LED (GPIO40 WS2812) via ESP32 RMT driver
void rgbOff() {
  neopixelWrite(40, 0, 0, 0);
}

void setup() {
  Serial.begin(115200);
  rgbOff();
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(200);
  bringUpPanel();
  lcdFill(A_BLACK);
  lcdTextC(180, "MUTILLIX", 3, A_BRIGHT, A_BLACK);
  lcdTextC(230, "rain v0.02", 2, A_DIM, A_BLACK);
  lcdTextC(252, "TAP BOOT = VIEWS", 2, A_DIM, A_BLACK);
  lcdTextC(272, "HOLD BOOT = BAND", 2, A_DIM, A_BLACK);

  scpFb = (uint8_t *)heap_caps_malloc(SCP_S * SCP_S * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!scpFb) scpFb = (uint8_t *)malloc(SCP_S * SCP_S * 2);
  if (!scpFb) {
    lcdTextC(310, "NO FRAMEBUFFER", 1, A_BRIGHT, A_BLACK);
    for (;;) delay(1000);
  }

  bleBegin();
  sdBegin();
  gameInit();
  // view history buffers use -128 (0x80) as "no data"; statics init to 0,
  // which would render as a full-bright waterfall / pinned strip lines
  memset(wfBuf, 0x80, sizeof(wfBuf));
  memset(scBuf, 0x80, sizeof(scBuf));
  battSample();
  lcdTextC(300, sdUp ? "SD LOG ON" : "SD DARK", 1, A_DIM, A_BLACK);
  delay(900);
  lcdFill(A_BLACK);
  trackBegin();
  Serial.println("ble radar up");
}

void loop() {
  static uint32_t lastScope = 0, lastUi = 0, lastBatt = 0;
  uint32_t now = millis();

  if (bandMode == BAND_ESP) {
    hunterPoll();
    static uint32_t lastDbg = 0;
    if (now - lastDbg > 5000) {
      lastDbg = now;
      Serial.printf("hunterRx: mgmt=%u action=%u match=%u ch=%d\n",
                    dbgMgmt, dbgAction, dbgMatch, hunterCh);
    }
    if (now - lastHop > 250) {
      lastHop = now;
      hunterCh = hunterCh % 13 + 1;
      esp_wifi_set_channel(hunterCh, WIFI_SECOND_CHAN_NONE);
    }
  } else if (bandMode == BAND_WIFI) {
    wifiPoll(now);
  } else {
    blePoll();
    blePump(now);
  }
  gameTick(now);
  gpsTick();
  wfTick(now);
  scTick(now);
  dgTick(now);
  logFlushTick(now);
  if (now - lastBatt > 2000) {
    lastBatt = now;
    battSample();
  }
  bootPoll();
  drawTopBar();
  watchTick(now);
  findTick(now);
  trackTick(now);
  drawAlertBanner();

  if (viewMode == 0) {
    drawRainView(now);
  } else if (viewMode == 1) {
    if (now - lastScope >= 66) {
      lastScope = now;
      sweepDeg += 6.0f;
      if (sweepDeg >= 360) sweepDeg -= 360;
      renderScope(now);
    }
    if (now - lastUi >= 500) {
      lastUi = now;
      drawStrip();
    }
  } else if (viewMode == 2) {
    drawListView();
  } else if (viewMode == 3) {
    drawGameView();
  } else if (viewMode == 4) {
    drawWaterfall();
  } else if (viewMode == 5) {
    drawStripChart();
  } else if (viewMode == 6) {
    drawDensity();
  } else if (viewMode == 7) {
    drawDetailView();
  } else if (viewMode == 8) {
    drawGpsView();
  } else if (viewMode == 9) {
    drawTrackerView();
  } else if (viewMode == 10) {
    drawFindView();
  } else if (viewMode == 11) {
    drawCreditsView();
  }
  delay(5);
}
