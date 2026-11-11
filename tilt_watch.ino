/*
  Tilt Watch - Waveshare ESP32-S3-LCD-1.28 (GC9A01 round LCD + QMI8658 IMU)

  A pocket watch with a white enamel dial and Roman numerals under a glass crystal.
    - the time comes from an NTP server over WiFi (set your WiFi and server below)
    - tilt the watch and the reflections on the glass slide across the dial
    - shake it hard and the glass cracks (up to three impacts); the cracks stay until you repair them
    - BOOT button: short press = repair the glass, hold ~1 s = recalibrate "level"

  Files in this sketch folder:
    tilt_watch.ino    this file (WiFi, time, sensor, main loop)
    watch_render.h    draws the watch (hands, glass reflections, cracks)
    watch_dial.h      the dial image (generated)

  Library needed: "GFX Library for Arduino" (moononournation). WiFi and time are built into the ESP32 core.
  Board: ESP32S3 Dev Module, USB CDC On Boot = Disabled, Flash 16MB, PSRAM = QSPI PSRAM (optional).
*/

#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include <WiFi.h>
#include <time.h>
#include <sys/time.h>
#include "esp_heap_caps.h"

#include "watch_render.h"

// ======================= EDIT THESE =======================
#define WIFI_SSID      "xxxxx"
#define WIFI_PASSWORD  "xxxxx"

#define NTP_SERVER     "pool.ntp.org"       // your NTP server of choice, e.g. "it.pool.ntp.org", "time.google.com"
#define NTP_SERVER2    "time.cloudflare.com" // fallback server

// POSIX time-zone rule. Default is Rome (CET/CEST with automatic daylight saving).
// Other examples: London "GMT0BST,M3.5.0/1,M10.5.0", New York "EST5EDT,M3.2.0,M11.1.0", UTC "UTC0"
#define TZ_INFO        "CET-1CEST,M3.5.0,M10.5.0/3"
// ==========================================================

// Newer Arduino_GFX versions removed the short colour names, so define the ones we use
#ifndef BLACK
#define BLACK  0x0000
#define WHITE  0xFFFF
#define RED    0xF800
#endif

// ---------------- Pins (non-touch ESP32-S3-LCD-1.28) ----------------
#define PIN_SCK   10
#define PIN_MOSI  11
#define PIN_CS     9
#define PIN_DC     8
#define PIN_RST   12
#define PIN_BL    40
#define PIN_SDA    6
#define PIN_SCL    7
#define PIN_BOOT   0

// ---------------- Tilt mapping (same meaning as in the other sketches) ----------------
#define SWAP_XY    0
#define INVERT_X   1
#define INVERT_Y   1
// 1 = the reflection slides toward the raised side of the tilted watch (like a real crystal),
// 0 = toward the lowered side
#define GLARE_TOWARD_HIGH 1
// 1 = the second hand ticks once a second with a tiny bounce, 0 = smooth sweep
#define SECOND_TICK 1
#define DEBUG_SERIAL 1

// ---------------- Feel ----------------
const float DEADZONE     = 0.02f;   // g
const float GLARE_X0     = 92.0f;   // resting position of the glare centre (px)
const float GLARE_Y0     = 86.0f;
//const float GLARE_K      = 110.0f;  // px of glare travel per g of tilt
const float GLARE_K      = 60.0f;  // px of glare travel per g of tilt
const float GLARE_TAU    = 0.05f;   // seconds, how quickly the glare follows the tilt

// ---------------- Shake detection ----------------
const float SHAKE_M0     = 0.50f;   // |a| deviation from 1 g where shaking starts to count
const float SHAKE_GAIN   = 6.0f;    // how fast shaking builds up
const float SHAKE_THRESH = 0.8f;    // energy needed to crack the glass (raise it to need a harder shake)
const float CALM_M       = 0.20f;   // below this deviation the watch counts as calm
const float REARM_CALM   = 1.0f;    // seconds of calm before another crack can happen

// ---------------- Display ----------------
Arduino_DataBus *bus = new Arduino_ESP32SPI(PIN_DC, PIN_CS, PIN_SCK, PIN_MOSI, GFX_NOT_DEFINED);
Arduino_GFX *gfx = new Arduino_GC9A01(bus, PIN_RST, 0 /*rotation*/, true /*IPS*/);

// ---------------- IMU (QMI8658) ----------------
#define QMI_WHO_AM_I 0x00
#define QMI_CTRL1    0x02
#define QMI_CTRL2    0x03
#define QMI_CTRL7    0x08
#define QMI_AX_L     0x35
#define QMI_RESET    0x60
#define ACC_LSB_PER_G 8192.0f       // +-4 g range

uint8_t imuAddr = 0;
float ax0 = 0, ay0 = 0;

bool i2cWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(imuAddr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool i2cRead(uint8_t reg, uint8_t *buf, int n) {
  Wire.beginTransmission(imuAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)imuAddr, (int)n) != n) return false;
  for (int i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

bool imuInit() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  const uint8_t addrs[2] = {0x6B, 0x6A};
  bool found = false;
  for (int i = 0; i < 2 && !found; i++) {
    imuAddr = addrs[i];
    uint8_t id = 0;
    if (i2cRead(QMI_WHO_AM_I, &id, 1) && id == 0x05) found = true;
  }
  if (!found) return false;
  i2cWrite(QMI_RESET, 0xB0);   // soft reset
  delay(30);
  i2cWrite(QMI_CTRL1, 0x40);   // address auto-increment
  i2cWrite(QMI_CTRL2, 0x15);   // accel +-4g, 250 Hz
  i2cWrite(QMI_CTRL7, 0x01);   // enable accelerometer
  delay(20);
  return true;
}

bool readAccel(float &ax, float &ay, float &az) {
  uint8_t b[6];
  if (!i2cRead(QMI_AX_L, b, 6)) return false;
  int16_t rx = (int16_t)(b[0] | (b[1] << 8));
  int16_t ry = (int16_t)(b[2] | (b[3] << 8));
  int16_t rz = (int16_t)(b[4] | (b[5] << 8));
  ax = rx / ACC_LSB_PER_G;
  ay = ry / ACC_LSB_PER_G;
  az = rz / ACC_LSB_PER_G;
  return true;
}

void centerText(const char *txt, int y, int size, uint16_t color) {
  int w = strlen(txt) * 6 * size;
  gfx->setTextSize(size);
  gfx->setTextColor(color);
  gfx->setCursor(120 - w / 2, y);
  gfx->print(txt);
}

void status(const char *l1, const char *l2 = nullptr) {
  gfx->fillScreen(BLACK);
  centerText(l1, l2 ? 100 : 112, 2, WHITE);
  if (l2) centerText(l2, 128, 2, WHITE);
}

void calibrateLevel() {
  status("Hold level");
  delay(1500);
  float sx = 0, sy = 0, az;
  int n = 0;
  for (int i = 0; i < 50; i++) {
    float x, y;
    if (readAccel(x, y, az)) { sx += x; sy += y; n++; }
    delay(10);
  }
  if (n) { ax0 = sx / n; ay0 = sy / n; }
}

// ---------------- WiFi and time ----------------
bool timeConfigured = false;
uint32_t lastWifiTry = 0;

bool wifiConfigured() { return strcmp(WIFI_SSID, "YOUR_WIFI_NAME") != 0; }
bool timeValid() { return time(nullptr) > 1700000000L; }     // anything after Nov 2023 means NTP has set the clock

void startTime() {
  configTzTime(TZ_INFO, NTP_SERVER, NTP_SERVER2);
  timeConfigured = true;
}

void connectAtBoot() {
  if (!wifiConfigured()) {
    status("Set WiFi in", "the sketch");
    Serial.println("WiFi name/password are still the placeholders: edit WIFI_SSID / WIFI_PASSWORD at the top of tilt_watch.ino");
    delay(2500);
    return;
  }
  status("Connecting", "WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(200);
  if (WiFi.status() != WL_CONNECTED) {
    status("No WiFi", "yet...");
    Serial.println("WiFi not connected, the watch will keep trying in the background");
    delay(1500);
    return;
  }
  Serial.printf("WiFi connected, IP %s\n", WiFi.localIP().toString().c_str());
  status("Setting", "the time...");
  startTime();
  t0 = millis();
  while (!timeValid() && millis() - t0 < 10000) delay(200);
  Serial.println(timeValid() ? "Time synchronised" : "NTP not answering yet, will keep trying");
}

// keeps trying in the background if the boot-time attempt did not work
void maintainTime() {
  if (!wifiConfigured()) return;
  if (WiFi.status() == WL_CONNECTED) {
    if (!timeConfigured) startTime();
  } else if (millis() - lastWifiTry > 20000) {
    lastWifiTry = millis();
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
}

// ---------------- Watch state ----------------
uint16_t *fb = nullptr;
float glareX = GLARE_X0, glareY = GLARE_Y0;
float flash = 0;                    // white flash when the glass cracks
float energy = 0, calmTime = 0;     // shake detector
bool  armed = true;
bool  repairing = false;
float repairT = 0;
uint32_t lastUs = 0, fpsMs = 0, pressStart = 0;
int frames = 0;
bool btnWasDown = false;

static const float D2R = 0.017453292f;

float easeOutBack(float t) {
  const float c1 = 1.70158f, c3 = c1 + 1.0f;
  t -= 1.0f;
  return 1.0f + c3 * t * t * t + c1 * t * t;
}

void crackTheGlass() {
  float a = (float)random(0, 6283) / 1000.0f;
  float r = 14.0f + (float)random(0, 720) / 10.0f;
  if (watch_crack_add(120.0f + cosf(a) * r, 120.0f + sinf(a) * r, esp_random())) {
    flash = 1.0f;
#if DEBUG_SERIAL
    Serial.println("crack!");
#endif
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BL, OUTPUT);
  digitalWrite(PIN_BL, HIGH);
  pinMode(PIN_BOOT, INPUT_PULLUP);

  if (!gfx->begin()) {
    Serial.println("Display init failed");
    while (true) delay(1000);
  }
  gfx->fillScreen(BLACK);

  if (!imuInit()) {
    status("IMU not found");
    Serial.println("QMI8658 not found");
    while (true) delay(1000);
  }

  fb = (uint16_t *)heap_caps_malloc(WW * WW * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!fb) fb = (uint16_t *)heap_caps_malloc(WW * WW * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!fb) {
    status("Out of memory");
    while (true) delay(1000);
  }
  memset(fb, 0, WW * WW * 2);
  watch_init();
  watch_crack_clear();
  randomSeed(esp_random());

  connectAtBoot();
  calibrateLevel();
  gfx->fillScreen(BLACK);
  lastUs = micros();
}

void loop() {
  // --- BOOT button: short = repair the glass, long = recalibrate ---
  bool down = (digitalRead(PIN_BOOT) == LOW);
  if (down && !btnWasDown) pressStart = millis();
  if (!down && btnWasDown) {
    if (millis() - pressStart > 1000) { calibrateLevel(); lastUs = micros(); }
    else if (w_ncracks > 0 && !repairing) { repairing = true; repairT = 0; }
  }
  btnWasDown = down;

  // --- time step ---
  uint32_t now = micros();
  float dt = (now - lastUs) / 1e6f;
  lastUs = now;
  if (dt > 0.1f) dt = 0.1f;

  maintainTime();

  // --- sensor ---
  float tiltX = 0, tiltY = 0, mag = 1.0f;
  float ax, ay, az;
  if (readAccel(ax, ay, az)) {
    mag = sqrtf(ax * ax + ay * ay + az * az);
    float gx = ax - ax0, gy = ay - ay0;
#if SWAP_XY
    float t = gx; gx = gy; gy = t;
#endif
#if INVERT_X
    gx = -gx;
#endif
#if INVERT_Y
    gy = -gy;
#endif
    if (fabsf(gx) < DEADZONE) gx = 0;
    if (fabsf(gy) < DEADZONE) gy = 0;
    // gx, gy > 0 mean the right / bottom side is lower
#if GLARE_TOWARD_HIGH
    tiltX = -gx; tiltY = -gy;
#else
    tiltX = gx;  tiltY = gy;
#endif
  }
  if (tiltX > 0.6f) tiltX = 0.6f; if (tiltX < -0.6f) tiltX = -0.6f;
  if (tiltY > 0.6f) tiltY = 0.6f; if (tiltY < -0.6f) tiltY = -0.6f;

  // --- glass reflection follows the tilt ---
  float k = 1.0f - expf(-dt / GLARE_TAU);
  glareX += (GLARE_X0 + GLARE_K * tiltX - glareX) * k;
  glareY += (GLARE_Y0 + GLARE_K * tiltY - glareY) * k;

  // --- shake detector -> crack ---
  float m = fabsf(mag - 1.0f);
  energy *= expf(-dt / 0.5f);
  if (m > SHAKE_M0) energy += (m - SHAKE_M0) * dt * SHAKE_GAIN;
  if (armed && !repairing && energy > SHAKE_THRESH) {
    crackTheGlass();
    armed = false; energy = 0; calmTime = 0;
  }
  if (!armed) {
    if (m < CALM_M) calmTime += dt; else calmTime = 0;
    if (calmTime >= REARM_CALM) armed = true;
  }

  // --- animations ---
  flash -= dt * 5.0f;
  if (flash < 0) flash = 0;
  watch_crack_update(dt);
  float crackFade = 1.0f, sweep = -1000.0f;
  if (repairing) {
    repairT += dt;
    float u = repairT / 0.9f;
    crackFade = 1.0f - (u < 0.15f ? 0.0f : (u - 0.15f) / 0.6f);
    if (crackFade < 0) crackFade = 0;
    sweep = -150.0f + 330.0f * u;                    // a bright band sweeps over the glass
    if (repairT >= 0.9f) { watch_crack_clear(); repairing = false; crackFade = 1.0f; sweep = -1000.0f; }
  }

  // --- time ---
  struct timeval tv;
  gettimeofday(&tv, NULL);
  bool synced = timeValid();
  int hh, mm, ss;
  float frac = tv.tv_usec / 1e6f;
  if (synced) {
    struct tm tmv;
    time_t t = tv.tv_sec;
    localtime_r(&t, &tmv);
    hh = tmv.tm_hour; mm = tmv.tm_min; ss = tmv.tm_sec;
  } else {
    // not synchronised yet: run freely from 10:09:30 (the classic watch pose) until NTP answers
    uint32_t ms = millis();
    uint32_t sec = 9 * 60 + 30 + ms / 1000;
    hh = 10 + (int)(sec / 3600); mm = (int)((sec / 60) % 60); ss = (int)(sec % 60);
    frac = (ms % 1000) / 1000.0f;
  }
  float minutes = mm + (ss + frac) / 60.0f;
  float hours = (hh % 12) + minutes / 60.0f;
#if SECOND_TICK
  float secPos = (frac < 0.16f) ? (ss - 1 + easeOutBack(frac / 0.16f)) : (float)ss;
#else
  float secPos = ss + frac;
#endif
  float aH = hours * 30.0f * D2R, aM = minutes * 6.0f * D2R, aS = secPos * 6.0f * D2R;

  // --- draw ---
  bool dot = !synced && (((millis() / 700) & 1) == 0);
  watch_render(fb, aH, aM, aS, glareX, glareY, 1.0f, flash, sweep, crackFade, dot);
  gfx->draw16bitRGBBitmap(0, 0, fb, WW, WW);

#if DEBUG_SERIAL
  frames++;
  if (millis() - fpsMs >= 2000) {
    Serial.printf("%.1f fps  %02d:%02d:%02d %s\n", frames * 1000.0f / (millis() - fpsMs), hh, mm, ss, synced ? "(NTP)" : "(not synced)");
    frames = 0;
    fpsMs = millis();
  }
#endif
}
