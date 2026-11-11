// watch_render.h - pocket-watch renderer (pure C++, no Arduino dependencies)
//
// Layers, back to front:
//   watch_dial   pre-rendered white enamel dial with Roman numerals, minute track and rim
//   hands        hour / minute / second with soft drop shadows, drawn with anti-aliasing
//   cracks       procedural cracks in the glass (impact point + radial cracks + web rings)
//   glass        reflections on the crystal: moving glare streaks, hot spot, rim glints, flashes
#pragma once
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "watch_dial.h"

#pragma GCC push_options
#pragma GCC optimize ("O3")

#define WW    240
#define W_CX  120.0f
#define W_CY  120.0f
#define W_MAXCRACKS 3

// ---------------------------------------------------------------- state and tables
static uint8_t  w_mask[WW * WW];               // crack mask: bits 0-5 = strength, bits 6-7 = which impact
static float    w_ccx[4], w_ccy[4], w_reveal[4];   // impact centres and how far each crack has spread (px)
static int      w_ncracks = 0;
static uint8_t  w_band[256];                   // glare streak profile across the band
static uint8_t  w_spot[300];                   // glare hot spot profile (by squared distance / 8)
static uint8_t  w_glassF[14700];               // how much glass reflection reaches a given squared radius
static uint32_t w_rng = 1;
static const uint8_t w_bayer[4][4] = {{0, 8, 2, 10}, {12, 4, 14, 6}, {3, 11, 1, 9}, {15, 7, 13, 5}};
static const float W_NX = 0.616f, W_NY = 0.788f;   // unit normal of the glare streaks
#define W_RIM2 (114 * 114)
#define W_TINT 17            // glass tint (0..255 scale): the dial is slightly shaded and the reflections lift it back to white

static inline float w_rnd() {                  // xorshift32 in [0,1)
  w_rng ^= w_rng << 13; w_rng ^= w_rng >> 17; w_rng ^= w_rng << 5;
  return (float)(w_rng & 0xFFFFFF) / 16777216.0f;
}
static inline float w_clampf(float v, float a, float b) { return v < a ? a : (v > b ? b : v); }
static inline int   w_clampi(int v, int a, int b) { return v < a ? a : (v > b ? b : v); }

static inline void w_unpack(uint16_t c, int &r, int &g, int &b) {
  int r5 = (c >> 11) & 31, g6 = (c >> 5) & 63, b5 = c & 31;
  r = (r5 << 3) | (r5 >> 2); g = (g6 << 2) | (g6 >> 4); b = (b5 << 3) | (b5 >> 2);
}
static inline uint16_t w_pack(int r, int g, int b, int t) {      // t = dither threshold 0..15
  r += t >> 1; g += t >> 2; b += t >> 1;
  r = w_clampi(r, 0, 255); g = w_clampi(g, 0, 255); b = w_clampi(b, 0, 255);
  return (uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
}
// mix a pixel toward colour (r,g,b) with weight a = 0..256
static inline uint16_t w_mix(uint16_t c, int r, int g, int b, int a) {
  int cr, cg, cb; w_unpack(c, cr, cg, cb);
  cr += ((r - cr) * a) >> 8; cg += ((g - cg) * a) >> 8; cb += ((b - cb) * a) >> 8;
  return w_pack(cr, cg, cb, 7);
}

static void watch_init() {
  for (int i = 0; i < 256; i++) {                        // streak profile along u = i/1.4222 - 80
    float u = (float)i / 1.4222f - 80.0f;
    float a = 0.15f * expf(-(u / 34.0f) * (u / 34.0f))                      // broad soft sheen
            + 0.40f * expf(-((u - 26.0f) / 6.5f) * ((u - 26.0f) / 6.5f))   // main streak
            + 0.20f * expf(-((u - 45.0f) / 3.0f) * ((u - 45.0f) / 3.0f));  // thin second streak
    w_band[i] = (uint8_t)(w_clampf(a, 0, 1) * 255.0f);
  }
  for (int i = 0; i < 300; i++) {                        // round hot spot, by squared distance / 8
    float d2 = (float)(i * 8);
    w_spot[i] = (uint8_t)(0.17f * expf(-d2 / (2.0f * 24.0f * 24.0f)) * 255.0f);
  }
  for (int r2 = 0; r2 < 14700; r2++) {                   // glare is strongest on the glass, weaker toward the rim
    float r = sqrtf((float)r2);
    float f = r < 108 ? 1.0f : (r < 114 ? 1.0f - 0.45f * (r - 108.0f) / 6.0f : 0.0f);
    w_glassF[r2] = (uint8_t)(f * 255.0f);
  }
}

// ---------------------------------------------------------------- hands
struct WHand { float tail, len, stem, ls, lm, le, lhw, tipHW; };
// half width along the hand: stem, a leaf ("spade") that swells and narrows, then a needle tip
static inline float w_hw(const WHand &h, float s) {
  if (s < -h.tail) return 0.0f;
  if (s < h.ls) return h.stem;
  if (s < h.lm) { float t = (s - h.ls) / (h.lm - h.ls); t = t * t * (3 - 2 * t); return h.stem + (h.lhw - h.stem) * t; }
  if (s < h.le) { float t = (s - h.lm) / (h.le - h.lm); t = t * t * (3 - 2 * t); return h.lhw + (h.stem - h.lhw) * t; }
  float t = (s - h.le) / (h.len - h.le); t = w_clampf(t, 0, 1);
  return h.stem + (h.tipHW - h.stem) * t;
}

static void w_draw_hand(uint16_t *fb, const WHand &h, float ang, float ox, float oy, int cr, int cg, int cb, int alpha) {
  float dx = sinf(ang), dy = -cosf(ang);
  float x0 = W_CX + ox, y0 = W_CY + oy;
  float ax = x0 - dx * h.tail, ay = y0 - dy * h.tail, bx = x0 + dx * h.len, by = y0 + dy * h.len;
  float m = h.lhw + 2.5f;
  int xa = w_clampi((int)floorf(fminf(ax, bx) - m), 0, WW - 1), xb = w_clampi((int)ceilf(fmaxf(ax, bx) + m), 0, WW - 1);
  int ya = w_clampi((int)floorf(fminf(ay, by) - m), 0, WW - 1), yb = w_clampi((int)ceilf(fmaxf(ay, by) + m), 0, WW - 1);
  for (int y = ya; y <= yb; y++) {
    uint16_t *row = fb + y * WW;
    for (int x = xa; x <= xb; x++) {
      float px = (float)x + 0.5f - x0, py = (float)y + 0.5f - y0;
      float s = px * dx + py * dy;
      if (s < -h.tail - 1.0f || s > h.len + 1.0f) continue;
      float n = fabsf(py * dx - px * dy);
      float cov = w_hw(h, s) + 0.5f - n;
      if (cov <= 0) continue;
      if (cov > 1) cov = 1;
      float e1 = s + h.tail + 0.5f; if (e1 < 1.0f) { if (e1 <= 0) continue; cov *= e1; }
      float e2 = h.len + 0.5f - s;  if (e2 < 1.0f) { if (e2 <= 0) continue; cov *= e2; }
      int a = (int)(cov * (float)alpha);
      if (a > 0) row[x] = w_mix(row[x], cr, cg, cb, a);
    }
  }
}

static void w_draw_disc(uint16_t *fb, float cx, float cy, float r, int cr, int cg, int cb, int alpha) {
  int xa = w_clampi((int)floorf(cx - r - 1), 0, WW - 1), xb = w_clampi((int)ceilf(cx + r + 1), 0, WW - 1);
  int ya = w_clampi((int)floorf(cy - r - 1), 0, WW - 1), yb = w_clampi((int)ceilf(cy + r + 1), 0, WW - 1);
  for (int y = ya; y <= yb; y++)
    for (int x = xa; x <= xb; x++) {
      float ddx = (float)x + 0.5f - cx, ddy = (float)y + 0.5f - cy;
      float cov = r + 0.5f - sqrtf(ddx * ddx + ddy * ddy);
      if (cov <= 0) continue;
      if (cov > 1) cov = 1;
      fb[y * WW + x] = w_mix(fb[y * WW + x], cr, cg, cb, (int)(cov * alpha));
    }
}

static const WHand W_HOUR = {14.0f, 66.0f, 2.3f, 28.0f, 42.0f, 56.0f, 6.0f, 0.4f};
static const WHand W_MIN  = {16.0f, 99.0f, 1.5f, 58.0f, 70.0f, 82.0f, 3.9f, 0.4f};
static const WHand W_SEC  = {26.0f, 98.0f, 0.65f, 0.0f, 1.0f, 2.0f, 0.65f, 0.4f};

static void w_hands(uint16_t *fb, float aH, float aM, float aS, bool unsynced) {
  const float sx = 2.4f, sy = 3.6f;                                  // shadow offset (light from the upper left)
  w_draw_hand(fb, W_HOUR, aH, sx, sy, 0, 0, 0, 62);
  w_draw_hand(fb, W_MIN,  aM, sx, sy, 0, 0, 0, 62);
  w_draw_hand(fb, W_SEC,  aS, sx * 1.4f, sy * 1.4f, 0, 0, 0, 40);
  w_draw_disc(fb, W_CX + sx, W_CY + sy, 5.6f, 0, 0, 0, 60);
  w_draw_hand(fb, W_HOUR, aH, 0, 0, 24, 26, 36, 256);
  w_draw_hand(fb, W_MIN,  aM, 0, 0, 24, 26, 36, 256);
  w_draw_hand(fb, W_SEC,  aS, 0, 0, 150, 28, 28, 256);
  w_draw_disc(fb, W_CX - (sinf(aS) * 19.0f), W_CY + (cosf(aS) * 19.0f), 3.4f, 150, 28, 28, 256);   // counterweight
  w_draw_disc(fb, W_CX, W_CY, 5.4f, 30, 30, 38, 256);                                               // centre cap
  w_draw_disc(fb, W_CX - 1.3f, W_CY - 1.4f, 1.7f, 170, 172, 186, 150);                              // glint on the cap
  w_draw_disc(fb, W_CX, W_CY, 1.9f, 150, 28, 28, 256);
  if (unsynced) w_draw_disc(fb, W_CX, 176.0f, 2.4f, 190, 30, 30, 230);                               // "no time sync yet"
}

// ---------------------------------------------------------------- cracks
static void w_splat(float x, float y, int id, int val) {
  int ix = (int)floorf(x), iy = (int)floorf(y);
  float fx = x - ix, fy = y - iy;
  for (int j = 0; j < 2; j++)
    for (int i = 0; i < 2; i++) {
      int px = ix + i, py = iy + j;
      if (px < 0 || py < 0 || px >= WW || py >= WW) continue;
      float w = (i ? fx : 1 - fx) * (j ? fy : 1 - fy);
      int v = (int)((float)val * fminf(1.0f, w * 1.7f));
      uint8_t &m = w_mask[py * WW + px];
      if ((int)(m & 63) < v) m = (uint8_t)((id << 6) | v);
    }
}
static void w_cline(float x0, float y0, float x1, float y1, int id, int v0, int v1) {
  float dx = x1 - x0, dy = y1 - y0;
  int n = (int)(sqrtf(dx * dx + dy * dy) * 2.0f) + 1;
  for (int i = 0; i <= n; i++) {
    float t = (float)i / (float)n;
    w_splat(x0 + dx * t, y0 + dy * t, id, v0 + (int)((float)(v1 - v0) * t));
  }
}
static bool w_inside(float x, float y) { float dx = x - W_CX, dy = y - W_CY; return dx * dx + dy * dy < 118.0f * 118.0f; }

static void watch_crack_clear() { memset(w_mask, 0, sizeof(w_mask)); w_ncracks = 0; }

// Add a new impact at (cx, cy). Returns false when the glass cannot take more cracks.
static bool watch_crack_add(float cx, float cy, uint32_t seed) {
  if (w_ncracks >= W_MAXCRACKS) return false;
  const int id = w_ncracks++;
  w_ccx[id] = cx; w_ccy[id] = cy; w_reveal[id] = 0;
  w_rng = seed ? seed : 7u;
  for (int i = 0; i < 5; i++) w_rnd();

  const int NR = 8 + (int)(w_rnd() * 4.0f), MAXP = 24;
  static float px_[12][24], py_[12][24], cum_[12][24];
  int np[12];
  for (int i = 0; i < NR; i++) {
    float ang = 6.2831853f * ((float)i + (w_rnd() - 0.5f) * 0.7f) / (float)NR;
    float x = cx, y = cy, total = 0, target = 80.0f + w_rnd() * 120.0f;
    int n = 0;
    px_[i][0] = x; py_[i][0] = y; cum_[i][0] = 0; n = 1;
    while (total < target && n < MAXP) {
      float step = 8.0f + w_rnd() * 10.0f;
      ang += (w_rnd() - 0.5f) * 0.55f;
      float nx = x + cosf(ang) * step, ny = y + sinf(ang) * step;
      bool last = false;
      if (!w_inside(nx, ny)) {                                    // stop at the edge of the glass
        float lo = 0, hi = 1;
        for (int k = 0; k < 6; k++) { float mid = 0.5f * (lo + hi); if (w_inside(x + (nx - x) * mid, y + (ny - y) * mid)) lo = mid; else hi = mid; }
        nx = x + (nx - x) * lo; ny = y + (ny - y) * lo; last = true;
      }
      int v0 = 63 - (int)(28.0f * total / target), v1 = 63 - (int)(28.0f * (total + step) / target);
      w_cline(x, y, nx, ny, id, v0, v1);
      if (!last && n > 1 && w_rnd() < 0.30f) {                    // side branch
        float ba = ang + (w_rnd() < 0.5f ? -1.0f : 1.0f) * (0.5f + w_rnd() * 0.4f);
        float bx = x, by = y, bl = 14.0f + w_rnd() * 36.0f, done = 0;
        while (done < bl) {
          float st = 6.0f + w_rnd() * 6.0f; ba += (w_rnd() - 0.5f) * 0.6f;
          float tx = bx + cosf(ba) * st, ty = by + sinf(ba) * st;
          if (!w_inside(tx, ty)) break;
          w_cline(bx, by, tx, ty, id, 40 - (int)(22.0f * done / bl), 40 - (int)(22.0f * (done + st) / bl));
          bx = tx; by = ty; done += st;
        }
      }
      total += step; x = nx; y = ny;
      px_[i][n] = x; py_[i][n] = y; cum_[i][n] = total; n++;
      if (last) break;
    }
    np[i] = n;
  }
  // web rings joining neighbouring cracks at a few distances from the impact
  const float rings[5] = {15.0f, 28.0f, 46.0f, 68.0f, 94.0f};
  for (int k = 0; k < 5; k++)
    for (int i = 0; i < NR; i++) {
      int j = (i + 1) % NR;
      if (w_rnd() > 0.72f) continue;
      float ax, ay, bx, by; bool oka = false, okb = false;
      for (int p = 1; p < np[i]; p++) if (cum_[i][p] >= rings[k]) { float t = (rings[k] - cum_[i][p - 1]) / (cum_[i][p] - cum_[i][p - 1]); ax = px_[i][p - 1] + (px_[i][p] - px_[i][p - 1]) * t; ay = py_[i][p - 1] + (py_[i][p] - py_[i][p - 1]) * t; oka = true; break; }
      for (int p = 1; p < np[j]; p++) if (cum_[j][p] >= rings[k]) { float t = (rings[k] - cum_[j][p - 1]) / (cum_[j][p] - cum_[j][p - 1]); bx = px_[j][p - 1] + (px_[j][p] - px_[j][p - 1]) * t; by = py_[j][p - 1] + (py_[j][p] - py_[j][p - 1]) * t; okb = true; break; }
      if (!oka || !okb) continue;
      float lx = ax, ly = ay, ex = bx - ax, ey = by - ay, len = sqrtf(ex * ex + ey * ey) + 0.001f;
      float nx = -ey / len, ny = ex / len;
      for (int sgm = 1; sgm <= 3; sgm++) {                        // jagged line between the two cracks
        float t = (float)sgm / 3.0f;
        float jx = ax + ex * t + (sgm < 3 ? nx * (w_rnd() - 0.5f) * 7.0f : 0), jy = ay + ey * t + (sgm < 3 ? ny * (w_rnd() - 0.5f) * 7.0f : 0);
        if (w_inside(jx, jy) && w_inside(lx, ly)) w_cline(lx, ly, jx, jy, id, 34 - k * 3, 30 - k * 3);
        lx = jx; ly = jy;
      }
    }
  // chips right at the impact
  for (int c = 0; c < 14; c++) {
    float a = w_rnd() * 6.2831853f, l = 2.5f + w_rnd() * 7.0f;
    w_cline(cx, cy, cx + cosf(a) * l, cy + sinf(a) * l, id, 63, 40);
  }
  w_splat(cx, cy, id, 63); w_splat(cx + 0.6f, cy, id, 63); w_splat(cx, cy + 0.6f, id, 63);
  return true;
}

// advance how far the cracks have spread (call every frame)
static void watch_crack_update(float dt) {
  for (int i = 0; i < w_ncracks; i++) w_reveal[i] += dt * 700.0f;
}

static void w_cracks(uint16_t *fb, float fade) {
  if (!w_ncracks || fade <= 0.0f) return;
  int fa = (int)(w_clampf(fade, 0, 1) * 256.0f);
  for (int y = 0; y < WW; y++) {
    const uint8_t *mrow = w_mask + y * WW;
    uint16_t *row = fb + y * WW;
    for (int x = 0; x < WW; x++) {
      uint8_t v = mrow[x];
      if (v & 63) {
        int id = v >> 6; float dx = (float)x + 0.5f - w_ccx[id], dy = (float)y + 0.5f - w_ccy[id];
        if (dx * dx + dy * dy > w_reveal[id] * w_reveal[id]) continue;
        int a = (((v & 63) * 240 / 63) * fa) >> 8;
        row[x] = w_mix(row[x], 50, 52, 62, a);
      } else if (x > 0 && y > 0) {
        uint8_t u = mrow[x - WW - 1];
        if (u & 63) {
          int id = u >> 6; float dx = (float)x - 0.5f - w_ccx[id], dy = (float)y - 0.5f - w_ccy[id];
          if (dx * dx + dy * dy > w_reveal[id] * w_reveal[id]) continue;
          int a = (((u & 63) * 170 / 63) * fa) >> 8;
          row[x] = w_mix(row[x], 255, 255, 255, a);
        }
      }
    }
  }
}

// ---------------------------------------------------------------- glass reflections
// (hx, hy): centre of the glare, strength 0..1, flash 0..1 (white flash), sweep: position of a bright
// sweep band in px along the glare normal (< -500 = off), crackFade: how visible cracks are (they catch the light)
static void w_glass(uint16_t *fb, float hx, float hy, float strength, float flash, float sweep, float crackFade) {
  const int S = (int)(w_clampf(strength, 0, 1) * 256.0f);
  const int F = (int)(w_clampf(flash, 0, 1) * 255.0f);
  const bool doSweep = sweep > -500.0f;
  const bool cr = w_ncracks > 0 && crackFade > 0.0f;
  float vx = hx - W_CX, vy = hy - W_CY, vl = sqrtf(vx * vx + vy * vy);
  float ux = -0.7071f, uy = -0.7071f;
  if (vl > 1.0f) { ux = vx / vl; uy = vy / vl; }
  const float rimW = 0.35f + 0.65f * fminf(1.0f, vl / 45.0f);

  for (int y = 0; y < WW; y++) {
    int dy = y - 120;
    int hw = (int)sqrtf(fmaxf(0.0f, 120.5f * 120.5f - (float)(dy * dy)));
    int xa = 120 - hw, xb = 120 + hw;
    if (xa < 0) xa = 0;
    if (xb > WW - 1) xb = WW - 1;
    uint16_t *row = fb + y * WW;
    const uint8_t *mrow = w_mask + y * WW;
    const uint8_t *bay = w_bayer[y & 3];
    float fy = (float)y + 0.5f - hy;

    for (int x = xa; x <= xb; x++) {
      int dx = x - 120;
      int r2 = dx * dx + dy * dy;
      int a = 0;
      if (r2 > W_RIM2) {                                           // metal rim: glints toward / away from the glare
        float r = sqrtf((float)r2) + 0.001f;
        float c = ((float)dx * ux + (float)dy * uy) / r;
        float c2 = c * c, c4 = c2 * c2, c6 = c4 * c2;
        float al = c > 0 ? 0.62f * c6 : 0.30f * c6;
        a = (int)(al * rimW * 255.0f) * S >> 8;
      } else {
        float fx = (float)x + 0.5f - hx;
        float u = fx * W_NX + fy * W_NY;
        int bi = (int)((u + 80.0f) * 1.4222f);
        if ((unsigned)bi < 256u) a = w_band[bi];
        int d2 = (int)(fx * fx + fy * fy) >> 3;
        if (d2 < 300) a += w_spot[d2];
        a = (a * w_glassF[r2] >> 8) * S >> 8;
        if (doSweep) {
          float su = ((float)x + 0.5f - W_CX) * W_NX + ((float)y + 0.5f - W_CY) * W_NY - sweep;
          float t = 1.0f - fabsf(su) * (1.0f / 14.0f);
          if (t > 0) a += (int)(190.0f * t * t);
        }
      }
      if (cr) { int m = mrow[x] & 63; if (m) a = a * 3 + (m >> 2); }
      a += F;
      if (a > 255) a = 255;
      int k = 0;
      if (r2 <= W_RIM2) k = (W_TINT * (255 - a)) >> 8;           // the glass slightly darkens whatever is not lit by a reflection
      if (a <= 0 && k <= 0) continue;
      int rr, gg, bb; w_unpack(row[x], rr, gg, bb);
      if (k) { rr -= (rr * k) >> 8; gg -= (gg * k) >> 8; bb -= (bb * k) >> 8; }
      rr += ((255 - rr) * a) >> 8; gg += ((255 - gg) * a) >> 8; bb += ((255 - bb) * a) >> 8;
      row[x] = w_pack(rr, gg, bb, bay[x & 3]);
    }
  }
}

// ---------------------------------------------------------------- whole frame
// aH/aM/aS: hand angles in radians (0 = 12 o'clock, clockwise).
static void watch_render(uint16_t *fb, float aH, float aM, float aS, float glareX, float glareY, float glareStrength,
                         float flash, float sweep, float crackFade, bool unsynced) {
  memcpy(fb, watch_dial, WW * WW * sizeof(uint16_t));
  w_hands(fb, aH, aM, aS, unsynced);
  w_cracks(fb, crackFade);
  w_glass(fb, glareX, glareY, glareStrength, flash, sweep, crackFade);
}

#pragma GCC pop_options
