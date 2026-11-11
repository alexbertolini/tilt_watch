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
