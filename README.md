# Patient Monitor (ESP32-S3)

A compact patient monitoring firmware for ESP32-S3 that reads an MLX90614 infrared thermometer, a DFRobot/MAX30102 SpO2/heart-rate sensor, and an ADS1115-connected pressure sensor for oscillometric blood-pressure measurement. The project includes a touchscreen/TFT UI (LVGL + TFT_eSPI) and receives commands over the serial monitor (`TEMP`, `HEART`, `BP`).

See `DOCUMENTATION.md` for full details: wiring, build/flash instructions, code walkthrough, calibration, safety notes, and troubleshooting.