# Patient Monitor — Documentation

This document explains how the code in `src/main.cpp` works, how to wire the hardware, how to build and flash the firmware (PlatformIO), the libraries used, and how to troubleshoot and calibrate the system.

## Project summary

This project runs on an ESP32-S3 board (PlatformIO env: `rymcu-esp32-s3-devkitc-1`). It implements a small patient-monitor system with three measurements:

- Temperature (infrared MLX90614: ambient & object)
- Heart rate and SpO2 (DFRobot/MAX30102 family via `DFRobot_BloodOxygen_S`)
- Blood pressure (oscillometric style using an analog pressure sensor read by an ADS1115 ADC, a pump driven by an L298N, and a solenoid valve)

A TFT display with LVGL shows the UI. Commands are issued over the serial monitor: `TEMP`, `HEART`, `BP`.

## Files of interest

- `src/main.cpp` — main firmware: initialization, measurement routines, LVGL UI creation and update helpers.
- `platformio.ini` — build configuration and library dependencies.

## Libraries / PlatformIO dependencies

Listed in `platformio.ini` (lib_deps):

- `adafruit/Adafruit ADS1X15` — ADS1115 ADC for pressure sensor.
- `dfrobot/DFRobot_BloodOxygen_S` — MAX30102 SpO2/heart sensor wrapper.
- `dfrobot/DFRobot_RTU` — dependency for DFRobot library.
- `adafruit/Adafruit MLX90614 Library` — MLX90614 IR thermometer.
- `adafruit/Adafruit BusIO` — common I2C helpers.
- `bodmer/TFT_eSPI` — TFT display driver.
- `lvgl/lvgl` — LittlevGL (LVGL) GUI library.

Platform: `espressif32`, Framework: `arduino` (see `platformio.ini`).

## Hardware overview and wiring

Important: verify pin numbering and wiring for your specific ESP32-S3 board. The code uses the following mappings (from `main.cpp`):

- I2C (Wire): `SDA_PIN` = 8, `SCL_PIN` = 9
  - MLX90614 (I2C) → SDA (pin 8) / SCL (pin 9)
  - ADS1115 (I2C) → SDA / SCL
  - MPX5050DP → A0 OF ADS1115
  - MAX30102 / DFRobot SpO2 (I2C) → SDA / SCL (I2C address in code: 0x57)
 L298N motor driver / pump / solenoid pins:
  - `ENA` = 15  (PWM enable for motor channel A)
  - `IN1` = 16  (direction control ch A)
  - `IN2` = 17  (direction control ch A)
  - `IN3` = 7   (direction control ch B)
  - `IN4` = 6   (direction control ch B)
  - `ENB` = 14  (named ENB but used as digital pin to toggle the solenoid in the code)

- TFT / LCD pins (from TFT_eSPI `User_Setup.h` included in the project):
  - `TFT_MISO` = 13
  - `TFT_MOSI` = 11
  - `TFT_SCLK` = 12
  - `TFT_CS`   = 5
  - `TFT_DC`   = 41
  - `TFT_RST`  = -1  (reset tied to board RST or not used)
  - `TFT_BL`   = 40  (backlight control)
  - `TOUCH_CS` = 39  (touch controller chip-select, if present)

- ADS1115 analog input used: A0 (read as channel 0 in `ads.readADC_SingleEnded(0)`).

Notes / assumptions about wiring:

- The code uses the Arduino `Wire` object initialized with `Wire.begin(SDA_PIN, SCL_PIN)`. Ensure your board allows I2C on those pins or change them to your hardware's SDA/SCL pins.
- The code toggles `ENB` directly to open/close the solenoid. On some L298N setups the ENB pin is a PWM input; in this code `ENB` is repurposed as a digital output to drive a solenoid transistor or driver.
- There is a potential mapping inconsistency: the `startPumpGradual()` helper uses `ENA`, `IN1`, and `IN2` (motor channel A). However during blood-pressure inflation the code sets `IN3`/`IN4` (motor channel B) before calling `startPumpGradual()`. This suggests either:
  - The pump is actually wired to channel A and the `IN3`/`IN4` writes were accidental, or
  - The pump is wired to channel B and `startPumpGradual()` should use `ENB` / `IN3`/`IN4` instead.

Check your physical wiring and adapt the code pins if necessary. We'll list this as a troubleshooting item later.

## Electrical safety and mechanical warnings

- This circuit controls a pump and a solenoid that will produce pressure on a cuff. Use caution: never apply excessive pressure to a human limb.
- The code includes a safety upper-pressure threshold `SAFETY_MAX = 180` mmHg to stop inflation.
- Always test with a capped cuff or test bench first and confirm the pressure reading logic against a calibrated manometer.

## How the measurements work (code walkthrough)

High-level flow in `loop()`:

1. The code keeps LVGL running by calling `lv_tick_inc()` and `lv_timer_handler()` periodically.
2. When a serial line is received the string is trimmed and uppercased. Valid commands: `TEMP`, `HEART`, `BP`.
3. Each command loads a single `scr_measure` LVGL screen and calls the corresponding measurement function:
   - `measureTemperature()` — MLX90614
   - `measureHeartSpO2()` — DFRobot MAX30102 wrapper
   - `measureBloodPressure()` — pump + ADS1115 oscillometry

### Temperature (`measureTemperature()`)

- Shows measurement screen and performs a 2-second averaging window.
- Reads:
  - `ambient = mlx.readAmbientTempC()` — ambient temperature in °C
  - `object = mlx.readObjectTempC()` — IR object temperature in °C (the target/patient surface)
- Samples approximately every 200 ms for 2 seconds, computes averages and writes them to `last_ambient` / `last_object` and displays the averaged object temperature.

### Heart rate & SpO2 (`measureHeartSpO2()`)

- Uses `DFRobot_BloodOxygen_S_I2C` wrapper:
  - Calls `spo2Sensor.sensorStartCollect()` to begin sampling.
  - Waits 4 seconds while showing a countdown on the display.
  - Calls `spo2Sensor.getHeartbeatSPO2()` which triggers the library to compute values.
  - Reads `spo2Sensor._sHeartbeatSPO2.SPO2` and `spo2Sensor._sHeartbeatSPO2.Heartbeat` and `spo2Sensor.getTemperature_C()` for the sensor temperature.
  - Stores `last_spo2`, `last_heart`, `last_sensor_temp` and displays the final results.

Notes:
- The DFRobot library may have its own internal sample rate and timings. The wrapper calls in the code assume that a 4-second window is enough to collect a stable result.

### Blood pressure (`measureBloodPressure()`)

This implements a basic oscillometric measurement using an ADS1115 and a pump + pulsed solenoid valve. Key parameters in `main.cpp`:

- `MA_LEN = 5` — moving average length for smoothing pressure samples.
- `TARGET_PRESSURE = 170` — inflation target (mmHg).
- `SAFETY_MAX = 180` — maximum allowed pressure (safety).
- `DEF_SAMPLE_INTERVAL = 50` ms — sample interval during hold/measurement loops.
- `HOLD_TIME = 15000` ms — total time to keep cuff inflated and record oscillations (15 s).
- `VALVE_OPEN_INTERVAL = 3000` ms — every 3 s the valve is briefly opened.
- `VALVE_OPEN_DURATION = 500` ms — valve open duration (500 ms).
- `PRESSURE_TOPUP_THRESHOLD = 2.0` mmHg — if pressure drops by more than this threshold, pump briefly top-ups.

Calibration constants that convert ADS1115 raw reading to mmHg:

- `ADC_ZERO = 714` (ADC reading representing 0 mmHg)
- `ADC_REF = 4825` (ADC reading representing reference pressure)
- `BP_REF = 90.0` (mmHg that corresponds to ADC_REF)

From these the code computes `mmHg_per_adc = BP_REF / (ADC_REF - ADC_ZERO)` and uses:

```
mmHgRaw = (raw_adc - ADC_ZERO) * mmHg_per_adc
```

The pressure reading is smoothed using a circular buffer `pressureBuffer` with length `MA_LEN`.

Flow:
1. Inflate cuff gradually by starting pump and monitoring `mmHg`. The pump is stopped when `mmHg >= TARGET_PRESSURE` or `mmHg > SAFETY_MAX`.
2. Pulse the solenoid valve and maintain the cuff while sampling pressure values for up to `HOLD_TIME`. During the hold the pump may top-up if pressure drops below `TARGET_PRESSURE - PRESSURE_TOPUP_THRESHOLD`.
3. While holding, collect arrays:
   - `pressures[]` — pressure values at each sample
   - `oscillations[]` — absolute difference between current and previous sample (simple oscillation magnitude)
4. After collection, find the index `peakIndex` where `oscillations[i]` was maximum (`maxOsc`).
5. Determine systolic pressure: the first pressure value before (or at) the peak where the oscillation magnitude >= 50% of maxOsc.
6. Determine diastolic pressure: the first pressure value after the peak where the oscillation magnitude <= 70% of maxOsc.

These thresholds (0.5 for systolic, 0.7 for diastolic) are hard-coded and are a simplified heuristic. Real oscillometric algorithms are more involved and often use curve-fitting and pulse amplitude envelopes; treat this as a proof-of-concept.

Finally the cuff is released by opening the solenoid and final `last_systole` and `last_diastole` are stored and shown.

## Important code sections (quick map)

- `setup()` — initializes serial, I2C, MLX90614, ADS1115, SpO2 sensor, L298N pins, TFT/LVGL UI.
- `loop()` — LVGL tick/handler + reads serial commands.
- `measureTemperature()` — 2s average of MLX90614 outputs.
- `measureHeartSpO2()` — 4s collection + library calculation for SpO2 & heart.
- `measureBloodPressure()` — inflation, hold, collect oscillations and derive systole/diastole.
- `readPressureSensor()` — reads ADS1115 channel 0, converts to mmHg and applies moving average.
- `startPumpGradual()` / `stopPump()` — helper to start/stop the pump (via motor driver pins).

## How to build & flash (PlatformIO)

In PowerShell (or the PlatformIO VSCode UI), from the project root run:

```powershell
# Build
pio run -e rymcu-esp32-s3-devkitc-1

# Upload/Flash (ensure board is in programming mode or auto-uploadable)
pio run -e rymcu-esp32-s3-devkitc-1 -t upload

# Open serial monitor (uses monitor_speed from platformio.ini)
pio device monitor -e rymcu-esp32-s3-devkitc-1
```

Alternatively use the PlatformIO UI in VSCode: select the `rymcu-esp32-s3-devkitc-1` environment and click Build / Upload / Monitor.

## Running & Commands

Open serial monitor at 115200 baud. Type any of the commands (case-insensitive) followed by Enter:

- `TEMP` — measure and display temperature (2s average)
- `HEART` — measure heart rate and SpO2 (4s sample)
- `BP` — measure blood pressure (inflates to `TARGET_PRESSURE`, holds and computes systolic/diastolic)

The TFT/LVGL UI will show progress during each measurement and the main screen shows the most recent values.

## Calibration

- Pressure calibration values (`ADC_ZERO`, `ADC_REF`, `BP_REF`) must be measured using a known manometer. The current values are placeholders used in this project.

To calibrate:
1. Apply a known pressure (e.g., 0 mmHg — open to atmosphere) and record the raw ADC output; set `ADC_ZERO` to that value.
2. Apply a second known pressure (e.g., 90 mmHg) and record the ADC; set `ADC_REF` to that ADC value and `BP_REF` to the corresponding mmHg (e.g., 90).
3. Recompute `mmHg_per_adc = BP_REF / (ADC_REF - ADC_ZERO)`.

## Edge cases and limitations

- ADS1115 range and gain: the code sets `ads.setGain(GAIN_ONE)` in `setup()`. Verify the selected gain matches the pressure sensor output voltage range.
- The SpO2 algorithm is library-dependent; ensure the DFRobot library documentation for sensor placement and measurement advice.
- The oscillometric BP algorithm is simplistic and may be inaccurate. Use with caution and validate with a clinical-grade reference.
- Pin mapping inconsistency: verify whether the pump uses channel A (`ENA`, `IN1`, `IN2`) or channel B (`ENB`, `IN3`, `IN4`). The code mixes `IN3`/`IN4` vs `IN1`/`IN2` which may be an error — confirm/wire accordingly.

## Troubleshooting checklist

- No I2C sensors found:
  - Verify `Wire.begin(SDA_PIN, SCL_PIN)` pins are correct. If using default I2C pins for your board, consider using `Wire.begin()` without parameters or adjust to the board's SDA/SCL pins.
  - Use an I2C scanner sketch to confirm device addresses.

- SpO2 values not stable:
  - Check finger placement, sensor contact, and that the sensor library is configured correctly.

- Pressure readings incorrect (always zero or negative):
  - Check wiring to ADS1115 A0.
  - Validate `ADC_ZERO` and `ADC_REF` calibration constants.

- Pump/solenoid do not actuate:
  - Verify L298N wiring, ensure power to motor driver and that the pump/solenoid is powered by an appropriate supply.
  - Check pin mapping in code vs wiring. The `ENB` pin is used as a digital output to toggle the solenoid; confirm whether your hardware expects PWM or digital.

- Build errors:
  - Ensure PlatformIO libraries are downloaded. Run `pio lib install` or use the VSCode PIO extension to fetch dependencies.

## Suggested improvements / next steps

- Fix/confirm pump pin mapping: align `startPumpGradual()` with the motor channel used for the pump.
- Improve oscillometric algorithm: compute pulse amplitude envelope, apply smoothing and curve fitting to derive MAP, systolic and diastolic more robustly.
- Add a configuration header (`include/config.h`) to hold pin definitions and calibration parameters for easier editing.
- Add unit-tests for conversion routines (ADC→mmHg) and small simulation harness.

## Quick reference — key constants (from `src/main.cpp`)

- I2C pins: `SDA_PIN = 8`, `SCL_PIN = 9`
- SpO2 I2C addr: `0x57`
- Pump / L298N pins: `ENA=15`, `IN1=16`, `IN2=17`, `IN3=7`, `IN4=6`, `ENB=14`
- Pressure conversion constants: `ADC_ZERO=714`, `ADC_REF=4825`, `BP_REF=90.0`
- Measurement commands: `TEMP`, `HEART`, `BP`


## Where to find more info

- LVGL: https://lvgl.io/
- TFT_eSPI: https://github.com/Bodmer/TFT_eSPI
- ADS1X15: Adafruit documentation / examples
- DFRobot MAX30102 wrapper: check the DFRobot library docs for sensor-specific notes