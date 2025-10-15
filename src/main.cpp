#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_ADS1X15.h>
#include "DFRobot_BloodOxygen_S.h"
#include <SPI.h>
#include <TFT_eSPI.h>
#include <cmath>
#include <lvgl.h>

// ===================== I2C SETUP =====================
#define SDA_PIN 8
#define SCL_PIN 9
#define SPO2_I2C_ADDR 0x57

// DFRobot MAX30102 (Heart rate + SpO2)
DFRobot_BloodOxygen_S_I2C spo2Sensor(&Wire, SPO2_I2C_ADDR);
// MLX90614 (Temperature)
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// ADS1115 (Pressure sensor MPX5050DP)
Adafruit_ADS1115 ads;

// ===================== L298N CONTROL =====================
#define ENA 15
#define IN1 16
#define IN2 17
#define IN3 7
#define IN4 6
#define ENB 14

// ===================== VARIABLES =====================
float mmHg = 0;
float mmHg_prev = 0;
float systole = 0;
float diastole = 0;
int mark = 0;

// ===================== DISPLAY (TFT_eSPI) =====================
TFT_eSPI tft = TFT_eSPI(); // Uses User_Setup in TFT_eSPI library

// Stored (last) values for main screen. Use NAN to indicate missing value.
float last_ambient = NAN;
float last_object = NAN;
float last_spo2 = NAN;
float last_heart = NAN;
float last_sensor_temp = NAN;
float last_systole = NAN;
float last_diastole = NAN;

// LVGL objects
lv_display_t *disp;
static uint8_t disp_buf[LV_HOR_RES_MAX * 40 * 2]; // 2 bytes per pixel for RGB565

lv_obj_t *scr_main;
lv_obj_t *lbl_ambient;
lv_obj_t *lbl_object;
lv_obj_t *lbl_spo2;
lv_obj_t *lbl_heart;
lv_obj_t *lbl_sensor_t;
lv_obj_t *lbl_bp;

lv_obj_t *scr_measure;
lv_obj_t *lbl_measure_title;
lv_obj_t *lbl_measure_status;
lv_obj_t *lbl_measure_value;

// LVGL timer helper
unsigned long last_lv_timer = 0;
// LVGL tick helper
unsigned long last_lv_tick = 0;

void lvgl_init_display();
void showMainScreen();
// New helpers: load the measurement screen once and update labels without reloading the screen
void loadMeasurementScreen(const char* title, const char* status, const String &value);
void updateMeasurementScreen(const char* status, const String &value);

// helper: format a float or return '-' when value is NaN
String fmtOrDash(float v, const char* fmt = "%.1f") {
  if (isnan(v)) return String("-");
  char buf[32];
  snprintf(buf, sizeof(buf), fmt, v);
  return String(buf);
}

// LVGL flush callback
static void my_flush_cb(lv_display_t * disp_ptr, const lv_area_t * area, uint8_t * px_map);

// ===================== FUNCTION DECLARATIONS =====================
void measureTemperature();
void measureHeartSpO2();
void measureBloodPressure();
float readPressureSensor();

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(500);

  // Initialize MLX90614
  if (!mlx.begin()) {
    Serial.println("Failed to find MLX90614 sensor!");
  } else {
    Serial.println("MLX90614 initialized.");
  }

  // Initialize ADS1115
  ads.begin();
  ads.setGain(GAIN_ONE);
  Serial.println("ADS1115 initialized.");

  // Initialize DFRobot MAX30102
  while (!spo2Sensor.begin()) {
    Serial.println("SpO2 sensor init failed, retrying...");
    delay(1000);
  }
  Serial.println("SpO2 sensor initialized successfully.");

  // L298N setup
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  Serial.println("\n=== ESP32-S3 IoT System Ready ===");
  Serial.println("Commands: TEMP | HEART | BP");

  // Initialize TFT
  tft.init();
  tft.setRotation(0); // portrait mode
  tft.fillScreen(TFT_BLACK);

  // Initialize LVGL and load UI
  lv_init();
  lvgl_init_display();
  showMainScreen();
  // Ensure LVGL renders the first screen
  lv_timer_handler();
  delay(50);
  // init tick
  last_lv_tick = millis();
}

// =====================================================
void loop() {
  // Run LVGL timer handler periodically so UI updates and screens are flushed
  unsigned long now = millis();
  // update LVGL tick
  if (now != last_lv_tick) {
    lv_tick_inc(now - last_lv_tick);
    last_lv_tick = now;
  }
  if (now - last_lv_timer >= 20) {
    lv_timer_handler();
    last_lv_timer = now;
  }
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "TEMP") {
      loadMeasurementScreen("Temperature", "Starting...", "");
      measureTemperature();
    } 
    else if (cmd == "HEART") {
      loadMeasurementScreen("Heart/SpO2", "Starting...", "");
      measureHeartSpO2();
    } 
    else if (cmd == "BP") {
      loadMeasurementScreen("Blood Pressure", "Starting...", "");
      measureBloodPressure();
    } 
    else {
      Serial.println("Unknown command. Use: TEMP | HEART | BP");
    }
  }
}

// =====================================================
// 🌡️ TEMPERATURE MEASUREMENT
void measureTemperature() {
  // Prepare UI: load measurement screen once and then update labels in the loop
  loadMeasurementScreen("Temperature", "Preparing...", "");
  // small pause to let LVGL render
  delay(200);

  Serial.println("\n[Temperature Measurement Started]");
  unsigned long startTime = millis();
  float ambientSum = 0;
  float objectSum = 0;
  int count = 0;

  while (millis() - startTime < 2000) { // measure for 2 seconds
    float ambient = mlx.readAmbientTempC();
    float object = mlx.readObjectTempC();

    ambientSum += ambient;
    objectSum += object;
    count++;

    {
      char _tmp[32];
      if (isnan(object)) {
        snprintf(_tmp, sizeof(_tmp), "- C");
      } else {
        snprintf(_tmp, sizeof(_tmp), "%.1f C", object);
      }
      // update status/value in-place (don't reload the whole screen)
    updateMeasurementScreen("Measuring...", String(_tmp));
    // keep LVGL ticks and handlers running so UI stays responsive
    lv_tick_inc(20);
      lv_timer_handler();
      delay(200);
  }
  }

  float ambientAvg = ambientSum / (count?count:1);
  float objectAvg = objectSum / (count?count:1);

  last_ambient = ambientAvg;
  last_object = objectAvg;

  Serial.print("Ambient Avg: "); Serial.print(ambientAvg); Serial.println(" °C");
  Serial.print("Object Avg: "); Serial.print(objectAvg); Serial.println(" °C");

  {
    char _tmp2[32];
    if (isnan(objectAvg)) {
      snprintf(_tmp2, sizeof(_tmp2), "- C");
    } else {
      snprintf(_tmp2, sizeof(_tmp2), "%.1f C", objectAvg);
    }
    updateMeasurementScreen("Done", String(_tmp2));
  }
  lv_timer_handler();
  delay(900);
  showMainScreen();
}


// =====================================================
// ❤️ HEART RATE + SPO2 MEASUREMENT
void measureHeartSpO2() {
  Serial.println("\n[Heart Rate & SpO2 Measurement]");
  // use load/update helpers so UI stays responsive and we don't reload screen every loop
  loadMeasurementScreen("Heart/SpO2", "Collecting...", "");
  spo2Sensor.sensorStartCollect();
  unsigned long start = millis();
  while (millis() - start < 4000) {
    updateMeasurementScreen("Collecting...", String((millis() - start) / 1000) + "s");
    delay(250);
  }
  spo2Sensor.getHeartbeatSPO2();

  float spo2 = spo2Sensor._sHeartbeatSPO2.SPO2;
  float heart = spo2Sensor._sHeartbeatSPO2.Heartbeat;
  float stemp = spo2Sensor.getTemperature_C();

  last_spo2 = spo2;
  last_heart = heart;
  last_sensor_temp = stemp;

  Serial.print("SpO2: "); Serial.print(spo2); Serial.println("%");
  Serial.print("Heart Rate: "); Serial.print(heart); Serial.println(" bpm");
  Serial.print("Sensor Temp: "); Serial.print(stemp); Serial.println(" °C");

  spo2Sensor.sensorEndCollect();

  updateMeasurementScreen("Done", String((int)spo2) + "%  " + String((int)heart) + " bpm");
  lv_timer_handler();
  delay(1200);
  showMainScreen();
}

// 🧮 READ PRESSURE (MPX5050DP)
float readPressureSensor() {
  int16_t raw = ads.readADC_SingleEnded(0);
  float voltage = raw * 0.1875 / 1000;  // ADS1115 conversion to volts
  float pressure_kPa = (voltage / 5.0 - 0.04) / 0.009;
  float mmHg = pressure_kPa * 7.5006;
  if (mmHg < 0) mmHg = 0; // approx conversion to mmHg
  Serial.print("Pressure: ");
  Serial.print(mmHg);
  Serial.println(" mmHg");
  return mmHg;
}


// =====================================================
// 🩸 BLOOD PRESSURE MEASUREMENT (MPX5050DP)

void measureBloodPressure() {
  Serial.println("\n[Blood Pressure Measurement Started]");
  mark = 0;
  systole = 0;
  diastole = 0;

  // load screen once and update in-place
  loadMeasurementScreen("Blood Pressure", "Inflating...", "");
  delay(200);

  // === Step 1: Inflate cuff ===
  Serial.println("Inflating cuff...");
  digitalWrite(ENB, HIGH);   // close solenoid (air can't escape)
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  digitalWrite(ENA, HIGH);   // start pump
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  while (true) {
    mmHg = readPressureSensor();
    // show live pressure while inflating
    updateMeasurementScreen("Inflating...", String((int)mmHg) + " mmHg");
    if (mmHg >= 135) { // target inflation
      Serial.println("Target pressure reached.");
      break;
    }
    if (mmHg > 150) {
      Serial.println("Safety stop: pressure >150 mmHg");
      break;
    }
    delay(100);
  }

  // === Step 2: Stop pump, hold air ===
  digitalWrite(ENA, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  delay(500);

  // === Step 3: Deflate slowly ===
  Serial.println("Deflating cuff...");
  digitalWrite(ENB, LOW); // open solenoid (air can escape)
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  unsigned long startDeflate = millis();
  while (millis() - startDeflate < 15000) {
    mmHg = readPressureSensor();
    // update screen while deflating
    updateMeasurementScreen("Deflating...", String((int)mmHg) + " mmHg");

    // Detect systolic (pressure falling past 110–100 mmHg)
    if ((mmHg <= 110) && (mark == 0)) {
      systole = mmHg;
      mark = 1;
      Serial.print("Systolic detected: ");
      Serial.println(systole);
  // show detected systolic
  updateMeasurementScreen("Systolic", String((int)systole) + " mmHg");
    }

    // Detect diastolic (pressure falling past 85–70 mmHg)
    if ((mmHg <= 80) && (mark == 1)) {
      diastole = mmHg;
      mark = 2;
      Serial.print("Diastolic detected: ");
      Serial.println(diastole);
  updateMeasurementScreen("Diastolic", String((int)diastole) + " mmHg");
      break;
    }

    if (mmHg < 30) break; // fully deflated
    // keep LVGL responsive
    lv_tick_inc(20);
    lv_timer_handler();
    delay(200);
  }

  // === Step 4: Ensure cuff is deflated ===
  digitalWrite(ENB, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  Serial.println("---------------------------");
  Serial.print("Final Systolic: "); Serial.println(systole);
  Serial.print("Final Diastolic: "); Serial.println(diastole);
  Serial.println("[Measurement Complete]");
  Serial.println("---------------------------");

  // store last values
  last_systole = systole;
  last_diastole = diastole;

  // show final results briefly then return to main
  updateMeasurementScreen("Done", String((int)last_systole) + "/" + String((int)last_diastole) + " mmHg");
  lv_timer_handler();
  delay(1200);
  showMainScreen();
}

// ===================== GUI HELPERS =====================
void my_flush_cb(lv_display_t * disp_ptr, const lv_area_t * area, uint8_t * px_map) {
  int32_t w = area->x2 - area->x1 + 1;
  int32_t h = area->y2 - area->y1 + 1;
  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.pushColors((uint16_t*)px_map, w * h, true);
  tft.endWrite();
  lv_display_flush_ready(disp_ptr);
}

void lvgl_init_display() {
  // create display
  disp = lv_display_create(LV_HOR_RES_MAX, LV_VER_RES_MAX);
  // set buffers (RGB565 -> 2 bytes per pixel)
  // ensure LVGL uses RGB565 to match TFT_eSPI
  lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);
  // set buffers (RGB565 -> 2 bytes per pixel) using PARTIAL render mode
  lv_display_set_buffers(disp, disp_buf, NULL, sizeof(disp_buf), LV_DISPLAY_RENDER_MODE_PARTIAL);
  // make this the default LVGL display
  lv_display_set_default(disp);
  // set flush callback
  lv_display_set_flush_cb(disp, my_flush_cb);

  // create screens and widgets
  scr_main = lv_obj_create(NULL);
  // --- logo (minimalist modern) ---
  lv_obj_t *logo_cont = lv_obj_create(scr_main);
  lv_obj_set_size(logo_cont, 48, 48);
  lv_obj_set_style_radius(logo_cont, 12, 0);
  lv_obj_set_style_bg_color(logo_cont, lv_color_hex(0x0B6E4F), 0);
  lv_obj_set_style_bg_grad_color(logo_cont, lv_color_hex(0x0F9D58), 0);
  lv_obj_set_style_bg_grad_dir(logo_cont, LV_GRAD_DIR_VER, 0);
  lv_obj_set_style_border_width(logo_cont, 0, 0);
  lv_obj_align(logo_cont, LV_ALIGN_TOP_LEFT, 8, 8);

  // two overlapping circles inside logo
  lv_obj_t *dot1 = lv_obj_create(logo_cont);
  lv_obj_set_size(dot1, 22, 22);
  lv_obj_set_style_radius(dot1, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(dot1, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_style_border_width(dot1, 0, 0);
  lv_obj_align(dot1, LV_ALIGN_LEFT_MID, 8, 0);

  lv_obj_t *dot2 = lv_obj_create(logo_cont);
  lv_obj_set_size(dot2, 22, 22);
  lv_obj_set_style_radius(dot2, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(dot2, lv_color_hex(0xFFFFFF), 0); // subtle overlay (white)
  lv_obj_set_style_bg_opa(dot2, LV_OPA_70, 0);
  lv_obj_set_style_border_width(dot2, 0, 0);
  lv_obj_align(dot2, LV_ALIGN_RIGHT_MID, -8, 0);

  // Title placed to the right of logo
  lv_obj_t *title = lv_label_create(scr_main);
  lv_label_set_text(title, "Patient Monitor");
  lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
  lv_obj_align_to(title, logo_cont, LV_ALIGN_OUT_RIGHT_MID, 12, 0);

  lbl_ambient = lv_label_create(scr_main);
  lv_label_set_text(lbl_ambient, "Ambient: - C");
  lv_obj_set_style_text_font(lbl_ambient, &lv_font_montserrat_14, 0);
  lv_obj_align(lbl_ambient, LV_ALIGN_TOP_LEFT, 8, 72);

  lbl_object = lv_label_create(scr_main);
  lv_label_set_text(lbl_object, "Object: - C");
  lv_obj_set_style_text_font(lbl_object, &lv_font_montserrat_14, 0);
  lv_obj_align(lbl_object, LV_ALIGN_TOP_LEFT, 8, 94);

  lbl_spo2 = lv_label_create(scr_main);
  lv_label_set_text(lbl_spo2, "SpO2: -");
  lv_obj_set_style_text_font(lbl_spo2, &lv_font_montserrat_14, 0);
  lv_obj_align(lbl_spo2, LV_ALIGN_TOP_LEFT, 8, 118);

  lbl_heart = lv_label_create(scr_main);
  lv_label_set_text(lbl_heart, "Heart: - bpm");
  lv_obj_set_style_text_font(lbl_heart, &lv_font_montserrat_14, 0);
  lv_obj_align(lbl_heart, LV_ALIGN_TOP_LEFT, 8, 140);

  lbl_sensor_t = lv_label_create(scr_main);
  lv_label_set_text(lbl_sensor_t, "Sensor T: - C");
  lv_obj_set_style_text_font(lbl_sensor_t, &lv_font_montserrat_14, 0);
  lv_obj_align(lbl_sensor_t, LV_ALIGN_TOP_LEFT, 8, 162);

  lbl_bp = lv_label_create(scr_main);
  lv_label_set_text(lbl_bp, "BP (S/D): -");
  lv_obj_set_style_text_font(lbl_bp, &lv_font_montserrat_14, 0);
  lv_obj_align(lbl_bp, LV_ALIGN_TOP_LEFT, 8, 186);

  lv_obj_t *footer = lv_label_create(scr_main);
  lv_label_set_text(footer, "Commands via Serial: TEMP | HEART | BP");
  lv_obj_set_style_text_font(footer, &lv_font_montserrat_12, 0);
  lv_obj_align(footer, LV_ALIGN_BOTTOM_LEFT, 8, -8);

  // measurement screen
  scr_measure = lv_obj_create(NULL);
  // small logo on measurement screen
  lv_obj_t *logo_small = lv_obj_create(scr_measure);
  lv_obj_set_size(logo_small, 36, 36);
  lv_obj_set_style_radius(logo_small, 10, 0);
  lv_obj_set_style_bg_color(logo_small, lv_color_hex(0x0B6E4F), 0);
  lv_obj_set_style_bg_grad_color(logo_small, lv_color_hex(0x0F9D58), 0);
  lv_obj_set_style_bg_grad_dir(logo_small, LV_GRAD_DIR_VER, 0);
  lv_obj_set_style_border_width(logo_small, 0, 0);
  lv_obj_align(logo_small, LV_ALIGN_TOP_LEFT, 8, 8);

  lbl_measure_title = lv_label_create(scr_measure);
  lv_label_set_text(lbl_measure_title, "");
  lv_obj_set_style_text_font(lbl_measure_title, &lv_font_montserrat_18, 0);
  lv_obj_align(lbl_measure_title, LV_ALIGN_TOP_MID, 0, 8);

  lbl_measure_status = lv_label_create(scr_measure);
  lv_label_set_text(lbl_measure_status, "");
  lv_obj_set_style_text_font(lbl_measure_status, &lv_font_montserrat_14, 0);
  lv_obj_align(lbl_measure_status, LV_ALIGN_TOP_LEFT, 8, 56);

  lbl_measure_value = lv_label_create(scr_measure);
  lv_label_set_text(lbl_measure_value, "");
  lv_obj_set_style_text_font(lbl_measure_value, &lv_font_montserrat_26, 0);
  lv_obj_align(lbl_measure_value, LV_ALIGN_CENTER, 0, 10);
}

void showMainScreen() {
  // Clear the screen
  lv_obj_clean(scr_main);

  // Create a container for the cards
  lv_obj_t *container = lv_obj_create(scr_main);
  lv_obj_set_size(container, LV_HOR_RES_MAX, LV_VER_RES_MAX);
  lv_obj_set_style_pad_all(container, 8, 0);
  lv_obj_set_style_bg_color(container, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_style_bg_opa(container, LV_OPA_COVER, 0);

  // Create the first card for temperature data (parameter on first line, value on second)
  const int horiz_margin = 16; // reduced side margin to fit more content
  const int card_h = 64; // increased height so parameter name and value are fully visible
  lv_obj_t *card_temp = lv_obj_create(container);
  lv_obj_set_size(card_temp, LV_HOR_RES_MAX - horiz_margin * 2, card_h);
  lv_obj_set_style_radius(card_temp, 8, 0);
  lv_obj_set_style_shadow_width(card_temp, 6, 0);
  lv_obj_set_style_shadow_color(card_temp, lv_color_hex(0xBDBDBD), 0);
  lv_obj_set_style_bg_color(card_temp, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_style_border_color(card_temp, lv_color_hex(0xE0E0E0), 0);
  lv_obj_set_style_border_width(card_temp, 0, 0);
  lv_obj_align(card_temp, LV_ALIGN_TOP_MID, 0, 56); // more space under title

  // Parameter label (bold, slightly larger)
  lv_obj_t *temp_param = lv_label_create(card_temp);
  lv_label_set_text(temp_param, "Temperature");
  lv_obj_set_style_text_font(temp_param, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(temp_param, lv_color_hex(0x212121), 0);
  // left-align the parameter and position it slightly above center so
  // the parameter + value pair are vertically centred within the card
  lv_obj_align(temp_param, LV_ALIGN_LEFT_MID, 12, -8);

  // Value label (prominent but smaller to fit)
  lv_obj_t *temp_value = lv_label_create(card_temp);
  {
    char _buf[48];
    if (isnan(last_object)) {
      snprintf(_buf, sizeof(_buf), "- C");
    } else {
      snprintf(_buf, sizeof(_buf), "%.1f C", last_object);
    }
    lv_label_set_text(temp_value, _buf);
  }
  lv_obj_set_style_text_font(temp_value, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(temp_value, lv_color_hex(0x1B5E20), 0);
  // place the value to the left and below the parameter so the pair
  // appears vertically centered and left-aligned within the card
  lv_obj_align(temp_value, LV_ALIGN_LEFT_MID, 12, 12);

  // Create the second card for heart rate and SpO2 data
  // Create the second card for heart rate and SpO2
  lv_obj_t *card_heart = lv_obj_create(container);
  lv_obj_set_size(card_heart, LV_HOR_RES_MAX - horiz_margin * 2, card_h);
  lv_obj_set_style_radius(card_heart, 8, 0);
  lv_obj_set_style_shadow_width(card_heart, 6, 0);
  lv_obj_set_style_shadow_color(card_heart, lv_color_hex(0xCFCFCF), 0);
  lv_obj_set_style_bg_color(card_heart, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_style_border_color(card_heart, lv_color_hex(0xE0E0E0), 0);
  lv_obj_set_style_border_width(card_heart, 0, 0);
  lv_obj_align_to(card_heart, card_temp, LV_ALIGN_OUT_BOTTOM_MID, 0, 24);

  lv_obj_t *heart_param = lv_label_create(card_heart);
  lv_label_set_text(heart_param, "Heart Rate & SpO2");
  lv_obj_set_style_text_font(heart_param, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(heart_param, lv_color_hex(0x212121), 0);
  // left-align the parameter and position it near the vertical middle
  lv_obj_align(heart_param, LV_ALIGN_LEFT_MID, 12, -8);

  lv_obj_t *heart_value = lv_label_create(card_heart);
  {
    char _buf2[64];
    snprintf(_buf2, sizeof(_buf2), "%s bpm  %s%%", isnan(last_heart) ? "--" : String((int)last_heart).c_str(), isnan(last_spo2) ? "--" : String((int)last_spo2).c_str());
    lv_label_set_text(heart_value, _buf2);
  }
  lv_obj_set_style_text_font(heart_value, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(heart_value, lv_color_hex(0xE53935), 0);
  // left-align value under the parameter so it's vertically centered
  lv_obj_align(heart_value, LV_ALIGN_LEFT_MID, 12, 12);

  // Create the third card for blood pressure data
  // Create the third card for blood pressure
  lv_obj_t *card_bp = lv_obj_create(container);
  lv_obj_set_size(card_bp, LV_HOR_RES_MAX - horiz_margin * 2, card_h);
  lv_obj_set_style_radius(card_bp, 8, 0);
  lv_obj_set_style_shadow_width(card_bp, 6, 0);
  lv_obj_set_style_shadow_color(card_bp, lv_color_hex(0xCFCFCF), 0);
  lv_obj_set_style_bg_color(card_bp, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_style_border_color(card_bp, lv_color_hex(0xE0E0E0), 0);
  lv_obj_set_style_border_width(card_bp, 0, 0);
  lv_obj_align_to(card_bp, card_heart, LV_ALIGN_OUT_BOTTOM_MID, 0, 24);

  lv_obj_t *bp_param = lv_label_create(card_bp);
  lv_label_set_text(bp_param, "Blood Pressure");
  lv_obj_set_style_text_font(bp_param, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(bp_param, lv_color_hex(0x212121), 0);
  // left-align the parameter and position it near the vertical middle
  lv_obj_align(bp_param, LV_ALIGN_LEFT_MID, 12, -8);

  lv_obj_t *bp_value = lv_label_create(card_bp);
  if (isnan(last_systole) || isnan(last_diastole)) {
    lv_label_set_text(bp_value, "-- / -- mmHg");
  } else {
    {
      char _buf3[48];
      snprintf(_buf3, sizeof(_buf3), "%d / %d mmHg", (int)last_systole, (int)last_diastole);
      lv_label_set_text(bp_value, _buf3);
    }
  }
  lv_obj_set_style_text_font(bp_value, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(bp_value, lv_color_hex(0x1565C0), 0);
  // left-align BP value under the parameter so the pair is centered
  lv_obj_align(bp_value, LV_ALIGN_LEFT_MID, 12, 12);

  // Set a background color for the main screen
  lv_obj_set_style_bg_color(scr_main, lv_color_hex(0xE8F5E9), 0); // Light green background
  lv_obj_set_style_bg_opa(scr_main, LV_OPA_COVER, 0);

  // Update the title style
  lv_obj_t *title = lv_label_create(scr_main);
  lv_label_set_text(title, "Health Monitor");
  lv_obj_set_style_text_font(title, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(title, lv_color_hex(0x1B5E20), 0); // Dark green text
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 8);

  // Add a decorative line below the title
  lv_obj_t *line = lv_obj_create(scr_main);
  lv_obj_set_size(line, LV_HOR_RES_MAX - 16, 2);
  lv_obj_set_style_bg_color(line, lv_color_hex(0x1B5E20), 0); // Dark green line
  lv_obj_set_style_bg_opa(line, LV_OPA_COVER, 0);
  lv_obj_align(line, LV_ALIGN_TOP_MID, 0, 40);

  // Load the updated screen
  lv_scr_load(scr_main);
  lv_timer_handler();
}

// NOTE: showMeasurementScreen removed in favor of loadMeasurementScreen/updateMeasurementScreen

// Load the measurement screen once (keeps title) and render
void loadMeasurementScreen(const char* title, const char* status, const String &value) {
  lv_label_set_text(lbl_measure_title, title);
  lv_label_set_text(lbl_measure_status, status);
  lv_label_set_text(lbl_measure_value, value.c_str());
  lv_scr_load(scr_measure);
  lv_timer_handler();
}

// Update status and value while the measurement screen is already loaded
void updateMeasurementScreen(const char* status, const String &value) {
  if (lbl_measure_status) lv_label_set_text(lbl_measure_status, status);
  if (lbl_measure_value) lv_label_set_text(lbl_measure_value, value.c_str());
  // flush UI and advance ticks so updates appear in real time
  lv_tick_inc(20);
  lv_timer_handler();
}
