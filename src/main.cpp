#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_ADS1X15.h>
#include "DFRobot_BloodOxygen_S.h"
#include <SPI.h>
#include <TFT_eSPI.h>
#include <cmath>

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

// Forward declarations for GUI
void drawMainScreen();
void drawMeasurementScreen(const char* param, const char* status, const String &value);
String fmtOrDash(float v, const char* fmt = "%.1f");

// ===================== FUNCTION DECLARATIONS =====================
void measureTemperature();
void measureHeartSpO2();
void measureBloodPressure();
float readPressureSensor();

// =====================================================
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
  // ILI9488 320x240 in landscape
  tft.setRotation(1); // landscape; adjust if your wiring requires another value
  tft.fillScreen(TFT_BLACK);
  drawMainScreen();
}

// =====================================================
void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "TEMP") {
      drawMeasurementScreen("Temperature", "Starting...", "");
      measureTemperature();
    } 
    else if (cmd == "HEART") {
      drawMeasurementScreen("Heart/SpO2", "Starting...", "");
      measureHeartSpO2();
    } 
    else if (cmd == "BP") {
      drawMeasurementScreen("Blood Pressure", "Starting...", "");
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
  // Step 0: Notify user and wait before starting
  drawMeasurementScreen("Temperature", "Preparing...", "");
  delay(3000); // 3-second delay before measuring

  Serial.println("\n[Temperature Measurement Started]");

  // Step 1: Measure for 3 seconds and average
  unsigned long startTime = millis();
  float ambientSum = 0;
  float objectSum = 0;
  int count = 0;

  while (millis() - startTime < 3000) { // measure for 3 seconds
    float ambient = mlx.readAmbientTempC();
    float object = mlx.readObjectTempC();

    ambientSum += ambient;
    objectSum += object;
    count++;

    drawMeasurementScreen("Temperature", "Measuring...", 
                          String(fmtOrDash(object).c_str()) + " C");
    delay(200); // small delay between readings
  }

  // Step 2: Compute average
  float ambientAvg = ambientSum / count;
  float objectAvg = objectSum / count;

  last_ambient = ambientAvg;
  last_object = objectAvg;

  // Step 3: Serial output
  Serial.print("Ambient Avg: "); Serial.print(ambientAvg); Serial.println(" °C");
  Serial.print("Object Avg: "); Serial.print(objectAvg); Serial.println(" °C");
  Serial.println("---------------------------");

  // Step 4: Display result briefly then return to main screen
  drawMeasurementScreen("Temperature", "Done", String(fmtOrDash(objectAvg).c_str()) + " C");
  delay(1200);
  drawMainScreen();
}


// =====================================================
// ❤️ HEART RATE + SPO2 MEASUREMENT
void measureHeartSpO2() {
  Serial.println("\n[Heart Rate & SpO2 Measurement]");
  drawMeasurementScreen("Heart/SpO2", "Collecting...", "");

  spo2Sensor.sensorStartCollect();
  // Wait while showing progress
  unsigned long start = millis();
  while (millis() - start < 4000) {
    // simple spinner/progress
    drawMeasurementScreen("Heart/SpO2", "Collecting...", String((millis() - start) / 1000) + "s");
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

  spo2Sensor.sensorEndCollect(); // optional

  // display results briefly
  drawMeasurementScreen("Heart/SpO2", "Done", String((int)spo2) + "%  " + String((int)heart) + " bpm");
  delay(1400);
  drawMainScreen();
}

// 🧮 READ PRESSURE (MPX5050DP)
float readPressureSensor() {
  int16_t raw = ads.readADC_SingleEnded(0);
  float voltage = raw * 0.1875 / 1000; // ADS1115 default: 0.1875mV/bit
  float mmHg = (voltage - 0.46) / 0.032; // approx conversion to mmHg
  if (mmHg < 0) mmHg = 0;
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

  drawMeasurementScreen("Blood Pressure", "Inflating...", "");

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
    drawMeasurementScreen("Blood Pressure", "Inflating...", String((int)mmHg) + " mmHg");
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
    drawMeasurementScreen("Blood Pressure", "Deflating...", String((int)mmHg) + " mmHg");

    // Detect systolic (pressure falling past 110–100 mmHg)
    if ((mmHg <= 110) && (mark == 0)) {
      systole = mmHg;
      mark = 1;
      Serial.print("Systolic detected: ");
      Serial.println(systole);
      // show detected systolic
      drawMeasurementScreen("Blood Pressure", "Systolic", String((int)systole) + " mmHg");
    }

    // Detect diastolic (pressure falling past 85–70 mmHg)
    if ((mmHg <= 80) && (mark == 1)) {
      diastole = mmHg;
      mark = 2;
      Serial.print("Diastolic detected: ");
      Serial.println(diastole);
      drawMeasurementScreen("Blood Pressure", "Diastolic", String((int)diastole) + " mmHg");
      break;
    }

    if (mmHg < 30) break; // fully deflated
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
  drawMeasurementScreen("Blood Pressure", "Done", String((int)last_systole) + "/" + String((int)last_diastole) + " mmHg");
  delay(1400);
  drawMainScreen();
}

// ===================== GUI HELPERS =====================
String fmtOrDash(float v, const char* fmt) {
  if (isnan(v)) return String("-");
  char buf[32];
  snprintf(buf, sizeof(buf), fmt, v);
  return String(buf);
}

void drawMainScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(8, 6);
  tft.print("Patient Monitor");

  tft.setTextSize(1);
  int y = 36;
  // Temperature Ambient / Object
  tft.setCursor(8, y); tft.print("Ambient:"); tft.setCursor(120, y); tft.print(fmtOrDash(last_ambient) + " C"); y += 18;
  tft.setCursor(8, y); tft.print("Object:");  tft.setCursor(120, y); tft.print(fmtOrDash(last_object) + " C"); y += 22;

  // SpO2 / Heart
  tft.setCursor(8, y); tft.print("SpO2:"); tft.setCursor(120, y); tft.print(isnan(last_spo2) ? String("-") : String((int)last_spo2) + "%"); y += 18;
  tft.setCursor(8, y); tft.print("Heart:"); tft.setCursor(120, y); tft.print(isnan(last_heart) ? String("-") : String((int)last_heart) + " bpm"); y += 22;

  // Sensor temp
  tft.setCursor(8, y); tft.print("Sensor T:"); tft.setCursor(120, y); tft.print(fmtOrDash(last_sensor_temp) + " C"); y += 22;

  // Blood pressure
  tft.setCursor(8, y); tft.print("BP (S/D):");
  String bpVal = (isnan(last_systole) || isnan(last_diastole)) ? String("-") : String((int)last_systole) + "/" + String((int)last_diastole) + " mmHg";
  tft.setCursor(120, y); tft.print(bpVal); y += 22;

  // Footer
  tft.setCursor(8, 210);
  tft.setTextSize(1);
  tft.print("Commands via Serial: TEMP | HEART | BP");
}

void drawMeasurementScreen(const char* param, const char* status, const String &value) {
  // simple full-screen update
  tft.fillScreen(TFT_NAVY);
  tft.setTextColor(TFT_WHITE, TFT_NAVY);
  tft.setTextSize(2);
  tft.setCursor(8, 8);
  tft.print(param);

  tft.setTextSize(1);
  tft.setCursor(8, 48);
  tft.print(status);

  if (value.length()) {
    tft.setTextSize(3);
    tft.setCursor(8, 90);
    tft.print(value);
  }
}
