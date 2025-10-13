#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_ADS1X15.h>
#include "DFRobot_BloodOxygen_S.h"
#include <LiquidCrystal_I2C.h>

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

// LCD 16x2 (I2C address 0x27)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ===================== L298N CONTROL =====================
#define ENA 15
#define IN1 16
#define IN2 17
#define IN3 12
#define IN4 13
#define ENB 14

// ===================== VARIABLES =====================
float mmHg = 0;
float mmHg_prev = 0;
float systole = 0;
float diastole = 0;
int mark = 0;

// ===================== FUNCTION DECLARATIONS =====================
void measureTemperature();
void measureHeartSpO2();
void measureBloodPressure();
float readPressureSensor();
void lcdShow(const String &line1, const String &line2);

// =====================================================
void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(500);

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcdShow("ESP32-S3 IoT", "Initializing...");
  delay(1000);

  // Initialize MLX90614
  if (!mlx.begin()) {
    Serial.println("Failed to find MLX90614 sensor!");
    lcdShow("MLX90614", "Init failed!");
  } else {
    Serial.println("MLX90614 initialized.");
    lcdShow("MLX90614", "OK");
  }

  // Initialize ADS1115
  ads.begin();
  ads.setGain(GAIN_ONE);
  Serial.println("ADS1115 initialized.");
  lcdShow("ADS1115", "OK");
  delay(500);

  // Initialize DFRobot MAX30102
  while (!spo2Sensor.begin()) {
    Serial.println("SpO2 sensor init failed, retrying...");
    lcdShow("SpO2 Sensor", "Retry...");
    delay(1000);
  }
  Serial.println("SpO2 sensor initialized successfully.");
  lcdShow("SpO2 Sensor", "OK");
  delay(500);

  // L298N setup
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, 255);
  analogWrite(ENB, 255);

  Serial.println("\n=== ESP32-S3 IoT System Ready ===");
  Serial.println("Commands: TEMP | HEART | BP");

  lcdShow("System Ready", "TEMP|HEART|BP");
}

// =====================================================
void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "TEMP") {
      measureTemperature();
    } 
    else if (cmd == "HEART") {
      measureHeartSpO2();
    } 
    else if (cmd == "BP") {
      measureBloodPressure();
    } 
    else {
      Serial.println("Unknown command. Use: TEMP | HEART | BP");
      lcdShow("Unknown CMD", "TEMP|HEART|BP");
    }
  }
}

// =====================================================
// 🌡️ TEMPERATURE MEASUREMENT
void measureTemperature() {
  float ambient = mlx.readAmbientTempC();
  float object = mlx.readObjectTempC();

  Serial.println("\n[Temperature Measurement]");
  Serial.print("Ambient: "); Serial.print(ambient); Serial.println(" °C");
  Serial.print("Object: "); Serial.print(object); Serial.println(" °C");
  Serial.println("---------------------------");

  lcdShow("TEMP (°C)", "Amb:" + String(ambient,1) + " Obj:" + String(object,1));
}

// =====================================================
// ❤️ HEART RATE + SPO2 MEASUREMENT
void measureHeartSpO2() {
  Serial.println("\n[Heart Rate & SpO2 Measurement]");
  lcdShow("Measuring", "Heart+SpO2...");
  spo2Sensor.sensorStartCollect();
  delay(4000); // Wait for stable data
  spo2Sensor.getHeartbeatSPO2();

  float spo2 = spo2Sensor._sHeartbeatSPO2.SPO2;
  float hr = spo2Sensor._sHeartbeatSPO2.Heartbeat;
  float temp = spo2Sensor.getTemperature_C();

  Serial.print("SpO2: "); Serial.print(spo2); Serial.println("%");
  Serial.print("Heart Rate: "); Serial.print(hr); Serial.println(" bpm");
  Serial.print("Sensor Temp: "); Serial.print(temp); Serial.println(" °C");
  Serial.println("---------------------------");

  lcdShow("SpO2:" + String(spo2,1) + "%", "HR:" + String(hr,1) + "bpm");
  delay(2000);
  lcdShow("Sensor Temp", String(temp,1) + " C");
  delay(2000);

  spo2Sensor.sensorEndCollect();
  lcdShow("Done", "----------------");
}

// =====================================================
// 🩸 BLOOD PRESSURE MEASUREMENT (MPX5050DP)
void measureBloodPressure() {
  Serial.println("\n[Blood Pressure Measurement Started]");
  lcdShow("BP Measure", "Inflating...");

  mark = 0;
  systole = 0;
  diastole = 0;
  mmHg_prev = 0;

  // Inflate cuff
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);  // Pump ON
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);  // Solenoid CLOSED
  delay(500);

  unsigned long startTime = millis();
  while (millis() - startTime < 15000) { // run for 15s max
    mmHg = readPressureSensor();

    // Detect systole
    if ((mmHg >= mmHg_prev + 2) && (mmHg > 100) && (mark == 0)) {
      systole = mmHg;
      mark = 1;
      Serial.print("Systole detected: ");
      Serial.print(systole);
      Serial.println(" mmHg");
      lcdShow("Systole Detected", String(systole,1) + " mmHg");
      digitalWrite(IN1, LOW); // Stop pump
    }

    // Detect diastole
    if ((mmHg >= mmHg_prev + 1) && (mmHg > 50) && (mmHg < 90) && (mark == 1)) {
      diastole = mmHg;
      mark = 2;
      Serial.print("Diastole detected: ");
      Serial.print(diastole);
      Serial.println(" mmHg");
      lcdShow("Diastole", String(diastole,1) + " mmHg");
    }

    // Stop inflation if too high
    if (mmHg > 150) {
      digitalWrite(IN1, LOW);
    }

    mmHg_prev = mmHg;
    delay(100);
  }

  // Deflate cuff
  lcdShow("Deflating", "...");
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); // Open solenoid
  delay(2000);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  Serial.println("---------------------------");
  Serial.print("Final Systolic: "); Serial.println(systole);
  Serial.print("Final Diastolic: "); Serial.println(diastole);
  Serial.println("[Measurement Complete]");
  Serial.println("---------------------------");

  lcdShow("Systolic: " + String(systole,1), "Diastolic: " + String(diastole,1));
  delay(4000);
  lcdShow("BP Done", "----------------");
}

// =====================================================
// 🧮 READ PRESSURE (MPX5050DP)
float readPressureSensor() {
  int16_t raw = ads.readADC_SingleEnded(0);
  float voltage = raw * 0.1875 / 1000; // ADS1115: 0.1875mV per bit
  float mmHg = (voltage - 0.46) / 0.032; // convert to mmHg
  if (mmHg < 0) mmHg = 0;
  Serial.print("Pressure: "); Serial.print(mmHg); Serial.println(" mmHg");
  return mmHg;
}

// =====================================================
// 🖥️ LCD HELPER
void lcdShow(const String &line1, const String &line2) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}
