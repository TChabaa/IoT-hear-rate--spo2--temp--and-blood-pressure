
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_ADS1X15.h>
#include "DFRobot_BloodOxygen_S.h"

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

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, 255);
  analogWrite(ENB, 255);

  Serial.println("\n=== ESP32-S3 IoT System Ready ===");
  Serial.println("Commands: TEMP | HEART | BP");
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
    }
  }
}

// =====================================================
// 🌡️ TEMPERATURE MEASUREMENT
void measureTemperature() {
  float ambient = mlx.readAmbientTempC();
  float object = mlx.readObjectTempC();

  Serial.println("\n[Temperature Measurement]");
  Serial.print("Ambient: ");
  Serial.print(ambient);
  Serial.println(" °C");
  Serial.print("Object: ");
  Serial.print(object);
  Serial.println(" °C");
  Serial.println("---------------------------");
}

// =====================================================
// ❤️ HEART RATE + SPO2 MEASUREMENT
void measureHeartSpO2() {
  Serial.println("\n[Heart Rate & SpO2 Measurement]");
  spo2Sensor.sensorStartCollect();
  delay(4000); // wait for sensor data
  spo2Sensor.getHeartbeatSPO2();

  Serial.print("SpO2: ");
  Serial.print(spo2Sensor._sHeartbeatSPO2.SPO2);
  Serial.println("%");
  Serial.print("Heart Rate: ");
  Serial.print(spo2Sensor._sHeartbeatSPO2.Heartbeat);
  Serial.println(" bpm");
  Serial.print("Sensor Temp: ");
  Serial.print(spo2Sensor.getTemperature_C());
  Serial.println(" °C");

  spo2Sensor.sensorEndCollect(); // optional
  Serial.println("---------------------------");
}

// =====================================================
// 🩸 BLOOD PRESSURE MEASUREMENT (MPX5050DP)
void measureBloodPressure() {
  Serial.println("\n[Blood Pressure Measurement Started]");
  mark = 0;
  systole = 0;
  diastole = 0;
  mmHg_prev = 0;

  // Inflate cuff
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW); // Pump ON
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW); // Solenoid CLOSED
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
      digitalWrite(IN1, LOW); // stop pump
    }

    // Detect diastole
    if ((mmHg >= mmHg_prev + 1) && (mmHg > 50) && (mmHg < 90) && (mark == 1)) {
      diastole = mmHg;
      mark = 2;
      Serial.print("Diastole detected: ");
      Serial.print(diastole);
      Serial.println(" mmHg");
    }

    // Stop inflation if too high
    if (mmHg > 150) {
      digitalWrite(IN1, LOW);
    }

    mmHg_prev = mmHg;
    delay(100);
  }

  // Deflate cuff
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); // Open solenoid
  delay(2000);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  Serial.println("---------------------------");
  Serial.print("Final Systolic: ");
  Serial.print(systole);
  Serial.println(" mmHg");
  Serial.print("Final Diastolic: ");
  Serial.print(diastole);
  Serial.println(" mmHg");
  Serial.println("[Measurement Complete]");
  Serial.println("---------------------------");
}

// =====================================================
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
