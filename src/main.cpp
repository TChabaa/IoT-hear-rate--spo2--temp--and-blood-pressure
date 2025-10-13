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

  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

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

    // Detect systolic (pressure falling past 110–100 mmHg)
    if ((mmHg <= 110) && (mark == 0)) {
      systole = mmHg;
      mark = 1;
      Serial.print("Systolic detected: ");
      Serial.println(systole);
    }

    // Detect diastolic (pressure falling past 85–70 mmHg)
    if ((mmHg <= 80) && (mark == 1)) {
      diastole = mmHg;
      mark = 2;
      Serial.print("Diastolic detected: ");
      Serial.println(diastole);
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
}
