/*
 * Autonomous Pipeline Inspection Drone
 * ESP32 Sensor Firmware
 *
 * Reads from 9-sensor payload at 10 Hz and streams structured
 * telemetry packets to Raspberry Pi over UART at 115200 baud.
 *
 * Sensors:
 *   MQ-2   (LPG, smoke)          -> GPIO34 (ADC)
 *   MQ-4   (Methane CH4)         -> GPIO35 (ADC)
 *   MQ-8   (Hydrogen H2)         -> GPIO32 (ADC)
 *   MQ-7   (Carbon monoxide CO)  -> GPIO33 (ADC)
 *   MQ-135 (NH3, NOx, CO2)       -> GPIO25 (ADC)
 *   MQ-3   (Alcohol vapors)      -> GPIO26 (ADC)
 *   DHT11  (Temp & humidity)     -> GPIO4  (Digital)
 *   BMP280 (Pressure & altitude) -> I2C (SDA=21, SCL=22)
 *   MPU6050 (IMU accel + gyro)   -> I2C (SDA=21, SCL=22)
 *   GP2Y1010AU0F (PM2.5)         -> GPIO27 (ADC), GPIO14 (LED control)
 *
 * Output packet format (115200 baud, '\n' terminated):
 *   M2=<val>; M4=<val>; M8=<val>; M7=<val>; M135=<val>; M3=<val>;
 *   PM25=<val>; TEMP=<val>; HUM=<val>; PRESS=<val>; ALT=<val>;
 *   AX=<val>; AY=<val>; AZ=<val>; GX=<val>; GY=<val>; GZ=<val>
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

// ── Pin definitions ───────────────────────────────────────────────────────────
#define PIN_MQ2    34
#define PIN_MQ4    35
#define PIN_MQ8    32
#define PIN_MQ7    33
#define PIN_MQ135  25
#define PIN_MQ3    26
#define PIN_PM25   27
#define PIN_PM_LED 14
#define PIN_DHT    4
#define DHT_TYPE   DHT11

// ── Sensor calibration constants ──────────────────────────────────────────────
// C = a * (RS/R0)^b  — power-law model per MQ datasheet curves
// R0 values set after two-point calibration in clean air + reference gas
struct MQCalib {
  float R0;   // baseline resistance in clean air (ohms, normalised)
  float a;    // curve constant
  float b;    // curve exponent (negative for most MQ sensors)
  float RL;   // load resistance on the breakout board (kohm)
};

MQCalib mq2   = { 9.83f,  574.25f, -2.222f, 5.0f };
MQCalib mq4   = { 4.40f,  1012.7f, -2.786f, 5.0f };
MQCalib mq8   = { 70.0f,  976.97f, -0.688f, 10.0f };
MQCalib mq7   = { 27.5f,  99.042f, -1.518f, 10.0f };
MQCalib mq135 = { 76.6f,  110.47f, -2.862f, 20.0f };
MQCalib mq3   = { 60.0f,  0.3934f, -1.504f, 200.0f };

// ── EMA state ────────────────────────────────────────────────────────────────
const float ALPHA = 0.15f;
float ema_mq2 = 0, ema_mq4 = 0, ema_mq8 = 0;
float ema_mq7 = 0, ema_mq135 = 0, ema_mq3 = 0, ema_pm25 = 0;

// ── Objects ──────────────────────────────────────────────────────────────────
DHT          dht(PIN_DHT, DHT_TYPE);
Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;

// ── Timing ───────────────────────────────────────────────────────────────────
const uint32_t SAMPLE_INTERVAL_MS = 100;   // 10 Hz
uint32_t lastSample = 0;

// ── Sea-level pressure reference (update for local conditions) ────────────────
const float SEA_LEVEL_HPA = 1013.25f;

// ── Warmup ───────────────────────────────────────────────────────────────────
const uint32_t WARMUP_MS = 60000;   // MQ sensors need 60s to stabilise
bool warmedUp = false;

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

float adcToVoltage(int raw) {
  return raw * (3.3f / 4095.0f);
}

/**
 * Convert raw ADC reading to ppm using the MQ power-law model.
 *   RS  = RL * (Vcc - Vout) / Vout
 *   ppm = a * (RS/R0)^b
 */
float mq_ppm(int rawADC, const MQCalib& cal) {
  float voltage = adcToVoltage(rawADC);
  if (voltage < 0.001f) return 0.0f;                   // sensor not connected
  float RS = cal.RL * (3.3f - voltage) / voltage;      // sensor resistance
  float ratio = RS / cal.R0;
  return cal.a * powf(ratio, cal.b);
}

/**
 * EMA low-pass filter.
 *   x_hat_t = alpha * x_t + (1 - alpha) * x_hat_{t-1}
 */
float ema(float newVal, float prevEma) {
  return ALPHA * newVal + (1.0f - ALPHA) * prevEma;
}

/**
 * GP2Y1010AU0F PM2.5 measurement.
 * Pulse the LED for 280 µs, sample at 280 µs, then wait 40 µs off.
 * Output voltage maps linearly to dust density (µg/m³).
 */
float readPM25() {
  digitalWrite(PIN_PM_LED, LOW);    // LED on (active low)
  delayMicroseconds(280);
  int raw = analogRead(PIN_PM25);
  delayMicroseconds(40);
  digitalWrite(PIN_PM_LED, HIGH);   // LED off
  delayMicroseconds(9680);

  float voltage = adcToVoltage(raw);
  // Datasheet linear model: density (µg/m³) = 170 * V - 0.1
  float density = 170.0f * voltage - 0.1f;
  return max(density, 0.0f);
}

// ─────────────────────────────────────────────────────────────────────────────
// Setup
// ─────────────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);

  pinMode(PIN_PM_LED, OUTPUT);
  digitalWrite(PIN_PM_LED, HIGH);   // PM2.5 LED off by default

  dht.begin();

  Wire.begin();

  if (!bmp.begin(0x76)) {
    Serial.println("ERR: BMP280 not found");
    while (1) delay(10);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  if (!mpu.begin()) {
    Serial.println("ERR: MPU6050 not found");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("INFO: Sensor warmup started (60s)...");
  delay(WARMUP_MS);
  warmedUp = true;
  Serial.println("INFO: Warmup complete. Streaming data.");
}

// ─────────────────────────────────────────────────────────────────────────────
// Main loop
// ─────────────────────────────────────────────────────────────────────────────

void loop() {
  uint32_t now = millis();
  if (now - lastSample < SAMPLE_INTERVAL_MS) return;
  lastSample = now;

  // ── Gas sensors (ppm) ────────────────────────────────────────────────────
  float raw_mq2   = mq_ppm(analogRead(PIN_MQ2),   mq2);
  float raw_mq4   = mq_ppm(analogRead(PIN_MQ4),   mq4);
  float raw_mq8   = mq_ppm(analogRead(PIN_MQ8),   mq8);
  float raw_mq7   = mq_ppm(analogRead(PIN_MQ7),   mq7);
  float raw_mq135 = mq_ppm(analogRead(PIN_MQ135), mq135);
  float raw_mq3   = mq_ppm(analogRead(PIN_MQ3),   mq3);
  float raw_pm25  = readPM25();

  // ── EMA filtering ────────────────────────────────────────────────────────
  ema_mq2   = ema(raw_mq2,   ema_mq2);
  ema_mq4   = ema(raw_mq4,   ema_mq4);
  ema_mq8   = ema(raw_mq8,   ema_mq8);
  ema_mq7   = ema(raw_mq7,   ema_mq7);
  ema_mq135 = ema(raw_mq135, ema_mq135);
  ema_mq3   = ema(raw_mq3,   ema_mq3);
  ema_pm25  = ema(raw_pm25,  ema_pm25);

  // ── DHT11 ────────────────────────────────────────────────────────────────
  float temp = dht.readTemperature();
  float hum  = dht.readHumidity();
  if (isnan(temp)) temp = -1.0f;
  if (isnan(hum))  hum  = -1.0f;

  // ── BMP280 ───────────────────────────────────────────────────────────────
  float pressure = bmp.readPressure() / 100.0f;   // Pa -> hPa
  float altitude = bmp.readAltitude(SEA_LEVEL_HPA);

  // ── MPU6050 ──────────────────────────────────────────────────────────────
  sensors_event_t accel_evt, gyro_evt, temp_evt;
  mpu.getEvent(&accel_evt, &gyro_evt, &temp_evt);
  float ax = accel_evt.acceleration.x;
  float ay = accel_evt.acceleration.y;
  float az = accel_evt.acceleration.z;
  float gx = gyro_evt.gyro.x;
  float gy = gyro_evt.gyro.y;
  float gz = gyro_evt.gyro.z;

  // ── Transmit packet ──────────────────────────────────────────────────────
  // Format: M2=<>;M4=<>;M8=<>;M7=<>;M135=<>;M3=<>;PM25=<>;
  //         TEMP=<>;HUM=<>;PRESS=<>;ALT=<>;
  //         AX=<>;AY=<>;AZ=<>;GX=<>;GY=<>;GZ=<>
  Serial.printf(
    "M2=%.1f;M4=%.1f;M8=%.1f;M7=%.1f;M135=%.1f;M3=%.1f;"
    "PM25=%.1f;TEMP=%.1f;HUM=%.1f;PRESS=%.2f;ALT=%.2f;"
    "AX=%.3f;AY=%.3f;AZ=%.3f;GX=%.3f;GY=%.3f;GZ=%.3f\n",
    ema_mq2, ema_mq4, ema_mq8, ema_mq7, ema_mq135, ema_mq3,
    ema_pm25, temp, hum, pressure, altitude,
    ax, ay, az, gx, gy, gz
  );
}
