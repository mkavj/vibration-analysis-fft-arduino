#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "arduinoFFT.h"

#define SAMPLES 64
#define SAMPLING_FREQ 50  // sampling frequency in Hz

Adafruit_MPU6050 czujnik;
ArduinoFFT<double> FFT = ArduinoFFT<double>();

double vReal[SAMPLES];
double vImag[SAMPLES];

float velocity_x = 0;
float position_x = 0;
unsigned long last_time = 0;
float calibration_offset_x = 0;

unsigned int sampling_period_us;
unsigned long microseconds;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!czujnik.begin()) {
    Serial.println("Error: GY-521 not detected. Check wiring (SDA->A4, SCL->A5).");
    while (1);
  }

  czujnik.setAccelerometerRange(MPU6050_RANGE_2_G);
  czujnik.setFilterBandwidth(MPU6050_BAND_44_HZ);
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQ));

  // calibration - keep sensor flat and still
  Serial.println("Calibrating... keep sensor flat.");
  delay(1000);
  float sum = 0;
  for(int i=0; i<100; i++) {
    sensors_event_t a, g, temp;
    czujnik.getEvent(&a, &g, &temp);
    sum += a.acceleration.x;
    delay(10);
  }
  calibration_offset_x = sum / 100.0;
  Serial.print("Calibration done. Offset: ");
  Serial.println(calibration_offset_x);
  delay(1000);
  last_time = micros();
}

void loop() {
  // collect 64 samples
  for(int i = 0; i < SAMPLES; i++) {
    microseconds = micros();
    sensors_event_t a, g, temp;
    czujnik.getEvent(&a, &g, &temp);

    unsigned long current_time = micros();
    float dt = (current_time - last_time) / 1000000.0;
    last_time = current_time;

    // remove gravity offset
    float accel_x = a.acceleration.x - calibration_offset_x;

    // dead zone filter to reduce drift
    if (abs(accel_x) < 0.15) accel_x = 0;

    // integrate: acceleration -> velocity -> position
    velocity_x += accel_x * dt;
    position_x += velocity_x * dt;

    Serial.print("AccX:"); Serial.print(accel_x);
    Serial.print("\tPosX:"); Serial.print(position_x);

    vReal[i] = a.acceleration.x;
    vImag[i] = 0;

    while(micros() - microseconds < sampling_period_us) {}
  }

  // FFT analysis - runs once per 64 samples
  Serial.println();
  Serial.println("--- FFT ANALYSIS ---");

  FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, SAMPLES);

  for(int i = 2; i < (SAMPLES/2); i++) {
    double frequency = (i * 1.0 * SAMPLING_FREQ) / SAMPLES;
    double accel_amplitude = vReal[i];

    // convert acceleration to displacement: D = A / omega^2
    double omega = 2.0 * PI * frequency;
    double displacement_mm = 0;

    if (frequency > 1.0) {
      displacement_mm = (accel_amplitude / (omega * omega)) * 1000.0;
    }

    if (displacement_mm > 0.01) {
      Serial.print("Freq: "); Serial.print(frequency, 1);
      Serial.print(" Hz -> Displacement: "); Serial.print(displacement_mm, 3);
      Serial.println(" mm");
    }
  }
  Serial.println("--------------------");
}
