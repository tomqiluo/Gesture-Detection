#include <Arduino_LSM9DS1.h>
#include <Wire.h>
#include "MAX30105.h"
// #include "spo2_algorithm.h" // Optional, if you plan to calculate SpO2

#define CONVERT_G_TO_MS2    9.80665f
#define FREQUENCY_HZ        50
#define INTERVAL_MS         (1000 / (FREQUENCY_HZ + 1))

static unsigned long last_interval_ms = 0;

MAX30105 particleSensor;

// Define min and max for scaling
float ax_min = -10.0f, ax_max = 10.0f;
float ay_min = -10.0f, ay_max = 10.0f;
float az_min = -10.0f, az_max = 10.0f;
float gx_min = -500.0f, gx_max = 500.0f;
float gy_min = -500.0f, gy_max = 500.0f;
float gz_min = -500.0f, gz_max = 500.0f;
float ir_min = 0.0f, ir_max = 150000.0f; // Adjust based on expected range

void setup() {
    Serial.begin(115200);
    delay(1000); // Allow time for serial port to initialize

    Serial.println("Started");

    // Initialize LSM9DS1 IMU
    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1);
    }

    // Initialize MAX30102 sensor
    if (!particleSensor.begin()) {
        Serial.println("MAX30102 was not found. Please check wiring/power.");
        while (1);
    }

    Serial.println("MAX30102 sensor found!");

    // Configure the MAX30102 sensor
    particleSensor.setup();
    particleSensor.setPulseAmplitudeRed(0x0A); // Red LED
    particleSensor.setPulseAmplitudeIR(0x1F);  // IR LED
}

// Scaling function to map data to [-20, 20]
float scaleToRange(float value, float min, float max) {
    return -20 + ((value - min) * 40 / (max - min));
}

void loop() {
    float ax, ay, az; // Accelerometer readings
    float gx, gy, gz; // Gyroscope readings
    long irValue;

    if (millis() > last_interval_ms + INTERVAL_MS) {
        last_interval_ms = millis();

        // Read accelerometer and gyroscope data
        if (IMU.readAcceleration(ax, ay, az) && IMU.readGyroscope(gx, gy, gz)) {

            // Scale accelerometer and gyroscope data to [-20, 20]
            float ax_scaled = scaleToRange(ax * CONVERT_G_TO_MS2, ax_min, ax_max);
            float ay_scaled = scaleToRange(ay * CONVERT_G_TO_MS2, ay_min, ay_max);
            float az_scaled = scaleToRange(az * CONVERT_G_TO_MS2, az_min, az_max);
            float gx_scaled = scaleToRange(gx, gx_min, gx_max);
            float gy_scaled = scaleToRange(gy, gy_min, gy_max);
            float gz_scaled = scaleToRange(gz, gz_min, gz_max);

            // Read IR value and scale to [-20, 20]
            irValue = particleSensor.getIR();
            float ir_scaled = scaleToRange((float)irValue, ir_min, ir_max);

            // Print scaled values with 2 decimal places
            Serial.print(ax_scaled, 2);
            Serial.print('\t');
            Serial.print(ay_scaled, 2);
            Serial.print('\t');
            Serial.print(az_scaled, 2);
            Serial.print('\t');
            Serial.print(gx_scaled, 2);
            Serial.print('\t');
            Serial.print(gy_scaled, 2);
            Serial.print('\t');
            Serial.print(gz_scaled, 2);
            Serial.print('\t');
            Serial.println(ir_scaled, 2);

        } else {
            Serial.println("Failed to read IMU data!");
        }
    }
}