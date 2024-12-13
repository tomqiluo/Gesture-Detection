#include <Gesture_Detection_inferencing.h>
#include <Arduino_LSM9DS1.h>
#include <Wire.h>
#include "MAX30105.h"

#define CONVERT_G_TO_MS2    9.80665f
#define MAX_ACCEPTED_RANGE  20.0f  // Scaled to match [-20, 20] range
#define FREQUENCY_HZ        50
#define INTERVAL_MS         (1000 / (FREQUENCY_HZ + 1))

const int bluePin = 5;
const int greenPin = 4;
const int yellowPin = 3;
const int motorPin1 = 6;
const int motorPin2 = 7;

static unsigned long last_interval_ms = 0;
MAX30105 particleSensor;
static float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };
static float inference_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
static rtos::Thread inference_thread(osPriorityLow);

// Scaling function to map data to [-20, 20]
float scaleToRange(float value, float min, float max) {
    return -20 + ((value - min) * 40 / (max - min));
}

void setup() {
    pinMode(bluePin, OUTPUT);
    pinMode(yellowPin, OUTPUT);
    pinMode(greenPin, OUTPUT);
    pinMode(motorPin1, OUTPUT);
    pinMode(motorPin2, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);
    while (!Serial);
    Serial.println("Edge Impulse Inferencing Demo");

    // Initialize LSM9DS1 IMU
    if (!IMU.begin()) {
        ei_printf("Failed to initialize IMU!\r\n");
        while (1);
    }

    // Initialize MAX30102 sensor
    if (!particleSensor.begin()) {
        ei_printf("MAX30102 was not found. Please check wiring/power.\r\n");
        while (1);
    }

    particleSensor.setup();
    particleSensor.setPulseAmplitudeRed(0x0A); // Red LED
    particleSensor.setPulseAmplitudeIR(0x1F);  // IR LED

    delay(1000);
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, HIGH);
    digitalWrite(bluePin, HIGH);
    delay(2000); 
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);

    inference_thread.start(mbed::callback(&run_inference_background));
}

void run_inference_background() {
    ei_classifier_smooth_t smooth;
    ei_classifier_smooth_init(&smooth, 10, 7, 0.8, 0.3);

    while (1) {
        // Copy the buffer to inference_buffer
        memcpy(inference_buffer, buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE * sizeof(float));

        // Create a signal from the buffer
        signal_t signal;
        int err = numpy::signal_from_buffer(inference_buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
        if (err != 0) {
            ei_printf("Failed to create signal from buffer (%d)\n", err);
            return;
        }

        ei_impulse_result_t result = { 0 };
        err = run_classifier(&signal, &result, false);
        if (err != EI_IMPULSE_OK) {
            ei_printf("Failed to run classifier (%d)\n", err);
            return;
        }

        ei_printf("Predictions: ");
        const char* prediction = ei_classifier_smooth_update(&smooth, &result);
        ei_printf("%s\n", prediction);

        if ((strcmp(prediction, "Pinch") == 0 )) {
            // Confirm
            digitalWrite(greenPin, HIGH);
            digitalWrite(bluePin, LOW);
            digitalWrite(yellowPin, LOW);
        } else if ((strcmp(prediction, "Clench") == 0 )) {
            // Remove message
            digitalWrite(yellowPin, HIGH);
            digitalWrite(greenPin, LOW);
            digitalWrite(bluePin, LOW);
        }
    }
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
            float ax_scaled = scaleToRange(ax * CONVERT_G_TO_MS2, -10.0f, 10.0f);
            float ay_scaled = scaleToRange(ay * CONVERT_G_TO_MS2, -10.0f, 10.0f);
            float az_scaled = scaleToRange(az * CONVERT_G_TO_MS2, -10.0f, 10.0f);
            float gx_scaled = scaleToRange(gx, -500.0f, 500.0f);
            float gy_scaled = scaleToRange(gy, -500.0f, 500.0f);
            float gz_scaled = scaleToRange(gz, -500.0f, 500.0f);

            // Read IR value and scale to [-20, 20]
            irValue = particleSensor.getIR();
            float ir_scaled = scaleToRange((float)irValue, 0.0f, 150000.0f);

            // Update the buffer with new data
            numpy::roll(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, -7);
            buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 7] = ax_scaled;
            buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 6] = ay_scaled;
            buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 5] = az_scaled;
            buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 4] = gx_scaled;
            buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 3] = gy_scaled;
            buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 2] = gz_scaled;
            buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 1] = ir_scaled;

            // Print scaled values for debugging
            // Serial.print(ax_scaled, 2);
            // Serial.print('\t');
            // Serial.print(ay_scaled, 2);
            // Serial.print('\t');
            // Serial.print(az_scaled, 2);
            // Serial.print('\t');
            // Serial.print(gx_scaled, 2);
            // Serial.print('\t');
            // Serial.print(gy_scaled, 2);
            // Serial.print('\t');
            // Serial.print(gz_scaled, 2);
            // Serial.print('\t');
            // Serial.println(ir_scaled, 2);

        } else {
            Serial.println("Failed to read IMU data!");
        }
    }
}
