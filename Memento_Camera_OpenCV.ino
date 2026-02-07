#include <Arduino.h>
#include "Adafruit_PyCamera.h"

#include <tflm_esp32.h>
#include <eloquent_tinyml.h>

#include "face_detect_model.h"

// ================= CONFIG =================
#define SHUTTER_PIN 0

#define IMG_W 96
#define IMG_H 96

#define TF_NUM_OPS 20
#define ARENA_SIZE 120000

// ================= OBJECTS =================
Adafruit_PyCamera camera;
Eloquent::TF::Sequential<TF_NUM_OPS, ARENA_SIZE> tf;

// INT8 input buffer
static int8_t inputBuffer[IMG_W * IMG_H];

// ================= SETUP =================
void setup() {
    Serial.begin(115200);
    delay(2000);

    pinMode(SHUTTER_PIN, INPUT_PULLUP);

    Serial.println("\nMEMENTO Mask Classifier (Arduino)");

    // ---- Camera init (control only) ----
    if (!camera.begin()) {
        Serial.println("❌ Camera init failed");
        while (1);
    }

    camera.setRotation(0);
    Serial.println("📷 Camera initialized (no pixel access)");

    // ---- ML config ----
    tf.setNumInputs(IMG_W * IMG_H);
    tf.setNumOutputs(3);

    tf.resolver.AddConv2D();
    tf.resolver.AddDepthwiseConv2D();
    tf.resolver.AddMaxPool2D();
    tf.resolver.AddFullyConnected();
    tf.resolver.AddReshape();
    tf.resolver.AddSoftmax();
    tf.resolver.AddQuantize();
    tf.resolver.AddDequantize();
    tf.resolver.AddLogistic();

    while (!tf.begin(face_detect_int8_tflite).isOk()) {
        Serial.println(tf.exception.toString());
        delay(1000);
    }

    Serial.println("✅ Model loaded");
    Serial.println("Press shutter to run inference");
}

// ================= LOOP =================
void loop() {
    static uint32_t lastPress = 0;

    if (digitalRead(SHUTTER_PIN) == LOW &&
        millis() - lastPress > 1000) {

        lastPress = millis();
        Serial.println("\n🔘 Button pressed");

        // -------------------------------------------------
        // PLACEHOLDER INPUT (neutral INT8 image)
        // -------------------------------------------------
        for (int i = 0; i < IMG_W * IMG_H; i++) {
            inputBuffer[i] = 0;   // INT8 neutral (≈ gray)
        }

        if (!tf.predict(inputBuffer).isOk()) {
            Serial.println(tf.exception.toString());
            return;
        }

        float noFace   = tf.output(0);
        float noMask   = tf.output(1);
        float withMask = tf.output(2);

        Serial.print("Scores → ");
        Serial.print(noFace, 4); Serial.print(" | ");
        Serial.print(noMask, 4); Serial.print(" | ");
        Serial.println(withMask, 4);

        int pred = 0;
        float maxVal = noFace;
        if (noMask > maxVal) { maxVal = noMask; pred = 1; }
        if (withMask > maxVal) { pred = 2; }

        Serial.print("Result: ");
        if (pred == 0) Serial.println("NO FACE");
        else if (pred == 1) Serial.println("FACE WITHOUT MASK");
        else Serial.println("FACE WITH MASK");

        Serial.print("Inference time: ");
        Serial.print(tf.benchmark.microseconds());
        Serial.println(" us");
    }
}
