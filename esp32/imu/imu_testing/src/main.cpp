#include <Arduino.h>
#include <Preferences.h>
#include "imu_ekf.hpp"

ImuEkf ekf;
Preferences prefs;           // ESP32 NVS (flash) key-value store

float DEFAULT_ROLL_OFFSET = -179.97f;
float DEFAULT_PITCH_OFFSET = -3.34f;

// Persistent mount offsets (degrees) – loaded at boot, applied every run
static float roll_off_deg  = 0.0f;
static float pitch_off_deg = 0.0f;

static float wrap180(float a){
  while (a >  180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

void loadOffsets() {
  prefs.begin("imu", true);       // read-only
  roll_off_deg  = prefs.getFloat("roll_off",  DEFAULT_ROLL_OFFSET);
  pitch_off_deg = prefs.getFloat("pitch_off", DEFAULT_PITCH_OFFSET);
  prefs.end();
}

void saveOffsets(float r_deg, float p_deg) {
  prefs.begin("imu", false);      // read-write
  prefs.putFloat("roll_off",  r_deg);
  prefs.putFloat("pitch_off", p_deg);
  prefs.end();
  roll_off_deg = r_deg;
  pitch_off_deg = p_deg;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  if (!ekf.begin(0x68)) {
    Serial.println("IMU not found at 0x68/0x69");
    while (true) { delay(1000); }
  }

  // Ignore yaw/magnetometer for balancing
  ekf.setTunings(
    1e-3f,   // Q_angle
    1e-5f,   // Q_bias
    2.0f,    // R_rp_deg (roll/pitch noise)
    1000.0f  // R_yaw_deg (huge -> yaw ignored)
  );

  loadOffsets();
  Serial.printf("IMU ready. Using saved offsets: roll_off=%.2f°, pitch_off=%.2f°\n",
                roll_off_deg, pitch_off_deg);
  Serial.println("Commands: 'z' save current R/P as offsets, 'x' clear offsets, 'p' print offsets.");
}

void loop() {
  if (ekf.update()) {
    float r, p, y;
    ekf.getRPY(r, p, y);  // absolute angles vs gravity (deg)

    // Apply persistent mount offsets so “perfectly flat” => 0
    float roll_for_control  = wrap180(r - roll_off_deg);
    float pitch_for_control = wrap180(p - pitch_off_deg);

    // ONE line per sample, nothing else:
    Serial.printf("$RY,%.2f,%.2f\n", roll_for_control, pitch_for_control);
  }

  // Simple serial commands
  if (Serial.available()) {
    int ch = Serial.read();
    if (ch=='z' || ch=='Z') {
      float r, p, y; ekf.getRPY(r, p, y);
      saveOffsets(r, p);
      Serial.printf("Saved new offsets: roll_off=%.2f°, pitch_off=%.2f°\n", r, p);
    } else if (ch=='x' || ch=='X') {
      saveOffsets(0.0f, 0.0f);
      Serial.println("Offsets cleared to 0,0");
    } else if (ch=='p' || ch=='P') {
      Serial.printf("Offsets: roll_off=%.2f°, pitch_off=%.2f°\n", roll_off_deg, pitch_off_deg);
    }
  }
}
