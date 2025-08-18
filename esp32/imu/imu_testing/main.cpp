#include <Arduino.h>
#include "imu_ekf.hpp"

ImuEkf ekf;

void setup() {
  Serial.begin(115200);
  delay(200);

  if (!ekf.begin(0x68)) {
    Serial.println("IMU not found at 0x68/0x69");
    while(true){ delay(1000); }
  }

  ekf.setDeclination(0.0f);           // set your local declination (deg)
  ekf.setYawOffset(0.0f);             // rotate to match your forward
  ekf.setTunings(1e-3f, 1e-5f, 2.0f, 5.0f);

  ekf.seedGyroBias(2000);             // keep still
  ekf.calibrateMag(10000);            // rotate all orientations

  Serial.println("Ready. Type 'c' to re-run mag calibration.");
}

void loop() {
  if (ekf.update()) {
    static uint32_t last=0;
    if (millis()-last > 100) {
      last = millis();
      float r,p,y;
      ekf.getRPY(r,p,y);
      Serial.printf("EKF RPY [deg]  roll=%7.2f  pitch=%7.2f  yaw=%7.2f | |B|=%5.1f µT | T %.2fC\n",
                    r, p, y, ekf.magneticField_uT(), ekf.temperature_C());
    }
  }

  if (Serial.available()) {
    int ch = Serial.read();
    if (ch=='c' || ch=='C') ekf.calibrateMag(10000);
  }
}
