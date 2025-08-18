#pragma once
#include <Arduino.h>
#include <MPU9250.h>

class ImuEkf {
public:
  // --- Lifecycle ---
  bool begin(uint8_t i2c_addr = 0x68, int sda = -1, int scl = -1, uint32_t i2c_hz = 400000);
  void setDeclination(float deg);     // +E / -W
  void setYawOffset(float deg);       // rotate to match your platform "forward"
  void setTunings(float Q_angle, float Q_bias, float R_rp_deg, float R_yaw_deg);

  // --- Calibrations ---
  void seedGyroBias(uint32_t ms = 2000);        // keep still
  void calibrateMag(uint32_t ms = 10000);       // rotate all orientations

  // --- Runtime update (call as fast as possible; returns true when a step ran) ---
  bool update();

  // --- Outputs (degrees) ---
  void getRPY(float &roll_deg, float &pitch_deg, float &yaw_deg) const;

  // Optional: additional outputs
  float magneticField_uT() const { return B_uT_; }
  float temperature_C()   const { return temp_C_; }

private:
  // Build roll/pitch (from accel) and tilt-comp yaw (from mag) in radians
  void rpyMeasRad_(float ax, float ay, float az,
                   float mx_uT, float my_uT, float mz_uT,
                   float &roll, float &pitch, float &yaw);

  // ===== Inner EKF: x = [phi, theta, psi, b_p, b_q, b_r]^T =====
  struct EKF {
    float x[6]  = {0};       // state (radians, rad/s)
    float P[36] = {0};       // 6x6 covariance (row-major)

    // Tunables
    float Q_angle = 1e-3f;   // rad^2/s (angles)
    float Q_bias  = 1e-5f;   // rad^2/s (bias random walk)
    float R_rp    = 2.0f * (float)M_PI / 180.0f;  // SD (rad) roll/pitch
    float R_yaw   = 5.0f * (float)M_PI / 180.0f;  // SD (rad) yaw

    // Small matrix helpers
    static void eye6(float A[36]);
    static void add6x6(float C[36], const float A[36], const float B[36]);
    static void mult6x6(float C[36], const float A[36], const float B[36]);
    static void mult6x6T(float C[36], const float A[36], const float BT[36]);
    static void mult3x6(float C[18], const float A[18], const float B[36]);
    static void mult6x3(float C[18], const float A[36], const float B[18]);
    static void mult3x6_6x3(float C[9], const float A[18], const float B[18]);
    static bool inv3x3(const float M[9], float Minv[9]);

    static void f(const float x[6], const float u[3], float xdot[6]);
    static void jacobian_A(const float x[6], const float u[3], float A[36]);

    void predict(float u[3], float dt);
    void update(const float z[3]);
  } ekf_;

  // Members
  MPU9250 imu_;
  uint8_t addr_ = 0x68;
  float decl_deg_ = 0.0f, yaw_off_deg_ = 0.0f;

  // Timing
  uint32_t t_prev_ms_ = 0;

  // Outputs
  float roll_deg_=0, pitch_deg_=0, yaw_deg_=0;
  float B_uT_=0, temp_C_=0;

  // Mag calibration (mG in driver)
  struct { float off[3]={0,0,0}; float scl[3]={1,1,1}; bool ready=false; } mag_;
};
