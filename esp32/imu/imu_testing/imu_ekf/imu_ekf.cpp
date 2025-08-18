#include "imu_ekf.hpp"
#include <Wire.h>
#include <math.h>
#include <string.h>

// -------- file-scope helpers (avoid scope issues) --------
static inline float deg2rad(float d){ return d * 0.017453292519943295f; }
static inline float rad2deg(float r){ return r * 57.29577951308232f; }
static inline float wrapPI(float a){ while(a >  M_PI) a -= 2*M_PI; while(a < -M_PI) a += 2*M_PI; return a; }
static inline float wrap2PI(float a){ a = fmodf(a, 2*M_PI); if(a < 0) a += 2*M_PI; return a; }

// ===== Public API =====
bool ImuEkf::begin(uint8_t i2c_addr, int sda, int scl, uint32_t i2c_hz){
  addr_ = i2c_addr;
  if (sda >= 0 && scl >= 0) Wire.begin(sda, scl);
  else Wire.begin();                       // ESP32 default SDA=21, SCL=22
  Wire.setClock(i2c_hz);

  // Try 0x68 then 0x69
  if (!imu_.setup(addr_)) {
    addr_ = (addr_ == 0x68) ? 0x69 : 0x68;
    if (!imu_.setup(addr_)) return false;
  }

  // Init covariance
  memset(ekf_.P, 0, sizeof(ekf_.P));
  ekf_.P[0*6+0] = deg2rad(5)*deg2rad(5);
  ekf_.P[1*6+1] = deg2rad(5)*deg2rad(5);
  ekf_.P[2*6+2] = deg2rad(10)*deg2rad(10);
  ekf_.P[3*6+3] = deg2rad(2)*deg2rad(2);
  ekf_.P[4*6+4] = deg2rad(2)*deg2rad(2);
  ekf_.P[5*6+5] = deg2rad(2)*deg2rad(2);

  t_prev_ms_ = millis();
  return true;
}

void ImuEkf::setDeclination(float deg){ decl_deg_ = deg; }
void ImuEkf::setYawOffset(float deg){  yaw_off_deg_ = deg; }

void ImuEkf::setTunings(float Q_angle, float Q_bias, float R_rp_deg, float R_yaw_deg){
  ekf_.Q_angle = Q_angle;
  ekf_.Q_bias  = Q_bias;
  ekf_.R_rp    = deg2rad(R_rp_deg);
  ekf_.R_yaw   = deg2rad(R_yaw_deg);
}

void ImuEkf::seedGyroBias(uint32_t ms){
  Serial.println("GYRO BIAS: keep sensor still...");
  uint32_t t0 = millis(); uint32_t n=0; double sx=0, sy=0, sz=0;
  while (millis() - t0 < ms) {
    if (imu_.update()) { sx+=imu_.getGyroX(); sy+=imu_.getGyroY(); sz+=imu_.getGyroZ(); n++; }
    delay(2);
  }
  float bx=0, by=0, bz=0; if (n>0){ bx=sx/n; by=sy/n; bz=sz/n; }
  ekf_.x[3] = deg2rad(bx);
  ekf_.x[4] = deg2rad(by);
  ekf_.x[5] = deg2rad(bz);
  Serial.printf("Gyro bias [dps]: %+6.3f %+6.3f %+6.3f\n", bx, by, bz);
}

void ImuEkf::calibrateMag(uint32_t ms){
  Serial.println("MAG CAL: rotate in ALL orientations...");
  float minv[3] = { 1e9f, 1e9f, 1e9f }, maxv[3] = { -1e9f, -1e9f, -1e9f };
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (imu_.update()) {
      float mxx = imu_.getMagX(), myy = imu_.getMagY(), mzz = imu_.getMagZ(); // mG
      if (mxx < minv[0]) minv[0]=mxx; if (myy < minv[1]) minv[1]=myy; if (mzz < minv[2]) minv[2]=mzz;
      if (mxx > maxv[0]) maxv[0]=mxx; if (myy > maxv[1]) maxv[1]=myy; if (mzz > maxv[2]) maxv[2]=mzz;
    }
    delay(10);
  }
  for (int i=0;i<3;i++) mag_.off[i] = 0.5f*(maxv[i]+minv[i]); // mG
  float r[3] = {0.5f*(maxv[0]-minv[0]), 0.5f*(maxv[1]-minv[1]), 0.5f*(maxv[2]-minv[2])};
  float r_avg = (r[0]+r[1]+r[2]) / 3.0f;
  for (int i=0;i<3;i++) mag_.scl[i] = r_avg / (r[i] > 1e-6f ? r[i] : 1.0f);
  mag_.ready = true;

  Serial.printf("MAG CAL done. Offsets (mG): %+6.1f %+6.1f %+6.1f  scales: %.3f %.3f %.3f\n",
                mag_.off[0], mag_.off[1], mag_.off[2],
                mag_.scl[0], mag_.scl[1], mag_.scl[2]);
}

bool ImuEkf::update(){
  if (!imu_.update()) return false;

  // dt
  uint32_t t_now = millis();
  float dt = (t_now - t_prev_ms_) * 0.001f;
  if (dt <= 0) dt = 1.0f/1000.0f;
  t_prev_ms_ = t_now;

  // Read sensors
  float ax = imu_.getAccX(), ay = imu_.getAccY(), az = imu_.getAccZ();        // g
  float gx_dps = imu_.getGyroX(), gy_dps = imu_.getGyroY(), gz_dps = imu_.getGyroZ(); // deg/s
  float mx_mG = imu_.getMagX(),  my_mG = imu_.getMagY(),  mz_mG = imu_.getMagZ();     // mG
  temp_C_ = imu_.getTemperature();

  // Apply mag calibration (mG), then µT
  if (mag_.ready) {
    mx_mG = (mx_mG - mag_.off[0]) * mag_.scl[0];
    my_mG = (my_mG - mag_.off[1]) * mag_.scl[1];
    mz_mG = (mz_mG - mag_.off[2]) * mag_.scl[2];
  }
  float mx_uT = mx_mG * 0.1f, my_uT = my_mG * 0.1f, mz_uT = mz_mG * 0.1f;
  B_uT_ = sqrtf(mx_uT*mx_uT + my_uT*my_uT + mz_uT*mz_uT);

  // Seed angles on first call
  static bool seeded=false;
  if (!seeded) {
    float r,p,y; rpyMeasRad_(ax,ay,az, mx_uT,my_uT,mz_uT, r,p,y);
    ekf_.x[0]=r; ekf_.x[1]=p; ekf_.x[2]=y;
    seeded=true;
  }

  // Inputs u = gyro (rad/s)
  float u[3] = { deg2rad(gx_dps), deg2rad(gy_dps), deg2rad(gz_dps) };

  // Predict + Update
  ekf_.predict(u, dt);
  float z[3]; rpyMeasRad_(ax,ay,az, mx_uT,my_uT,mz_uT, z[0],z[1],z[2]);
  ekf_.update(z);

  // Store outputs (deg)
  roll_deg_  = rad2deg(ekf_.x[0]);
  pitch_deg_ = rad2deg(ekf_.x[1]);
  yaw_deg_   = rad2deg(ekf_.x[2]); if (yaw_deg_ < 0) yaw_deg_ += 360.0f;

  return true;
}

void ImuEkf::getRPY(float &roll_deg, float &pitch_deg, float &yaw_deg) const {
  roll_deg  = roll_deg_;
  pitch_deg = pitch_deg_;
  yaw_deg   = yaw_deg_;
}

// ===== Private helpers =====
void ImuEkf::rpyMeasRad_(float ax, float ay, float az,
                         float mx_uT, float my_uT, float mz_uT,
                         float &roll, float &pitch, float &yaw)
{
  roll  = atan2f(ay, az);
  pitch = -atan2f(ax, sqrtf(ay*ay + az*az));
  float cr = cosf(roll),  sr = sinf(roll);
  float cp = cosf(pitch), sp = sinf(pitch);
  float mxh = mx_uT*cp + mz_uT*sp;
  float myh = mx_uT*sr*sp + my_uT*cr - mz_uT*sr*cp;
  yaw = atan2f(-myh, mxh);
  yaw = wrap2PI(yaw + deg2rad(decl_deg_ + yaw_off_deg_));
}

// ===== EKF internals =====
void ImuEkf::EKF::eye6(float A[36]){ memset(A,0,36*sizeof(float)); for(int i=0;i<6;i++) A[i*6+i]=1; }
void ImuEkf::EKF::add6x6(float C[36], const float A[36], const float B[36]){ for(int i=0;i<36;i++) C[i]=A[i]+B[i]; }
void ImuEkf::EKF::mult6x6(float C[36], const float A[36], const float B[36]){
  for(int i=0;i<6;i++) for(int j=0;j<6;j++){
    float s=0; for(int k=0;k<6;k++) s+=A[i*6+k]*B[k*6+j]; C[i*6+j]=s;
  }
}
void ImuEkf::EKF::mult6x6T(float C[36], const float A[36], const float BT[36]){
  for(int i=0;i<6;i++) for(int j=0;j<6;j++){
    float s=0; for(int k=0;k<6;k++) s+=A[i*6+k]*BT[j*6+k]; C[i*6+j]=s;
  }
}
void ImuEkf::EKF::mult3x6(float C[18], const float A[18], const float B[36]){
  for(int i=0;i<3;i++) for(int j=0;j<6;j++){
    float s=0; for(int k=0;k<6;k++) s+=A[i*6+k]*B[k*6+j]; C[i*6+j]=s;
  }
}
void ImuEkf::EKF::mult6x3(float C[18], const float A[36], const float B[18]){
  for(int i=0;i<6;i++) for(int j=0;j<3;j++){
    float s=0; for(int k=0;k<6;k++) s+=A[i*6+k]*B[k*3+j]; C[i*3+j]=s;
  }
}
void ImuEkf::EKF::mult3x6_6x3(float C[9], const float A[18], const float B[18]){
  for(int i=0;i<3;i++) for(int j=0;j<3;j++){
    float s=0; for(int k=0;k<6;k++) s+=A[i*6+k]*B[k*3+j]; C[i*3+j]=s;
  }
}
bool ImuEkf::EKF::inv3x3(const float M[9], float Minv[9]){
  float a=M[0], b=M[1], c=M[2], d=M[3], e=M[4], f=M[5], g=M[6], h=M[7], i=M[8];
  float A=e*i - f*h, B=-(d*i - f*g), C=d*h - e*g;
  float D=-(b*i - c*h), E=a*i - c*g, F=-(a*h - b*g);
  float G=b*f - c*e, H=-(a*f - c*d), I=a*e - b*d;
  float det = a*A + b*B + c*C;
  if (fabsf(det) < 1e-9f) return false;
  float inv=1.0f/det;
  Minv[0]=A*inv; Minv[1]=D*inv; Minv[2]=G*inv;
  Minv[3]=B*inv; Minv[4]=E*inv; Minv[5]=H*inv;
  Minv[6]=C*inv; Minv[7]=F*inv; Minv[8]=I*inv;
  return true;
}

void ImuEkf::EKF::f(const float x[6], const float u[3], float xdot[6]){
  float phi = x[0], th = x[1];
  float bp = x[3],  bq = x[4],  br = x[5];
  float p = u[0] - bp, q = u[1] - bq, r = u[2] - br;

  float ct = cosf(th),  st = sinf(th);
  float sp = sinf(phi), cp = cosf(phi);

  // Robust tan(theta) and 1/cos(theta)
  float tt = (fabsf(ct) < 1e-3f) ? (st / ((ct >= 0) ? 1e-3f : -1e-3f)) : tanf(th);
  float inv_ct = (fabsf(ct) < 1e-3f) ? ((ct >= 0) ? 1e3f : -1e3f) : (1.0f / ct);

  float phi_dot   = p + q*sp*tt + r*cp*tt;
  float theta_dot = q*cp - r*sp;
  float psi_dot   = (q*sp + r*cp) * inv_ct;

  xdot[0] = phi_dot;
  xdot[1] = theta_dot;
  xdot[2] = psi_dot;
  xdot[3] = 0.0f;
  xdot[4] = 0.0f;
  xdot[5] = 0.0f;
}


void ImuEkf::EKF::jacobian_A(const float x[6], const float u[3], float A[36]){
  const float eps = 1e-5f;
  float f0[6]; f(x,u,f0);
  for(int j=0;j<6;j++){
    float xx[6]; memcpy(xx,x,6*sizeof(float));
    xx[j]+=eps; float fj[6]; f(xx,u,fj);
    for(int i=0;i<6;i++) A[i*6+j] = (fj[i]-f0[i]) / eps;
  }
}

void ImuEkf::EKF::predict(float u[3], float dt){
  float xdot[6]; f(x,u,xdot);
  for(int i=0;i<6;i++) x[i] += xdot[i]*dt;
  x[0]=wrapPI(x[0]); x[1]=wrapPI(x[1]); x[2]=wrap2PI(x[2]);

  float A[36], F[36], I6[36]; jacobian_A(x,u,A); eye6(I6);
  for(int i=0;i<36;i++) F[i] = I6[i] + dt*A[i];

  float Qd[36]={0};
  Qd[0*6+0] = Q_angle*dt; Qd[1*6+1] = Q_angle*dt; Qd[2*6+2] = Q_angle*dt;
  Qd[3*6+3] = Q_bias *dt; Qd[4*6+4] = Q_bias *dt; Qd[5*6+5] = Q_bias *dt;

  float FP[36], FPFt[36];
  mult6x6(FP, F, P);
  mult6x6T(FPFt, FP, F);
  add6x6(P, FPFt, Qd);
}

void ImuEkf::EKF::update(const float z[3]){
  // H (3x6) = [I3 | 0]
  float H[18] = {0};
  for (int i=0;i<3;i++) H[i*6 + i] = 1.0f;

  // H^T (6x3)
  float HT[18] = {0};
  HT[0*3+0] = 1.0f;
  HT[1*3+1] = 1.0f;
  HT[2*3+2] = 1.0f;
  // rows 3..5 remain zero

  // R (measurement covariance)
  float R[9] = {
    R_rp*R_rp, 0, 0,
    0, R_rp*R_rp, 0,
    0, 0, R_yaw*R_yaw
  };

  // S = H P H^T + R
  float HP[18];         // 3x6
  mult3x6(HP, H, P);
  float S[9];           // 3x3
  mult3x6_6x3(S, HP, HT);
  S[0] += R[0]; S[4] += R[4]; S[8] += R[8];

  // Guard: if S has NaN/Inf or is near-singular, skip this update
  if (!isfinite(S[0]) || !isfinite(S[4]) || !isfinite(S[8])) return;
  float Sinv[9];
  if (!inv3x3(S, Sinv)) return;

  // PH^T = P * H^T  (first 3 columns of P)
  float PHt[18]; // 6x3
  for (int i=0;i<6;i++)
    for (int j=0;j<3;j++)
      PHt[i*3 + j] = P[i*6 + j];

  // K = PH^T * S^-1
  float K[18];
  for (int i=0;i<6;i++) {
    for (int j=0;j<3;j++) {
      float s = 0.0f;
      for (int k=0;k<3;k++) s += PHt[i*3+k] * Sinv[k*3+j];
      K[i*3 + j] = s;
    }
  }

  // Innovation y = z - h(x) ; h(x) = [phi, theta, psi]
  float yv[3] = { z[0] - x[0], z[1] - x[1], z[2] - x[2] };
  yv[0] = wrapPI(yv[0]);
  yv[1] = wrapPI(yv[1]);
  yv[2] = wrapPI(yv[2]);

  // x = x + K y
  for (int i=0;i<6;i++) {
    x[i] += K[i*3+0]*yv[0] + K[i*3+1]*yv[1] + K[i*3+2]*yv[2];
  }
  x[0] = wrapPI(x[0]);
  x[1] = wrapPI(x[1]);
  x[2] = wrap2PI(x[2]);

  // P = (I - K H) P
  float KH[36] = {0};
  for (int i=0;i<6;i++)
    for (int j=0;j<3;j++)
      KH[i*6 + j] = K[i*3 + j];  // because H is [I|0]

  float I6[36]; eye6(I6);
  float IKH[36];
  for (int n=0;n<36;n++) IKH[n] = I6[n] - KH[n];

  float newP[36];
  mult6x6(newP, IKH, P);
  memcpy(P, newP, sizeof(newP));
}

