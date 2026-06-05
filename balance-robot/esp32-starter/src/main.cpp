#include <Arduino.h>
#include <BluetoothSerial.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>

// ── Controller gains ────────────────────────────────────────────────────────
const float K_THETA     = -120.0f; //160    120
const float K_THETA_DOT = -100.0f; //55     100
const float K_XDOT      =   0.0f; // velocity gain (m/s → rad/s) — tune from 0
const float VEL_CMD     =   0.05f; // target speed when w/s held (m/s)
const float VEL_RAMP    =   0.003f;  // m/s per 10ms loop → ~50ms to full speed


// ── Kalman filter — process model from calculations2.py ──────────────────────
// State: [theta, theta_dot]
// Ad = expm(Ac * 0.01), Ac = [[0,1],[a_tt,0]]
// m=0.84,l=0.105,M=0.46,I=0.02 → D0=0.03026, a_tt=37.17, ω=6.097
// Ad = [[cosh(ωTs), sinh(ωTs)/ω], [ω·sinh(ωTs), cosh(ωTs)]]
static const float KF_AD00 =  1.001859f, KF_AD01 =  0.010003f;
static const float KF_AD10 =  0.371830f, KF_AD11 =  1.001859f;
// Process noise (tune: larger = trust measurements more)
static const float KF_Q00  =  1e-4f;   // angle
static const float KF_Q11  =  2e-3f;   // rate
// Measurement noise variances (tune to match sensor noise floor)
static const float KF_R_ACCEL =  5000.0f;  // theta from accelerometer
static const float KF_R_GYRO  =  500.0f;  // theta_dot from gyro

// ── Controller settings ─────────────────────────────────────────────────────
const float U_MAX          = 30.0f;
const float GYRO_BIAS      = -0.014f;
const float CF_ALPHA       = 0.98f;
const float THETA_OFFSET   = 0.1167f;
const float GYRO_LPF       = 0.95f;
const float COM_HEIGHT_M   = 0.105f;  // ← height of centre of mass above wheel axle

// ── Pins ──────────────────────────────────────────────────────────────────────
const int STEPPER1_DIR_PIN  = 16;
const int STEPPER1_STEP_PIN = 17;
const int STEPPER2_DIR_PIN  = 4;
const int STEPPER2_STEP_PIN = 14;
const int STEPPER_EN_PIN    = 15;
const int TOGGLE_PIN        = 32;

const int   LOOP_INTERVAL_MS    = 10;
const int   PRINT_INTERVAL_MS   = 50;
const int   STEPPER_INTERVAL_US = 20;
const float DT = LOOP_INTERVAL_MS / 1000.0f;
const float WHEEL_RADIUS_M      = 0.034f;  // ← measure your wheel radius in metres

static bool capActive = false;

// ── Objects ───────────────────────────────────────────────────────────────────
BluetoothSerial BT;
ESP32Timer ITimer(3);
Adafruit_MPU6050 mpu;
step step1(STEPPER_INTERVAL_US, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
step step2(STEPPER_INTERVAL_US, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

bool TimerHandler(void *)
{
  step1.runStepper();
  step2.runStepper();
  return true;
}

template<typename T> void logPrint(T v)           { Serial.print(v);      BT.print(v); }
template<typename T> void logPrint(T v, int dp)   { Serial.print(v, dp);  BT.print(v, dp); }
template<typename T> void logPrintln(T v)         { Serial.println(v);    BT.println(v); }
template<typename T> void logPrintln(T v, int dp) { Serial.println(v,dp); BT.println(v,dp); }


void setup()
{
  Serial.begin(115200);
  BT.begin("BalanceRobot");
  pinMode(TOGGLE_PIN, OUTPUT);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);

  if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
    Serial.println("Stepper ISR failed");
    while (1) delay(10);
  }

  step1.setAccelerationRad(500.0f);
  step2.setAccelerationRad(500.0f);

  pinMode(STEPPER_EN_PIN, OUTPUT);
  digitalWrite(STEPPER_EN_PIN, LOW);

  pinMode(0, INPUT_PULLUP);
  Serial.println("Press BOOT to start controller...");
  while (digitalRead(0) == HIGH) delay(10);
  while (digitalRead(0) == LOW)  delay(10);
}

void loop()
{
  static unsigned long loopTimer  = millis();
  static unsigned long printTimer = millis();
  static float theta_filt     = 0.0f;
  static float theta_dot_filt = 0.0f;
  static float vel_filt       = 0.0f;
  static float kf_th  = 0.0f, kf_thd = 0.0f;
  static float kfP00 = 1.0f, kfP01 = 0.0f, kfP11 = 1.0f;
  static char          keyState   = '-';   // 'w', 's', or '-'
  static unsigned long lastKeyMs  = 0;
  static const unsigned long KEY_TIMEOUT_MS = 150;

  if (millis() < loopTimer) return;
  loopTimer += LOOP_INTERVAL_MS;

  // ── Bluetooth / Serial keyboard input (runs at 100 Hz) ───────────────────
  int ch = -1;
  if      (BT.available())     ch = BT.read();
  else if (Serial.available()) ch = Serial.read();
  if (ch != -1) {
    char c = (char)ch;
    if      (c == 'w' || c == 'W') { keyState = 'w'; lastKeyMs = millis(); }
    else if (c == 's' || c == 'S') { keyState = 's'; lastKeyMs = millis(); }
    else                             keyState = '-';
  }
  if (keyState != '-' && (millis() - lastKeyMs) > KEY_TIMEOUT_MS) keyState = '-';

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // 3-sample median filter on raw gyro
  static float gyrobuf[3] = {0, 0, 0};
  static uint8_t gyrobuf_i = 0;
  gyrobuf[gyrobuf_i] = g.gyro.y;
  gyrobuf_i = (gyrobuf_i + 1) % 3;
  float ga = gyrobuf[0], gb = gyrobuf[1], gc = gyrobuf[2];
  float gyro_med = max(min(ga, gb), min(max(ga, gb), gc));

  float theta_dot   = gyro_med - GYRO_BIAS;
  float theta_accel = asinf(constrain(a.acceleration.z / 9.81f, -1.0f, 1.0f)) - THETA_OFFSET;

  theta_dot_filt = GYRO_LPF * theta_dot_filt + (1.0f - GYRO_LPF) * theta_dot;
  theta_filt     = CF_ALPHA * (theta_filt + theta_dot_filt * DT) + (1.0f - CF_ALPHA) * theta_accel;

  // ── Kalman filter ────────────────────────────────────────────────────────
  // Predict: x = Ad * x
  float kf_th_p  = KF_AD00*kf_th  + KF_AD01*kf_thd;
  float kf_thd_p = KF_AD10*kf_th  + KF_AD11*kf_thd;
  // P = Ad * P * Ad^T + Q
  float tA = KF_AD00*kfP00 + KF_AD01*kfP01;
  float tB = KF_AD00*kfP01 + KF_AD01*kfP11;
  float tC = KF_AD10*kfP00 + KF_AD11*kfP01;
  float tD = KF_AD10*kfP01 + KF_AD11*kfP11;
  float p00 = tA*KF_AD00 + tB*KF_AD01 + KF_Q00;
  float p01 = tA*KF_AD10 + tB*KF_AD11;
  float p11 = tC*KF_AD10 + tD*KF_AD11 + KF_Q11;
  // Update 1: theta from accelerometer (H = [1,0])
  float S1  = p00 + KF_R_ACCEL;
  float K00 = p00 / S1;
  float K10 = p01 / S1;
  float inn1 = theta_accel - kf_th_p;
  kf_th_p  += K00 * inn1;
  kf_thd_p += K10 * inn1;
  float p00a = (1.0f - K00)*p00;
  float p01a = (1.0f - K00)*p01;
  float p11a = p11 - K10*p01;
  // Update 2: theta_dot from gyro (H = [0,1])
  float S2   = p11a + KF_R_GYRO;
  float K01  = p01a / S2;
  float K11b = p11a / S2;
  float inn2 = theta_dot - kf_thd_p;
  kf_th  = kf_th_p  + K01  * inn2;
  kf_thd = kf_thd_p + K11b * inn2;
  kfP00  = p00a  - K01  * p01a;
  kfP01  = p01a  * (1.0f - K11b);
  kfP11  = p11a  * (1.0f - K11b);


  // Body velocity: wheel surface speed + pendulum tip correction
  // v_body = r*omega_wheel + L*theta_dot*cos(theta)  (exact for rigid body)
  float vel_raw = step1.getSpeedRad() * WHEEL_RADIUS_M  + COM_HEIGHT_M * theta_dot_filt * cosf(theta_filt);
  vel_filt = GYRO_LPF * vel_filt + (1.0f - GYRO_LPF) * vel_raw;

  static float vel_target = 0.0f;

  float vel_desired = (keyState == 'w') ?  VEL_CMD : (keyState == 's') ? -VEL_CMD : 0.0f;

  if      (vel_target < vel_desired - VEL_RAMP) vel_target += VEL_RAMP;
  else if (vel_target > vel_desired + VEL_RAMP) vel_target -= VEL_RAMP;
  else                                           vel_target  = vel_desired;

  float u_th   = -K_THETA     * theta_filt;
  float u_thd  = -K_THETA_DOT * theta_dot_filt;
  float u_vel  = -K_XDOT      * (vel_filt - vel_target);
  float u_raw  = u_th + u_thd + u_vel;
  float u      = constrain(u_raw, -U_MAX, U_MAX);

  step1.setTargetSpeedRad(u);
  step2.setTargetSpeedRad(-u);

  if (!capActive && millis() >= printTimer) {
    printTimer += PRINT_INTERVAL_MS;
    logPrint("t=");      logPrint(millis());
    logPrint(" th=");     logPrint(theta_filt,    4);
    logPrint(" th_d=");  logPrint(theta_dot_filt, 4);
    logPrint(" td_r=");  logPrint(theta_dot,      4);
    logPrint(" kf_th="); logPrint(kf_th,          4);
    logPrint(" kf_td="); logPrint(kf_thd,         4);
    logPrint(" vel=");   logPrint(vel_filt,        4);
    logPrint(" u_th=");  logPrint(u_th,             4);
    logPrint(" u_thd="); logPrint(u_thd,            4);
    logPrint(" u_vel="); logPrint(u_vel,            4);
    logPrint(" u=");     logPrint(u,                4);
    logPrint(" spd=");   logPrintln(step1.getSpeedRad(), 4);
    logPrint(" key=");   Serial.println(keyState); BT.println(keyState);
  }
}

//python -m serial.tools.miniterm COM5 115200
