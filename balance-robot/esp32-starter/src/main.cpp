#include <Arduino.h>
#include <BluetoothSerial.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>

// ── Controller gains ────────────────────────────────────────────────────────
const float K_THETA     = -120.0f; //160    120
const float K_THETA_DOT = -90.0f; //55     100
// K_XDOT removed — velocity regulated solely by outer PI; do not re-enable both
const float VEL_CMD     =   0.05f; // target speed when w/s held (m/s)
const float VEL_RAMP    =   0.003f;  // m/s per 10ms loop → ~50ms to full speed
const float TURN_CMD    =   0.5f;  // differential speed offset (rad/s) for a/d turns


// ── Controller settings ─────────────────────────────────────────────────────
const float U_MAX          = 30.0f;
float       Kp_v           =   1.0f;// outer PI: proportional gain (m/s → rad)
float       Ki_v           =   0.0f;//outer PI: integral gain
const float THETA_REF_MAX  =  0.03f;   // max commanded lean (rad)
const float GYRO_BIAS      = -0.007f;
const float CF_ALPHA       = 0.98f;
const float THETA_OFFSET   = 0.1050f;
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

  step1.setAccelerationRad(100.0f);
  step2.setAccelerationRad(100.0f);

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
  static float theta_filt        = 0.0f;
  static float theta_dot_filt    = 0.0f;
  static float theta_dot_filt_07 = 0.0f;
  static float gyro_bias_est     = GYRO_BIAS;
  static float vel_filt          = 0.0f;
  static char          keyState   = '-';   // 'w', 's', or '-'
  static char          turnState  = '-';   // 'a', 'd', or '-'
  static unsigned long lastKeyMs  = 0;
  static unsigned long lastTurnMs = 0;
  static const unsigned long KEY_TIMEOUT_MS = 150;

  if (millis() < loopTimer) return;
  loopTimer += LOOP_INTERVAL_MS;

  // ── Bluetooth / Serial keyboard input (runs at 100 Hz) ───────────────────
  int ch = -1;
  if      (BT.available())     ch = BT.read();
  else if (Serial.available()) ch = Serial.read();
  if (ch != -1) {
    char c = (char)ch;
    if      (c == 'w' || c == 'W') { keyState = 'w'; lastKeyMs  = millis(); }
    else if (c == 's' || c == 'S') { keyState = 's'; lastKeyMs  = millis(); }
    else if (c == 'a' || c == 'A') { turnState = 'a'; lastTurnMs = millis(); }
    else if (c == 'd' || c == 'D') { turnState = 'd'; lastTurnMs = millis(); }
    else { keyState = '-'; turnState = '-'; }
  }
  if (keyState  != '-' && (millis() - lastKeyMs)  > KEY_TIMEOUT_MS) keyState  = '-';
  if (turnState != '-' && (millis() - lastTurnMs) > KEY_TIMEOUT_MS) turnState = '-';

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // 3-sample median filter on raw gyro
  static float gyrobuf[3] = {0, 0, 0};
  static uint8_t gyrobuf_i = 0;
  gyrobuf[gyrobuf_i] = g.gyro.y;
  gyrobuf_i = (gyrobuf_i + 1) % 3;
  float ga = gyrobuf[0], gb = gyrobuf[1], gc = gyrobuf[2];
  float gyro_med = max(min(ga, gb), min(max(ga, gb), gc));

  float theta_dot   = gyro_med - gyro_bias_est;
  float theta_accel = asinf(constrain(a.acceleration.z / 9.81f, -1.0f, 1.0f)) - THETA_OFFSET;

  theta_dot_filt    = GYRO_LPF  * theta_dot_filt    + (1.0f - GYRO_LPF)  * theta_dot;
  theta_dot_filt_07 = 0.8f      * theta_dot_filt_07 + 0.2f               * theta_dot;
  theta_filt = CF_ALPHA * (theta_filt + theta_dot_filt * DT) + (1.0f - CF_ALPHA) * theta_accel;

  // Online gyro bias correction: adapt slowly when robot is near-upright and not spinning
  // if (fabsf(theta_filt) < 0.05f && fabsf(theta_dot_filt) < 0.1f)
  //   gyro_bias_est += 0.0003f * (gyro_med - gyro_bias_est);

  // Body velocity: wheel surface speed + pendulum tip correction
  // v_body = r*omega_wheel + L*theta_dot*cos(theta)  (exact for rigid body)
  float vel_raw = step1.getSpeedRad() * WHEEL_RADIUS_M  + COM_HEIGHT_M * theta_dot_filt * cosf(theta_filt);
  vel_filt = GYRO_LPF * vel_filt + (1.0f - GYRO_LPF) * vel_raw;

  static float vel_target   = 0.0f;
  static float vel_integral = 0.0f;

  float vel_desired = (keyState == 's') ?  VEL_CMD : (keyState == 'w') ? -VEL_CMD : 0.0f;

  if      (vel_target < vel_desired - VEL_RAMP) vel_target += VEL_RAMP;
  else if (vel_target > vel_desired + VEL_RAMP) vel_target -= VEL_RAMP;
  else                                           vel_target  = vel_desired;

  // ── Outer PI velocity loop → tilt setpoint ────────────────────────────────
  float e_v            = vel_target - vel_filt;
  float theta_ref_raw  = Kp_v * e_v + Ki_v * vel_integral;
  bool  pi_sat         = fabsf(theta_ref_raw) >= THETA_REF_MAX;
  bool  pi_wind        = (theta_ref_raw > 0.0f && e_v > 0.0f) || (theta_ref_raw < 0.0f && e_v < 0.0f);
  if (!(pi_sat && pi_wind)) vel_integral += e_v * DT;
  if (Ki_v != 0.0f)
    vel_integral = constrain(vel_integral, -THETA_REF_MAX / fabsf(Ki_v), THETA_REF_MAX / fabsf(Ki_v));
  float theta_ref = constrain(Kp_v * e_v + Ki_v * vel_integral, -THETA_REF_MAX, THETA_REF_MAX);

  float u_th   = -K_THETA     * (theta_filt - theta_ref);
  float u_thd  = -K_THETA_DOT * theta_dot_filt;
  float u_raw  = u_th + u_thd;
  float u      = constrain(u_raw, -U_MAX, U_MAX);

  float turn_cmd = (turnState == 'd') ? TURN_CMD : (turnState == 'a') ? -TURN_CMD : 0.0f;
  step1.setTargetSpeedRad(u + turn_cmd);
  step2.setTargetSpeedRad(-(u - turn_cmd));

  if (!capActive && millis() >= printTimer) {
    printTimer += PRINT_INTERVAL_MS;
    logPrint("t=");      logPrint(millis());
    logPrint(" th=");     logPrint(theta_filt,    4);
    logPrint(" th_d=");  logPrint(theta_dot_filt,    4);
    logPrint(" th_d7="); logPrint(theta_dot_filt_07, 4);
    logPrint(" td_r=");  logPrint(theta_dot,       4);
   // logPrint(" gbias="); logPrint(gyro_bias_est,   6);
   // logPrint(" vel=");   logPrint(vel_filt,        4);
    logPrint(" u_th=");  logPrint(u_th,             4);
    logPrint(" u_thd="); logPrint(u_thd,            4);
   // logPrint(" tref=");  logPrint(theta_ref,          4);
   // logPrint(" vi=");    logPrint(vel_integral,       4);
    logPrint(" u=");     logPrint(u,                4);
    logPrint(" spd=");   logPrintln(step1.getSpeedRad(), 4);
    logPrint(" key=");   Serial.println(keyState); BT.println(keyState);
    logPrint(" turn=");  Serial.println(turnState); BT.println(turnState);
  }
}

//python -m serial.tools.miniterm COM5 115200