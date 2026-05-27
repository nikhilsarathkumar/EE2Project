#include <Arduino.h>
#include <BluetoothSerial.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>

// ── Controller gains ────────────────────────────────────────────────────────
const float K_THETA     = -70.0f; //17
const float K_THETA_DOT = -50.0f; // 15

// ── Controller limits ───────────────────────────────────────────────────────
const float U_MAX        = 50.0f;  // motor speed saturation (rad/s)
const float GYRO_BIAS    = -0.015f;
const float CF_ALPHA     = 0.98f;
const float THETA_OFFSET = 0.1147f;  
const float GYRO_LPF = 0.9f;  // lower = more smoothing, try 0.7–0.9

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

static bool   capActive = false;

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

template<typename T> void logPrint(T v)          { Serial.print(v);      BT.print(v); }
template<typename T> void logPrint(T v, int dp)  { Serial.print(v, dp);  BT.print(v, dp); }
template<typename T> void logPrintln(T v)        { Serial.println(v);    BT.println(v); }
template<typename T> void logPrintln(T v, int dp){ Serial.println(v,dp); BT.println(v,dp); }


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

  logPrintln("Running. Send 'c' to capture 3s of data.");
}

void loop()
{
  static unsigned long loopTimer  = millis();
  static unsigned long printTimer = millis();
  static float theta_filt = 0.0f;
  static float theta_dot_filt = 0.0f;

  if (millis() < loopTimer) return;
  loopTimer += LOOP_INTERVAL_MS;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  static float gyrobuf[3] = {0, 0, 0};
  static uint8_t gyrobuf_i = 0;
  gyrobuf[gyrobuf_i] = g.gyro.y;
  gyrobuf_i = (gyrobuf_i + 1) % 3;
  float ga = gyrobuf[0], gb = gyrobuf[1], gc = gyrobuf[2];
  float gyro_med = max(min(ga, gb), min(max(ga, gb), gc));

  float theta_dot   = gyro_med - GYRO_BIAS;
  float theta_accel = a.acceleration.z / 9.81f - THETA_OFFSET;


  theta_dot_filt = GYRO_LPF * theta_dot_filt + (1.0f - GYRO_LPF) * theta_dot;
  theta_filt = CF_ALPHA * (theta_filt + theta_dot_filt * DT) + (1.0f - CF_ALPHA) * theta_accel;

  float u = -(K_THETA * theta_filt + K_THETA_DOT * theta_dot_filt);
 
  u = constrain(u, -U_MAX, U_MAX);

  step1.setTargetSpeedRad(u);
  step2.setTargetSpeedRad(-u);

  // ── Regular live print ────────────────────────────────────────────────────
  if (!capActive && millis() >= printTimer) {
    printTimer += PRINT_INTERVAL_MS;
    logPrint("t=");     logPrint(millis());
    logPrint(" th=");   logPrint(theta_filt, 4);
    logPrint(" th_d="); logPrint(theta_dot_filt,  4);
    logPrint(" u=");    logPrintln(u,        4);
  }
}



//python -m serial.tools.miniterm COM5 115200
//-0.25, 1.2
//1.2, 2.5