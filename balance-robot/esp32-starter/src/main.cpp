/*
// ── Balance Robot Starter Code ────────────────────────────────────────────────
#include <Arduino.h>
#include <SPI.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>

// The Stepper pins
const int STEPPER1_DIR_PIN  = 16;
const int STEPPER1_STEP_PIN = 17;
const int STEPPER2_DIR_PIN  = 4;
const int STEPPER2_STEP_PIN = 14;
const int STEPPER_EN_PIN    = 15; 

//ADC pins
const int ADC_CS_PIN        = 5;
const int ADC_SCK_PIN       = 18;
const int ADC_MISO_PIN      = 19;
const int ADC_MOSI_PIN      = 23;

// Diagnostic pin for oscilloscope
const int TOGGLE_PIN        = 32;

const int PRINT_INTERVAL    = 500;
const int LOOP_INTERVAL     = 10;
const int STEPPER_INTERVAL_US = 20;

const float kx = 20.0;
const float VREF = 4.096;

//Global objects
ESP32Timer ITimer(3);
Adafruit_MPU6050 mpu;         //Default pins for I2C are SCL: IO22, SDA: IO21

step step1(STEPPER_INTERVAL_US,STEPPER1_STEP_PIN,STEPPER1_DIR_PIN );
step step2(STEPPER_INTERVAL_US,STEPPER2_STEP_PIN,STEPPER2_DIR_PIN );


//Interrupt Service Routine for motor update
//Note: ESP32 doesn't support floating point calculations in an ISR
bool TimerHandler(void * timerNo)
{
  static bool toggle = false;

  //Update the stepper motors
  step1.runStepper();
  step2.runStepper();

  //Indicate that the ISR is running
  digitalWrite(TOGGLE_PIN,toggle);  
  toggle = !toggle;
	return true;
}

uint16_t readADC(uint8_t channel) {
  uint8_t tx0 = 0x06 | (channel >> 2);  // Command Byte 0 = Start bit + single-ended mode + MSB of channel
  uint8_t tx1 = (channel & 0x03) << 6;  // Command Byte 1 = Remaining 2 bits of channel

  digitalWrite(ADC_CS_PIN, LOW); 

  SPI.transfer(tx0);                    // Send Command Byte 0
  uint8_t rx0 = SPI.transfer(tx1);      // Send Command Byte 1 and receive high byte of result
  uint8_t rx1 = SPI.transfer(0x00);     // Send dummy byte and receive low byte of result

  digitalWrite(ADC_CS_PIN, HIGH); 

  uint16_t result = ((rx0 & 0x0F) << 8) | rx1; // Combine high and low byte into 12-bit result
  return result;
}

void setup()
{
  Serial.begin(115200);
  pinMode(TOGGLE_PIN,OUTPUT);

  // Try to initialize Accelerometer/Gyroscope
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  //Attach motor update ISR to timer to run every STEPPER_INTERVAL_US μs
  if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
    Serial.println("Failed to start stepper interrupt");
    while (1) delay(10);
    }
  Serial.println("Initialised Interrupt for Stepper");

  //Set motor acceleration values
  step1.setAccelerationRad(10.0);
  step2.setAccelerationRad(10.0);

  //Enable the stepper motor drivers
  pinMode(STEPPER_EN_PIN,OUTPUT);
  digitalWrite(STEPPER_EN_PIN, false);

  //Set up ADC and SPI
  pinMode(ADC_CS_PIN, OUTPUT);
  digitalWrite(ADC_CS_PIN, HIGH);
  SPI.begin(ADC_SCK_PIN, ADC_MISO_PIN, ADC_MOSI_PIN, ADC_CS_PIN);

}

void loop()
{
  //Static variables are initialised once and then the value is remembered betweeen subsequent calls to this function
  static unsigned long printTimer = 0;  //time of the next print
  static unsigned long loopTimer = 0;   //time of the next control update
  static float tiltx = 0.0;             //current tilt angle
  
  //Run the control loop every LOOP_INTERVAL ms
  if (millis() > loopTimer) {
    loopTimer += LOOP_INTERVAL;
    
    // Fetch data from MPU6050
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    //Calculate Tilt using accelerometer and sin x = x approximation for a small tilt angle
    tiltx = a.acceleration.z/9.67;

    //Set target motor speed proportional to tilt angle
    //Note: this is for demonstrating accelerometer and motors - it won't work as a balance controller
    step1.setTargetSpeedRad(tiltx*kx);
    step2.setTargetSpeedRad(-tiltx*kx);
  }
  
  //Print updates every PRINT_INTERVAL ms
  //Line format: X-axis tilt, Motor speed, A0 Voltage
  if (millis() > printTimer) {
    printTimer += PRINT_INTERVAL;
    Serial.print(tiltx*1000);
    Serial.print(' ');
    Serial.print(step1.getSpeedRad());
    Serial.print(' ');
    Serial.print((readADC(0) * VREF)/4095.0);
    Serial.println();
  }
}





/*


// ── State logger (archived) ───────────────────────────────────────────────────
#include <Arduino.h>
#include <SPI.h>
#include <BluetoothSerial.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>

const float WHEEL_RADIUS = 0.034f;
const float GYRO_BIAS    = 0.0f;
const float CF_ALPHA     = 0.98f;

const int STEPPER1_DIR_PIN  = 16;
const int STEPPER1_STEP_PIN = 17;
const int STEPPER2_DIR_PIN  = 4;
const int STEPPER2_STEP_PIN = 14;
const int STEPPER_EN_PIN    = 15;
const int TOGGLE_PIN        = 32;

const int   LOOP_INTERVAL_MS    = 10;
const int   STEPPER_INTERVAL_US = 20;
const float DT = LOOP_INTERVAL_MS / 1000.0f;

BluetoothSerial BT;
ESP32Timer ITimer(3);
Adafruit_MPU6050 mpu;
step step1(STEPPER_INTERVAL_US, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
step step2(STEPPER_INTERVAL_US, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

bool TimerHandler(void *) { step1.runStepper(); step2.runStepper(); return true; }

template<typename T> void logPrint(T v)          { Serial.print(v);      BT.print(v); }
template<typename T> void logPrint(T v, int dp)  { Serial.print(v, dp);  BT.print(v, dp); }
template<typename T> void logPrintln(T v)        { Serial.println(v);    BT.println(v); }
template<typename T> void logPrintln(T v, int dp){ Serial.println(v,dp); BT.println(v,dp); }

void setup() {
  Serial.begin(115200);
  BT.begin("BalanceRobot");
  pinMode(TOGGLE_PIN, OUTPUT);
  if (!mpu.begin()) { Serial.println("MPU6050 not found"); while(1) delay(10); }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) { while(1) delay(10); }
  step1.setAccelerationRad(10.0f);
  step2.setAccelerationRad(10.0f);
  pinMode(STEPPER_EN_PIN, OUTPUT);
  digitalWrite(STEPPER_EN_PIN, LOW);
  pinMode(0, INPUT_PULLUP);
  Serial.println("Press BOOT to start logging...");
  while (digitalRead(0) == HIGH) delay(10);
  while (digitalRead(0) == LOW)  delay(10);
  logPrintln("t_ms,theta,theta_dot,x,x_dot");
}

void loop() {
  static unsigned long loopTimer = millis();
  static float theta_filt = 0.0f;
  if (millis() < loopTimer) return;
  loopTimer += LOOP_INTERVAL_MS;
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float theta_dot   = g.gyro.y - GYRO_BIAS;
  float theta_accel = a.acceleration.z / 9.81f;
  theta_filt = CF_ALPHA * (theta_filt + theta_dot * DT) + (1.0f - CF_ALPHA) * theta_accel;
  float x     = step1.getPositionRad() * WHEEL_RADIUS;
  float x_dot = step1.getSpeedRad()    * WHEEL_RADIUS;
  logPrint(millis());       logPrint(',');
  logPrint(theta_filt, 5);  logPrint(',');
  logPrint(theta_dot,  5);  logPrint(',');
  logPrint(x,          5);  logPrint(',');
  logPrintln(x_dot,    5);
}

*/


// ── LQR balance controller ────────────────────────────────────────────────────
#include <Arduino.h>
#include <BluetoothSerial.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>

// ── LQR gains  u = -(K_theta*theta + K_theta_dot*theta_dot + K_x*x + K_x_dot*x_dot)
// UPDATE these with your identified values
const float K_THETA     = 20.5f/2;
const float K_THETA_DOT = 2.43f/2;
const float K_X         = 0.125f/2;
const float K_X_DOT     = 0.5f/2;

const float U_MAX        = 50.0f;   // motor speed saturation (rad/s)
const float WHEEL_RADIUS = 0.034f;  // metres
const float GYRO_BIAS    = 0.0f;    // rad/s — update from noise analysis
const float CF_ALPHA     = 0.98f;

// ── Pins ──────────────────────────────────────────────────────────────────────
const int STEPPER1_DIR_PIN  = 16;
const int STEPPER1_STEP_PIN = 17;
const int STEPPER2_DIR_PIN  = 4;
const int STEPPER2_STEP_PIN = 14;
const int STEPPER_EN_PIN    = 15;
const int TOGGLE_PIN        = 32;

const int   LOOP_INTERVAL_MS    = 10;
const int   PRINT_INTERVAL_MS   = 50;   // 20 Hz log rate (separate from 100 Hz control)
const int   STEPPER_INTERVAL_US = 20;
const float DT = LOOP_INTERVAL_MS / 1000.0f;

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
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
    Serial.println("Stepper ISR failed");
    while (1) delay(10);
  }

  // High acceleration so motors track the control signal with minimal lag
  step1.setAccelerationRad(100.0f);
  step2.setAccelerationRad(100.0f);

  pinMode(STEPPER_EN_PIN, OUTPUT);
  digitalWrite(STEPPER_EN_PIN, LOW);

  pinMode(0, INPUT_PULLUP);
  Serial.println("Press BOOT to start controller...");
  while (digitalRead(0) == HIGH) delay(10);
  while (digitalRead(0) == LOW)  delay(10);

  logPrintln("t_ms,theta,theta_dot,x,x_dot,u");
}

void loop()
{
  static unsigned long loopTimer  = millis();
  static unsigned long printTimer = millis();
  static float theta_filt = 0.0f;

  if (millis() < loopTimer) return;
  loopTimer += LOOP_INTERVAL_MS;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float theta_dot   = g.gyro.y - GYRO_BIAS;
  float theta_accel = a.acceleration.z / 9.81f;
  theta_filt = CF_ALPHA * (theta_filt + theta_dot * DT)
             + (1.0f - CF_ALPHA) * theta_accel;

  float x     = step1.getPositionRad() * WHEEL_RADIUS;
  float x_dot = step1.getSpeedRad()    * WHEEL_RADIUS;

  // LQR control law
  float u = -(K_THETA     * theta_filt
            + K_THETA_DOT * theta_dot
            + K_X         * x
            + K_X_DOT     * x_dot);

  u = constrain(u, -U_MAX, U_MAX);

  step1.setTargetSpeedRad(u);
  step2.setTargetSpeedRad(-u);

  if (millis() >= printTimer) {
    printTimer += PRINT_INTERVAL_MS;
    logPrint("t=");        logPrint(millis());
    logPrint(" th=");      logPrint(theta_filt, 4);
    logPrint(" th_d=");    logPrint(theta_dot,  4);
    logPrint(" x=");       logPrint(x,          4);
    logPrint(" x_d=");     logPrint(x_dot,      4);
    logPrint(" u=");       logPrintln(u,        4);
  }
}




//python -m serial.tools.miniterm COM5 115200