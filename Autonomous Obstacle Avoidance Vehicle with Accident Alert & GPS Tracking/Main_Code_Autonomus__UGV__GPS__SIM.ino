#include <ESP32Servo.h>   // Use this library on ESP32

Servo myServo;

#include <Wire.h>
#include <HardwareSerial.h>
#include <math.h>

// =================== GPS / MPU6050 / SIM800L Add-on ===================
// Recommended pins (must match your wiring)
#define GPS_RX_PIN     16   // ESP32 RX2  <- GPS TX
#define GPS_TX_PIN     17   // ESP32 TX2  -> GPS RX

#define GSM_RX_PIN     18   // ESP32 RX1  <- SIM800L TX
#define GSM_TX_PIN      5   // ESP32 TX1  -> SIM800L RX

#define MPU_SDA_PIN    21
#define MPU_SCL_PIN    19

// Alert settings
const String EMERGENCY_NUMBER = "+91XXXXXXXXXX";   // <-- replace with your number
const float ACCEL_THRESHOLD_G  = 2.0;               // crash/vibration threshold
const float ANGLE_THRESHOLD_DEG = 35.0;             // sudden angle change threshold
const unsigned long ALERT_COOLDOWN_MS = 60000;      // 1 min between SMS alerts

HardwareSerial GPS_Serial(2);
HardwareSerial GSM_Serial(1);

struct GPSData {
  bool valid = false;
  float lat = 0.0;
  float lon = 0.0;
};

static float mpuBaselineRoll = 0.0;
static float mpuBaselinePitch = 0.0;
static bool mpuBaselineReady = false;
static unsigned long lastAlertMs = 0;
static bool accidentAlreadyHandled = false;

void initGPSModule();
void initGSMModule();
void initMPU6050Module();
void updateMPUBaseline();
bool readGPSLocation(GPSData &data);
String gpsCoordToDecimal(const String &value, const String &hemisphere);
bool getMPU6050Angle(float &roll, float &pitch, float &accelG);
bool accidentDetected();
String buildEmergencyMessage(const GPSData &gps);
void sendSMS(const String &number, const String &message);
void alertAccidentWithGPS();
void checkAccidentAndAlert();
void sendATCommand(const String &cmd, unsigned long waitMs = 500);
// =================== Pin Definitions ===================
#define TRIG_PIN      4
#define ECHO_PIN      34

#define SERVO_PIN     13
int Car_Length = 10; // the size of Car

#define IN1           26  // L298N IN1 - Left motor
#define IN2           25  // L298N IN2 - Left motor
#define IN3           33  // L298N IN3 - Right motor
#define IN4           32  // L298N IN4 - Right motor

#define ENA           22  // PWM - Left motor speed
#define ENB           23  // PWM - Right motor speed

// =================== PWM Setup ===================
#define PWM_CH_A      0
#define PWM_CH_B      1
#define PWM_FREQ     30000
#define PWM_RES      8

// =================== Speed Values (0–255) ===================
const int SPEED_FORWARD = 180;
const int SPEED_TURN    = 150;
const int SPEED_REVERSE = 160;

// =================== Distance Thresholds (cm) ===================
const int SAFE_START_DIST = 20;  // D > 20 → ok to start moving
const int SAFE_RUN_DIST   = 30;  // D > 30 → keep moving
const int STOP_DIST       = 10;  // D < 10 → too close, must avoid
const int BACK_SAFE_DIST  = 20;  // Reverse until D ≥ 20

// =================== Servo Angles ===================
// Adjust according to your servo mounting
float SERVO_FRONT(){ // front 
  float sum = 0;
    for (int i=75;i<=105;i+=3){
      float A = getDistanceAtAngle(i);
      sum += A; //value stored in sum
    }
  float Front_Distance = sum / 11;
  
  if(Front_Distance<10.00){
    if(0.5233*Front_Distance > Car_Length){  // calculate the ARC of imaginary circle of radious Average distance.
      return Front_Distance;
    }
    else{
      return 9;
    }
  }
  else{
    return Front_Distance;
  }

}

float SERVO_LEFT(){  // left 
  float sum = 0;
    for (int i=135;i<=165;i+=3){
      float A = getDistanceAtAngle(i);
      sum += A; //value stored in sum
    }
  float Left_Distance = sum / 11;

  if(Left_Distance<10.00){
    if(0.5233*Left_Distance > Car_Length){ // calculate the ARC of imaginary circle of radious Average distance.
      return Left_Distance;
    }
    else{
      return 20;
    }
  }
  else{
    return Left_Distance;
  }

}

float SERVO_RIGHT(){  // right
  float sum = 0;
    for (int i=15;i<=45;i+=3){
      float A = getDistanceAtAngle(i);
      sum += A; //value stored in sum
    }
  float RIGHT_Distance = sum / 11;

  if(RIGHT_Distance<10.00){
    if(0.5233*RIGHT_Distance > Car_Length){ // calculate the ARC of imaginary circle of radious Average distance.
      return RIGHT_Distance;
    }
    else{
      return 20;
    }
  }
  else{
    return RIGHT_Distance;
  }
}

// =================== Motor Control Helpers ===================
void stopCar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(PWM_CH_A, 0);
  ledcWrite(PWM_CH_B, 0);
}

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledcWrite(PWM_CH_A, SPEED_FORWARD);
  ledcWrite(PWM_CH_B, SPEED_FORWARD);
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(PWM_CH_A, SPEED_REVERSE);
  ledcWrite(PWM_CH_B, SPEED_REVERSE);
}

void turnLeftFor(unsigned long ms) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledcWrite(PWM_CH_A, SPEED_TURN);
  ledcWrite(PWM_CH_B, SPEED_TURN);
  delay(ms);
  stopCar();
  myServo.write(90); // Angle for 90 of FRONT of RC car
  delay(50);
}

void turnRightFor(unsigned long ms) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(PWM_CH_A, SPEED_TURN);
  ledcWrite(PWM_CH_B, SPEED_TURN);
  delay(ms);
  stopCar();
  myServo.write(90); // Angle for 90 of FRONT of RC car
  delay(50);
}

// Rotate car roughly 180° (adjust delay experimentally)
void rotate180() {
  int rotationCount = 0;

  while (rotationCount < 4) {   // max 4 × 90° attempts
    // Rotate approx 90°
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    delay(300);   // adjust for ~90°
    stopCar();
    delay(150);

    myServo.write(90); // Angle for 90 of FRONT of RC car
    delay(150);
    // Check front distance after rotation
    float frontD = readDistanceCmRaw();

    if (frontD > SAFE_START_DIST) {
      // Front is clear → rotation successful
      return;
    }

    rotationCount++;
  }

  // Safety stop (if something goes wrong)
  stopCar();
}

// =================== Ultrasonic Distance ===================
float readDistanceCmRaw() {
  // Trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read echo; timeout ~ 30 ms
  long duration = pulseIn(ECHO_PIN, HIGH, 6000);
  if (duration == 0) {
    return 999; // no echo, assume very far
  }

  // Speed of sound: ~343 m/s -> 29.1 µs per cm (round-trip ~58 µs per cm)
  float distance = duration / 58.0;
  return distance;
}

// Read distance while pointing servo at given angle
float getDistanceAtAngle(int angle) {
  myServo.write(angle);
  delay(150);           // allow servo to reach position
  float d = readDistanceCmRaw();
  return d;
}

// =================== Backward Until Safe ===================
// Implements: "go backward until D ≥ 20"
void reverseUntilSafe() {
  unsigned long start = millis();
  while (true) {
    checkAccidentAndAlert();
    moveBackward();
    float d = SERVO_FRONT();
    if (d >= BACK_SAFE_DIST) {
      break;
    }
    // safety timeout so it doesn't reverse forever if sensor fails
    if (millis() - start > 3000) {
      break;
    }
  }
  stopCar();
}

// =================== Condition (2): Forward with Avoidance ===================
// Your logic (2):
// D > 30 go forward until D < 10
// if D < 10: see LEFT & RIGHT
// → go L/R direction which has D > 20 : go their L/R
// if D < 10 and LEFT & RIGHT both direction D < 20 : go backward until D ≥ 20
// → after REVERSE (Front is 180 Rotation) the CAR : and again the condition of first
void runCondition2();    // forward declaration
void runFirstCondition(); // forward declaration

void runCondition2() {
  myServo.write(90);
  delay(80);

  while (true) {
    checkAccidentAndAlert();
    float dFront = SERVO_FRONT();

    if (dFront > SAFE_RUN_DIST) {
      // D > 30 → go forward
      moveForward();
      // small delay to actually move
      delay(50);
    } else if (dFront <= STOP_DIST) {
      // D < 10 → must handle obstacle
      stopCar();

      // Check left and right
      float dLeft = SERVO_LEFT();
      float dRight = SERVO_RIGHT();

      if (dLeft > SAFE_START_DIST || dRight > SAFE_START_DIST) {
        // At least one side is free: choose the better side (larger D)
        if (dLeft > dRight) {
          turnLeftFor(300);
        } else {
          turnRightFor(300);
        }
        // After turning to L/R, again apply Condition (2)
        continue;
      } else {
        // Front close and both L & R blocked: go backward until D ≥ 20
        reverseUntilSafe();

        // Now rotate car 180° and then go back to "first" logic
        rotate180();
        stopCar();
        return;  // leave condition (2); loop() will call first condition again
      }
    } else {
      // if 10 < D ≤ 30, you can choose:
      // either treat as safe or as caution; here we still go forward slowly
      moveForward();
      delay(50);
    }
  }
}

// =================== First Condition ===================
// Your logic (1):
// 1) SERVO angle 0 and check forward distance
//    if D > 20 : car go forward
//    else : check LEFT and RIGHT direction
//    → go L/R (which is satisfy D > 20) : if go L/R
//    →→ again Condition (2)
//
// 2) (inside) D > 30 go forward until D < 10 (this is same as runCondition2)
void runFirstCondition() {
  myServo.write(90);
  delay(80);

  // Servo to front and check forward distance
  float dFront = SERVO_FRONT();


  if (dFront > SAFE_START_DIST) {
    // D > 20 → directly go to Condition (2)
    runCondition2();
  } else {
    // Front blocked, check left and right
    float dLeft = SERVO_LEFT();
    float dRight = SERVO_RIGHT();

    if (dLeft > SAFE_START_DIST || dRight > SAFE_START_DIST) {
      // choose the side with greater distance (>20)
      if (dLeft > dRight) {
        turnLeftFor(300);
      } else {
        turnRightFor(300);
      }
      // After turning L/R → again Condition (2)
      runCondition2();
    } else {
      // front, left, right all blocked
      // go backward until D ≥ 20
      reverseUntilSafe();

      // rotate 180° and then loop back to first condition in main loop
      rotate180();
      stopCar();
    }
  }
}


// =================== GPS / MPU6050 / SIM800L Add-on Functions ===================
void sendATCommand(const String &cmd, unsigned long waitMs) {
  GSM_Serial.print(cmd);
  GSM_Serial.print("\r");
  delay(waitMs);
  while (GSM_Serial.available()) {
    Serial.write(GSM_Serial.read());
  }
}

void initGPSModule() {
  GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}

void initGSMModule() {
  GSM_Serial.begin(9600, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);
  delay(1000);
  sendATCommand("AT");
  sendATCommand("ATE0");
  sendATCommand("AT+CMGF=1");   // SMS text mode
}

void initMPU6050Module() {
  Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);   // PWR_MGMT_1
  Wire.write(0x00);   // wake up MPU6050
  Wire.endTransmission(true);
  delay(100);
}

void updateMPUBaseline() {
  float roll = 0, pitch = 0, accelG = 0;
  if (getMPU6050Angle(roll, pitch, accelG)) {
    mpuBaselineRoll = roll;
    mpuBaselinePitch = pitch;
    mpuBaselineReady = true;
  }
}

bool getMPU6050Angle(float &roll, float &pitch, float &accelG) {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  Wire.requestFrom(0x68, 14, true);
  if (Wire.available() < 14) return false;

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  int16_t tempRaw = (Wire.read() << 8) | Wire.read();
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();
  (void)tempRaw;
  (void)gx;
  (void)gy;
  (void)gz;

  const float accelScale = 16384.0; // ±2g
  float axg = ax / accelScale;
  float ayg = ay / accelScale;
  float azg = az / accelScale;

  accelG = sqrt(axg * axg + ayg * ayg + azg * azg);

  roll  = atan2(ayg, azg) * 180.0 / PI;
  pitch = atan2(-axg, sqrt(ayg * ayg + azg * azg)) * 180.0 / PI;
  return true;
}

String gpsCoordToDecimal(const String &value, const String &hemisphere) {
  if (value.length() < 3) return "0";
  int dot = value.indexOf('.');
  if (dot < 0) return value;
  int degLen = (hemisphere == "N" || hemisphere == "S") ? 2 : 3;
  if (dot < degLen) return value;

  String degPart = value.substring(0, degLen);
  String minPart = value.substring(degLen);

  float deg = degPart.toFloat();
  float minutes = minPart.toFloat();
  float decimal = deg + (minutes / 60.0);

  if (hemisphere == "S" || hemisphere == "W") decimal = -decimal;
  return String(decimal, 6);
}

bool readGPSLocation(GPSData &data) {
  unsigned long start = millis();
  String line = "";

  while (millis() - start < 1500) {
    while (GPS_Serial.available()) {
      char c = GPS_Serial.read();
      if (c == '\n') {
        if (line.startsWith("$GPRMC") || line.startsWith("$GNRMC")) {
          int comma = 0;
          String fields[12];
          String token = "";
          for (unsigned int i = 0; i < line.length(); i++) {
            char ch = line[i];
            if (ch == ',' || ch == '*') {
              if (comma < 12) fields[comma] = token;
              token = "";
              comma++;
              if (ch == '*') break;
            } else {
              token += ch;
            }
          }

          // RMC fields:
          // 1=time, 2=status, 3=lat, 4=N/S, 5=lon, 6=E/W
          if (fields[2] == "A") {
            data.lat = gpsCoordToDecimal(fields[3], fields[4]).toFloat();
            data.lon = gpsCoordToDecimal(fields[5], fields[6]).toFloat();
            data.valid = true;
            return true;
          }
        }
        line = "";
      } else if (c != '\r') {
        line += c;
      }
    }
  }
  return false;
}

bool accidentDetected() {
  float roll = 0, pitch = 0, accelG = 0;
  if (!getMPU6050Angle(roll, pitch, accelG)) {
    return false;
  }

  if (!mpuBaselineReady) {
    mpuBaselineRoll = roll;
    mpuBaselinePitch = pitch;
    mpuBaselineReady = true;
    return false;
  }

  float angleChange = fmaxf(fabsf(roll - mpuBaselineRoll), fabsf(pitch - mpuBaselinePitch));

  if (accelG >= ACCEL_THRESHOLD_G) return true;
  if (angleChange >= ANGLE_THRESHOLD_DEG) return true;

  return false;
}

String buildEmergencyMessage(const GPSData &gps) {
  String msg = "Accident detected!";

  if (gps.valid) {
    msg += "\nLive GPS:";
    msg += "\nLat: " + String(gps.lat, 6);
    msg += "\nLon: " + String(gps.lon, 6);
    msg += "\nGoogle Maps: https://maps.google.com/?q=" + String(gps.lat, 6) + "," + String(gps.lon, 6);
  } else {
    msg += "\nGPS signal not valid";
  }

  return msg;
}

void sendSMS(const String &number, const String &message) {
  sendATCommand("AT+CMGF=1", 300);
  GSM_Serial.print("AT+CMGS=\"");
  GSM_Serial.print(number);
  GSM_Serial.println("\"");
  delay(500);
  GSM_Serial.print(message);
  GSM_Serial.write(26);   // Ctrl+Z
  delay(5000);
}

void alertAccidentWithGPS() {
  GPSData gps;
  readGPSLocation(gps);

  String msg = buildEmergencyMessage(gps);
  sendSMS(EMERGENCY_NUMBER, msg);
}

void checkAccidentAndAlert() {
  if (accidentAlreadyHandled) return;

  if (millis() - lastAlertMs < ALERT_COOLDOWN_MS) return;

  if (accidentDetected()) {
    accidentAlreadyHandled = true;
    lastAlertMs = millis();
    stopCar();
    alertAccidentWithGPS();
  }
}
// =================== Setup & Loop ===================
void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcSetup(PWM_CH_A, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA, PWM_CH_A);
  ledcAttachPin(ENB, PWM_CH_B);

  myServo.attach(SERVO_PIN);
  myServo.write(90);
  delay(500);

  initGPSModule();
  initGSMModule();
  initMPU6050Module();
  updateMPUBaseline();

  stopCar();

}

void loop() {
  checkAccidentAndAlert();
  // Always start from the "first" condition
  runFirstCondition();
  // small delay for stability
  delay(20);
}
