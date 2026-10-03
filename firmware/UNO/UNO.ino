#include <Arduino.h>
#include <math.h>
#include <Servo.h>
#include <Wire.h>

/* =========================================================
   UNO + CNC SHIELD V3 + ROBOT ARM IK + TCA9548A + 2 AS5600

   - Nhận lệnh qua Serial RX/TX D0/D1
   - Điều khiển XYZ bằng G-code:
       G28
       G90
       G1 X40 Y160 Z100 F20

   - Jog tay:
       w: X+
       s: X-
       q: Y+
       e: Y-
       a: Z+
       d: Z-
       x: STOP

   - AS5600:
       LOW  shank  -> TCA channel 1
       HIGH shank  -> TCA channel 0

   - Z hiện đang chạy qua driver A:
       Z_STEP = 12
       Z_DIR  = 13
   ========================================================= */

/* ================= SERIAL SETTING ================= */
#define SERIAL_BAUD 115200

#define DEBUG_LOG 1

#if DEBUG_LOG
  #define DBG_PRINT(x)   Serial.print(x)
  #define DBG_PRINTLN(x) Serial.println(x)
#else
  #define DBG_PRINT(x)
  #define DBG_PRINTLN(x)
#endif

/* ================= CNC SHIELD V3 PIN ================= */
#define X_STEP 2
#define Y_STEP 3

// Đang dùng socket A làm trục Z
#define Z_STEP 12

#define X_DIR 5
#define Y_DIR 6

// Đang dùng socket A làm trục Z
#define Z_DIR 13

#define ENABLE_PIN 8   // EN CNC Shield V3, LOW = bật driver

/* ================= SERVO GRIPPER ================= */
#define SERVO_PIN A3

Servo gripperServo;

int servoAngle = 90;
#define SERVO_OPEN_ANGLE  30
#define SERVO_CLOSE_ANGLE 90

/* ================= TCA9548A + AS5600 ================= */
#define TCA_ADDR     0x70
#define AS5600_ADDR  0x36

#define AS5600_RAW_ANGLE_H 0x0C
#define AS5600_RAW_ANGLE_L 0x0D

// Kênh TCA đang cắm AS5600
#define AS5600_LOW_CH   1
#define AS5600_HIGH_CH  0

// Chu kỳ in encoder ra Serial
#define ENCODER_PRINT_INTERVAL_MS 500

// Nếu muốn in encoder trong lúc đang chạy G-code thì để 1
// Nếu motor bị khựng nhẹ do I2C/Serial thì đổi thành 0
#define ENCODER_PRINT_DURING_MOVE 0

// In dòng ENC chi tiết cho Serial Monitor.
// Khi dùng Unity nên để 0 để Unity chỉ cần lọc dòng A,...
#define ENCODER_VERBOSE_PRINT_ENABLE 0

// ================= UNITY DIGITAL TWIN FEEDBACK =================
// Gửi dữ liệu góc về Unity theo format:
// A,baseDeg,lowJointDeg,highJointDeg
// Ví dụ: A,0.00,32.50,87.20
#define UNITY_FEEDBACK_ENABLE 1
#define UNITY_FEEDBACK_INTERVAL_MS 50

unsigned long lastEncoderPrintMs = 0;
unsigned long lastUnityFeedbackMs = 0;

// Zero encoder, sau này calib thì chỉnh
#define LOW_AS5600_ZERO_DEG   0.0f
#define HIGH_AS5600_ZERO_DEG  0.0f

// LOW AS5600 đang nằm trên trục stepper/pulley GT2 20T
#define LOW_ENCODER_ON_MOTOR_SIDE   true

// HIGH nếu gắn trên motor thì true, nếu gắn trực tiếp khớp thì false
#define HIGH_ENCODER_ON_MOTOR_SIDE  true

/* ================= BOOT SAFETY ================= */
unsigned long bootTimeMs = 0;

// Bỏ qua rác Serial từ ESP32-C3 trong 2.5 giây đầu
#define SERIAL_IGNORE_AFTER_BOOT_MS 2500

/* ================= SPEED SETTING ================= */
#define STEP_INTERVAL_US 2000
#define STEP_PULSE_US 5
#define MIN_STEP_INTERVAL_US 900

/* ================= ROBOT GEOMETRY - IK ================= */
#define LOW_SHANK_LENGTH     140.0f
#define HIGH_SHANK_LENGTH    140.0f
#define END_EFFECTOR_OFFSET  95.0f

#define HOME_X 0.0f
#define HOME_Y 174.0f
#define HOME_Z 120.0f

/* ================= STEPPER CONFIG ================= */
#define MOTOR_STEPS_PER_REV 200.0f
#define MICROSTEPS 16.0f

/* ===== TỈ SỐ TRUYỀN LOW SHANK GT2 =====
   Pulley stepper GT2 20T:
   Chu vi = 20 × 2mm = 40mm
   Bán kính pulley nhỏ = 40 / (2π) = 6.366mm

   Bánh lớn low shank R = 27.6mm

   LOW_GEAR_RATIO = 27.6 / 6.366 ≈ 4.335
*/
#define GT2_PITCH_MM              2.0f
#define GT2_MOTOR_TEETH           20.0f
#define LOW_BIG_PULLEY_RADIUS_MM  27.6f

#define GT2_MOTOR_PULLEY_RADIUS_MM  ((GT2_MOTOR_TEETH * GT2_PITCH_MM) / (2.0f * PI))
#define LOW_GEAR_RATIO              (LOW_BIG_PULLEY_RADIUS_MM / GT2_MOTOR_PULLEY_RADIUS_MM)

// Trục X và high shank tạm giữ 4.5
#define ROT_GEAR_RATIO   4.5f
#define HIGH_GEAR_RATIO  4.5f

#define ROT_STEPS_PER_RAD  ((MOTOR_STEPS_PER_REV * MICROSTEPS * ROT_GEAR_RATIO)  / (2.0f * PI))
#define LOW_STEPS_PER_RAD  ((MOTOR_STEPS_PER_REV * MICROSTEPS * LOW_GEAR_RATIO)  / (2.0f * PI))
#define HIGH_STEPS_PER_RAD ((MOTOR_STEPS_PER_REV * MICROSTEPS * HIGH_GEAR_RATIO) / (2.0f * PI))

/* ================= DIRECTION CONFIG ================= */
#define INVERT_X_DIR false
#define INVERT_Y_DIR true
#define INVERT_Z_DIR true

/* ================= ANGLE OFFSET ================= */
#define ROT_OFFSET_RAD  0.0f
#define LOW_OFFSET_RAD  0.0f
#define HIGH_OFFSET_RAD 0.0f

/* ================= MOTOR STATE ================= */
bool xRun = false;
bool yRun = false;
bool zRun = false;

int xJogSign = 1;
int yJogSign = 1;
int zJogSign = 1;

unsigned long lastXStep = 0;
unsigned long lastYStep = 0;
unsigned long lastZStep = 0;

/* ================= POSITION STATE ================= */
float currentX = HOME_X;
float currentY = HOME_Y;
float currentZ = HOME_Z;

long currentXSteps = 0;
long currentYSteps = 0;
long currentZSteps = 0;

bool absoluteMode = true;
bool isHomed = false;

/* ================= SERIAL BUFFER ================= */
char lineBuffer[100];
uint8_t lineIndex = 0;

/* ================= IK RESULT ================= */
struct IKResult {
  float rotRad;
  float lowRad;
  float highRad;
};

/* ================= BASIC FUNCTIONS ================= */
float clampFloat(float v, float minV, float maxV) {
  if (v < minV) return minV;
  if (v > maxV) return maxV;
  return v;
}

float wrap180(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg <= -180.0f) deg += 360.0f;
  return deg;
}

void delayUsLong(unsigned long us) {
  while (us > 16000) {
    delayMicroseconds(16000);
    us -= 16000;
  }

  delayMicroseconds(us);
}

void stepMotor(uint8_t stepPin) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(stepPin, LOW);
}

void setAxisDir(uint8_t dirPin, bool positive, bool invertDir) {
  bool dirState = positive ? HIGH : LOW;

  if (invertDir) {
    dirState = !dirState;
  }

  digitalWrite(dirPin, dirState);
}

/* ================= AS5600 FUNCTIONS ================= */
bool tcaSelect(uint8_t channel) {
  if (channel > 7) return false;

  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  uint8_t err = Wire.endTransmission();

  return err == 0;
}

bool readAS5600Raw(uint8_t channel, uint16_t &raw) {
  raw = 0;

  if (!tcaSelect(channel)) {
    return false;
  }

  delayMicroseconds(300);

  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_RAW_ANGLE_H);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  uint8_t n = Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2);

  if (n < 2) {
    return false;
  }

  uint8_t highByte = Wire.read();
  uint8_t lowByte  = Wire.read();

  raw = ((highByte & 0x0F) << 8) | lowByte;

  return true;
}

float rawToDeg(uint16_t raw) {
  return raw * 360.0f / 4096.0f;
}

void printEncodersNow() {
  uint16_t rawLow = 0;
  uint16_t rawHigh = 0;

  bool okLow = readAS5600Raw(AS5600_LOW_CH, rawLow);
  bool okHigh = readAS5600Raw(AS5600_HIGH_CH, rawHigh);

  Serial.print("ENC | ");

  if (okLow) {
    float lowDeg = rawToDeg(rawLow);
    float lowRelDeg = wrap180(lowDeg - LOW_AS5600_ZERO_DEG);
    float lowJointDeg = lowRelDeg;

    if (LOW_ENCODER_ON_MOTOR_SIDE) {
      lowJointDeg = lowRelDeg / LOW_GEAR_RATIO;
    }

    Serial.print("LOW raw=");
    Serial.print(rawLow);
    Serial.print(" deg=");
    Serial.print(lowDeg, 2);
    Serial.print(" rel=");
    Serial.print(lowRelDeg, 2);
    Serial.print(" joint=");
    Serial.print(lowJointDeg, 2);
  } else {
    Serial.print("LOW ERROR");
  }

  Serial.print("  ||  ");

  if (okHigh) {
    float highDeg = rawToDeg(rawHigh);
    float highRelDeg = wrap180(highDeg - HIGH_AS5600_ZERO_DEG);
    float highJointDeg = highRelDeg;

    if (HIGH_ENCODER_ON_MOTOR_SIDE) {
      highJointDeg = highRelDeg / HIGH_GEAR_RATIO;
    }

    Serial.print("HIGH raw=");
    Serial.print(rawHigh);
    Serial.print(" deg=");
    Serial.print(highDeg, 2);
    Serial.print(" rel=");
    Serial.print(highRelDeg, 2);
    Serial.print(" joint=");
    Serial.print(highJointDeg, 2);
  } else {
    Serial.print("HIGH ERROR");
  }

  Serial.println();
}

void showEncodersPeriodic() {
#if ENCODER_VERBOSE_PRINT_ENABLE
  unsigned long nowMs = millis();

  if (nowMs - lastEncoderPrintMs >= ENCODER_PRINT_INTERVAL_MS) {
    lastEncoderPrintMs = nowMs;
    printEncodersNow();
  }
#endif
}

/* ================= UNITY FEEDBACK FUNCTIONS =================
   Unity chỉ cần đọc các dòng bắt đầu bằng "A,"
   Format chuẩn:
   A,baseDeg,lowJointDeg,highJointDeg

   Lưu ý:
   - Base hiện chưa có AS5600 riêng, nên baseDeg đang lấy theo currentXSteps.
   - LOW/HIGH lấy từ AS5600 thật.
   - Nếu Unity xoay ngược chiều, chỉnh invert trong Unity trước.
   - Nếu lệch góc ban đầu, chỉnh LOW_AS5600_ZERO_DEG / HIGH_AS5600_ZERO_DEG.
*/
bool readJointAngles(float &lowJointDeg, float &highJointDeg) {
  uint16_t rawLow = 0;
  uint16_t rawHigh = 0;

  bool okLow = readAS5600Raw(AS5600_LOW_CH, rawLow);
  bool okHigh = readAS5600Raw(AS5600_HIGH_CH, rawHigh);

  if (!okLow || !okHigh) {
    return false;
  }

  float lowDeg = rawToDeg(rawLow);
  float lowRelDeg = wrap180(lowDeg - LOW_AS5600_ZERO_DEG);

  lowJointDeg = lowRelDeg;

  if (LOW_ENCODER_ON_MOTOR_SIDE) {
    lowJointDeg = lowRelDeg / LOW_GEAR_RATIO;
  }

  float highDeg = rawToDeg(rawHigh);
  float highRelDeg = wrap180(highDeg - HIGH_AS5600_ZERO_DEG);

  highJointDeg = highRelDeg;

  if (HIGH_ENCODER_ON_MOTOR_SIDE) {
    highJointDeg = highRelDeg / HIGH_GEAR_RATIO;
  }

  return true;
}

void sendUnityFeedback() {
#if UNITY_FEEDBACK_ENABLE
  float lowJointDeg = 0.0f;
  float highJointDeg = 0.0f;

  bool ok = readJointAngles(lowJointDeg, highJointDeg);

  if (!ok) {
    Serial.println("A,ERR");
    return;
  }

  // Base chưa có AS5600 riêng.
  // Tạm ước lượng theo số step của trục X.
  // Khi lắp AS5600 cho base, thay dòng này bằng góc đọc từ AS5600 base.
  float baseRad = ((float)currentXSteps / ROT_STEPS_PER_RAD) - ROT_OFFSET_RAD;
  float baseDeg = baseRad * 180.0f / PI;

  Serial.print("A,");
  Serial.print(baseDeg, 2);
  Serial.print(",");
  Serial.print(lowJointDeg, 2);
  Serial.print(",");
  Serial.println(highJointDeg, 2);
#endif
}

void showUnityFeedbackPeriodic() {
#if UNITY_FEEDBACK_ENABLE
  unsigned long nowMs = millis();

  if (nowMs - lastUnityFeedbackMs >= UNITY_FEEDBACK_INTERVAL_MS) {
    lastUnityFeedbackMs = nowMs;
    sendUnityFeedback();
  }
#endif
}


void scanTCAChannels() {
  Serial.println("Scan TCA9548A channels...");

  for (uint8_t ch = 0; ch < 8; ch++) {
    uint16_t raw = 0;
    bool ok = readAS5600Raw(ch, raw);

    Serial.print("TCA CH");
    Serial.print(ch);
    Serial.print(": ");

    if (ok) {
      Serial.print("AS5600 OK raw=");
      Serial.print(raw);
      Serial.print(" deg=");
      Serial.println(rawToDeg(raw), 2);
    } else {
      Serial.println("not found");
    }
  }
}

/* ================= STOP ALL ================= */
void stopAll() {
  xRun = false;
  yRun = false;
  zRun = false;

  DBG_PRINTLN("STOP ALL");
}

/* ================= MANUAL JOG ================= */
void runOnlyX(bool dir) {
  xRun = true;
  yRun = false;
  zRun = false;

  xJogSign = dir == HIGH ? 1 : -1;
  setAxisDir(X_DIR, dir == HIGH, INVERT_X_DIR);

  DBG_PRINTLN(dir == HIGH ? "X+" : "X-");
}

void runOnlyY(bool dir) {
  xRun = false;
  yRun = true;
  zRun = false;

  yJogSign = dir == HIGH ? 1 : -1;
  setAxisDir(Y_DIR, dir == HIGH, INVERT_Y_DIR);

  DBG_PRINTLN(dir == HIGH ? "Y+" : "Y-");
}

void runOnlyZ(bool dir) {
  xRun = false;
  yRun = false;
  zRun = true;

  zJogSign = dir == HIGH ? 1 : -1;
  setAxisDir(Z_DIR, dir == HIGH, INVERT_Z_DIR);

  DBG_PRINTLN(dir == HIGH ? "Z+" : "Z-");
}

bool handleSingleKey(char c) {
  if (c == 'w') {
    runOnlyX(HIGH);
    return true;
  }
  else if (c == 's') {
    runOnlyX(LOW);
    return true;
  }
  else if (c == 'q') {
    runOnlyY(HIGH);
    return true;
  }
  else if (c == 'e') {
    runOnlyY(LOW);
    return true;
  }
  else if (c == 'a') {
    runOnlyZ(HIGH);
    return true;
  }
  else if (c == 'd') {
    runOnlyZ(LOW);
    return true;
  }
  else if (c == 'x' || c == ' ') {
    stopAll();
    return true;
  }
  else if (c == 'o') {
    gripperServo.write(SERVO_OPEN_ANGLE);
    servoAngle = SERVO_OPEN_ANGLE;
    DBG_PRINTLN("SERVO OPEN");
    return true;
  }
  else if (c == 'p') {
    gripperServo.write(SERVO_CLOSE_ANGLE);
    servoAngle = SERVO_CLOSE_ANGLE;
    DBG_PRINTLN("SERVO CLOSE");
    return true;
  }

  return false;
}

/* ================= IK FUNCTION ================= */
bool calcIK(float x, float y, float z, IKResult &out) {
  const float L1 = LOW_SHANK_LENGTH;
  const float L2 = HIGH_SHANK_LENGTH;
  const float EE = END_EFFECTOR_OFFSET;

  float r_ee = sqrt(x * x + y * y);

  if (r_ee < 0.001f) {
    DBG_PRINTLN("IK ERROR: r_ee qua nho");
    return false;
  }

  float r = r_ee - EE;

  if (r <= 0.0f) {
    DBG_PRINTLN("IK ERROR: diem qua gan truc robot");
    return false;
  }

  float d = sqrt(r * r + z * z);

  if (d > (L1 + L2)) {
    DBG_PRINTLN("IK ERROR: diem nam ngoai tam voi");
    return false;
  }

  if (d < fabs(L1 - L2)) {
    DBG_PRINTLN("IK ERROR: diem nam trong vung chet");
    return false;
  }

  out.rotRad = atan2(x, y);

  float cElbow = (L1 * L1 + L2 * L2 - d * d) / (2.0f * L1 * L2);
  cElbow = clampFloat(cElbow, -1.0f, 1.0f);

  float elbow = PI - acos(cElbow);

  float cLow = (L1 * L1 - L2 * L2 + d * d) / (2.0f * L1 * d);
  cLow = clampFloat(cLow, -1.0f, 1.0f);

  float low;

  if (z > 0) {
    low = acos(z / d) - acos(cLow);
  } else {
    low = PI - asin(r / d) - acos(cLow);
  }

  out.lowRad = low;
  out.highRad = elbow + low;

  return true;
}

/* ================= ANGLE TO STEP ================= */
long rotRadToSteps(float rad) {
  return lround((rad + ROT_OFFSET_RAD) * ROT_STEPS_PER_RAD);
}

long lowRadToSteps(float rad) {
  return lround((rad + LOW_OFFSET_RAD) * LOW_STEPS_PER_RAD);
}

long highRadToSteps(float rad) {
  return lround((rad + HIGH_OFFSET_RAD) * HIGH_STEPS_PER_RAD);
}

/* ================= SOFTWARE HOME ================= */
void softwareHome() {
  IKResult ik;

  if (!calcIK(HOME_X, HOME_Y, HOME_Z, ik)) {
    DBG_PRINTLN("HOME IK ERROR");
    return;
  }

  currentX = HOME_X;
  currentY = HOME_Y;
  currentZ = HOME_Z;

  currentXSteps = rotRadToSteps(ik.rotRad);
  currentYSteps = lowRadToSteps(ik.lowRad);
  currentZSteps = highRadToSteps(ik.highRad);

  isHomed = true;

  DBG_PRINTLN("G28 OK - SOFTWARE HOME SET");
}

/* ================= EMERGENCY CHECK DURING MOVE ================= */
bool checkEmergencyStopDuringMove() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == 'x' || c == ' ') {
      stopAll();
      DBG_PRINTLN("MOVE ABORTED BY STOP");
      return true;
    }
  }

  return false;
}

/* ================= SIMULTANEOUS STEPPER MOVE ================= */
bool moveStepsLinear(long targetXSteps, long targetYSteps, long targetZSteps, float feed_mm_s, float distance_mm) {
  long dx = targetXSteps - currentXSteps;
  long dy = targetYSteps - currentYSteps;
  long dz = targetZSteps - currentZSteps;

  long ax = labs(dx);
  long ay = labs(dy);
  long az = labs(dz);

  long maxSteps = max(ax, max(ay, az));

  if (maxSteps == 0) {
    DBG_PRINTLN("NO MOVE");
    return true;
  }

  setAxisDir(X_DIR, dx >= 0, INVERT_X_DIR);
  setAxisDir(Y_DIR, dy >= 0, INVERT_Y_DIR);
  setAxisDir(Z_DIR, dz >= 0, INVERT_Z_DIR);

  if (feed_mm_s <= 0.0f) {
    feed_mm_s = 20.0f;
  }

  if (distance_mm < 0.001f) {
    distance_mm = 1.0f;
  }

  unsigned long totalTimeUs = (unsigned long)((distance_mm / feed_mm_s) * 1000000.0f);
  unsigned long stepIntervalUs = totalTimeUs / maxSteps;

  if (stepIntervalUs < MIN_STEP_INTERVAL_US) {
    stepIntervalUs = MIN_STEP_INTERVAL_US;
  }

  long errX = 0;
  long errY = 0;
  long errZ = 0;

  DBG_PRINT("MOVE STEPS: ");
  DBG_PRINT(dx);
  DBG_PRINT(", ");
  DBG_PRINT(dy);
  DBG_PRINT(", ");
  DBG_PRINTLN(dz);

  for (long i = 0; i < maxSteps; i++) {
    if ((i % 100) == 0) {
      if (checkEmergencyStopDuringMove()) {
        return false;
      }

      if (ENCODER_PRINT_DURING_MOVE) {
        showEncodersPeriodic();
      }
    }

    errX += ax;
    errY += ay;
    errZ += az;

    if (errX >= maxSteps) {
      stepMotor(X_STEP);
      currentXSteps += (dx >= 0) ? 1 : -1;
      errX -= maxSteps;
    }

    if (errY >= maxSteps) {
      stepMotor(Y_STEP);
      currentYSteps += (dy >= 0) ? 1 : -1;
      errY -= maxSteps;
    }

    if (errZ >= maxSteps) {
      stepMotor(Z_STEP);
      currentZSteps += (dz >= 0) ? 1 : -1;
      errZ -= maxSteps;
    }

    delayUsLong(stepIntervalUs);
  }

  return true;
}

/* ================= MOVE TO XYZ USING IK ================= */
void moveToXYZ(float targetX, float targetY, float targetZ, float feed_mm_s) {
  stopAll();

  if (!isHomed) {
    DBG_PRINTLN("ERROR: Chua G28. Hay dat robot ve HOME roi gui G28 truoc.");
    return;
  }

  IKResult ik;

  DBG_PRINT("TARGET XYZ: ");
  DBG_PRINT(targetX);
  DBG_PRINT(", ");
  DBG_PRINT(targetY);
  DBG_PRINT(", ");
  DBG_PRINTLN(targetZ);

  if (!calcIK(targetX, targetY, targetZ, ik)) {
    DBG_PRINTLN("G1 FAILED: IK khong hop le");
    return;
  }

  long targetXSteps = rotRadToSteps(ik.rotRad);
  long targetYSteps = lowRadToSteps(ik.lowRad);
  long targetZSteps = highRadToSteps(ik.highRad);

  float dx = targetX - currentX;
  float dy = targetY - currentY;
  float dz = targetZ - currentZ;

  float distance = sqrt(dx * dx + dy * dy + dz * dz);

  bool ok = moveStepsLinear(targetXSteps, targetYSteps, targetZSteps, feed_mm_s, distance);

  if (ok) {
    currentX = targetX;
    currentY = targetY;
    currentZ = targetZ;

    DBG_PRINTLN("G1 OK");
  } else {
    DBG_PRINTLN("G1 STOPPED");
  }
}

/* ================= GCODE PARAM PARSER ================= */
bool getParam(const char *line, char key, float &value) {
  const char *p = strchr(line, key);

  if (p == NULL) {
    return false;
  }

  value = atof(p + 1);
  return true;
}

void toUpperCase(char *s) {
  while (*s) {
    if (*s >= 'a' && *s <= 'z') {
      *s = *s - 'a' + 'A';
    }

    s++;
  }
}

/* ================= SERVO FUNCTIONS ================= */
void setServoAngle(int angle) {
  angle = constrain(angle, 0, 180);

  servoAngle = angle;
  gripperServo.write(servoAngle);

  DBG_PRINT("SERVO ANGLE = ");
  DBG_PRINTLN(servoAngle);
}

void servoOpen() {
  setServoAngle(SERVO_OPEN_ANGLE);
  DBG_PRINTLN("SERVO OPEN");
}

void servoClose() {
  setServoAngle(SERVO_CLOSE_ANGLE);
  DBG_PRINTLN("SERVO CLOSE");
}

void printRobotStatus() {
  Serial.print("STATUS XYZ = ");
  Serial.print(currentX);
  Serial.print(", ");
  Serial.print(currentY);
  Serial.print(", ");
  Serial.println(currentZ);

  Serial.print("STATUS STEPS = ");
  Serial.print(currentXSteps);
  Serial.print(", ");
  Serial.print(currentYSteps);
  Serial.print(", ");
  Serial.println(currentZSteps);

  printEncodersNow();
}

/* ================= PROCESS GCODE LINE ================= */
void processLine(char *line) {
  toUpperCase(line);

  while (*line == ' ') {
    line++;
  }

  if (strlen(line) == 0) {
    return;
  }

  DBG_PRINT("CMD: ");
  DBG_PRINTLN(line);

  if (strncmp(line, "M17", 3) == 0) {
    digitalWrite(ENABLE_PIN, LOW);
    DBG_PRINTLN("M17 OK - DRIVER ENABLED");
    return;
  }

  if (strncmp(line, "M18", 3) == 0 || strncmp(line, "M84", 3) == 0) {
    digitalWrite(ENABLE_PIN, HIGH);
    stopAll();
    DBG_PRINTLN("DRIVER DISABLED");
    return;
  }

  if (strncmp(line, "M114", 4) == 0) {
    printRobotStatus();
    return;
  }

  if (strncmp(line, "M280", 4) == 0) {
    float sVal;

    if (getParam(line, 'S', sVal)) {
      setServoAngle((int)sVal);
      DBG_PRINTLN("M280 OK");
    } else {
      DBG_PRINTLN("M280 ERROR: thieu S angle");
    }

    return;
  }

  if (strncmp(line, "M3", 2) == 0) {
    servoClose();
    return;
  }

  if (strncmp(line, "M5", 2) == 0) {
    servoOpen();
    return;
  }

  if (strncmp(line, "G28", 3) == 0) {
    stopAll();
    softwareHome();
    return;
  }

  if (strncmp(line, "G90", 3) == 0) {
    absoluteMode = true;
    DBG_PRINTLN("G90 OK - ABSOLUTE MODE");
    return;
  }

  if (strncmp(line, "G91", 3) == 0) {
    absoluteMode = false;
    DBG_PRINTLN("G91 OK - RELATIVE MODE");
    return;
  }

  if (strncmp(line, "G0", 2) == 0 || strncmp(line, "G1", 2) == 0) {
    float xVal, yVal, zVal, fVal;

    bool hasX = getParam(line, 'X', xVal);
    bool hasY = getParam(line, 'Y', yVal);
    bool hasZ = getParam(line, 'Z', zVal);
    bool hasF = getParam(line, 'F', fVal);

    float targetX = currentX;
    float targetY = currentY;
    float targetZ = currentZ;

    if (absoluteMode) {
      if (hasX) targetX = xVal;
      if (hasY) targetY = yVal;
      if (hasZ) targetZ = zVal;
    } else {
      if (hasX) targetX = currentX + xVal;
      if (hasY) targetY = currentY + yVal;
      if (hasZ) targetZ = currentZ + zVal;
    }

    float feed = hasF ? fVal : 20.0f;

    moveToXYZ(targetX, targetY, targetZ, feed);
    return;
  }

  DBG_PRINTLN("UNKNOWN COMMAND");
}

/* ================= READ SERIAL ================= */
void readSerialCommand() {
  // Bỏ qua tín hiệu rác trong vài giây đầu sau khi cấp điện
  if (millis() - bootTimeMs < SERIAL_IGNORE_AFTER_BOOT_MS) {
    while (Serial.available() > 0) {
      Serial.read();
    }

    return;
  }

  while (Serial.available() > 0) {
    char c = Serial.read();

    if (lineIndex == 0) {
      if (handleSingleKey(c)) {
        return;
      }
    }

    if (c == '\n' || c == '\r') {
      if (lineIndex > 0) {
        lineBuffer[lineIndex] = '\0';
        processLine(lineBuffer);
        lineIndex = 0;
      }
    } else {
      if (lineIndex < sizeof(lineBuffer) - 1) {
        lineBuffer[lineIndex++] = c;
      }
    }
  }
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(SERIAL_BAUD);
  bootTimeMs = millis();

  Wire.begin();
  Wire.setClock(100000);

  pinMode(X_STEP, OUTPUT);
  pinMode(Y_STEP, OUTPUT);
  pinMode(Z_STEP, OUTPUT);

  pinMode(X_DIR, OUTPUT);
  pinMode(Y_DIR, OUTPUT);
  pinMode(Z_DIR, OUTPUT);

  pinMode(ENABLE_PIN, OUTPUT);

  // An toàn khi mới cấp điện: tắt driver trước
  // EN HIGH = tắt driver
  digitalWrite(ENABLE_PIN, HIGH);

  digitalWrite(X_STEP, LOW);
  digitalWrite(Y_STEP, LOW);
  digitalWrite(Z_STEP, LOW);

  digitalWrite(X_DIR, LOW);
  digitalWrite(Y_DIR, LOW);
  digitalWrite(Z_DIR, LOW);

  xRun = false;
  yRun = false;
  zRun = false;

  gripperServo.attach(SERVO_PIN);
  gripperServo.write(servoAngle);

  delay(1500);

  while (Serial.available() > 0) {
    Serial.read();
  }

  // Bật driver sau khi ổn định
  // EN LOW = bật driver
  digitalWrite(ENABLE_PIN, LOW);

  Serial.println("=== UNO ROBOT IK + TCA9548A + 2 AS5600 READY ===");
  Serial.print("LOW_GEAR_RATIO = ");
  Serial.println(LOW_GEAR_RATIO, 4);

  Serial.println("Commands:");
  Serial.println("G28");
  Serial.println("G90");
  Serial.println("G1 X0 Y174 Z120 F20");
  Serial.println("G1 X40 Y160 Z100 F20");
  Serial.println("G1 X-40 Y160 Z100 F20");
  Serial.println("M114 -> show XYZ + AS5600");
  Serial.println("Jog: w/s/q/e/a/d/x");
  Serial.println("Unity feedback: A,baseDeg,lowDeg,highDeg");

  scanTCAChannels();

  lastEncoderPrintMs = millis();
  lastUnityFeedbackMs = millis();
}

/* ================= LOOP ================= */
void loop() {
  readSerialCommand();

  showEncodersPeriodic();
  showUnityFeedbackPeriodic();

  unsigned long now = micros();

  if (xRun && now - lastXStep >= STEP_INTERVAL_US) {
    lastXStep = now;
    stepMotor(X_STEP);
    currentXSteps += xJogSign;
  }

  if (yRun && now - lastYStep >= STEP_INTERVAL_US) {
    lastYStep = now;
    stepMotor(Y_STEP);
    currentYSteps += yJogSign;
  }

  if (zRun && now - lastZStep >= STEP_INTERVAL_US) {
    lastZStep = now;
    stepMotor(Z_STEP);
    currentZSteps += zJogSign;
  }
}