#include <avr/interrupt.h>

#define RDIR 11
#define LDIR 9
#define LPWM 5
#define RPWM 6

#define BUTTON1 7
#define BUTTON2 8
#define BUTTON3 12

#define BUZZER 4
#define RED 3
#define GREEN 2
#define BLUE 13

#define LEFT_TRIM   5
#define RIGHT_TRIM -5

// ====== LINE SENSOR ======
#define NO_LINE    0b00000000
#define FULL_BLACK 0b11111111

// ====== CHỈNH THEO XE ======
#define START_IGNORE_COUNT 100
#define TURN_180_COUNT 290

// Lắc tìm line
// Nếu lắc chưa đủ rộng thì tăng SEARCH_45_COUNT
// Nếu lắc quá rộng thì giảm SEARCH_45_COUNT
#define SEARCH_45_COUNT 85
#define SEARCH_TURN_SPEED 35

// Tốc độ sau khi đã tìm thấy line
#define RUN_SPEED_AFTER_FOUND 50

volatile int lastPos = 0;
volatile unsigned char isCalib = 1;
volatile int servoPwm = 0;
volatile unsigned char sensor = 0;
volatile int cnt = 0;

unsigned int sensorValue[8];
unsigned int sensorPID[8];
unsigned int black_value[8];
unsigned int white_value[8];
unsigned int compare_value[8];

int speed_run_forward = 35;

unsigned char pattern = 0;
unsigned char start = 0;

// 0 = quay trái 45
// 1 = quay phải 90
// 2 = quay trái 90
unsigned char searchPhase = 0;

int kp = 1;
int kd = 3;

void setup() {
  pinMode(LDIR, OUTPUT);
  pinMode(RDIR, OUTPUT);

  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUTTON3, INPUT_PULLUP);

  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  digitalWrite(RDIR, LOW);
  digitalWrite(LDIR, LOW);

  speed_run(0, 0);
  initCalibValue();

  Serial.begin(9600);
  timer_init();

  Serial.println("=== BOOT AGAIN ===");
  Serial.println("Nhan BUTTON1 hoac gui so 1 de chay.");
  beep(100);
}

void initCalibValue() {
  for (int i = 0; i < 8; i++) {
    black_value[i] = 1023;
    white_value[i] = 0;
    compare_value[i] = 500;
    sensorPID[i] = 0;
  }
}

void timer_init() {
  TCCR2A = 0;
  TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);
  TCNT2 = 0xB2;
  TIMSK2 = (1 << TOIE2);
}

ISR(TIMER2_OVF_vect) {
  TCNT2 = 0xB2;
  read_sensor();
  cnt++;
}

void loop() {
  // ================== CHẾ ĐỘ CHỜ ==================
  if (start == 0) {
    speed_run(0, 0);

    if (isCalib == 1) {
      updateLine();
    }

    if (checkStartCommand()) {
      startRun(RUN_SPEED_AFTER_FOUND);
    }

    return;
  }

  // ================== CHẾ ĐỘ ĐANG CHẠY ==================
  switch (pattern) {
    // ================== TÌM LINE ==================
    case 5:
      // Nếu đã thấy line thì bắt đầu bám line
      if (sensor != NO_LINE) {
        speed_run(0, 0);
        delay(80);

        cnt = 0;
        pattern = 10;

        Serial.print("FOUND LINE: ");
        printSensorBinary(sensor);
        break;
      }

      searchLine();
      break;

    // ================== VỪA XUẤT PHÁT ==================
    case 10:
      runforwardline(speed_run_forward);

      if (cnt >= START_IGNORE_COUNT) {
        cnt = 0;
        pattern = 11;
      }
      break;

    // ================== BÁM LINE CHÍNH ==================
    case 11:
      // Gặp full đen thì quay 180
      if (sensor == FULL_BLACK) {
        speed_run(0, 0);
        delay(80);

        cnt = 0;
        pattern = 20;

        Serial.println("FULL BLACK -> TURN 180");
        break;
      }

      // Nếu mất line thì chuyển sang lắc tìm line
      if (sensor == NO_LINE) {
        speed_run(0, 0);
        delay(50);

        cnt = 0;
        searchPhase = 0;
        pattern = 5;

        Serial.println("NO LINE -> SEARCH LINE");
        break;
      }

      runforwardline(speed_run_forward);
      break;

    // ================== QUAY 180 ==================
    case 20:
      RGB(1);
      speed_run(35, -35);

      if (cnt >= TURN_180_COUNT) {
        finishCycle();
      }
      break;

    default:
      finishCycle();
      break;
  }
}

bool checkStartCommand() {
  bool request = false;

  // Nhận số 1 từ Serial Monitor hoặc ESP32-C3 gửi sang
  if (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '1') {
      request = true;
    }
  }

  // Nhấn BUTTON1
  if (digitalRead(BUTTON1) == LOW) {
    speed_run(0, 0);
    delay(30);

    if (digitalRead(BUTTON1) == LOW) {
      while (digitalRead(BUTTON1) == LOW) {
        speed_run(0, 0);

        if (isCalib == 1) {
          updateLine();
        }
      }

      delay(30);
      request = true;
    }
  }

  return request;
}

void startRun(int spd) {
  start = 1;
  isCalib = 0;
  cnt = 0;
  speed_run_forward = spd;
  searchPhase = 0;

  // Nếu lúc bắt đầu chưa thấy line thì lắc tìm line trước
  if (sensor == NO_LINE) {
    pattern = 5;
    Serial.println("START -> NO LINE -> SEARCH");
  } else {
    pattern = 10;
    Serial.println("START -> LINE FOUND -> RUN");
  }

  beep(100);
}

void finishCycle() {
  speed_run(0, 0);
  RGB(3);

  start = 0;
  pattern = 0;
  cnt = 0;
  searchPhase = 0;

  Serial.println("DONE -> STOP AND WAIT BUTTON1 / SERIAL 1");
  beep(80);
}

// ================== LẮC TÌM LINE ==================
void searchLine() {
  RGB(2);

  // Giai đoạn 0: quay trái khoảng 45 độ
  if (searchPhase == 0) {
    turnLeftSearch();

    if (cnt >= SEARCH_45_COUNT) {
      cnt = 0;
      searchPhase = 1;
    }

    return;
  }

  // Giai đoạn 1: quay phải khoảng 90 độ
  // Tức là từ trái 45 sang phải 45
  if (searchPhase == 1) {
    turnRightSearch();

    if (cnt >= SEARCH_45_COUNT * 2) {
      cnt = 0;
      searchPhase = 2;
    }

    return;
  }

  // Giai đoạn 2: quay trái khoảng 90 độ
  // Tức là từ phải 45 sang trái 45
  if (searchPhase == 2) {
    turnLeftSearch();

    if (cnt >= SEARCH_45_COUNT * 2) {
      cnt = 0;
      searchPhase = 1;
    }

    return;
  }
}

void turnLeftSearch() {
  speed_run(-SEARCH_TURN_SPEED, SEARCH_TURN_SPEED);
}

void turnRightSearch() {
  speed_run(SEARCH_TURN_SPEED, -SEARCH_TURN_SPEED);
}

// Cảm biến giữa: cho phép 1 hoặc 2 mắt giữa đều được coi là bám line
bool isCenterLine() {
  return sensor == 0b00011000 ||
         sensor == 0b00010000 ||
         sensor == 0b00001000 ||
         sensor == 0b00111000 ||
         sensor == 0b00011100 ||
         sensor == 0b00111100 ||
         sensor == 0b00110000 ||
         sensor == 0b00001100;
}

// Nếu có dính một trong hai mắt giữa thì vẫn coi là đang ở gần giữa line
bool touchCenterSensor() {
  return (sensor & 0b00011000) != 0;
}

void runforwardline(int tocdo) {
  // Mất line thì không đứng yên chết ở đây nữa.
  // Pattern 11 sẽ đưa xe qua chế độ search.
  if (sensor == NO_LINE) {
    speed_run(0, 0);
    return;
  }

  // Full đen để pattern chính xử lý quay 180
  if (sensor == FULL_BLACK) {
    delay(1000);
    speed_run(tocdo, tocdo);
    return;
  }

  // Nếu dính cảm biến giữa 4-5 thì bám line bình thường
  if (isCenterLine() || touchCenterSensor()) {
    kd = 3;
    RGB(0);
    handleAndSpeed(servoPwm, tocdo);
    return;
  }

  // Line lệch về một phía
  if (
    sensor == 0b00001100 ||
    sensor == 0b00000100 ||
    sensor == 0b00001110 ||
    sensor == 0b00011110 ||
    sensor == 0b00000110 ||
    sensor == 0b00000010 ||
    sensor == 0b00000111 ||
    sensor == 0b00000011 ||
    sensor == 0b00000001 ||
    sensor == 0b00001111 ||
    sensor == 0b00011111 ||
    sensor == 0b00111111
  ) {
    kd = 3;
    RGB(1);
    handleAndSpeed(servoPwm, tocdo);
    return;
  }

  // Line lệch về phía còn lại
  if (
    sensor == 0b00110000 ||
    sensor == 0b00100000 ||
    sensor == 0b01110000 ||
    sensor == 0b01111000 ||
    sensor == 0b01100000 ||
    sensor == 0b01000000 ||
    sensor == 0b11100000 ||
    sensor == 0b11000000 ||
    sensor == 0b10000000 ||
    sensor == 0b11110000 ||
    sensor == 0b11111000 ||
    sensor == 0b11111100
  ) {
    kd = 3;
    RGB(2);
    handleAndSpeed(servoPwm, tocdo);
    return;
  }

  // Các mẫu lạ nhưng vẫn có line thì vẫn cho PID xử lý, không dừng xe
  RGB(0);
  handleAndSpeed(servoPwm, tocdo);
}

void updateLine() {
  for (int i = 0; i < 8; i++) {
    if (black_value[i] == 1023) black_value[i] = sensorValue[i];

    if (sensorValue[i] < black_value[i]) black_value[i] = sensorValue[i];
    if (sensorValue[i] > white_value[i]) white_value[i] = sensorValue[i];

    compare_value[i] = (black_value[i] + white_value[i]) / 2;
  }

  Serial.print("SENSOR: ");
  printSensorBinary(sensor);
}

void read_sensor() {
  unsigned char temp = 0;
  unsigned int sum = 0;
  unsigned long avg = 0;

  sensorValue[0] = 1023 - analogRead(A0);
  sensorValue[1] = 1023 - analogRead(A1);
  sensorValue[2] = 1023 - analogRead(A2);
  sensorValue[3] = 1023 - analogRead(A3);
  sensorValue[4] = 1023 - analogRead(A4);
  sensorValue[5] = 1023 - analogRead(A5);
  sensorValue[6] = 1023 - analogRead(A6);
  sensorValue[7] = 1023 - analogRead(A7);

  for (int j = 0; j < 8; j++) {
    temp = temp << 1;

    if (sensorValue[j] > compare_value[j]) {
      temp |= 0x01;
    }

    if (isCalib == 0) {
      int low = black_value[j];
      int high = white_value[j];

      if (high <= low) {
        sensorPID[j] = 0;
      } else {
        sensorValue[j] = constrain(sensorValue[j], low, high);
        sensorPID[j] = map(sensorValue[j], low, high, 0, 1000);
      }
    }
  }

  sensor = temp;

  if (isCalib == 1) return;

  for (int j = 0; j < 8; j++) {
    avg += (long)sensorPID[j] * (j * 1000);
    sum += sensorPID[j];
  }

  if (sum == 0) return;

  int i = (int)((avg / sum) - 3500);

  kp = 1;
  int iP = kp * i;
  int iD = kd * (lastPos - i);
  int iRet = iP - iD;

  servoPwm = iRet / 75;
  servoPwm = constrain(servoPwm, -15, 15);

  lastPos = i;
}

void handleAndSpeed(int angle, int speed1) {
  int speedLeft = speed1 - angle;
  int speedRight = speed1 + angle;

  speedLeft = constrain(speedLeft, 10, 120);
  speedRight = constrain(speedRight, 10, 120);

  speed_run(speedLeft, speedRight);
}

void speed_run(int speedDC_left, int speedDC_right) {
  if (speedDC_left > 0) speedDC_left += LEFT_TRIM;
  if (speedDC_right > 0) speedDC_right += RIGHT_TRIM;

  speedDC_left = constrain(speedDC_left, -255, 255);
  speedDC_right = constrain(speedDC_right, -255, 255);

  if (speedDC_left < 0) {
    analogWrite(LPWM, 255 + speedDC_left);
    digitalWrite(LDIR, HIGH);
  } else {
    analogWrite(LPWM, speedDC_left);
    digitalWrite(LDIR, LOW);
  }

  if (speedDC_right < 0) {
    analogWrite(RPWM, 255 + speedDC_right);
    digitalWrite(RDIR, HIGH);
  } else {
    analogWrite(RPWM, speedDC_right);
    digitalWrite(RDIR, LOW);
  }
}

void RGB(int color) {
  switch (color) {
    case 0:
      // xanh lá: đang bám giữa line
      digitalWrite(RED, 0);
      digitalWrite(GREEN, 1);
      digitalWrite(BLUE, 1);
      break;

    case 1:
      // đỏ: lệch một bên
      digitalWrite(RED, 1);
      digitalWrite(GREEN, 0);
      digitalWrite(BLUE, 1);
      break;

    case 2:
      // xanh dương: lệch bên còn lại / đang tìm line
      digitalWrite(RED, 1);
      digitalWrite(GREEN, 1);
      digitalWrite(BLUE, 0);
      break;

    default:
      digitalWrite(RED, 1);
      digitalWrite(GREEN, 1);
      digitalWrite(BLUE, 1);
      break;
  }
}

void beep(int timer) {
  digitalWrite(BUZZER, 1);
  delay(timer);
  digitalWrite(BUZZER, 0);
}

void printSensorBinary(unsigned char value) {
  for (int i = 7; i >= 0; i--) {
    Serial.print((value >> i) & 1);
  }
  Serial.println();
}