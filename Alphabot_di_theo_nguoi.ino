#include <Arduino.h>

// ====== Cảm biến IR ======
const int LEFT_SENSOR  = 7;
const int RIGHT_SENSOR = 8;

// ====== Cảm biến siêu âm (HC-SR04) ======
const int TRIG_PIN = 9;
const int ECHO_PIN = 10;

// ====== Động cơ L298N ======
const int IN1 = A0;
const int IN2 = A1;
const int ENA = 5;
const int IN3 = A2;
const int IN4 = A3;
const int ENB = 6;

// ====== Tốc độ ======
const int NORMAL_SPEED = 130;

// ====== Ngưỡng khoảng cách ======
const int TOO_CLOSE = 7;   // <10 cm → dừng
const int TOO_FAR   = 50;   // >30 cm → dừng
const int FOLLOW_MIN = 8;  // 12–25 cm → tiến theo
const int FOLLOW_MAX = 40;

// ====== Hàm điều khiển động cơ ======
void setMotors(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // === Bánh trái ===
  if (leftSpeed > 0) {  // tiến
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else if (leftSpeed < 0) {  // lùi
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  // === Bánh phải (đảo chiều logic để khắc phục lỗi ngược) ===
  if (rightSpeed > 0) {  // tiến
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (rightSpeed < 0) {  // lùi
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  analogWrite(ENA, abs(leftSpeed));
  analogWrite(ENB, abs(rightSpeed));
}

// ====== Đo khoảng cách ======
long getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 999; // không phản hồi
  return duration * 0.034 / 2; // cm
}

// ====== Setup ======
void setup() {
  Serial.begin(115200);
  pinMode(LEFT_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  setMotors(0, 0);
  Serial.println("🚗 Xe theo người (fix logic motor phải)");
}

// ====== Loop ======
void loop() {
  bool left  = (digitalRead(LEFT_SENSOR) == LOW);
  bool right = (digitalRead(RIGHT_SENSOR) == LOW);
  long dist = getDistance();

  Serial.print("Khoảng cách: ");
  Serial.print(dist);
  Serial.print(" cm | Trái:");
  Serial.print(left);
  Serial.print(" Phải:");
  Serial.println(right);

  // ---- Điều khiển theo khoảng cách ----
  if (dist > TOO_FAR) {
    Serial.println("📏 Người quá xa → dừng");
    setMotors(0, 0);
  }
  else if (dist < TOO_CLOSE) {
    Serial.println("⛔ Người quá gần → dừng");
    setMotors(0, 0);
  }
  else if (dist >= FOLLOW_MIN && dist <= FOLLOW_MAX) {
    // Trong khoảng phù hợp: tiến hoặc rẽ
    if (left && !right) {
      Serial.println("↩️ Người lệch trái → rẽ trái nhẹ");
      setMotors(NORMAL_SPEED / 2, NORMAL_SPEED);
    }
    else if (right && !left) {
      Serial.println("↪️ Người lệch phải → rẽ phải nhẹ");
      setMotors(NORMAL_SPEED, NORMAL_SPEED / 2);
    }
    else {
      Serial.println("🚶 Người ở trước → tiến thẳng");
      setMotors(NORMAL_SPEED, NORMAL_SPEED);
    }
  }
  else {
    Serial.println("🤔 Không rõ tín hiệu → dừng");
    setMotors(0, 0);
  }

  delay(150);
}
