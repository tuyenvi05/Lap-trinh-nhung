#include <Arduino.h>

// ====== Chân cảm biến IR ======
const int LEFT_SENSOR  = 7; // D7
const int RIGHT_SENSOR = 8; // D8

// ====== Chân cảm biến siêu âm HC-SR04 ======
const int TRIG_PIN = 9;
const int ECHO_PIN = 10;

// ====== Chân điều khiển động cơ ======
const int IN1 = A0;
const int IN2 = A1;
const int ENA = 5;
const int IN3 = A2;
const int IN4 = A3;
const int ENB = 6;

// ====== Tốc độ ======
const int FORWARD_SPEED = 90;   // tốc độ tiến
const int TURN_SPEED    = 20;   // tốc độ rẽ
const int BACK_SPEED    = 120;   // tốc độ lùi

// ====== Ngưỡng khoảng cách (cm) ======
const int DIST_THRESHOLD = 20;

// ====== Biến trạng thái ======
bool obstacleL = false;
bool obstacleR = false;
bool obstacleFront = false;

// ====== Hàm điều khiển động cơ ======
void setMotors(int leftSpeed, int rightSpeed) {
  analogWrite(ENA, abs(leftSpeed));
  analogWrite(ENB, abs(rightSpeed));

  // bánh trái
  if(leftSpeed >= 0){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }

  // bánh phải (đã đảo)
  if(rightSpeed >= 0){
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
}

// ====== Setup ======
void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  pinMode(LEFT_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.println("AlphaBot - IR + Ultrasonic Obstacle Avoidance");
}

// ====== Đọc cảm biến IR ======
bool readLeftSensor() {
  return digitalRead(LEFT_SENSOR) == LOW;
}

bool readRightSensor() {
  return digitalRead(RIGHT_SENSOR) == LOW;
}

// ====== Đọc cảm biến siêu âm ======
long readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance = duration * 0.034 / 2; // cm
  return distance;
}

// ====== Hàm tránh vật cản mượt ======
void avoidObstacle(bool left, bool right, bool front) {
  if(front) {
    Serial.println("↑ Vật cản trước mặt: lùi thẳng rồi rẽ ngẫu nhiên");
    setMotors(-BACK_SPEED, -BACK_SPEED);
    delay(400);
    if(random(2) == 0) setMotors(FORWARD_SPEED, FORWARD_SPEED / 2);
    else setMotors(FORWARD_SPEED / 2, FORWARD_SPEED);
    delay(400);

  } else if(left && !right) {
    Serial.println("← Vật cản bên trái: lùi + rẽ phải cả hai bánh");
    // Cả hai động cơ lùi nhưng bên trái yếu hơn để quay sang phải
    setMotors(-BACK_SPEED / 2, -BACK_SPEED);
    delay(300);
    // Cả hai động cơ tiến nhưng bên phải yếu hơn để quay sang phải
    setMotors(FORWARD_SPEED, FORWARD_SPEED / 2);
    delay(400);

  } else if(right && !left) {
    Serial.println("→ Vật cản bên phải: lùi + rẽ trái cả hai bánh");
    // Cả hai động cơ lùi nhưng bên phải yếu hơn để quay sang trái
    setMotors(-BACK_SPEED, -BACK_SPEED / 2);
    delay(300);
    // Cả hai động cơ tiến nhưng bên trái yếu hơn để quay sang trái
    setMotors(FORWARD_SPEED / 2, FORWARD_SPEED);
    delay(400);

  } else {
    Serial.println("Không vật cản: tiến thẳng");
    setMotors(FORWARD_SPEED, FORWARD_SPEED);
  }
}

// ====== Main loop ======
void loop() {
  obstacleL = readLeftSensor();
  obstacleR = readRightSensor();
  obstacleFront = readUltrasonic() < DIST_THRESHOLD;

  avoidObstacle(obstacleL, obstacleR, obstacleFront);
  delay(50);
}
