#include <Arduino.h>
#include <SCServo.h>

SCSCL sts; // ライブラリインスタンス

const byte servoID = 1;
const int center = 2048;
const int maxAngle = 1023; // おおよそ 300°に相当（約0.29°/unit）
const float Kp = 0.8;       // 簡易比例ゲイン
int initialPos = 0;         // 電源投入時の初期角度

// 目標角度（例：任意に制御可能）
int targetPos = 2500;

void setup() {
  Serial.begin(9600);
  delay(100);
  Serial.println("Start: Feedback Control");

  Serial1.begin(1000000);
  sts.pSerial = &Serial1;

  delay(500); // 安定化待ち

  // 初期角度の取得
  int readPos = sts.ReadPos(servoID);
  if (readPos != -1) {
    initialPos = readPos;
    Serial.print("Initial Position: ");
    Serial.println(initialPos);
  } else {
    Serial.println("Failed to read initial position!");
  }
}

void loop() {
  // 1. 現在位置を取得
  int currentPos = sts.ReadPos(servoID);
  if (currentPos == -1) {
    Serial.println("Error reading position");
    delay(100);
    return;
  }

  // 2. 角度差から操作量を決定（比例制御）
  int error = targetPos - currentPos;
  int controlPos = currentPos + int(Kp * error);

  // 安全制限（物理可動域を超えないように）
  controlPos = constrain(controlPos, center - maxAngle, center + maxAngle);

  // 3. 指令を送信
  sts.WritePos(servoID, controlPos, 0, 0);

  // 4. シリアルに出力
  Serial.print("Current: ");
  Serial.print(currentPos);
  Serial.print(" / Target: ");
  Serial.print(targetPos);
  Serial.print(" / Out: ");
  Serial.println(controlPos);

  delay(20); // 制御周期20ms
}
