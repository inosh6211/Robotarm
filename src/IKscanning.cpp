#include <Arduino.h>
#include <math.h>

// === パラメータ ===
const float Hbase = 80.0f;
const float L1 = 125.0f;
const float L2 = 125.0f;
const float L3 = 100.0f;

#define POSITION_REGISTER 56
#define MAX_DATA_LENGTH 64
byte buffer[MAX_DATA_LENGTH];

// === チェックサム計算 ===
byte calcChecksum(byte *data, int length){
  int sum = 0;
  for(int i = 2; i < length - 1; i++){
    sum += data[i];
  }
  return ~((byte)(sum & 0xFF));
}

// === 正運動学 ===
void computeFK(float th2, float th3, float th4, float *x_out, float *z_out) {
  float t2 = (th2 - 180.0f) * DEG_TO_RAD;
  float t3 = (th3 - 180.0f) * DEG_TO_RAD;
  float t4 = (th4 - 180.0f) * DEG_TO_RAD;

  float t23 = t2 + t3;
  float t234 = t23 + t4;
  
  *x_out = L1 * sin(t2) + L2 * sin(t23) + L3 * sin(t234);
  *z_out = Hbase + L1 * cos(t2) + L2 * cos(t23) + L3 * cos(t234);
}

// === サーボへ目標角送信 ===
void sendPositionCommand(uint8_t id, uint16_t position, uint16_t time_ms) {
  uint8_t packet[11];
  packet[0] = 0xFF;
  packet[1] = 0xFF;
  packet[2] = id;
  packet[3] = 7;
  packet[4] = 0x03;
  packet[5] = 0x2A;
  packet[6] = position & 0xFF;
  packet[7] = (position >> 8) & 0xFF;
  packet[8] = time_ms & 0xFF;
  packet[9] = (time_ms >> 8) & 0xFF;
  packet[10] = calcChecksum(packet, 11);
  Serial1.write(packet, 11);
  Serial1.flush();
}

// === deg → pos(0~4095) ===
uint16_t degToPos(float deg) {
  return (uint16_t)(deg / 360.0f * 4095.0f);
}

// === 逆運動学スキャン ===
void solveIK(float x_target, float z_target) {
  float best_err = 1e2;
  float best_t2 = 180, best_t3 = 180, best_t4 = 180;

  for (int t2 = 100; t2 <= 260; t2 +=3) {
    for (int t3 = 100; t3 <= 260; t3 += 3) {
      for (int t4 = 100; t4 <= 260; t4 += 3) {
        float x, z;
        computeFK(t2, t3, t4, &x, &z);
        float err = sqrt(pow(x - x_target, 2) + pow(z - z_target, 2));
        if (err < best_err) {
          best_err = err;
          best_t2 = t2;
          best_t3 = t3;
          best_t4 = t4;
        }
      }
    }
  }

  Serial.print("Best θ2: "); Serial.print(best_t2);
  Serial.print(", θ3: "); Serial.print(best_t3);
  Serial.print(", θ4: "); Serial.print(best_t4);
  Serial.print(", err: "); Serial.println(best_err);

  // シリアルサーボへ送信（角度→位置変換が必要）
  sendPositionCommand(2, degToPos(best_t2), 500);
  sendPositionCommand(3, degToPos(best_t3), 500);
  sendPositionCommand(4, degToPos(best_t4), 500);
}

// === setup ===
void setup() {
  Serial.begin(115200);
  Serial1.begin(1000000); // STS用の通信速度

  delay(500);
  Serial.println("=== IK Solver Start ===");

  float x_target = 150.0;
  float z_target = 250.0;

  solveIK(x_target, z_target);
}

// === loop ===
void loop() {
  // 一度だけ動かす
}
