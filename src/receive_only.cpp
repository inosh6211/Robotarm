/*アームが延長上方向に一直線上になっている状態でのID=2~4のサーボの角度を180°とする
また、土台のサーボについてはペイロードを上から見て短辺にアームが平行になっている状態を180°とする
*/

#include <Arduino.h>

#define POSITION_REGISTER 56 //レジスタの先頭番地
#define MAX_DATA_LENGTH 64
byte buffer[MAX_DATA_LENGTH];

//Hardware information
const float Hbase = 80.0f;
const float L1 = 125.0f;
const float L2 = 125.0f;
const float L3 = 100.0f;

byte calcChecksum(byte *data, int length){
  int sum = 0;
  for(int i = 2; i < length - 1; i++){
    sum += data[i];
  }
  return ~((byte)(sum & 0xFF));
}

void sendReadPosPacket(byte id){
  byte packet[8];
  packet[0] = 0xFF;
  packet[1] = 0xFF;
  packet[2] = id;
  packet[3] = 4;
  packet[4] = 0x02;
  packet[5] = POSITION_REGISTER;
  packet[6] = 2; //読み込みbyte数
  packet[7] = calcChecksum(packet, 8);

  Serial1.write(packet, 8);
  Serial1.flush();
}

int receivePosition(uint16_t *position) {
  int len = 8; // 応答パケット長
  unsigned long start = millis();
  int count = 0;

  while (count < len && millis() - start < 100) {
    if (Serial1.available()) {
      buffer[count++] = Serial1.read();
    }
  }
  if (count < len) return -1;
  *position = buffer[5] | (buffer[6] << 8);
  return 0;
}

// --- STS角度値 → degree変換（0~4095 → 0~360°） ---
float posToDeg(uint16_t pos) {
  return (pos / 4095.0f) * 360.0f;
}

// --- FK計算（θ2〜θ4で x,z を求める）---
void computeFK(float th1, float th2, float th3, float th4, float *x_out, float *z_out, float *x1_out, float *z1_out, float *x12_out, float *z12_out) {
  // θ2〜θ4: 180°が直立。そこからの相対角度に変換
  float t2 = (th2 - 180.0f) * DEG_TO_RAD;
  float t3 = (th3 - 180.0f) * DEG_TO_RAD;
  float t4 = (th4 - 180.0f) * DEG_TO_RAD;

  float t23 = t2 + t3;
  float t234 = t23 + t4;

  float z1 = Hbase + L1*cos(t2);
  float z12 = Hbase + L1*cos(t2) + L2*cos(t23);
  float z = Hbase + L1 * cos(t2) + L2 * cos(t23) + L3 * cos(t234);
  float x1 = L1*sin(t2);
  float x12 = L1*sin(t2) + L2*sin(t23);
  float x = L1 * sin(t2) + L2 * sin(t23) + L3 * sin(t234);

  // θ1で回転（ID=1）
  float t1 = th1 * DEG_TO_RAD;
  //float x = x_proj * cos(t1); // 本来は3D化できるが、Yは使わずXに投影

  *x_out = x;
  *z_out = z;
  *x1_out = x1;
  *z1_out = z1;
  *x12_out = x12;
  *z12_out = z12;
}

// --- サーボIDから角度を取得 ---
bool getAngle(byte id, float *angle_deg) {
  sendReadPosPacket(id);
  uint16_t raw;
  if (receivePosition(&raw) == 0) {
    *angle_deg = posToDeg(raw);
    return true;
  }
  return false;
}

void setup() {
  Serial.begin(115200); //serialprint用
  Serial1.begin(1000000);  // STSサーボ通信
  delay(500);
  Serial.println("角度読取→FK計算 開始");
}

void loop() {
  float th1, th2, th3, th4;
  bool ok1 = getAngle(1, &th1);
  bool ok2 = getAngle(2, &th2);
  bool ok3 = getAngle(3, &th3);
  bool ok4 = getAngle(4, &th4);

  if (ok1 && ok2 && ok3 && ok4) {
    float x1, x12, x, z1, z12, z;
    computeFK(th1, th2, th3, th4  , &x, &z, &x1, &z1, &x12, &z12);

    Serial.print("θ1: "); Serial.print(th1); Serial.print("°, ");
    Serial.print("θ2: "); Serial.print(th2); Serial.print("°, ");
    Serial.print("θ3: "); Serial.print(th3); Serial.print("°, ");
    Serial.print("θ4: "); Serial.print(th4); Serial.println("°");

    // Serial.print("第1関節まで:x1 = ");
    // Serial.print(x1);
    // Serial.print(" mm, z1 = ");
    // Serial.print(z1);
    // Serial.println(" mm");

    // Serial.print("第2関節まで:x12 = ");
    // Serial.print(x12);
    // Serial.print(" mm, z12 = ");
    // Serial.print(z12);
    // Serial.println(" mm");

    Serial.print("→ EndEffector座標: x = ");
    Serial.print(x);
    Serial.print(" mm, z = ");
    Serial.print(z);
    Serial.println(" mm");

  } else {
    Serial.println("角度読取失敗");
  }

  delay(1000);
}

/*
// バッファサイズ
#define MAX_DATA_LENGTH 64
byte buffer[MAX_DATA_LENGTH];

// 読み取りレジスタ（現在位置）
#define POSITION_REGISTER 56

// チェックサム計算
byte calcChecksum(byte *data, int length) {
  int sum = 0;
  for (int i = 2; i < length - 1; i++) {
    sum += data[i];
  }
  return ~((byte)(sum & 0xFF));
}

// 読み取りパケット送信
void sendReadPosPacket(byte id) {
  byte packet[8];
  packet[0] = 0xFF;
  packet[1] = 0xFF;
  packet[2] = id;
  packet[3] = 4;
  packet[4] = 0x02;  // READ命令
  packet[5] = POSITION_REGISTER;
  packet[6] = 2;
  packet[7] = calcChecksum(packet, 8);

  Serial1.write(packet, 8);
  Serial1.flush();
}

// 応答を受信して位置値を取得
int receivePosition(uint16_t *position) {
  int len = 8; // 応答パケット長（FF FF ID LEN ERR POSL POSH CHK）

  unsigned long start = millis();
  int count = 0;
  while (count < len && millis() - start < 100) {
    if (Serial1.available()) {
      buffer[count++] = Serial1.read();
    }
  }

  if (count < len) return -1; // タイムアウト

  *position = buffer[5] | (buffer[6] << 8);
  return 0;
}

// 位置値（0〜4095）を角度（0〜360°）へ変換
float posToDegrees(uint16_t pos) {
  return (pos / 4095.0f) * 360.0f;
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(1000000);  // シリアル1：STSサーボ通信

  delay(500);
  Serial.println("2個のサーボ角度読み取り開始");
}

void loop() {
  byte id1 = 1;
  byte id2 = 2;

  // --- サーボ1 ---
  sendReadPosPacket(id1);
  delay(5);  // 応答待ち（最小限）

  uint16_t pos1;
  if (receivePosition(&pos1) == 0) {
    float deg1 = posToDegrees(pos1);
    Serial.print("サーボID ");
    Serial.print(id1);
    Serial.print(": ");
    Serial.print(deg1);
    Serial.println("°");
  } else {
    Serial.println("サーボ1の読み取り失敗");
  }

  // --- サーボ2 ---
  sendReadPosPacket(id2);
  delay(5);  // 応答待ち

  uint16_t pos2;
  if (receivePosition(&pos2) == 0) {
    float deg2 = posToDegrees(pos2);
    Serial.print("サーボID ");
    Serial.print(id2);
    Serial.print(": ");
    Serial.print(deg2);
    Serial.println("°");
  } else {
    Serial.println("サーボ2の読み取り失敗");
  }

  delay(1000); // 1秒ごとに更新
}
*/
