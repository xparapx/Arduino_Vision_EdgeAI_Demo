/* ============================================================
 * Vision Gauge - 제어 노드 (Nano ESP32 + Grove OLED 1.12" V2)
 * ------------------------------------------------------------
 * BLE Central로서 비전 노드에 연결해 인식 결과를 OLED에 표시.
 *
 * 핵심 동작:
 *   1) 비전 노드("VisionGauge") 탐색 -> 연결
 *   2) 결과 특성(0x...002) 구독 -> 3바이트 수신 시 OLED 갱신
 *   3) 상단에 BLE 연결/수신 상태 인디케이터 표시
 *
 *   ※ 라벨은 아래 LABELS[]에 직접 입력 (BLE 자동 동기화 대신 수동).
 *      비전 노드의 EI 라벨 순서(알파벳순)와 정확히 일치시킬 것!
 *
 * OLED: Grove 1.12" V2 (SH1107G, 128x128, I2C)
 * 라이브러리: U8g2 by oliver, ESP32 내장 BLE
 * ============================================================ */
#include <BLEDevice.h>
#include <Wire.h>
#include <U8g2lib.h>

// ---------- Grove OLED 1.12" V2 (SH1107G, 128x128) ----------
U8G2_SH1107_SEEED_128X128_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

// ---------- BLE UUID (비전 노드와 동일해야 함) ----------
#define SERVICE_UUID  "7e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHAR_UUID     "7e400002-b5a3-f393-e0a9-e50e24dcca9e"  // 결과 (3바이트)

/* ============================================================
 * ★ 라벨 직접 입력 (가장 중요!) ★
 * ------------------------------------------------------------
 * 비전 노드 시리얼의 LBL: 줄에 나오는 순서 그대로 입력하세요.
 *   예) 시리얼에 "LBL:arduino,keyboard,mouse" 가 보이면
 *       -> 아래를 {"arduino", "keyboard", "mouse"} 로
 *
 * EI는 라벨을 알파벳순으로 정렬하므로, 보통 a→z 순서입니다.
 * 클래스(모델)를 바꾸면 이 배열도 반드시 같이 수정하세요.
 * ============================================================ */
String LABELS[] = {
  "arduino",    // 0
  "keyboard",   // 1
  "mouse"       // 2
};
const int numLabels = sizeof(LABELS) / sizeof(LABELS[0]);

// ---------- 상태 변수 ----------
volatile int  curClass  = -1;    // 현재 표시 중인 클래스 인덱스
volatile int  curConf   =  0;    // 확신도 (0~100)
volatile bool updated   = false; // 새 데이터 도착 플래그
volatile uint32_t lastRxMs = 0;  // 마지막 수신 시각 (수신 인디케이터용)
bool bleConnected = false;

BLEClient* pClient = nullptr;
BLERemoteCharacteristic* pResultChar = nullptr;

/* ---------- 결과 특성 notify 콜백 ----------
 * 비전 노드가 200ms마다 푸시하는 3바이트 [class_id, conf, seq] 수신 */
static void notifyCB(BLERemoteCharacteristic* c,
                     uint8_t* data, size_t len, bool isNotify) {
  if (len < 3) return;
  curClass = data[0];        // 클래스 인덱스 (LABELS[]에서 이름 조회)
  curConf  = data[1];        // 확신도 0~100 정수
  updated  = true;
  lastRxMs = millis();       // 수신 시각 기록 -> 인디케이터 점멸
}

/* ---------- 비전 노드 연결 + 라벨 수신 ---------- */
bool connectVisionNode() {
  BLEScan* scan = BLEDevice::getScan();
  scan->setActiveScan(true);

  // esp32 2.0.x: start()가 값을 반환 (3.x면 BLEScanResults* + res->)
  BLEScanResults res = scan->start(5);
  for (int i = 0; i < res.getCount(); i++) {
    BLEAdvertisedDevice d = res.getDevice(i);
    if (d.haveServiceUUID() && d.isAdvertisingService(BLEUUID(SERVICE_UUID))) {
      pClient = BLEDevice::createClient();
      if (!pClient->connect(&d)) return false;

      BLERemoteService* svc = pClient->getService(SERVICE_UUID);
      if (!svc) return false;

      // 라벨은 상단 LABELS[]에 직접 입력했으므로 BLE read 불필요.

      // --- 결과 특성 구독 ---
      pResultChar = svc->getCharacteristic(CHAR_UUID);
      if (!pResultChar || !pResultChar->canNotify()) return false;
      pResultChar->registerForNotify(notifyCB);
      return true;
    }
  }
  return false;
}

/* ---------- 상단 상태 바 그리기 ----------
 * 좌측: BLE 연결 아이콘 / 우측: 수신 점멸 표시 */
void drawStatusBar() {
  oled.setFont(u8g2_font_5x8_tr);

  // BLE 연결 상태 (좌상단)
  if (bleConnected) {
    oled.drawStr(2, 8, "BLE:ON");
    // 연결 아이콘 (채워진 원)
    oled.drawDisc(40, 5, 3);
  } else {
    oled.drawStr(2, 8, "BLE:--");
    oled.drawCircle(40, 5, 3);   // 빈 원
  }

  // 수신 인디케이터 (우상단) - 최근 600ms 안에 수신했으면 채움
  bool rxActive = (millis() - lastRxMs) < 600;
  oled.drawStr(86, 8, "RX");
  if (rxActive) {
    oled.drawBox(104, 1, 8, 8);          // 채워진 사각 (수신 중)
  } else {
    oled.drawFrame(104, 1, 8, 8);        // 빈 사각 (대기)
  }

  oled.drawHLine(0, 12, 128);            // 구분선
}

/* ---------- 메인 화면 그리기 ---------- */
void drawScreen() {
  oled.clearBuffer();
  drawStatusBar();

  if (!bleConnected) {
    oled.setFont(u8g2_font_ncenB10_tr);
    oled.drawStr(8, 60, "connecting");
    oled.drawStr(8, 80, "...");
  } else if (curClass < 0) {
    oled.setFont(u8g2_font_ncenB10_tr);
    oled.drawStr(8, 60, "waiting");
    oled.drawStr(8, 80, "for data");
  } else {
    // --- 인식된 라벨 ---
    // LABELS[]가 비었거나 범위 밖이면 인덱스로 fallback (디버깅용)
    String lab;
    if (curClass >= 0 && curClass < numLabels && LABELS[curClass].length() > 0) {
      lab = LABELS[curClass];
    } else {
      lab = "C" + String(curClass);   // 라벨 미수신 시 "C0","C1"... 로 표시
    }

    // 라벨 길이에 따라 폰트 자동 선택 (긴 라벨도 안 잘림)
    if (lab.length() <= 6)        oled.setFont(u8g2_font_ncenB18_tr);
    else if (lab.length() <= 9)   oled.setFont(u8g2_font_ncenB14_tr);
    else                          oled.setFont(u8g2_font_ncenB10_tr);
    oled.drawStr(6, 50, lab.c_str());

    // --- 확신도 숫자 ---
    char buf[24];
    snprintf(buf, sizeof(buf), "conf %d%%", curConf);
    oled.setFont(u8g2_font_6x12_tr);
    oled.drawStr(6, 78, buf);

    // --- 확신도 bar 게이지 ---
    int barX = 6, barY = 88, barW = 116, barH = 16;
    oled.drawFrame(barX, barY, barW, barH);             // 외곽
    int fillW = (curConf * (barW - 4)) / 100;           // 채움 길이
    oled.drawBox(barX + 2, barY + 2, fillW, barH - 4);  // 내부 채움

    // --- 눈금 (25/50/75%) ---
    for (int p = 25; p < 100; p += 25) {
      int tx = barX + 2 + (p * (barW - 4)) / 100;
      oled.drawVLine(tx, barY + barH + 2, 3);
    }
    oled.setFont(u8g2_font_4x6_tr);
    oled.drawStr(barX, barY + barH + 12, "0");
    oled.drawStr(barX + barW - 16, barY + barH + 12, "100");
  }

  oled.sendBuffer();
}

void setup() {
  Serial.begin(115200);
  delay(500);                   // 시리얼 안정화 대기
  Serial.println();
  Serial.println("=== 제어 노드 부팅 시작 ===");

  Wire.begin();
  Serial.println("1) I2C 초기화 완료");

  oled.begin();
  Serial.println("2) OLED 초기화 완료");
  drawScreen();                 // "connecting..." 표시

  BLEDevice::init("");
  Serial.println("3) BLE 초기화 완료 - 비전 노드 탐색 시작");

  int tries = 0;
  while (!connectVisionNode()) {
    tries++;
    Serial.print("비전 노드 재탐색... ("); Serial.print(tries); Serial.println("회)");
    delay(1000);
  }
  bleConnected = true;
  Serial.println("4) 비전 노드 연결 성공!");
  drawScreen();                 // "waiting for data" 표시
}

void loop() {
  // 새 데이터가 왔거나, 수신 인디케이터 점멸을 위해 주기적으로 다시 그림
  static uint32_t lastDraw = 0;
  if (updated || (millis() - lastDraw > 200)) {
    updated = false;
    lastDraw = millis();
    drawScreen();
  }
  delay(20);
}
