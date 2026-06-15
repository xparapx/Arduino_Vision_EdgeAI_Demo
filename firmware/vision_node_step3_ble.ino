/* ============================================================
 * Vision Gauge - STEP 4-C : BLE 송신 레이어 추가
 *
 * step2와의 차이 (이 파일에서 새로 추가된 것):
 *   [+ STEP 4-C] #include <ArduinoBLE.h>
 *   [+ STEP 4-C] BLE 서비스/특성 객체 전역 선언 (결과 + 라벨)
 *   [+ STEP 4-C] setup()에서 BLE.begin -> advertise (5단계)
 *   [+ STEP 4-C] loop() 첫 줄 BLE.poll() (생략 시 끊김!)
 *   [+ STEP 4-C] 200ms마다 최신 추론 결과를 3바이트 notify 송신 (실시간)
 *
 * step2와 동일한 것 (변경 없음):
 *   - 카메라 글루, EI 추론, argmax
 *   - USB 스트리밍 (LBL/RES/IMG)
 *
 * BLE 송신 방식 (절충안 - 실시간):
 *   안정화 필터(STABLE) 없이 매 추론의 최신 결과를 200ms마다 송신.
 *   확신도가 낮아도 즉시 반영되고, 200ms 쓰로틀로 OLED 깜빡임을 억제.
 *
 * BLE 송신 데이터:
 *   3바이트 [class_id, conf*100, seq]
 *     class_id : 클래스 인덱스 (0~N-1, 알파벳순)
 *     conf*100 : 확신도 정수 (0~100, 0.85 -> 85)
 *     seq      : 시퀀스 번호 (0~255 순환, 송신마다 증가)
 *
 * 검증 절차:
 *   1) USB 시리얼로 RES/IMG/TX 줄이 흐르는지 (4-B + TX)
 *   2) 폰 nRF Connect 앱으로 "VisionGauge" 검색/연결
 *   3) 0x7e400002 특성 구독 후 물체 변경 시 3바이트 갱신 확인
 * ============================================================ */

#include <object_clf_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "camera.h"
#include "gc2145.h"
#include <ea_malloc.h>
#include <ArduinoBLE.h>                  // [+ STEP 4-C] BLE Peripheral API

/* ============================================================
 * 1. 설정값
 * ============================================================ */
#define LOOP_INTERVAL_MS   100   // 추론 시도 간격(ms)
#define IMG_EVERY_N        3     // IMG 송신 빈도 (3번에 1번)
#define BLE_TX_INTERVAL_MS 200   // [절충안] BLE 송신 주기 - 200ms마다 최신 결과
                                 //   짧으면 반응 빠르나 OLED 깜빡임/대역폭↑
                                 //   길면 부드러우나 반응 느림. 200ms가 균형점

/* ============================================================
 * [+ STEP 4-C 추가] 2. BLE 서비스/특성 정의
 * ------------------------------------------------------------
 * UUID는 임의의 128비트 값 - "이 서비스는 다른 BLE 기기와 충돌하지 않는다"
 * 는 의미만 가짐. 0x7e40 prefix는 우리 프로젝트 고유 식별자로 사용.
 * 제어 노드(ESP32)의 코드도 이 UUID와 정확히 일치해야 함.
 * ============================================================ */
#define BLE_DEVICE_NAME    "VisionGauge"   // nRF Connect 스캔에 표시될 이름
#define SERVICE_UUID       "7e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHAR_UUID          "7e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define LABEL_UUID         "7e400003-b5a3-f393-e0a9-e50e24dcca9e"  // [+ 라벨 특성]

BLEService        visionService(SERVICE_UUID);
// 결과 특성 - 3바이트 [class_id, conf*100, seq], 자주 갱신
//   BLERead    : Central이 값을 읽을 수 있음
//   BLENotify  : 값이 바뀌면 Central에 자동 푸시 (구독 시)
BLECharacteristic resultChar(CHAR_UUID, BLERead | BLENotify, 3);

// [+ 라벨 특성] 라벨 목록 문자열 "label1,label2,unknown"
//   부팅 시 1회 기록, 제어 노드가 연결 직후 read해서 LABELS[] 채움.
//   BLEStringCharacteristic: 문자열 길이를 정확히 처리 (고정 BLECharacteristic은
//   const char* write 시 길이가 0으로 보이는 문제가 있어 String 전용 타입 사용)
//   최대 64바이트 (라벨이 길거나 많으면 늘릴 것)
BLEStringCharacteristic labelChar(LABEL_UUID, BLERead, 64);

/* ============================================================
 * 3. 카메라 글루 (step1/step2와 100% 동일)
 * ============================================================ */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS  320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS  240
#define EI_CAMERA_RAW_FRAME_BYTE_SIZE      2
#define ALIGN_PTR(p,a) ((p & (a-1)) ? (((uintptr_t)p + a) & ~(uintptr_t)(a-1)) : p)

GC2145 galaxyCore;
Camera cam(galaxyCore);
FrameBuffer fb;

static bool is_initialised = false;
static uint8_t *ei_camera_capture_out = NULL;
static uint8_t *ei_camera_frame_mem;
static uint8_t *ei_camera_frame_buffer;

typedef struct { size_t width; size_t height; } resize_t;

bool RBG565ToRGB888(uint8_t *src, uint8_t *dst, uint32_t src_len) {
  uint32_t pix = src_len / 2;
  for (uint32_t i = 0; i < pix; i++) {
    uint8_t hb = *src++, lb = *src++;
    *dst++ = hb & 0xF8;
    *dst++ = (hb & 0x07) << 5 | (lb & 0xE0) >> 3;
    *dst++ = (lb & 0x1F) << 3;
  }
  return true;
}

int calc_resize(uint32_t out_w, uint32_t out_h,
                uint32_t *col, uint32_t *row, bool *do_resize) {
  const resize_t list[] = {{64,64},{96,96},{160,120},{160,160},{320,240}};
  *col = EI_CAMERA_RAW_FRAME_BUFFER_COLS;
  *row = EI_CAMERA_RAW_FRAME_BUFFER_ROWS;
  *do_resize = false;
  for (size_t i = 0; i < 5; i++) {
    if (out_w <= list[i].width && out_h <= list[i].height) {
      *col = list[i].width; *row = list[i].height; *do_resize = true; break;
    }
  }
  return 0;
}

bool ei_camera_init() {
  if (is_initialised) return true;
  if (!cam.begin(CAMERA_R320x240, CAMERA_RGB565, -1)) return false;
  ei_camera_frame_mem = (uint8_t*)ei_malloc(
    EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS
    * EI_CAMERA_RAW_FRAME_BYTE_SIZE + 32);
  if (!ei_camera_frame_mem) return false;
  ei_camera_frame_buffer = (uint8_t*)ALIGN_PTR((uintptr_t)ei_camera_frame_mem, 32);
  fb.setBuffer(ei_camera_frame_buffer);
  is_initialised = true;
  return true;
}

bool ei_camera_capture(uint32_t w, uint32_t h) {
  if (!is_initialised) return false;
  ei_camera_capture_out = (uint8_t*)ea_malloc(
    EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * 3 + 32);
  ei_camera_capture_out = (uint8_t*)ALIGN_PTR((uintptr_t)ei_camera_capture_out, 32);
  if (cam.grabFrame(fb, 100) != 0) return false;
  if (!RBG565ToRGB888(ei_camera_frame_buffer, ei_camera_capture_out, cam.frameSize())) return false;
  uint32_t rc, rr; bool do_resize;
  calc_resize(w, h, &rc, &rr, &do_resize);
  if (do_resize) {
    ei::image::processing::crop_and_interpolate_rgb888(
      ei_camera_capture_out, EI_CAMERA_RAW_FRAME_BUFFER_COLS, EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
      ei_camera_capture_out, rc, rr);
  }
  return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
  size_t pix_ix = offset * 3;
  for (size_t i = 0; i < length; i++) {
    out_ptr[i] = (ei_camera_capture_out[pix_ix    ] << 16)
               + (ei_camera_capture_out[pix_ix + 1] <<  8)
               +  ei_camera_capture_out[pix_ix + 2];
    pix_ix += 3;
  }
  return 0;
}

/* ============================================================
 * 4. base64 인코더 (step2에서 추가, 그대로 유지)
 * ============================================================ */
static const char b64chars[] =
  "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

void serialPrintBase64(const uint8_t *data, size_t len) {
  for (size_t i = 0; i < len; i += 3) {
    uint32_t v = data[i] << 16;
    if (i + 1 < len) v |= data[i + 1] << 8;
    if (i + 2 < len) v |= data[i + 2];
    Serial.print(b64chars[(v >> 18) & 0x3F]);
    Serial.print(b64chars[(v >> 12) & 0x3F]);
    Serial.print(i + 1 < len ? b64chars[(v >> 6) & 0x3F] : '=');
    Serial.print(i + 2 < len ? b64chars[ v       & 0x3F] : '=');
  }
}

/* ============================================================
 * 5. 송신 상태 변수
 * ============================================================ */
uint8_t  seqNum       =  0;   // BLE 송신 시퀀스 번호 (0~255 순환)
uint32_t loopCount    =  0;   // IMG 송신 빈도 카운터

/* 라벨 목록 USB 송신 함수 (부팅 1회 + loop 주기 재전송)
 *   monitor.py가 나중에 연결돼도 라벨을 받아 "#2" 대신 이름 표시 */
void sendLabels() {
  Serial.print("LBL:");
  for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    if (i) Serial.print(",");
    Serial.print(ei_classifier_inferencing_categories[i]);
  }
  Serial.println();
}

/* ============================================================
 * 6. setup() - step2 + [+ STEP 4-C] BLE 초기화 5단계
 * ============================================================ */
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000) ;

  Serial.println("# Vision Gauge v3 - USB + BLE");
  malloc_addblock((void*)0x30000000, 288 * 1024);

  if (!ei_camera_init()) { Serial.println("# FATAL: 카메라"); while(1); }

  /* [+ STEP 4-C 추가] BLE 초기화 5단계 (순서 바꾸면 동작 안 함)
   *   1) BLE.begin()                 BLE 스택 기동
   *   2) setLocalName()               기기 이름 (스캔 시 표시)
   *   3) setAdvertisedService()       광고에 서비스 UUID 포함
   *   4) addCharacteristic + addService  특성을 서비스에 등록
   *   5) advertise()                  광고 시작 (Central이 발견 가능)
   */
  if (!BLE.begin()) {
    Serial.println("# FATAL: BLE.begin failed");
    while(1);
  }
  BLE.setLocalName(BLE_DEVICE_NAME);
  BLE.setAdvertisedService(visionService);
  visionService.addCharacteristic(resultChar);
  visionService.addCharacteristic(labelChar);   // [+ 라벨 특성] 등록
  BLE.addService(visionService);

  // 초기값 [0, 0, 0] - 첫 연결 시 Central이 읽으면 이 값을 받음
  uint8_t initVal[3] = {0, 0, 0};
  resultChar.writeValue(initVal, 3);

  // [+ 라벨 특성] 라벨 목록을 쉼표로 이어 문자열로 기록
  //   예: "bottle,marker,unknown"
  //   제어 노드(ESP32)가 연결 직후 이 값을 read해서 LABELS[]를 채움
  String labelStr = "";
  for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    if (i) labelStr += ",";
    labelStr += ei_classifier_inferencing_categories[i];
  }
  labelChar.writeValue(labelStr);   // BLEStringCharacteristic은 String 직접 전달
  Serial.print("# 라벨 특성 기록: "); Serial.print(labelStr);
  Serial.print(" (len="); Serial.print(labelStr.length()); Serial.println(")");

  BLE.advertise();
  Serial.print("# BLE advertising as '");
  Serial.print(BLE_DEVICE_NAME);
  Serial.println("'");

  // 라벨 목록 송신 (sendLabels로 분리 - loop에서 주기 재전송)
  sendLabels();
  Serial.println("# ready");
}

/* ============================================================
 * 7. loop() - step2 + [+ STEP 4-C] BLE.poll() + BLE notify
 * ============================================================ */
void loop() {
  /* [+ STEP 4-C 추가] BLE 이벤트 처리 - 매 루프 반드시 호출!
   * 누락 시 증상:
   *   - 연결은 되지만 1~2초 후 끊어짐 (Central은 PHY layer만 보고 alive 판단)
   *   - 학생 차시 BLE 문제의 80%가 이 한 줄 누락
   * 위치는 loop() 첫 줄이 권장 - 추론 사이에 끼면 추론 시간이 길어짐
   */
  BLE.poll();

  static uint32_t lastTick = 0;
  if (millis() - lastTick < LOOP_INTERVAL_MS) return;
  lastTick = millis();

  // 라벨 목록 주기적 재전송 (5초마다) - monitor.py 나중 연결 대비
  static uint32_t lastLbl = 0;
  if (Serial && millis() - lastLbl > 5000) {
    lastLbl = millis();
    sendLabels();
  }

  uint32_t t0 = millis();
  if (!ei_camera_capture(EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT)) {
    ea_free(ei_camera_capture_out); return;
  }

  // 추론 (step1/step2와 동일)
  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data     = &ei_camera_get_data;
  ei_impulse_result_t result = {0};
  if (run_classifier(&signal, &result, false) != EI_IMPULSE_OK) {
    ea_free(ei_camera_capture_out); return;
  }

  // argmax
  float maxConf = 0; int maxIdx = -1;
  for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    if (result.classification[i].value > maxConf) {
      maxConf = result.classification[i].value; maxIdx = i;
    }
  }
  uint32_t totalMs = millis() - t0;

  /* USB 송신 (step2와 동일) - BLE와 독립적으로 동작 */
  if (Serial) {
    Serial.print("RES:");
    Serial.print(maxIdx); Serial.print(",");
    Serial.print(maxConf, 3); Serial.print(",");
    Serial.println(totalMs);

    if (loopCount % IMG_EVERY_N == 0) {
      Serial.print("IMG:");
      size_t pixBytes = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT * 3;
      serialPrintBase64(ei_camera_capture_out, pixBytes);
      Serial.println();
    }
  }
  loopCount++;

  /* [+ STEP 4-C / 절충안] BLE 실시간 송신
   * STABLE 조건 없이 매 추론 결과를 200ms마다 최신값으로 송신.
   *   - 즉각 반응 (확신도 낮아도 보임)
   *   - 200ms 쓰로틀로 OLED 깜빡임 억제 (매 추론마다 보내면 너무 잦음)
   * monitor.py는 이 송신을 BLE 인디케이터에 반영. */
  static uint32_t lastBleMs = 0;
  if (millis() - lastBleMs >= BLE_TX_INTERVAL_MS) {
    lastBleMs = millis();
    seqNum++;

    // 3바이트 [class_id, conf*100, seq] - 가장 확신도 높은 클래스 그대로
    uint8_t bleBuf[3] = {
      (uint8_t)maxIdx,
      (uint8_t)(maxConf * 100),   // 0.0~1.0 -> 0~100 정수
      seqNum
    };
    resultChar.writeValue(bleBuf, 3);

    // 사람용 로그 (monitor.py가 BLE 송신 인디케이터 트리거로 사용)
    Serial.print("# TX: ");
    Serial.print(ei_classifier_inferencing_categories[maxIdx]);
    Serial.print(" "); Serial.print(maxConf, 2);
    if (BLE.connected()) {
      Serial.println(" (BLE central에 notify 전송)");
    } else {
      Serial.println(" (BLE 미연결 - 수신자 없음)");
    }
  }

  ea_free(ei_camera_capture_out);
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "이 펌웨어는 카메라 입력 모델 전용입니다"
#endif
