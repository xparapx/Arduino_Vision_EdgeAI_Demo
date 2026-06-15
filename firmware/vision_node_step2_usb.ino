/* ============================================================
 * Vision Gauge - STEP 4-B : USB 스트리밍 + 모니터 UI 레이어 추가
 *
 * step1과의 차이 (이 파일에서 새로 추가된 것):
 *   [+ STEP 4-B] base64 인코더 함수 (serialPrintBase64)
 *   [+ STEP 4-B] setup() 끝에 LBL: 라벨 목록 송신
 *   [+ STEP 4-B] loop()의 USB 스트리밍 블록 (RES: / IMG: 줄)
 *   [+ STEP 4-B] STABLE 시 사람용 로그 형식 변경 (# STABLE: ...)
 *   [+ STEP 4-B] USB 미연결 시 송신 생략 가드 (if (Serial))
 *
 * step1과 동일한 것 (변경 없음):
 *   - 카메라 글루, EI 추론, argmax, 안정화 필터 로직
 *
 * PC쪽 monitor.py가 받는 줄 단위 프로토콜:
 *   LBL:<label1>,<label2>,...    부팅 시 1회 (학습한 라벨 목록)
 *   RES:<idx>,<conf>,<t_ms>      매 추론 (가공 전 raw 결과)
 *   IMG:<base64 64x64 RGB888>    3번에 1번 (모델이 본 영상)
 *   # STABLE: <label> <conf> seq=<n>   안정화 확정 시
 * ============================================================ */

#include <object_clf_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "camera.h"
#include "gc2145.h"
#include <ea_malloc.h>

/* ============================================================
 * 1. 설정값 (step1과 동일 + IMG 송신 빈도만 추가)
 * ============================================================ */
#define CONF_THRESHOLD     0.70f
#define STABLE_REQUIRED    3
#define LOOP_INTERVAL_MS   100
#define IMG_EVERY_N        3      // [+ STEP 4-B] 3번 추론마다 1프레임 전송 (영상 부드러움)
                                  //   매 추론마다 영상을 보내면 USB 대역폭 초과
                                  //   IMG는 약 12KB 텍스트 - 5fps 정도가 적절

/* ============================================================
 * 2. 카메라 글루 (step1과 100% 동일 - 변경 없음)
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
 * [+ STEP 4-B 추가] 3. base64 인코더
 * ------------------------------------------------------------
 * 왜 필요한가:
 *   카메라 영상은 RGB888 바이너리 데이터 (값에 0x0A=개행, 0x00=NULL 등이 섞임).
 *   바이너리를 그대로 시리얼로 보내면 줄 단위 파싱이 깨짐.
 *   -> base64로 변환하면 알파벳+숫자+/+ 만 사용해 텍스트 라인으로 안전 전송.
 * 비용:
 *   3바이트 -> 4글자로 늘어남 (33% 오버헤드). 64x64x3=12,288바이트 -> 16,384글자.
 * ============================================================ */
static const char b64chars[] =
  "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

// 인코드 결과를 그대로 Serial로 흘려보냄 (메모리 절약 - 큰 버퍼 안 잡음)
void serialPrintBase64(const uint8_t *data, size_t len) {
  for (size_t i = 0; i < len; i += 3) {
    // 3바이트씩 묶어 24비트 정수 v로 합침
    uint32_t v = data[i] << 16;
    if (i + 1 < len) v |= data[i + 1] << 8;
    if (i + 2 < len) v |= data[i + 2];
    // 24비트를 6비트씩 4조각으로 잘라 base64 문자 인덱스로 사용
    Serial.print(b64chars[(v >> 18) & 0x3F]);
    Serial.print(b64chars[(v >> 12) & 0x3F]);
    Serial.print(i + 1 < len ? b64chars[(v >> 6) & 0x3F] : '=');  // 패딩
    Serial.print(i + 2 < len ? b64chars[ v       & 0x3F] : '=');
  }
}

/* ============================================================
 * 4. 안정화 필터 상태 (step1과 동일 + 송신용 변수 추가)
 * ============================================================ */
int      lastClass    = -1;
int      stableCount  =  0;
int      confirmedCls = -1;
uint8_t  seqNum       =  0;   // [+ STEP 4-B] STABLE 시 sequence 번호 (PC측 ID)
uint32_t loopCount    =  0;   // [+ STEP 4-B] IMG 송신 빈도 카운터

/* [+ STEP 4-B] 라벨 목록 송신 함수
 *   monitor.py가 이 LBL: 줄을 받아 "인덱스 -> 라벨명" 매핑을 만듦.
 *   부팅 시 1회 + loop에서 주기적으로 재전송 (monitor가 나중에 연결돼도 라벨 수신) */
void sendLabels() {
  Serial.print("LBL:");
  for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    if (i) Serial.print(",");
    Serial.print(ei_classifier_inferencing_categories[i]);
  }
  Serial.println();
}

/* ============================================================
 * 5. setup() - step1 + 라벨 송신 추가
 * ============================================================ */
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000) ;

  // [+ STEP 4-B] 시리얼 출력 형식 변경
  //   step1: 사람 친화적인 긴 메시지
  //   step2: monitor.py가 파싱하기 쉽도록 # 주석 + LBL/RES/IMG 줄 위주
  Serial.println("# Vision Gauge v2 - USB 스트리밍");

  malloc_addblock((void*)0x30000000, 288 * 1024);

  if (!ei_camera_init()) { Serial.println("# FATAL: 카메라"); while(1); }

  // [+ STEP 4-B] 라벨 목록 송신 (sendLabels 함수로 분리 - loop에서 주기 재전송)
  sendLabels();
  Serial.println("# ready");
}

/* ============================================================
 * 6. loop() - step1 추론 + [+ STEP 4-B] USB 스트리밍 블록
 * ============================================================ */
void loop() {
  static uint32_t lastTick = 0;
  if (millis() - lastTick < LOOP_INTERVAL_MS) return;
  lastTick = millis();

  // [+ STEP 4-B] 라벨 목록 주기적 재전송 (5초마다)
  //   monitor.py를 나중에 실행해도 라벨을 받을 수 있도록 - "#2" 대신 이름 표시
  static uint32_t lastLbl = 0;
  if (Serial && millis() - lastLbl > 5000) {
    lastLbl = millis();
    sendLabels();
  }

  uint32_t t0 = millis();
  if (!ei_camera_capture(EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT)) {
    ea_free(ei_camera_capture_out); return;
  }

  // 추론 (step1과 동일)
  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data     = &ei_camera_get_data;
  ei_impulse_result_t result = {0};
  if (run_classifier(&signal, &result, false) != EI_IMPULSE_OK) {
    ea_free(ei_camera_capture_out); return;
  }

  // argmax (step1과 동일)
  float maxConf = 0; int maxIdx = -1;
  for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    if (result.classification[i].value > maxConf) {
      maxConf = result.classification[i].value; maxIdx = i;
    }
  }
  uint32_t totalMs = millis() - t0;

  /* ----- [+ STEP 4-B 추가] USB 송신 블록 -----
   * if (Serial) 가드:
   *   USB가 연결 안 됐을 때는 송신을 통째로 건너뜀.
   *   안 그러면 시리얼 버퍼가 꽉 차서 다음 추론이 늦어질 수 있음.
   *   (시연 시 PC 없이 보드만 동작시킬 때 중요)
   */
  if (Serial) {
    // RES: 줄 - 매 추론마다 (가공 전 raw 결과)
    Serial.print("RES:");
    Serial.print(maxIdx); Serial.print(",");
    Serial.print(maxConf, 3); Serial.print(",");
    Serial.println(totalMs);

    // IMG: 줄 - 3번에 1번 (모델이 본 영상을 base64 텍스트로)
    if (loopCount % IMG_EVERY_N == 0) {
      Serial.print("IMG:");
      size_t pixBytes = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT * 3;
      serialPrintBase64(ei_camera_capture_out, pixBytes);
      Serial.println();
    }
  }
  loopCount++;

  // 안정화 필터 (step1과 거의 동일, STABLE 출력 형식만 변경)
  if (maxConf >= CONF_THRESHOLD) {
    if (maxIdx == lastClass) stableCount++;
    else { stableCount = 1; lastClass = maxIdx; }

    if (stableCount >= STABLE_REQUIRED && maxIdx != confirmedCls) {
      confirmedCls = maxIdx;
      seqNum++;
      // [+ STEP 4-B] 사람용 로그 형식 - monitor.py가 "# STABLE:"으로 파싱
      //   PC쪽 모니터에서 "확정 결과" 박스를 갱신하는 트리거
      Serial.print("# STABLE: ");
      Serial.print(ei_classifier_inferencing_categories[confirmedCls]);
      Serial.print(" "); Serial.print(maxConf, 2);
      Serial.print(" seq="); Serial.println(seqNum);
      // [+ STEP 4-C 예정] 여기서 BLE notify로 3바이트 송신 (step3에서 추가)
    }
  } else {
    stableCount = 0; lastClass = -1;
  }

  ea_free(ei_camera_capture_out);
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "이 펌웨어는 카메라 입력 모델 전용입니다"
#endif
