/* ============================================================
 * Vision Gauge — STEP 4-A · 코어 테스트 펌웨어
 *
 * 이 파일에서 검증하는 것:
 *   1) 카메라(GC2145)가 정상 캡처되는가
 *   2) Edge Impulse 모델이 보드에서 추론을 돌리는가
 *   3) 안정화 필터(같은 답 3회 + conf >= 0.70)가 의도대로 작동하는가
 *
 * 이 파일에 없는 것 (다음 단계에서 추가):
 *   - USB 시리얼 영상 스트리밍 (STEP 4-B 에서 추가)
 *   - BLE 무선 송신          (STEP 4-C 에서 추가)
 *   - OLED 화면 출력         (STEP 06, 제어 노드 측)
 *
 * 결과 확인: Arduino IDE 시리얼 모니터 (115200 baud)
 *   매 추론마다       RES: <label> (<conf>) <ms>ms
 *   안정화 통과 시    >>> STABLE: <label> (<conf>) <<<
 * ============================================================ */

#include <object_clf_inferencing.h>          // EI Studio에서 받은 라이브러리 헤더
                                             //   학습된 모델 가중치, 라벨, run_classifier() 함수가 모두 들어 있음
#include "edge-impulse-sdk/dsp/image/image.hpp"  // RGB565 -> RGB888 변환, 리사이즈 함수
#include "camera.h"                          // Arduino Nicla Camera 클래스
#include "gc2145.h"                          // GC2145 센서 드라이버
#include <ea_malloc.h>                       // mbed 환경의 정렬 메모리 할당

/* ============================================================
 * 1. 설정값 (여기만 조정하면 동작이 바뀜)
 * ============================================================ */
#define CONF_THRESHOLD   0.70f   // 이 확신도 이상일 때만 결과로 인정 (0.0 ~ 1.0)
                                 //   올리면 더 엄격 (놓치는 결과 늘고, 오인식 줄어듦)
                                 //   낮추면 더 관대 (반응 빠르고, 오인식 늘어남)
#define STABLE_REQUIRED  3       // 같은 답이 N번 연속이면 "확정"으로 판정
                                 //   바늘 떨림 / OLED 깜빡임을 방지하는 핵심 장치
#define LOOP_INTERVAL_MS 100     // 추론 시도 간격(ms) - 실제 추론은 ~32ms 걸리므로
                                 //   대략 100ms 주기 = 초당 10번 시도 (5fps급)

/* ============================================================
 * 2. 카메라 글루 코드 (EI 공식 예제에서 무수정 이식)
 * ------------------------------------------------------------
 * 이 영역은 GC2145 센서를 EI의 추론 파이프라인에 연결하는 "어댑터".
 * 학생 차시에서 직접 손댈 일은 거의 없고, 그대로 두고 동작.
 *
 * 동작 흐름:
 *   카메라 -> 320x240 RGB565 프레임 -> RGB888로 변환
 *          -> 모델 입력 크기(64x64)로 리사이즈/크롭
 *          -> EI 추론기에 픽셀 데이터 공급
 * ============================================================ */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS  320   // 카메라 출력 폭 (센서 고정)
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS  240   // 카메라 출력 높이
#define EI_CAMERA_RAW_FRAME_BYTE_SIZE      2   // RGB565는 픽셀당 2바이트
// 32바이트 정렬 매크로 - DMA 전송 시 정렬되지 않으면 성능 저하 또는 오동작
#define ALIGN_PTR(p,a) ((p & (a-1)) ? (((uintptr_t)p + a) & ~(uintptr_t)(a-1)) : p)

GC2145 galaxyCore;          // GC2145 센서 드라이버 인스턴스
Camera cam(galaxyCore);     // Arduino Camera API 래퍼
FrameBuffer fb;             // 프레임 버퍼 객체

static bool is_initialised = false;
static uint8_t *ei_camera_capture_out = NULL;   // RGB888 변환/리사이즈 결과 버퍼
static uint8_t *ei_camera_frame_mem;            // 원본 RGB565 프레임 메모리
static uint8_t *ei_camera_frame_buffer;         // 32바이트 정렬된 포인터

typedef struct { size_t width; size_t height; } resize_t;

/* RGB565(2바이트/픽셀) -> RGB888(3바이트/픽셀) 변환
 * 카메라는 메모리 절약을 위해 RGB565를 출력하지만, 모델은 RGB888을 입력으로 받음 */
bool RBG565ToRGB888(uint8_t *src, uint8_t *dst, uint32_t src_len) {
  uint32_t pix = src_len / 2;
  for (uint32_t i = 0; i < pix; i++) {
    uint8_t hb = *src++, lb = *src++;
    *dst++ = hb & 0xF8;                              // R (상위 5비트)
    *dst++ = (hb & 0x07) << 5 | (lb & 0xE0) >> 3;    // G (중간 6비트)
    *dst++ = (lb & 0x1F) << 3;                       // B (하위 5비트)
  }
  return true;
}

/* 출력 해상도에 맞는 중간 리사이즈 크기 결정.
 * EI 라이브러리가 단계적으로 줄여나가는 방식을 사용 */
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

/* 카메라 초기화 - setup()에서 한 번만 호출 */
bool ei_camera_init() {
  if (is_initialised) return true;
  if (!cam.begin(CAMERA_R320x240, CAMERA_RGB565, -1)) return false;

  // 원본 RGB565 프레임 메모리 할당 (320 * 240 * 2 = 153,600 byte + 정렬 여유)
  ei_camera_frame_mem = (uint8_t*)ei_malloc(
    EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS
    * EI_CAMERA_RAW_FRAME_BYTE_SIZE + 32);
  if (!ei_camera_frame_mem) return false;

  // 32바이트 정렬된 위치로 포인터 조정 (DMA 효율)
  ei_camera_frame_buffer = (uint8_t*)ALIGN_PTR((uintptr_t)ei_camera_frame_mem, 32);
  fb.setBuffer(ei_camera_frame_buffer);
  is_initialised = true;
  return true;
}

/* 한 프레임 캡처 + RGB 변환 + 리사이즈
 * 호출 후 ei_camera_capture_out 에 모델 입력 크기로 준비된 RGB888 픽셀이 들어 있음 */
bool ei_camera_capture(uint32_t w, uint32_t h) {
  if (!is_initialised) return false;

  // RGB888 출력 버퍼 할당 (320 * 240 * 3 = 230,400 byte + 정렬)
  // 함수 종료 시 ea_free로 반환 - 단편화 방지
  ei_camera_capture_out = (uint8_t*)ea_malloc(
    EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * 3 + 32);
  ei_camera_capture_out = (uint8_t*)ALIGN_PTR((uintptr_t)ei_camera_capture_out, 32);

  // 1) 카메라에서 RGB565 프레임 한 장 받기 (100ms 타임아웃)
  if (cam.grabFrame(fb, 100) != 0) return false;

  // 2) RGB888로 변환
  if (!RBG565ToRGB888(ei_camera_frame_buffer, ei_camera_capture_out, cam.frameSize()))
    return false;

  // 3) 모델 입력 크기로 리사이즈 + 중앙 크롭
  uint32_t rc, rr; bool do_resize;
  calc_resize(w, h, &rc, &rr, &do_resize);
  if (do_resize) {
    ei::image::processing::crop_and_interpolate_rgb888(
      ei_camera_capture_out, EI_CAMERA_RAW_FRAME_BUFFER_COLS, EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
      ei_camera_capture_out, rc, rr);
  }
  return true;
}

/* EI 추론기가 픽셀을 가져갈 때 호출하는 콜백.
 * RGB888 3바이트를 32비트 정수 하나로 패킹해 모델에 전달 */
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
 * 3. 안정화 필터 상태 (전역 변수)
 * ------------------------------------------------------------
 * 안정화 필터의 핵심 아이디어:
 *   카메라가 흔들리거나 빛이 변하면 모델이 잠깐 다른 답을 내기도 함.
 *   -> "확신도 0.70 이상" + "같은 답이 3번 연속"을 모두 통과한 결과만
 *      "STABLE = 확정"으로 인정. 그 외에는 무시.
 * 이 필터 덕분에 OLED 표시가 깜빡이지 않고, 서보 바늘이 떨지 않음.
 * ============================================================ */
int lastClass    = -1;   // 직전 추론에서 가장 확신도 높았던 클래스
int stableCount  =  0;   // 같은 답이 몇 번 연속됐는지 카운트
int confirmedCls = -1;   // 최근 "STABLE"로 확정된 클래스 (변화 시점 감지용)

/* ============================================================
 * 4. setup() - 부팅 시 한 번만 실행
 * ============================================================ */
void setup() {
  Serial.begin(115200);
  // USB 시리얼 연결을 5초간 기다림 (PC가 안 꽂혀 있어도 진행)
  while (!Serial && millis() < 5000) ;

  Serial.println();
  Serial.println("================================================");
  Serial.println("Vision Gauge - STEP 4-A : Nicla 코어 테스트");
  Serial.println("================================================");

  // [중요] Nicla Vision의 M4 코어 RAM 영역을 추가 할당 영역으로 등록.
  //   기본 M7 RAM(512KB)만으로는 EI TFLite arena가 부족해 부팅 직후 리셋됨.
  //   이 한 줄로 M4 영역에서 288KB를 추가 확보 - Arduino 공식 권장.
  malloc_addblock((void*)0x30000000, 288 * 1024);

  if (!ei_camera_init()) {
    Serial.println("FATAL: 카메라 초기화 실패");
    while(1);   // 무한 루프 (FATAL 표시 후 정지)
  }
  Serial.println("OK: 카메라 초기화 완료");

  // 학습된 모델 정보를 시리얼에 출력 - 학생이 자기 라벨이 맞는지 확인 가능
  Serial.print("모델 입력: ");
  Serial.print(EI_CLASSIFIER_INPUT_WIDTH); Serial.print("x");
  Serial.println(EI_CLASSIFIER_INPUT_HEIGHT);
  Serial.print("클래스 수: "); Serial.println(EI_CLASSIFIER_LABEL_COUNT);
  for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    Serial.print("  ["); Serial.print(i); Serial.print("] ");
    // 라벨은 EI Studio에서 자동으로 알파벳순으로 정렬됨
    //   STEP 06의 LABELS[] 배열 순서가 이 순서와 같아야 함
    Serial.println(ei_classifier_inferencing_categories[i]);
  }
  Serial.print("필터: conf>="); Serial.print(CONF_THRESHOLD);
  Serial.print(", stable="); Serial.println(STABLE_REQUIRED);
  Serial.println("추론 루프 시작...");
  Serial.println();
}

/* ============================================================
 * 5. loop() - 계속 반복 실행
 * ------------------------------------------------------------
 * 매 LOOP_INTERVAL_MS(=100ms)마다:
 *   1) 카메라 캡처   2) EI 추론   3) 가장 확신도 높은 클래스 선택
 *   4) 시리얼에 결과 출력   5) 안정화 필터 적용
 * ============================================================ */
void loop() {
  // 100ms 간격 유지 - 매 루프 즉시 추론하면 보드가 과열되고 USB도 막힘
  static uint32_t lastTick = 0;
  if (millis() - lastTick < LOOP_INTERVAL_MS) return;
  lastTick = millis();

  uint32_t t0 = millis();   // 추론 시간 측정 시작

  // 1) 카메라 캡처
  if (!ei_camera_capture(EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT)) {
    ea_free(ei_camera_capture_out);
    return;
  }

  // 2) 추론 - EI가 제공하는 signal_t 구조를 통해 픽셀 데이터를 모델에 전달
  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data     = &ei_camera_get_data;     // 위에서 정의한 콜백

  ei_impulse_result_t result = {0};
  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);
  if (err != EI_IMPULSE_OK) {
    Serial.print("ERR: run_classifier "); Serial.println(err);
    ea_free(ei_camera_capture_out);
    return;
  }

  // 3) argmax - 모델은 모든 클래스의 확률을 반환. 그중 가장 큰 것 하나만 선택
  float maxConf = 0;
  int maxIdx = -1;
  for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    if (result.classification[i].value > maxConf) {
      maxConf = result.classification[i].value;
      maxIdx  = i;
    }
  }

  uint32_t totalMs = millis() - t0;   // 캡처+추론 총 소요시간

  // 4) 시리얼에 매 추론 결과 출력 (사람이 읽을 수 있도록 라벨명까지)
  Serial.print("RES: ");
  Serial.print(ei_classifier_inferencing_categories[maxIdx]);
  Serial.print(" ("); Serial.print(maxConf, 2); Serial.print(") ");
  Serial.print(totalMs); Serial.println("ms");

  // 5) 안정화 필터 - "같은 답 N회 + conf >= 임계값" 통과 시에만 STABLE 확정
  if (maxConf >= CONF_THRESHOLD) {
    if (maxIdx == lastClass) {
      stableCount++;                // 같은 답 -> 카운트 증가
    } else {
      stableCount = 1;              // 다른 답 -> 카운트 리셋
      lastClass = maxIdx;
    }

    // STABLE 통과 + "직전 확정과 다른 새 답"일 때만 STABLE 줄 출력
    // (같은 답이 계속 STABLE로 들어와 줄이 도배되는 걸 방지)
    if (stableCount >= STABLE_REQUIRED && maxIdx != confirmedCls) {
      confirmedCls = maxIdx;
      Serial.print(">>> STABLE: ");
      Serial.print(ei_classifier_inferencing_categories[confirmedCls]);
      Serial.print(" ("); Serial.print(maxConf, 2); Serial.println(") <<<");
    }
  } else {
    // 확신도가 임계값 미만 - "모름" 상태로 간주, 카운터 리셋
    //   (이게 곧 unknown 클래스 없이도 "모름"을 처리하는 핵심 메커니즘)
    stableCount = 0;
    lastClass = -1;
  }

  ea_free(ei_camera_capture_out);   // 캡처 버퍼 반환 - 단편화 방지
}

/* ============================================================
 * 6. 안전 검증 - EI 라이브러리가 카메라 입력 모델인지 확인
 * ============================================================ */
#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "이 펌웨어는 카메라 입력 모델 전용입니다 (EI 프로젝트 설정 확인)"
#endif
