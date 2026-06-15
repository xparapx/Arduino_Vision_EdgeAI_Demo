# Arduino Vision EdgeAI Demo (Vision Gauge)

> 카메라 보드가 **클라우드 없이 직접** 사물을 인식하고, 판단 결과만 무선으로 보내 OLED와 대시보드에 실시간 표시하는 온디바이스 비전 AI 프로젝트.

고등학교 SW·AI 융합수업용으로 제작되었습니다. 학생이 **데이터 수집 → 모델 학습 → 온디바이스 추론 → 무선 전송 → 시각화**의 전 과정을 직접 경험합니다.

---

## 프로젝트 개요

`docs/index.html`을 브라우저로 열면 프로젝트 요약·준비물·기술스택·프로세스 흐름도를 한눈에 볼 수 있습니다.

| 단계 | 내용 | 도구 |
|---|---|---|
| ① 데이터 수집 | 사물별 이미지 촬영·라벨링 | Nicla Vision |
| ② 모델 학습 | 이미지 분류 모델 학습·양자화 | Edge Impulse |
| ③ 온디바이스 추론 | 보드가 직접 인식 (클라우드 X) | int8 / TFLite |
| ④ 무선 전송 | 판단 결과(3바이트)만 BLE 송신 | ESP32 · OLED |
| ⑤ 시각화 | 영상·게이지 실시간 표시 | Streamlit |

---

## 하드웨어 구성

- **비전 노드**: Arduino Nicla Vision — 카메라 + 온디바이스 추론
- **제어 노드**: ESP32 + Grove OLED 1.12" V2 (SH1107G, 128×128, I2C)
- 비전 노드 → ESP32: **BLE** 3바이트 `[class_id, conf×100, seq]`
- 비전 노드 → PC: **USB Serial** (영상 + 결과)

### BLE 사양
- Service UUID: `7e400001-b5a3-f393-e0a9-e50e24dcca9e`
- 결과 Characteristic: `7e400002-...` (3바이트, Read + Notify)
- 기기명: `VisionGauge`

---

## 폴더 구조

```
firmware/    보드 펌웨어 (.ino)
dashboard/   PC 대시보드 (Streamlit)
manuals/     학생·교사 매뉴얼 (PDF)
docs/        프로젝트 개요 (HTML)
```

### firmware
| 파일 | 역할 |
|---|---|
| `vision_node_step1_test.ino` | 카메라·추론 동작 확인 |
| `vision_node_step2_usb.ino` | USB로 영상·결과 송신 |
| `vision_node_step3_ble.ino` | BLE로 결과 송신 |
| `control_node_oled.ino` | ESP32 OLED 표시 (라벨 수동 입력) |

---

## 대시보드 실행

```bash
cd dashboard
uv run streamlit run monitor.py
```

또는 `dashboard/대시보드_실행.bat`을 더블클릭 (Windows).

**필요 패키지**: streamlit, pyserial, numpy (uv 프로젝트로 관리)

> ⚠️ `python monitor.py`로는 실행되지 않습니다. 반드시 `uv run streamlit run`을 사용하세요.

---

## 사용 라이브러리

- **비전 노드**: ArduinoBLE, Edge Impulse SDK (`object_clf_inferencing.h`)
- **제어 노드**: U8g2 (by oliver), ESP32 내장 BLE (`BLEDevice.h`)
- **대시보드**: Streamlit, pyserial, numpy

---

## 라벨 변경 시 주의

모델의 클래스를 바꾸면 두 곳을 함께 수정해야 합니다:
1. 비전 노드: Edge Impulse 모델 재학습 후 헤더 교체
2. 제어 노드: `control_node_oled.ino` 상단 `LABELS[]` 배열을 비전 노드 시리얼의 `LBL:` 순서와 동일하게 수정

> Edge Impulse는 라벨을 알파벳순으로 정렬하므로, 보통 a→z 순서입니다.

---

## 작업 로그

- **2026-06**: BLE 라벨 자동 동기화 → ESP32 수동 입력 방식으로 전환 (안정성 개선)
- **2026-06**: BLE 송신을 200ms 실시간 송신으로 변경 (STABLE 조건 제거)
- **2026-06**: 대시보드 반원형 게이지 UI 적용

---

*Maintainer: physics-jh*
