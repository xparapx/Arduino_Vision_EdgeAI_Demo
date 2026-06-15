# Vision Gauge — Edge AI 카메라 데모

> 카메라 보드가 **클라우드 없이 직접** 사물을 인식, 판단 결과만 무선으로 OLED·대시보드에 실시간 표시하는 온디바이스 비전 AI 프로젝트.

🔗 **프로젝트 개요:** https://xparapx.github.io/Arduino_Vision_EdgeAI_Demo/

`Nicla Vision` · `Edge Impulse` · `BLE` · `ESP32 · OLED` · `Streamlit` · `TinyML` · `SW·AI 융합 교육`

---

## 01. 프로젝트 요약

> "AI가 사물을 알아본다"를 직접 학습·탑재·확인하는 프로젝트.

카메라로 사물 이미지를 모아 **Edge Impulse**에서 분류 모델 학습 후 **Nicla Vision** 보드에 탑재. 인터넷 없이 보드 단독 추론(온디바이스 AI), 인식 결과(클래스·확신도)만 **BLE**로 ESP32 제어 노드에 전송 → **OLED** 게이지 표시. 동시에 USB로 PC 연결, **Streamlit 대시보드**에서 카메라 영상·인식 결과 실시간 확인.

## 02. 프로세스 흐름도

데이터가 픽셀에서 시작해 판단이 되고, 무선을 거쳐 화면에 닿기까지:

```
 ① 데이터 수집       ② 모델 학습        ③ 온디바이스 추론      ④ 무선 전송        ⑤ 시각화
  이미지 촬영·라벨링    분류 모델            보드가 직접 인식         판단 결과만         영상·게이지
                      학습·양자화          (클라우드 X)           BLE 송신           실시간 표시
 [Nicla Vision] ─업로드→ [Edge Impulse] ─.zip→ [int8/TFLite] ─3byte→ [ESP32·OLED] ─USB→ [Streamlit]
```

## 03. 준비물

### 하드웨어 (실물)
| 항목 | 비고 |
|---|---|
| Arduino Nicla Vision | 비전 노드 |
| ESP32 개발보드 | 제어 노드 |
| Grove OLED 1.12" V2 | SH1107G · 128×128 |
| USB-C 케이블 | ×2 |
| 점퍼 와이어 (I2C) | SDA·SCL·V·G |
| 인식 대상 사물 | 예: 아두이노·키보드·마우스 |

### 소프트웨어 · 라이브러리
| 도구 | 비고 |
|---|---|
| Arduino IDE | 2.3+ |
| Edge Impulse | 웹 · 모델 학습 |
| ArduinoBLE | 비전 노드 |
| U8g2 (by oliver) | OLED 구동 |
| ESP32 보드 패키지 | `BLEDevice.h` |
| Python + uv | streamlit · pyserial · numpy |

## 04. 기술 스택

- **AI** — 이미지 분류 · TinyML, int8 양자화 추론
- **MCU** — Nicla Vision (Cortex-M7), ESP32
- **통신** — BLE (GATT), USB Serial, I2C (OLED)
- **SW** — Streamlit, Python · pyserial, C++ (Arduino)

## 05. 저장소 구조

```
Arduino_Vision_EdgeAI_Demo/
├─ firmware/            # 보드 펌웨어 (.ino)
│  ├─ vision_node_step1_test.ino   # 카메라·추론 확인
│  ├─ vision_node_step2_usb.ino    # USB 영상·결과 송신
│  ├─ vision_node_step3_ble.ino    # BLE 결과 송신
│  └─ control_node_oled.ino        # ESP32 OLED 표시
├─ dashboard/           # PC 대시보드
│  ├─ monitor.py                   # Streamlit 앱
│  └─ 대시보드_실행.bat             # 더블클릭 실행
├─ manuals/             # 매뉴얼 (PDF)
├─ docs/                # 프로젝트 개요 (index.html)
├─ README.md
└─ .gitignore
```

---

## 실행 방법

```bash
cd dashboard
uv run streamlit run monitor.py
```
또는 Windows에서 `dashboard/대시보드_실행.bat` 더블클릭.

> ⚠️ `python monitor.py`로는 실행 불가. 반드시 `uv run streamlit run` 사용.
> **필요 패키지**: streamlit, pyserial, numpy (uv 프로젝트로 관리)

## 핵심 사양

**BLE**
- Service UUID: `7e400001-b5a3-f393-e0a9-e50e24dcca9e`
- 결과 Characteristic: `7e400002-…` (3바이트, Read + Notify)
- 기기명: `VisionGauge`
- 패킷: `[class_id, conf×100, seq]`

**라벨 변경 시 주의** — 모델 클래스 변경 시 두 곳 함께 수정:
1. 비전 노드: Edge Impulse 모델 재학습 후 헤더(`object_clf_inferencing.h`) 교체
2. 제어 노드: `control_node_oled.ino` 상단 `LABELS[]`를 비전 노드 시리얼 `LBL:` 순서와 동일하게 수정
> Edge Impulse는 라벨 알파벳순 정렬 → 보통 a→z 순서.

## 작업 로그

- **2026-06** BLE 라벨 자동 동기화 → ESP32 수동 입력 방식 전환 (안정성 개선)
- **2026-06** BLE 송신을 200ms 실시간 송신으로 변경 (STABLE 조건 제거)
- **2026-06** 대시보드 반원형 게이지 UI 적용

---

*Maintainer: xparapx*
