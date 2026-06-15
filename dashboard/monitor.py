"""
============================================================
Vision Gauge - 코어측 모니터 UI (STEP 4-B / STEP 05)
------------------------------------------------------------
Nicla Vision의 USB 시리얼을 읽어 PC 화면에 표시하는 Streamlit 앱.

Nicla Vision이 보내는 줄 단위 프로토콜:
  LBL:<라벨1>,<라벨2>,...    부팅 1회 (학습한 라벨 목록)
  RES:<idx>,<conf>,<t_ms>    매 추론 (raw 결과)
  IMG:<base64 64x64 RGB888>  3번에 1번 (모델이 본 영상)
  # STABLE: <라벨> <conf> seq=<n>   안정화 확정 시
  # ...                       그 외 주석 (디버그용, UI에는 미사용)

화면 구성:
  좌측 - 64x64 영상 (5배 확대)
  우측 - 실시간 인식 카드 (클래스별 색상, 큰 폰트, 퍼센트)
  하단 - 최근 20개 이력 표

실행:
  uv run streamlit run monitor.py
============================================================
"""

import base64
import io
import threading
import time
from collections import deque
from dataclasses import dataclass, field

import numpy as np
import serial
import serial.tools.list_ports
import streamlit as st
from PIL import Image

# ---------- 상수 ----------
IMG_W, IMG_H = 64, 64       # 모델 입력 해상도 (펌웨어와 일치해야 함)
BAUD = 115200               # 시리얼 통신 속도 (펌웨어 Serial.begin과 같아야)
POLL_MS = 150               # UI 갱신 주기 (150ms = 초당 약 6.7회, 영상 부드러움)

# 클래스별 색상 팔레트 - 다크 배경에서 서로 명확히 대비되는 테마 컬러맵
# (Tableau 10 계열을 다크 테마에 맞게 밝기 보정 - 인접 색이 확연히 구분됨)
CLASS_COLORS = [
    "#4FC3F7",  # 0 하늘색
    "#FF7043",  # 1 주황
    "#66BB6A",  # 2 초록
    "#BA68C8",  # 3 보라
    "#FFCA28",  # 4 노랑
    "#EC407A",  # 5 핑크
    "#26C6DA",  # 6 청록
    "#9CCC65",  # 7 연두
]

def class_color(idx: int) -> str:
    """클래스 인덱스 -> 색상 (개수 초과 시 순환)"""
    return CLASS_COLORS[idx % len(CLASS_COLORS)]


def semicircle_gauge(pct: int, color: str) -> str:
    """반원형(180도) 게이지 SVG 생성.
    pct: 0~100 확신도, color: 채움 색상.
    바늘이 좌(0%)에서 우(100%)로 회전, 중심축 하단에 Confidence 표기."""
    import math
    # 반원: 왼쪽(180도)에서 오른쪽(0도)으로. pct에 따라 바늘 각도 계산
    # 180도(왼쪽 끝) ~ 0도(오른쪽 끝)
    angle_deg = 180 - (pct / 100) * 180
    angle_rad = math.radians(angle_deg)
    cx, cy, r = 130, 130, 100        # 중심, 반지름
    # 바늘 끝점
    nx = cx + r * 0.82 * math.cos(angle_rad)
    ny = cy - r * 0.82 * math.sin(angle_rad)

    # 호(arc) 경로 - 배경(회색 전체 반원) + 채움(색상, pct만큼)
    def arc_point(p):
        a = math.radians(180 - (p / 100) * 180)
        return (cx + r * math.cos(a), cy - r * math.sin(a))
    bg_start = arc_point(0)
    bg_end = arc_point(100)
    fill_end = arc_point(pct)
    large = 0  # 반원이므로 항상 0
    sweep_bg = 1
    # 채움 호: 0%(왼쪽)에서 pct까지
    fill_path = (f"M {bg_start[0]:.1f} {bg_start[1]:.1f} "
                 f"A {r} {r} 0 0 1 {fill_end[0]:.1f} {fill_end[1]:.1f}")
    bg_path = (f"M {bg_start[0]:.1f} {bg_start[1]:.1f} "
               f"A {r} {r} 0 0 1 {bg_end[0]:.1f} {bg_end[1]:.1f}")

    return f"""
<svg viewBox="0 0 260 170" style="width:100%;max-width:340px;">
  <!-- 배경 호 (회색) -->
  <path d="{bg_path}" fill="none" stroke="#1e293b" stroke-width="20"
        stroke-linecap="round"/>
  <!-- 채움 호 (색상) -->
  <path d="{fill_path}" fill="none" stroke="{color}" stroke-width="20"
        stroke-linecap="round"/>
  <!-- 바늘 -->
  <line x1="{cx}" y1="{cy}" x2="{nx:.1f}" y2="{ny:.1f}"
        stroke="#f1f5f9" stroke-width="4" stroke-linecap="round"/>
  <!-- 중심축 -->
  <circle cx="{cx}" cy="{cy}" r="9" fill="#f1f5f9"/>
  <circle cx="{cx}" cy="{cy}" r="4" fill="{color}"/>
  <!-- 퍼센트 숫자 (호 안쪽 상단, 바늘과 겹치지 않게) -->
  <text x="{cx}" y="78" text-anchor="middle" fill="#f1f5f9"
        font-size="36" font-weight="800">{pct}<tspan font-size="18" fill="#94a3b8">%</tspan></text>
  <!-- Confidence 라벨 (중심축 하단) -->
  <text x="{cx}" y="158" text-anchor="middle" fill="#94a3b8"
        font-size="15" font-weight="600" letter-spacing="0.1em">Confidence</text>
</svg>"""


# ============================================================
# 1. 백그라운드 스레드용 공유 상태
# ------------------------------------------------------------
# 시리얼 읽기는 블로킹 작업이므로 별도 스레드에서 처리.
# Streamlit 메인 스레드는 이 State를 읽어 화면만 그림.
# ============================================================
@dataclass
class State:
    labels: list = field(default_factory=list)   # LBL:로 받은 라벨 목록
    last_img: np.ndarray = None                  # 마지막 IMG: 디코드 결과
    last_res: dict = None                        # 최근 RES: {"label","conf","t_ms"}
    history: deque = field(default_factory=lambda: deque(maxlen=20))  # 이력
    stable: dict = None                          # 최근 STABLE 결과
    connected: bool = False                      # 시리얼 연결 상태
    error: str = ""                              # 에러 메시지
    ble_tx: str = "idle"                         # BLE 송신 상태: idle/sent/waiting
    ble_tx_time: float = 0.0                     # 마지막 BLE 송신 시각


# ============================================================
# 2. 시리얼 읽기 백그라운드 루프
# ------------------------------------------------------------
# - PySerial로 한 줄씩 읽기
# - 머리말(LBL:/RES:/IMG:/# STABLE:)로 분기해 State 갱신
# - stop_flag로 종료 신호 받음
# ============================================================
def reader_loop(port: str, state: State, stop_flag: list):
    try:
        ser = serial.Serial(port, BAUD, timeout=1)
        state.connected = True
        state.error = ""
    except Exception as e:
        state.error = f"포트 열기 실패: {e}"
        return

    try:
        while not stop_flag[0]:
            try:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
            except Exception:
                continue
            if not line:
                continue

            # ---- LBL: 라벨 목록 (부팅 1회) ----
            if line.startswith("LBL:"):
                state.labels = line[4:].split(",")

            # ---- RES: 매 추론 결과 ----
            elif line.startswith("RES:"):
                try:
                    idx_str, conf_str, ms_str = line[4:].split(",")
                    idx = int(idx_str)
                    conf = float(conf_str)
                    t_ms = int(ms_str)
                    label = state.labels[idx] if 0 <= idx < len(state.labels) else f"#{idx}"
                    state.last_res = {"label": label, "conf": conf, "t_ms": t_ms, "idx": idx}
                    state.history.append({
                        "시각": time.strftime("%H:%M:%S", time.localtime()),
                        "라벨": label,
                        "확신도": f"{int(round(conf*100))}%",
                    })
                except ValueError:
                    pass   # 파싱 실패는 무시 (잡음 줄)

            # ---- IMG: 영상 base64 ----
            elif line.startswith("IMG:"):
                try:
                    raw = base64.b64decode(line[4:])
                    # 길이 검증 - 줄이 잘리지 않았는지 확인
                    if len(raw) == IMG_W * IMG_H * 3:
                        arr = np.frombuffer(raw, dtype=np.uint8).reshape(IMG_H, IMG_W, 3)
                        state.last_img = arr
                except Exception:
                    pass

            # ---- # STABLE: 안정화 확정 ----
            #   예: "# STABLE: bottle 0.89 seq=3"
            elif line.startswith("# STABLE:"):
                try:
                    parts = line[len("# STABLE:"):].strip().split()
                    label = parts[0]
                    conf = float(parts[1])
                    # 라벨 -> 인덱스 (색상 매핑용)
                    idx = state.labels.index(label) if label in state.labels else 0
                    state.stable = {"label": label, "conf": conf, "idx": idx,
                                    "time": time.strftime("%H:%M:%S")}
                except Exception:
                    pass

            # ---- BLE 송신 상태 (step3가 # TX: 줄로 출력) ----
            #   "... (BLE central에 notify 전송)"  -> 송신 성공
            #   "... (BLE 미연결 ...)"             -> 수신자 없음
            elif "BLE central" in line and "notify" in line:
                state.ble_tx = "sent"
                state.ble_tx_time = time.time()
            elif "BLE 미연결" in line:
                state.ble_tx = "waiting"
                state.ble_tx_time = time.time()

            # 그 외 "# ..." 주석 줄은 무시 (디버그용)
    finally:
        ser.close()
        state.connected = False


# ============================================================
# 3. Streamlit UI - 메인 스레드
# ============================================================
st.set_page_config(page_title="Vision Gauge Monitor", layout="wide")
st.title("🎥 Vision Gauge Monitor")
st.caption("Nicla Vision의 USB 시리얼 출력을 실시간으로 확인")

# ---- 사이드바: 포트 선택 + 연결 토글 ----
with st.sidebar:
    st.subheader("연결")
    ports = [p.device for p in serial.tools.list_ports.comports()]
    if not ports:
        st.warning("시리얼 포트를 찾을 수 없습니다. 보드가 USB로 연결되었는지 확인하세요.")
        st.stop()

    selected_port = st.selectbox("포트", ports)

    if "thread" not in st.session_state:
        st.session_state.thread = None
        st.session_state.state = State()
        st.session_state.stop_flag = [False]

    if st.button("연결" if not st.session_state.state.connected else "연결 끊기"):
        if st.session_state.thread is None or not st.session_state.thread.is_alive():
            # 시작
            st.session_state.state = State()
            st.session_state.stop_flag = [False]
            t = threading.Thread(
                target=reader_loop,
                args=(selected_port, st.session_state.state, st.session_state.stop_flag),
                daemon=True,
            )
            t.start()
            st.session_state.thread = t
        else:
            # 종료
            st.session_state.stop_flag[0] = True
            st.session_state.thread = None

    # 상태 표시
    state: State = st.session_state.state
    if state.connected:
        st.success(f"연결됨: {selected_port}")
    elif state.error:
        st.error(state.error)
    else:
        st.info("대기 중")

    # ---- BLE 송신 상태 인디케이터 ----
    #   비전 노드가 STABLE 시 BLE notify를 보내는지 USB 로그로 추적
    st.markdown("**BLE 송신**")
    ble_fresh = (time.time() - state.ble_tx_time) < 3.0   # 3초 내 활동
    if state.ble_tx == "sent" and ble_fresh:
        st.success("📡 송신 중 (제어 노드 수신)")
    elif state.ble_tx == "waiting" and ble_fresh:
        st.warning("⚠ 수신자 없음 (제어 노드 미연결)")
    else:
        st.caption("⬚ 대기 — STABLE 확정 시 송신")

    if state.labels:
        st.markdown("**라벨**")
        for i, lab in enumerate(state.labels):
            st.write(f"`[{i}]` {lab}")

# ---- 메인 영역: 영상 + 결과 (둘 다 정사각형, 1:1 배치) ----
col_img, col_res = st.columns([1, 1])

with col_img:
    st.subheader("모델이 보는 영상")
    img_area = st.empty()

with col_res:
    st.subheader("실시간 인식 결과")
    res_area = st.empty()

st.subheader("최근 20개 이력")
hist_area = st.empty()

# ---- POLL_MS마다 화면 갱신 (67회 x 150ms 약 10초 후 rerun) ----
# Streamlit은 자동 새로고침이 없으므로 명시적으로 st.rerun() 호출
POLL_MS_total = POLL_MS / 1000
for _ in range(67):
    state: State = st.session_state.state

    # 영상 - 정사각형(aspect-ratio 1:1). base64로 HTML img에 삽입
    if state.last_img is not None:
        img = Image.fromarray(state.last_img).resize((IMG_W * 5, IMG_H * 5),
                                                     Image.NEAREST)
        buf = io.BytesIO()
        img.save(buf, format="PNG")
        b64 = base64.b64encode(buf.getvalue()).decode()
        img_area.markdown(f"""
<div style="aspect-ratio:1/1;border-radius:18px;overflow:hidden;margin-top:8px;
            background:#000;display:flex;align-items:center;justify-content:center;">
  <img src="data:image/png;base64,{b64}"
       style="width:100%;height:100%;object-fit:cover;
              image-rendering:pixelated;"/>
</div>
""", unsafe_allow_html=True)
    else:
        img_area.markdown("""
<div style="aspect-ratio:1/1;border-radius:18px;margin-top:8px;background:#1e293b;
            display:flex;align-items:center;justify-content:center;
            color:#94a3b8;font-size:16px;">영상 대기 중...</div>
""", unsafe_allow_html=True)

    # ---- 실시간 인식 (영상과 같은 높이의 큰 카드, 클래스별 색상) ----
    if state.last_res:
        r = state.last_res
        # 라벨을 아직 못 받아 인덱스(#N)로 표시되는 경우 - 동기화 안내
        if r["label"].startswith("#"):
            res_area.markdown("""
<div style="background:#1e293b;border:2px dashed #475569;border-radius:18px;
            box-sizing:border-box;padding:40px 32px;margin-top:8px;text-align:center;
            display:flex;flex-direction:column;justify-content:center;align-items:center;
            aspect-ratio:1/1;">
  <div style="font-size:22px;color:#94a3b8;">라벨 동기화 중...</div>
  <div style="font-size:15px;color:#64748b;margin-top:10px;">
    비전 노드에서 라벨 목록을 받는 중입니다 (몇 초 소요)</div>
</div>
""", unsafe_allow_html=True)
        else:
            color = class_color(r["idx"])
            pct = int(round(r["conf"] * 100))
            # 절충안(실시간 송신): STABLE 개념 대신 확신도로 상태 구분
            if pct >= 70:
                check = "● 안정적 인식"
            elif pct >= 50:
                check = "● 인식 중"
            else:
                check = "○ 탐색 중"

            gauge = semicircle_gauge(pct, color)
            res_area.markdown(f"""
<div style="background:linear-gradient(135deg,{color}26,{color}0a);
            border:2px solid {color};border-radius:18px;box-sizing:border-box;
            padding:28px 32px;margin-top:8px;aspect-ratio:1/1;
            display:flex;flex-direction:column;align-items:center;
            justify-content:center;text-align:center;">
  <div style="font-size:16px;color:{color};letter-spacing:0.08em;
              font-weight:600;margin-bottom:10px;">{check}</div>
  <div style="font-size:60px;font-weight:800;color:{color};
              line-height:1.05;margin-bottom:8px;
              word-break:break-all;text-shadow:0 2px 12px {color}40;">{r['label']}</div>
  {gauge}
</div>
""", unsafe_allow_html=True)
    else:
        res_area.markdown("""
<div style="background:#1e293b;border:2px dashed #475569;border-radius:18px;
            box-sizing:border-box;padding:40px 32px;margin-top:8px;text-align:center;
            display:flex;flex-direction:column;justify-content:center;align-items:center;
            aspect-ratio:1/1;">
  <div style="font-size:22px;color:#94a3b8;">추론 결과 대기 중...</div>
</div>
""", unsafe_allow_html=True)

    # 이력 표
    if state.history:
        rows = list(state.history)[::-1]
        hist_area.dataframe(rows, use_container_width=True, hide_index=True)
    else:
        hist_area.empty()

    time.sleep(POLL_MS_total)

st.rerun()   # 약 10초마다 페이지 자동 새로고침 - Streamlit 특성상 필요
