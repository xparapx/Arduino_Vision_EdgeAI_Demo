@echo off
REM ============================================================
REM  Vision Gauge 대시보드 실행 (monitor.py)
REM  - 이 .bat 파일을 monitor.py 와 같은 폴더에 두고 더블클릭하세요.
REM  - 매번 터미널에서 경로 이동 / 명령 입력할 필요가 없습니다.
REM ============================================================

REM 한글 깨짐 방지 (콘솔을 UTF-8 로)
chcp 65001 >nul

REM 이 배치파일이 있는 폴더로 이동 (%~dp0 = 배치파일의 경로)
cd /d "%~dp0"

echo.
echo ============================================
echo   Vision Gauge 대시보드를 시작합니다...
echo   (종료하려면 이 창에서 Ctrl + C)
echo ============================================
echo.

REM uv 프로젝트로 streamlit 실행
uv run streamlit run monitor.py

REM 오류로 꺼졌을 때 창이 바로 닫히지 않도록 대기
echo.
echo ----- 종료되었습니다. 아무 키나 누르면 창이 닫힙니다. -----
pause >nul
