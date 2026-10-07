#!/usr/bin/env bash
# =============================================================================
#  Hailo 가속기 장착 후 점검 스크립트  (읽기 전용 — 아무것도 바꾸지 않는다)
#
#  사용법:  bash ~/ros2_ws/src/drain_vision/deploy/check_hailo.sh
#
#  모듈을 꽂고 이 스크립트를 돌려 [FAIL] 이 나온 항목부터 해결하면 된다.
#  설치 절차 자체는 drain_vision/README.md 참고.
# =============================================================================
set -u

pass() { printf '  \033[32m[ OK ]\033[0m %s\n' "$1"; }
fail() { printf '  \033[31m[FAIL]\033[0m %s\n' "$1"; FAILED=$((FAILED+1)); }
info() { printf '         %s\n' "$1"; }
FAILED=0

echo "=== 1. PCIe 활성화 (config.txt) ==="
if grep -qE '^\s*dtparam=pciex1' /boot/firmware/config.txt 2>/dev/null; then
    pass "dtparam=pciex1 설정됨"
else
    fail "config.txt 에 dtparam=pciex1 없음 — PCIe 슬롯이 안 켜진다"
    info "추가할 줄:  dtparam=pciex1"
    info "            pciex1_gen=3      # Hailo-8 권장. 불안정하면 이 줄만 빼고 gen2로"
    info "고친 뒤 재부팅 필요"
fi

echo "=== 2. PCIe 장치 인식 ==="
if lspci 2>/dev/null | grep -qi hailo; then
    pass "$(lspci | grep -i hailo)"
else
    fail "lspci 에 Hailo 없음 — 모듈 미장착이거나 PCIe 미활성"
    info "현재 lspci: $(lspci 2>/dev/null | tr '\n' ' | ')"
fi

echo "=== 3. 커널 드라이버 (/dev/hailo0) ==="
if ls /dev/hailo* >/dev/null 2>&1; then
    pass "$(ls /dev/hailo*)"
else
    fail "/dev/hailo* 없음 — hailo_pci 드라이버 미설치/미로드"
    info "dkms 상태: $(dkms status 2>/dev/null || echo 'dkms 미설치')"
fi

echo "=== 4. 빌드 전제조건 (드라이버 dkms 빌드에 필요) ==="
command -v dkms >/dev/null 2>&1 && pass "dkms 설치됨" || fail "dkms 없음 → sudo apt install dkms"
[ -d "/usr/src/linux-headers-$(uname -r)" ] \
    && pass "커널 헤더 있음 ($(uname -r))" \
    || fail "커널 헤더 없음 → sudo apt install linux-headers-$(uname -r)"

echo "=== 5. HailoRT 런타임 ==="
if command -v hailortcli >/dev/null 2>&1; then
    pass "hailortcli 있음 — $(hailortcli --version 2>&1 | head -1)"
    echo "     --- fw-control identify ---"
    hailortcli fw-control identify 2>&1 | sed 's/^/     /'
else
    fail "hailortcli 없음 → HailoRT deb 설치 필요"
fi

echo "=== 6. 파이썬 바인딩 ==="
python3 - <<'PY' 2>&1 | sed 's/^/  /'
import sys, site
p = site.getusersitepackages()
if p not in sys.path:
    sys.path.insert(0, p)
try:
    import hailo_platform
    print("\033[32m[ OK ]\033[0m hailo_platform:", hailo_platform.__file__)
except Exception as e:
    print("\033[31m[FAIL]\033[0m hailo_platform import 실패:", e)
PY

echo "=== 7. 모델 파일 ==="
#  [2026-08-29] v2 로 교체. 여기가 옛 이름 그대로면 점검이 쓰지도 않는 모델을 본다.
#  detector_params.yaml 의 model_path 와 같은 값을 유지할 것.
HEF=~/ros2_ws/install/drain_vision/share/drain_vision/models/yolov11s_v2.hef
if [ -f "$HEF" ]; then
    pass "$(du -h "$HEF" | cut -f1)  $HEF"
else
    fail "설치된 .hef 없음 — colcon build 를 먼저 할 것"
fi

echo
if [ "$FAILED" -eq 0 ]; then
    echo "전부 통과. 이제 실행:  ros2 launch drain_vision detection.launch.py"
else
    echo "미해결 $FAILED 건. 위 [FAIL] 항목부터 처리할 것."
fi
