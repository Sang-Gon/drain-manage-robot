#!/usr/bin/env bash
# =============================================================================
#  CanManager 설치 스크립트 (라즈베리파이5 / Ubuntu 24.04 / ROS2 Jazzy)
# =============================================================================
#
#  [원본과 달라진 점]
#    원본 install_on_pi.sh는 can_manager_ws 폴더를 통째로 받아와서
#    ~/ros2_ws/src로 "복사"하는 것을 전제로 했다.
#    이 Pi에서는 소스가 이미 ~/ros2_ws/src 안에 직접 배치되어 있으므로
#    복사 단계를 제거하고, 의존성 설치 / 빌드 / systemd 등록만 수행한다.
#
#  사용법:
#    cd ~/ros2_ws/src/can_manager/deploy
#    chmod +x install_on_pi.sh
#    ./install_on_pi.sh              # 의존성 + 빌드 (systemd 등록은 안 함)
#    ./install_on_pi.sh --service    # 위 + can0-up.service 등록까지
#
#  전제조건:
#    - ROS2 Jazzy가 /opt/ros/jazzy에 설치되어 있을 것
#    - `ip link show can0`에 can0이 보일 것
#      (안 보이면 ../README.md의 "1단계 CAN 하드웨어 활성화"를 먼저 수행)
# =============================================================================
set -euo pipefail

WITH_SERVICE=0
[[ "${1:-}" == "--service" ]] && WITH_SERVICE=1

DEPLOY_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$DEPLOY_DIR/../../.." && pwd)"     # .../ros2_ws

echo "=============================================="
echo " CanManager 설치"
echo "   워크스페이스 : $WS"
echo "   systemd 등록 : $([[ $WITH_SERVICE == 1 ]] && echo 'YES' || echo 'no (--service로 활성화)')"
echo "=============================================="

# --- [1/4] can-utils ---------------------------------------------------------
echo ""
echo "==> [1/4] can-utils 설치"
sudo apt update
sudo apt install -y can-utils

# --- [2/4] python-can --------------------------------------------------------
echo ""
echo "==> [2/4] python-can 설치"
if python3 -c "import can" 2>/dev/null; then
    echo "  이미 설치되어 있음 — 건너뜀"
elif apt-cache show python3-can >/dev/null 2>&1; then
    # noble/universe에 python3-can 4.3.1이 있다. apt로 받는 쪽이
    # PEP 668을 우회하지 않고 패키지 관리자가 계속 관리해 주므로 낫다.
    echo "  apt의 python3-can 사용"
    sudo apt install -y python3-can
else
    # 저장소에 없을 때만 pip 폴백. Ubuntu 24.04는 PEP 668 때문에
    # --break-system-packages가 필요하다.
    echo "  apt에 python3-can이 없음 — pip 폴백"
    pip install python-can --break-system-packages
fi

# --- [3/4] 빌드 --------------------------------------------------------------
echo ""
echo "==> [3/4] 의존성 확인 및 빌드"
source /opt/ros/jazzy/setup.bash
cd "$WS"
rosdep install --from-paths src --ignore-src -r -y || \
    echo "  (rosdep 경고는 대부분 무시 가능 — 아래 빌드 결과를 확인하세요)"

# robot_interfaces가 먼저 빌드되어야 can_manager가 메시지를 찾는다.
# (colcon이 의존관계를 보고 알아서 순서를 잡지만 명시적으로 적어 둔다)
# (기존 라이다 패키지들이 비-symlink로 빌드되어 있어 워크스페이스를 통일한다.
#  config/can_params.yaml을 자주 고칠 거라면 --symlink-install을 붙여도 된다)
colcon build --packages-select robot_interfaces can_manager

# --- [4/4] can0 자동 기동 서비스 ---------------------------------------------
echo ""
if [[ $WITH_SERVICE == 1 ]]; then
    echo "==> [4/4] can0 자동 기동 systemd 서비스 등록"
    sudo cp "$DEPLOY_DIR/can0-up.service" /etc/systemd/system/can0-up.service
    sudo systemctl daemon-reload
    sudo systemctl enable can0-up.service
    echo "  등록 완료 — 다음 부팅부터 can0이 자동으로 올라온다."
    echo "  지금 바로 올리려면: sudo systemctl start can0-up.service"
else
    echo "==> [4/4] systemd 등록 건너뜀"
    echo "  하위 ECU 배선·종단저항 검증 전이라면 수동으로 올려서 먼저 확인할 것:"
    echo "    sudo ip link set can0 up type can bitrate 500000 sample-point 0.875 restart-ms 100"
    echo "    candump can0"
    echo "  검증이 끝난 뒤 ./install_on_pi.sh --service 로 등록하면 된다."
fi

cat << EOF

=============================================
 설치 완료
=============================================

1. ~/.bashrc에 워크스페이스 오버레이가 없다면 추가:
     echo "source $WS/install/setup.bash" >> ~/.bashrc
     source ~/.bashrc

2. 실행 (★바퀴는 반드시 띄운 상태로★):
     ros2 launch can_manager can_manager.launch.py

   하드웨어 없이 로직만 검증하려면:
     sudo modprobe vcan
     sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
     ros2 launch can_manager can_manager.launch.py --ros-args -p can_interface:=vcan0

EOF
