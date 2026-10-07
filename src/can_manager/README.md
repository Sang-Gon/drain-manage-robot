# CanManager — ROS2 ↔ CAN 게이트웨이

Main ECU(라즈베리파이5, Ubuntu 24.04, ROS2 Jazzy)에서 하위 ECU와 CAN으로 통신하는 노드.

이 Pi에서는 별도 `can_manager_ws`를 만들지 않고, 기존 라이다 워크스페이스인
`~/ros2_ws` 안에 직접 배치했다. `deploy/`와 이 README는 `can_manager` 패키지
안으로 들어와 패키지와 함께 움직인다.

```
~/ros2_ws/src/
├── robot_interfaces/                    # 공용 메시지·서비스 (TaskManager도 사용)
│   ├── msg/  MotorStatus, MotorStatusArray, TofStatus, CanHealth
│   ├── srv/  ClearMotorFault
│   ├── package.xml
│   └── CMakeLists.txt
├── can_manager/
│   ├── can_manager/
│   │   ├── can_protocol.py              ← 프로토콜 정의는 여기에만 존재
│   │   └── can_manager_node.py          ← 노드 본체
│   ├── config/can_params.yaml           ← 튜닝값 (재빌드 없이 수정 가능)
│   ├── launch/can_manager.launch.py
│   ├── package.xml / setup.py / setup.cfg
│   ├── resource/can_manager
│   ├── deploy/
│   │   ├── install_on_pi.sh             ← 설치 스크립트 (이 배치에 맞게 수정됨)
│   │   └── can0-up.service              ← can0 부팅 시 자동 기동
│   └── README.md                        ← 이 파일
│
├── sllidar_ros2/                        # [기존] RPLIDAR 드라이버 → /scan
└── drain_lidar_avoidance/               # [기존] 회피 로직 → /cmd_vel
```

## 기존 라이다 스택과의 관계

```
 sllidar_ros2 ──/scan──> drain_lidar_avoidance ──/cmd_vel──> can_manager ──CAN──> 하위 ECU
```

`drain_lidar_avoidance`가 발행하는 `/cmd_vel`을 `can_manager`가 그대로 받는다.
토픽·노드 이름이 겹치지 않으므로 두 스택은 동시에 띄워도 된다.

> **참고 — TF 트리**
> `can_manager`는 `odom → base_link` TF를, 라이다는 `laser` 프레임의 `/scan`을
> 내보낸다. 둘을 잇는 `base_link → laser` 변환이 아직 없어서 TF 트리가 끊겨 있다.
> 현재 회피 노드는 스캔 인덱스만 쓰므로 동작에는 문제가 없지만,
> SLAM이나 RViz에 붙이려면 라이다 장착 위치를 실측해 static transform을 추가해야 한다.
>
> ```bash
> ros2 run tf2_ros static_transform_publisher \
>     <x> <y> <z> <yaw> <pitch> <roll> base_link laser
> ```

> **참고 — 제거된 패키지**
> 회피 테스트용 임시 노드였던 `drain_can_manager`는 이 패키지와 역할이
> 완전히 겹쳐(둘 다 `/cmd_vel` → `0x200`) 제거했다.
> 백업: `~/backup/drain_can_manager_20260817/`

---

## 1. 시스템 구성

```
        [ Main ECU · 라즈베리파이5 ]
          비전 · LiDAR · SLAM
                  ↓
             TaskManager          ← 판단 (아직 미구현)
                  ↓ /cmd_vel
             CanManager           ← 변환 (이 패키지)
                  ↓
        ══════ CAN 버스 500kbps ══════
           ↓         ↓         ↓
       BLDC ECU   BAT ECU   TOF ECU
```

CanManager는 **변환기**다. "무엇을 할지"는 TaskManager가 정하고, 이 노드는
"어떻게 전달할지"만 담당한다. 단, 안전 조건에 걸리면 상위 명령을 거부할 권한이 있다.

## 2. 인터페이스

| 방향 | 토픽/서비스 | 타입 | CAN |
|---|---|---|---|
| 구독 | `/cmd_vel` | `geometry_msgs/Twist` | → `0x200` RPM 필드 (100ms) |
| 구독 | `/emergency_stop` | `std_msgs/Bool` | → `0x200` EMERGENCY (즉시) |
| 서비스 | `/clear_motor_fault` | `ClearMotorFault` | → `0x200` Fault Clear (500ms 펄스) |
| 발행 | `/motor/status` | `MotorStatusArray` | ← `0x210` / `0x211` |
| 발행 | `/battery/state` | `sensor_msgs/BatteryState` | ← `0x310` |
| 발행 | `/tof/status` | `TofStatus` | ← `0x410` |
| 발행 | `/odom` + TF | `nav_msgs/Odometry` | (실제 RPM 적분 → SLAM용) |
| 발행 | `/can/health` | `CanHealth` | (진단) |

## 3. 안전 게이트 (0x200 제어 모드)

매 송신 주기마다 안전 조건을 확인해 제어 모드를 자동 결정한다.

| 상황 | 모드 | RPM |
|---|---|---|
| 정상 | `RUN` | 목표 RPM |
| `/emergency_stop` = true | `EMERGENCY` | 0 |
| 모터 Fault 코드 ≠ 0 | `EMERGENCY` | 0 |
| `/cmd_vel` 300ms 미수신 | `IDLE` | 0 |
| BLDC 상태 프레임 300ms 두절 | `IDLE` | 0 |
| 노드 종료 시 | `EMERGENCY` | 0 (3회 송신) |

Fault가 걸리면 자동 클리어되지 않는다. `/clear_motor_fault` 서비스로 명시적으로 지워야 한다.

```bash
ros2 service call /clear_motor_fault \
    robot_interfaces/srv/ClearMotorFault "{clear_all: true}"
```

---

# 라즈베리파이5 설치 가이드

전체 흐름: **① CAN 하드웨어 → ② 로우레벨 검증 → ③ 파일 전송 → ④ 빌드 → ⑤ 실행**

소프트웨어보다 ①②단계에서 막힐 확률이 훨씬 높다. 여기서 시간을 아끼려 하지 말 것.

## 1단계 — CAN 하드웨어 활성화

라즈베리파이는 CAN 포트가 내장되어 있지 않다. 사용하는 하드웨어에 따라 갈린다.

### A. MCP2515 계열 SPI CAN HAT (PiCAN, Waveshare 등)

```bash
sudo nano /boot/firmware/config.txt
```

> Ubuntu on Pi5는 경로가 `/boot/firmware/config.txt`다.
> 옛날 자료의 `/boot/config.txt`가 아니다.

파일 맨 아래에 추가한다. **`oscillator` 값은 HAT 크리스탈 주파수와 일치해야 한다**
(보통 8MHz 또는 16MHz — 구매처 스펙 확인 필수):

```
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25
```

```bash
sudo reboot

# 재부팅 후 확인
dmesg | grep -i mcp2515      # "MCP2515 successfully initialized" 가 나와야 정상
ip link show can0             # can0 인터페이스가 보여야 정상
```

`can0`이 안 보이면 → `oscillator` 값 오류, `interrupt` 핀 번호 오류, SPI 미활성 순으로 확인.

### B. USB-CAN 어댑터 (CANable, PCAN-USB 등)

device tree 작업이 필요 없다. 꽂으면 커널이 자동 인식한다.

```bash
lsusb
dmesg | tail -20
ip link show
```

## 2단계 — can-utils로 로우레벨 검증

**ROS2를 건드리기 전에 반드시 여기서 통신이 되는지 확인한다.**
나중에 노드가 안 되는 걸 디버깅하다 사실 배선 문제였다는 걸 알면 시간 낭비가 크다.

```bash
sudo apt update && sudo apt install -y can-utils

# 인터페이스 수동 기동
sudo ip link set can0 up type can bitrate 500000 sample-point 0.875 restart-ms 100

# 상태 확인
ip -details link show can0
```

`state ERROR-ACTIVE`면 정상. `state BUS-OFF`가 반복되면:

- 버스 양 끝 종단저항(120Ω) 미장착
- 비트레이트 불일치 (모든 ECU가 500kbps인지)
- CAN_H / CAN_L 배선 반대

하위 ECU 전원을 켜고 배선을 연결한 상태에서:

```bash
candump can0
```

`210#...`, `211#...`, `310#...`, `410#...` 프레임이 찍히면 배선 문제는 해결된 것이다.
**이 화면이 안 뜨면 다음 단계로 넘어가지 말 것.**

## 3단계 — 파일 배치

**이 Pi에서는 이미 완료되었다.** 소스는 `~/ros2_ws/src/robot_interfaces`와
`~/ros2_ws/src/can_manager`에 배치되어 있다.

다른 기기에 새로 올릴 때만 해당:

```bash
scp -r ~/ros2_ws/src/robot_interfaces ~/ros2_ws/src/can_manager \
    pi@<라즈베리파이_IP>:~/ros2_ws/src/
```

## 4단계 — 설치 스크립트 실행

```bash
cd ~/ros2_ws/src/can_manager/deploy
./install_on_pi.sh              # 의존성 + 빌드
./install_on_pi.sh --service    # 위 + can0-up.service 등록까지
```

스크립트가 하는 일:

1. `can-utils` 설치
2. `python-can` 설치 — `sudo apt install python3-can` 우선.
   noble/universe에 4.3.1이 있어서 PEP 668을 우회할 필요가 없다.
   저장소에 없을 때만 `pip install python-can --break-system-packages`로 폴백한다.
3. `rosdep install` → `colcon build`
4. (`--service` 지정 시) `can0-up.service` 등록 — 부팅 시 can0 자동 기동

소스가 이미 `~/ros2_ws/src`에 있으므로 원본에 있던 "복사" 단계는 제거했다.
`--service`를 기본에서 뺀 이유는, 배선·종단저항 검증 전에 can0이 자동으로
올라오면 2단계 진단이 오히려 헷갈리기 때문이다.

환경 설정을 `.bashrc`에 추가한다:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 5단계 — 재부팅 후 실행

```bash
sudo reboot
```

재부팅 후:

```bash
ip -details link show can0    # 서비스가 자동으로 올렸는지 확인
candump can0                   # 여전히 프레임이 찍히는지
```

**★ 바퀴는 반드시 띄운 상태로 시작할 것 ★**

```bash
ros2 launch can_manager can_manager.launch.py
```

다른 터미널에서:

```bash
ros2 topic echo /motor/status
ros2 topic echo /battery/state
ros2 topic echo /tof/status
ros2 topic echo /can/health
```

## 6단계 — 실차 검증 순서

수신부터 확인하고 송신으로 넘어간다. 순서를 지킬 것.

1. **바퀴를 띄운다.**
2. BLDC ECU 딥스위치를 CAN 모드로 설정.
3. `/motor/status` 수신 확인 ← **수신이 먼저다.**
4. `teleop_twist_keyboard`로 저속 명령 → 실제 RPM이 목표를 추종하는지 확인.
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
5. 좌우 회전 방향이 반대면 → `can_params.yaml`의 `invert_left` / `invert_right` 수정.
6. 1m 직진 → `/odom` 값과 실측 비교 → `wheel_radius` 보정.
7. 제자리 360도 회전 → `/odom` 각도 비교 → `wheel_separation` 보정.
8. E-stop 발행 → 즉시 정지 확인.
   ```bash
   ros2 topic pub --once /emergency_stop std_msgs/Bool "{data: true}"
   ```
9. CAN 선을 뽑아 두절 재현 → 정지 및 `/can/health` 경보 확인.

---

## 하드웨어 없이 테스트 (가상 CAN)

실제 하드웨어를 기다리는 동안 로직만 먼저 검증할 수 있다.

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

ros2 launch can_manager can_manager.launch.py --ros-args -p can_interface:=vcan0

# 다른 터미널: 송신 프레임 확인
candump vcan0

# 또 다른 터미널: 주행 명령 주입
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

`200#...` 프레임이 100ms 주기로 흐르고, 키를 떼면 300ms 뒤 RPM이 0으로 떨어지면서
5번째 바이트(제어 모드)가 `01`(RUN) → `00`(IDLE)로 바뀌면 정상이다.

## CAN 세팅에 대한 참고

BLDC/BAT ECU는 STM32 FDCAN 페리페럴을 CubeMX에서 직접 레지스터 설정
(Prescaler·Seg1·Seg2)하지만, **Main ECU(Linux/SocketCAN)는 `ip link` 한 줄이면
커널이 이 계산을 대신 해준다.**

CanManager 파이썬 코드는 비트타이밍에 전혀 관여하지 않는다.
반드시 인터페이스가 UP된 뒤에 노드를 띄워야 한다.

---

## 문제 해결

| 증상 | 원인 |
|---|---|
| `can0`이 안 보임 | oscillator/interrupt 설정 오류, 재부팅 안 함, SPI 비활성 |
| `Cannot find device "can0"` | 위와 동일 — `dmesg \| grep spi`로 드라이버 로드 확인 |
| `state BUS-OFF` 반복 | 종단저항 없음, 비트레이트 불일치, CAN_H/L 반대 |
| `candump`엔 뜨는데 토픽엔 안 뜸 | `source install/setup.bash` 안 함 |
| `ModuleNotFoundError: can` | `sudo apt install python3-can` (없으면 `pip install python-can --break-system-packages`) |
| `rosidl_adapter` UnicodeDecodeError | `.msg`/`.srv` **주석**에 줄 끝 역슬래시(`\`)가 있는지 확인 |
| colcon build rosidl 에러 | `robot_interfaces`를 먼저 빌드했는지 확인 |
| 노드는 뜨는데 모터가 안 돔 | `/can/health`의 `motor_stale` 확인, BLDC 딥스위치 CAN 모드인지 확인 |

---

## 프로토콜 확정 현황

네 프레임 모두 팀 CAN 명세서(2026 캡스톤디자인 / 개발 관련 / CAN) 기준 **확정**.
모두 명세서 Ex 값으로 왕복 검증 완료.

| ID | 방향 | 내용 | 주기 |
|---|---|---|---|
| `0x200` | MAIN → BLDC | 좌/우 RPM + 제어모드 + Fault Clear | 100ms |
| `0x210`/`0x211` | BLDC → MAIN | M0(좌)/M1(우) 상태 | 100ms |
| `0x310` | BAT → MAIN | 전류·온도·전압·SOC·에러 | 미기재 |
| `0x410` | TOF → MAIN | 거리센서 6개 + 평균 | 미기재 |

**MAIN이 버스에 송신하는 프레임은 `0x200` 하나뿐이다.** 명세서에 없던 자체 정의
프레임(`0x100` 시스템 모드 · `0x101` heartbeat · `0x110` 진공 흡입)은 모두 제거했다.

- `0x110` — 흡입 기능 자체가 폐기됨
- `0x100` — E-stop은 `0x200` byte4(`3 = EMERGENCY`)가 이미 전달한다. 완전 중복이었다
- `0x101` — `0x200`이 100ms마다 나가므로 **`0x200`의 도착 자체가 MAIN 생존 증거**다.
  BLDC가 `0x200`에 RX 워치독을 걸면 heartbeat 없이 두절을 감지할 수 있다

수신자가 없는 프레임을 지운 것이라 기능 손실은 없다. 다만 `0x100`/`0x101`은
`0x200`보다 ID가 낮아 **버스 조정에서 모터 명령보다 앞섰다** — 제거로 그 역전도 해소됐다.

## 남은 확인 사항

1. **BLDC측 `0x200` RX 워치독** 유무와 타임아웃 값 — 100ms 송신 주기가 충분히 빠른지 검증 필요.
   **`0x101` heartbeat를 제거했으므로 MAIN 두절 보호는 전적으로 이 워치독에 달렸다**
2. **EMERGENCY 모드 복귀 방식** — Fault Clear 없이 자동 RUN 복귀되는지, 래치 상태인지
   (현재 코드는 자동 복귀를 가정. 래치라면 `_tx_motor`에 복귀 로직 추가 필요)
3. **TOF 센서 6개의 물리적 배치**(전/후/좌/우) — TaskManager 회피 로직에 필수
4. **BAT / TOF 송신 주기** — 현재 각각 1s, 100ms로 가정
5. 모터 오류 코드 범위 (명세서 "0~4" vs 설명 "0~5" — 0~5로 반영)
6. BAT 온도 / TOF 거리 필드 부호 (명세서 int8, 범위상 uint8로 반영)
7. BAT "소모 전류"의 충전/방전 방향 판별 방법 (현재 항상 방전으로 가정)
8. 실측 바퀴 반경 / 좌우 바퀴 간격 / 감속비 (6단계에서 보정)
