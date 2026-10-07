#!/usr/bin/env python3
# =============================================================================
#  can_protocol.py  —  CAN 프로토콜 정의 (Single Source of Truth)
# =============================================================================
#
#  [이 파일의 역할]
#    CAN 프레임의 ID / 바이트 배치 / 스케일 팩터는 **오직 이 파일에만** 존재한다.
#    노드 코드(can_manager_node.py)는 아래의 pack_* / unpack_* 함수만 호출하므로,
#    펌웨어 프로토콜이 바뀌면 이 파일 하나만 수정하면 된다.
#
#  [수정할 때 보는 곳]
#    - CAN ID 변경        -> "SECTION 1. CAN ID 맵"
#    - 바이트 배치 변경   -> "SECTION 5" (송신) / "SECTION 6" (수신)의 해당 함수
#    - 코드값/이름 추가   -> "SECTION 2~4"의 상수 정의
#
#  [프로토콜 확정 현황]  기준: 2026 캡스톤디자인 / 개발 관련 / CAN (팀 노션)
#    [O] MAIN -> BLDC  모터 명령      0x200         (송신 주기 100ms, BLDC팀 확인)
#    [O] BLDC -> MAIN  모터 상태      0x210 / 0x211 (수신 주기 100ms)
#    [O] BAT  -> MAIN  배터리 상태    0x310         (명세 미기재 · 2026-08-29 실측 101.2ms)
#    [O] TOF  -> MAIN  거리 센서      0x410         (명세 미기재 · 2026-08-25 실측 1.000s)
#
#    반대 방향(MAIN -> BAT, MAIN -> TOF) 통신은 없다.
#    MAIN이 송신하는 프레임은 0x200 하나뿐이고, 그 수신자는 BLDC 하나뿐이다.
#    체크섬 / alive 카운터는 실제 프로토콜에 없다. 프레임 신선도는
#    각 상태 프레임의 수신 타임스탬프만으로 판단한다.
#
#  [미확인 사항 — 펌웨어팀 확인 필요]
#    (1) 모터 오류 코드 범위 표기가 "0~4"인데 설명에는 5(HALL fault)까지 있다.
#        오타로 보고 0~5로 반영했다.
#    (2) BAT 온도 / TOF 거리 필드가 명세서엔 int8인데 범위가 0~255다.
#        범위 기준으로 uint8이 맞다고 판단해 반영했다.
#    (3) BAT / TOF 프레임의 바이트 순서는 명세서 표의 필드 나열 순서를 그대로
#        따랐다고 가정했다. 실제 프레임 캡처로 검증 전까지는 가정이다.
#    (4) EMERGENCY 모드가 Fault Clear 없이 RUN으로 자동 복귀되는지, 아니면
#        반드시 Fault Clear를 보내야 빠져나오는 래치 상태인지 미확인.
#        현재 CanManager는 "안전 조건이 풀리면 자동 RUN 복귀"를 가정한다.
#    (5) BLDC측에 0x200 RX 워치독(일정 시간 미수신 시 자동 정지)이 있는지,
#        있다면 타임아웃 값이 얼마인지 미확인.
#
# =============================================================================

import struct
from dataclasses import dataclass, field


# =============================================================================
#  SECTION 1. CAN ID 맵
# =============================================================================
#  CAN은 ID가 작을수록 버스 조정(arbitration) 우선순위가 높다.
#  따라서 안전 관련 프레임일수록 낮은 ID를 할당한다.
#  (우선순위는 소프트웨어가 아니라 이 ID 값에 의해 하드웨어 레벨에서 결정된다)
# -----------------------------------------------------------------------------
CAN_ID_MOTOR_CMD     = 0x200   # Main -> BLDC : 좌/우 목표 RPM + 모드  (100ms)
CAN_ID_MOTOR_STATUS0 = 0x210   # BLDC -> Main : M0(좌) 상태            (100ms)
CAN_ID_MOTOR_STATUS1 = 0x211   # BLDC -> Main : M1(우) 상태            (100ms)
CAN_ID_BAT_STATUS    = 0x310   # BAT  -> Main : 전류/온도/전압/SOC     (1s 가정)
CAN_ID_TOF_STATUS    = 0x410   # TOF  -> Main : 거리센서 6개 + 평균    (실측 1.000s)

# 모터 상태 프레임 ID 묶음 (수신 라우팅용)
MOTOR_STATUS_IDS = (CAN_ID_MOTOR_STATUS0, CAN_ID_MOTOR_STATUS1)


# =============================================================================
#  SECTION 2. 스케일 팩터
# =============================================================================
#  [주의] ECU마다 방식이 다르다.
#    - BLDC : 전류에만 스케일 적용 (raw * 0.01 = A)
#    - BAT  : 스케일 없음. 정수가 그대로 물리값 (1557 -> 1557mA)
#             단, 전류만 mA -> A 변환이 필요해 unpack 함수 안에서 직접 나눈다.
#    - TOF  : 스케일 없음. 정수가 그대로 cm
# -----------------------------------------------------------------------------
SCALE_MOTOR_CURRENT_A = 0.01   # BLDC 전류: int16, 0.01 A/LSB

#  [2026-08-29 신규 명세] BAT 전압: int16 big-endian, 0.1 V/LSB
#  명세 전송 범위 0~566 은 raw 값이고 실제로는 0~56.6 V 를 뜻한다.
#  실측 raw 369~370 -> 36.9~37.0 V (36V 계통과 맞는다)
SCALE_BAT_VOLTAGE_V = 0.1


# =============================================================================
#  SECTION 3. 송신용 상수 (MAIN -> 하위 ECU)
# =============================================================================

# --- 모터 제어 모드 (0x200 byte 4) ------------------------------------------
MOTOR_MODE_IDLE      = 0   # 정지 (복구 가능한 통신 문제 시 사용)
MOTOR_MODE_RUN       = 1   # 정상 주행
MOTOR_MODE_FREE      = 2   # 프리휠 (무동력, 수동으로 밀 수 있는 상태)
MOTOR_MODE_EMERGENCY = 3   # 비상정지 (E-stop / 하드웨어 Fault 시 사용)

MOTOR_MODE_NAMES = {
    MOTOR_MODE_IDLE:      'IDLE',
    MOTOR_MODE_RUN:       'RUN',
    MOTOR_MODE_FREE:      'FREE',
    MOTOR_MODE_EMERGENCY: 'EMERGENCY',
}


def get_motor_mode_name(mode: int) -> str:
    """제어 모드 번호를 로그용 문자열로 변환."""
    return MOTOR_MODE_NAMES.get(mode, f'알수없음(0x{mode:02X})')


# --- Fault Clear 비트필드 (0x200 byte 5) ------------------------------------
#  평상시에는 0을 보내고, Fault를 지울 때만 해당 비트를 세운다.
MOTOR_CLEAR_M0  = 0x01  # bit0 : M0(좌) 클리어
MOTOR_CLEAR_M1  = 0x02  # bit1 : M1(우) 클리어
MOTOR_CLEAR_ALL = 0x04  # bit2 : 전체 클리어


# =============================================================================
#  SECTION 4. 수신용 상수 (하위 ECU -> MAIN)
# =============================================================================

# --- 모터 오류 코드 (0x210/0x211 byte 5) ------------------------------------
MOTOR_FAULT_NONE         = 0   # 정상
MOTOR_FAULT_OVERCURRENT  = 1   # 과전류
MOTOR_FAULT_OVERHEAT     = 2   # 과열
MOTOR_FAULT_UNDERVOLTAGE = 3   # 저전압
MOTOR_FAULT_ESTOP        = 4   # E-stop
MOTOR_FAULT_HALL         = 5   # HALL 센서 이상

MOTOR_FAULT_NAMES = {
    MOTOR_FAULT_NONE:         '정상',
    MOTOR_FAULT_OVERCURRENT:  '과전류',
    MOTOR_FAULT_OVERHEAT:     '과열',
    MOTOR_FAULT_UNDERVOLTAGE: '저전압',
    MOTOR_FAULT_ESTOP:        'E-stop',
    MOTOR_FAULT_HALL:         'HALL fault',
}


def get_motor_fault_name(code: int) -> str:
    """Fault 코드를 로그·UI용 한글 문자열로 변환.
    정의되지 않은 코드는 16진수 그대로 보여준다."""
    return MOTOR_FAULT_NAMES.get(code, f'알수없음(0x{code:02X})')


# --- 모터 상태 플래그 비트 (0x210/0x211 byte 6) -----------------------------
#  [!] bit1 "PI"의 정확한 의미(PI 제어기 동작 중으로 추정)는 펌웨어팀 확인 필요.
MOTOR_STATE_ACTIVE   = 0x01  # bit0 : 활성화
MOTOR_STATE_PI       = 0x02  # bit1 : PI 제어 동작 중 (추정)
MOTOR_STATE_ROTATING = 0x04  # bit2 : 회전 중


def motor_state_active(flags: int) -> bool:
    """모터가 활성화 상태인지."""
    return bool(flags & MOTOR_STATE_ACTIVE)


def motor_state_rotating(flags: int) -> bool:
    """모터가 실제로 회전 중인지."""
    return bool(flags & MOTOR_STATE_ROTATING)


# =============================================================================
#  SECTION 5. 송신 — pack_* (ROS2 데이터 -> CAN 바이트)
# =============================================================================

def pack_motor_cmd(rpm_left: int, rpm_right: int, mode: int,
                   fault_clear: int = 0) -> bytes:
    """0x200 : MAIN(CanManager) -> BLDC 모터 명령.   [배치 확정]

    byte 0-1 : int16 LE  M0(좌) 목표 RPM  (LSB=1, 범위 -3000~+3000, 음수=후진)
    byte 2-3 : int16 LE  M1(우) 목표 RPM  (동일)
    byte 4   : uint8     제어 모드   (0=IDLE 1=RUN 2=FREE 3=EMERGENCY)
    byte 5   : uint8     Fault Clear (bit0=M0 bit1=M1 bit2=전체) — 평상시 0
    byte 6-7 : reserved  (0 고정)

    struct 포맷 '<hhBBBB' 대응:
        <  = little-endian
        h  = byte0-1 (좌 RPM),  h = byte2-3 (우 RPM)
        B  = byte4 (모드),      B = byte5 (fault clear)
        BB = byte6-7 (reserved)
    """
    return struct.pack('<hhBBBB',
                       _clamp_i16(rpm_left),    # byte 0-1
                       _clamp_i16(rpm_right),   # byte 2-3
                       mode & 0xFF,             # byte 4
                       fault_clear & 0xFF,      # byte 5
                       0, 0)                    # byte 6-7 (reserved)


# =============================================================================
#  SECTION 6. 수신 — unpack_* (CAN 바이트 -> 파이썬 dataclass)
# =============================================================================

@dataclass
class MotorStatusFrame:
    """0x210 / 0x211 파싱 결과."""
    motor_id: int           # 0 = M0(좌), 1 = M1(우)
    rpm: int                # 실제 RPM (음수 = 후진)
    current_a: float        # 최대 상전류 [A]
    temperature_c: float    # NTC 온도 [°C]
    fault_code: int         # MOTOR_FAULT_* 참조
    state_flags: int        # MOTOR_STATE_* 비트 참조


@dataclass
class BatteryStatusFrame:
    """0x310 파싱 결과."""
    current_a: float        # 음수 = 방전 (ROS BatteryState 관례에 맞춤)
    temperature_c: float    # 36V 파워 라인 온도 [°C]
    voltage_v: float        # 팩 전압 [V]
    soc_percent: int        # 잔량 [%]
    error: bool             # Error Interrupt (ECU 단 에러 유무)
    seq: int = 0            # 프레임 카운터 0~255 (프레임 유실 검출용)


@dataclass
class TofStatusFrame:
    """0x410 파싱 결과."""
    total_cm: int                       # 6개 센서 평균 거리 [cm]
    sensor_cm: list = field(default_factory=list)  # 길이 6, 센서1~6 순서


def unpack_motor_status(can_id: int, data: bytes) -> MotorStatusFrame:
    """0x210 / 0x211 : BLDC 모터 상태.   [배치 확정]

    M0(0x210) = 좌측 모터, M1(0x211) = 우측 모터. 각각 100ms 주기 수신.

    byte 0-1 : int16  RPM         (LSB=1, 범위 -3000~+3000, 음수=후진)
    byte 2-3 : int16  최대 상전류 (LSB=0.01A, 범위 0~327 -> 0~3.27A)
    byte 4   : int8   온도        (LSB=1°C, 범위 0~100)
    byte 5   : int8   Fault 코드  (MOTOR_FAULT_* 참조)
    byte 6   : int8   상태 플래그 (MOTOR_STATE_* 비트)
    byte 7   : uint8  예약 (명세서에 정의 없음)

    struct 포맷 '<hhbbBB' 대응:
        h h = byte0-1(RPM), byte2-3(전류)
        b b = byte4(온도), byte5(fault)   ← 온도는 음수 가능성 고려해 signed
        B B = byte6(플래그), byte7(예약)
    """
    # 프레임이 8바이트보다 짧게 와도 파싱이 깨지지 않도록 0으로 패딩
    if len(data) < 8:
        data = bytes(data) + bytes(8 - len(data))

    rpm, cur_raw, temp, fault, state, _ = struct.unpack('<hhbbBB', data[:8])

    return MotorStatusFrame(
        motor_id=0 if can_id == CAN_ID_MOTOR_STATUS0 else 1,
        rpm=rpm,
        current_a=cur_raw * SCALE_MOTOR_CURRENT_A,
        temperature_c=float(temp),
        fault_code=fault,
        state_flags=state,
    )


# =============================================================================
#  ★★ [2026-08-29] BAT 프레임 배치를 새 명세로 전면 교체했다 ★★
#
#  옛 코드는 `<hBhbbB` (little-endian · byte2=온도 · byte3-4=전압) 였고,
#  실측값을 넣으면 **전압 11378V · 전류 32A** 라는 불가능한 값이 나왔다.
#  사용자가 받아 온 새 명세표(~/Downloads/수정이미지.png)로 고쳤다.
#
#  ★ 실측 316프레임으로 검증한 것 (2026-08-29) ★
#    - **big-endian 확정.** little-endian 이면 전류 -32512~-31744 mA,
#      byte2-3 = 28929~29185 로 **둘 다 명세 범위 밖**이다.
#      big-endian 이면 전류 129~132 mA · 전압 raw 369~370 으로 **둘 다 범위 안**.
#    - **byte7 은 seq 가 맞다.** 316프레임 전부 직전값 +1 (mod 256) 이었다.
#    - 전압 raw 369~370 x0.1 = **36.9~37.0 V** — 36V 계통과 맞는다.
#    - 온도 44~46°C · SoC 100% · error 0.
#
#  ★ [2026-08-29 사용자 확인] byte2-3 = 전압 · byte4 = 온도 로 확정됐다 ★
#    표의 열 밀림 때문에 두 갈래로 읽혔으나 실측과 사용자 확인이 같은 답이었다.
#    **명세표 자체는 여전히 밀려 있으니 표만 보고 고치지 말 것.**
#
#  ★ 아직 확정 안 된 것 하나 ★
#    전압 스케일 x0.1 은 명세에 없다. 범위 0~566 과 예시 36[V] 에서 역산했고
#    실측(raw 369~370 -> 36.9~37.0V)이 36V 계통과 맞았을 뿐이다.
#    **멀티미터로 팩 전압을 한 번 재면 닫힌다.**
#
#  ★ 주기도 명세와 다르다 ★ "1s 가정"이 아니라 **실측 101.2ms**
#    (316프레임/32초 · min 96.7 / max 105.6). can_params.yaml 의
#    battery_status_timeout: 3.0 은 실제 주기의 30배라 두절 감지가 늦다.
# =============================================================================
def unpack_battery_status(data: bytes) -> BatteryStatusFrame:
    """0x310 : BAT ECU 상태.   [2026-08-29 새 명세로 전면 수정]

    ★ 배치가 통째로 바뀌었다 ★ 옛 코드는 byte2=온도(uint8) · byte3-4=전압 이었고
    little-endian 이었다. 새 명세는 byte2-3=전압(int16) · byte4=온도(int8) 이고
    **big-endian** 이다. 옛 코드로 읽으면 전압 11378V · 전류 32A 가 나왔다.

    byte 0-1 : int16  소모 전류 [mA]  (범위 -32000~32000, 음수=충전 방향)
    byte 2-3 : int16  배터리 전압     (범위 0~566 raw, x0.1 -> 0~56.6 V)
    byte 4   : int8   배터리 온도 [°C] (범위 -40~125)
    byte 5   : int8   배터리 잔량 [%]  (범위 0~100)
    byte 6   : int8   Error 검출 플래그 (0=OK, 1=Error)
    byte 7   : uint8  seq — 프레임 카운터 (0~255, 매 프레임 +1)

    ★ 엔디안은 big-endian 이다 (2026-08-29 실측 316프레임으로 확정) ★
      little-endian 으로 읽으면 전류 -32512~-31744 mA, byte2-3 = 28929~29185 로
      **둘 다 명세 범위를 벗어난다.** big-endian 이면 전류 129~132 mA,
      전압 raw 369~370 (36.9~37.0 V) 로 **둘 다 범위 안**이다.

    ★ byte2-3 = 전압 · byte4 = 온도. 2026-08-29 사용자 확인 완료. ★
    [!] 다만 **받은 명세표 자체는 열이 한 줄 밀려 있다** — `데이터`/`전송 범위`/`형식`
        열은 위와 같이 읽히는데 `단위`/`Ex`/`설명` 열은 그 반대다.
        **표만 보고 고치면 전압과 온도가 뒤바뀐다.** 다음 사람을 위해 남겨 둔다.
        (실측도 같은 답이었다: byte4 를 전압으로 보면 44~46V 라 36V 만충 42V 를 넘어 성립 안 함)

    [!] 전압 스케일(x0.1)은 명세에 명시돼 있지 않다. 전송 범위 0~566 과
        예시 "36 [V]" 로부터 역산한 값이고 실측(raw 370 -> 37.0V)과 맞는다.
        **멀티미터로 한 번 재서 확정할 것.**
    """
    if len(data) < 8:
        data = bytes(data) + bytes(8 - len(data))

    #  '>' = big-endian / h h = byte0-1(전류) byte2-3(전압)
    #  b b b = byte4(온도) byte5(SoC) byte6(error) / B = byte7(seq)
    cur_ma, volt_raw, temp, soc, error, seq = struct.unpack('>hhbbbB', data[:8])

    return BatteryStatusFrame(
        # "소모 전류"이므로 양수 = 방전이다. ROS BatteryState 는 방전이 음수라
        # 부호를 반전한다.
        # [2026-08-29] 새 명세의 범위가 -32000~32000 으로 **부호가 있다**.
        # 옛 명세(0~2000)와 달리 방향 정보가 있으므로, 반전만 해 두면
        # 소모(+) -> ROS 음수(방전) · 충전(-) -> ROS 양수(충전) 로 둘 다 맞는다.
        current_a=-(cur_ma / 1000.0),   # mA -> A 변환 + 부호 관례 변환
        temperature_c=float(temp),
        voltage_v=float(volt_raw) * SCALE_BAT_VOLTAGE_V,
        soc_percent=soc,
        error=bool(error),
        seq=seq,
    )


def unpack_tof_status(data: bytes) -> TofStatusFrame:
    """0x410 : TOF 센서 ECU 상태.   [배치 확정]

    BAT와 마찬가지로 스케일 없이 정수값이 그대로 cm 단위다.

    byte 0 : uint8  total 거리 (6개 평균) [cm] (범위 0~255)
    byte 1 : uint8  센서1 거리 [cm]
    byte 2 : uint8  센서2 거리 [cm]
    byte 3 : uint8  센서3 거리 [cm]
    byte 4 : uint8  센서4 거리 [cm]
    byte 5 : uint8  센서5 거리 [cm]
    byte 6 : uint8  센서6 거리 [cm]
    byte 7 : uint8  예약 (명세서에 정의 없음)

    [!] 명세서엔 int8로 적혀 있으나 범위(0~255)상 uint8이 맞다고 판단.
    [!] 센서 6개가 로봇 몸체에 어떻게 배치되어 있는지(전/후/좌/우 등)
        명세서에 없다. TaskManager에서 방향별 회피 로직을 짜려면 배치도 필요.
    """
    if len(data) < 8:
        data = bytes(data) + bytes(8 - len(data))

    total, s1, s2, s3, s4, s5, s6, _ = struct.unpack('<BBBBBBBB', data[:8])

    return TofStatusFrame(
        total_cm=total,
        sensor_cm=[s1, s2, s3, s4, s5, s6],
    )


# =============================================================================
#  SECTION 7. 내부 헬퍼
# =============================================================================

def _clamp_i16(v) -> int:
    """int16 범위로 안전하게 자른다 (struct.pack 오버플로 예외 방지)."""
    return max(-32768, min(32767, int(round(v))))
