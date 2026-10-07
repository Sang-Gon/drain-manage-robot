# CAN 버스 및 각 ECU 통신 상태 진단 (1Hz 발행)

std_msgs/Header header
bool   bus_ok               # CAN 소켓이 정상 오픈 상태인지
uint32 rx_count             # 누적 수신 프레임 수
uint32 tx_count             # 누적 송신 프레임 수
uint32 error_count          # 누적 에러 (송신 실패 + 에러 프레임)
uint32 bus_off_recoveries   # bus-off 자동 복구 시도 횟수

# 각 ECU별 프레임 신선도. true = 타임아웃 초과 (해당 ECU 통신 두절 의심)
bool   motor_stale
bool   battery_stale
bool   tof_stale
