# TOF 거리 센서 ECU 상태 (0x410에서 변환)
#
# [주의] 센서 6개의 물리적 배치(전/후/좌/우 등)는 아직 미확정이다.
#        여기서는 명세서 표의 순서 그대로 전달하며, 방향 해석은
#        TaskManager가 배치도를 근거로 수행해야 한다.

std_msgs/Header header
uint8    total_distance      # 6개 센서 평균 거리 [cm]
uint8[6] sensor_distances    # 센서1~6 거리 [cm]
bool     stale               # true면 타임아웃 초과
