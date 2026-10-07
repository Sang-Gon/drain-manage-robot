# TaskManager 의 현재 임무 상태.  감시와 사후 기록용
#
# [주의] 이 파일의 주석에 줄 끝 역슬래시를 쓰지 말 것.

std_msgs/Header header

string state          # BOOT_WAIT / DRIVING / SETTLE / MOUNTING / MEASURING /
                      # ADVANCING / CAPTURING / REPORTING / DEPARTING /
                      # DONE / PAUSED / FAULT / ABORT
                      # MOUNTING 은 서보가 선 자리에서 배수구 위로 n 만큼
                      # 더 들어가는 구간이다 (2026-08-24 신설)

uint32 sequence       # 지금 처리 중인 배수구 순번 (1부터, 아직이면 0)

# verdict 값은 AssessDrain 과 같은 번호를 쓴다
uint8 VERDICT_UNKNOWN=0
uint8 VERDICT_NOT_NEEDED=1
uint8 VERDICT_NEEDED=2
uint8 verdict         # 아직 판정 전이면 VERDICT_UNKNOWN

# 지금 mux 로 내보내고 있는 게이트 값. 상태와 어긋나면 버그다
bool allow_drive
bool servo_enable
bool task_cmd_active

string detail         # 사람이 읽는 한 줄 (카운트다운 · 실패 사유 등)
