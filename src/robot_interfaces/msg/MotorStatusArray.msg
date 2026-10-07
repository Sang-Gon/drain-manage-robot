# 좌/우 모터 상태를 한 메시지로 묶어 발행

std_msgs/Header header
MotorStatus[] motors    # 보통 길이 2 (M0, M1 순서)
