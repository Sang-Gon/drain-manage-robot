# BLDC ECU 모터 1개의 상태 (0x210 / 0x211에서 변환)

uint8   motor_id       # 0 = M0(좌), 1 = M1(우)
int16   rpm            # 실제 RPM (음수 = 후진)
float32 current        # 최대 상전류 [A]
float32 temperature    # NTC 온도 [degC]
uint8   fault_code     # 0=정상 1=과전류 2=과열 3=저전압 4=E-stop 5=HALL fault
uint8   state_flags    # bit0=활성 bit1=PI bit2=회전중
bool    stale          # true면 타임아웃 초과 (값을 믿으면 안 됨)
