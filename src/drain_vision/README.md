# drain_vision — 카메라 + Hailo YOLOv11s 배수로 탐지

비전팀이 **다른 라즈베리파이5에서 개발한 `my_opencv_tutorials`** 를 Main ECU 워크스페이스
(`~/ros2_ws`)에 맞춰 정리한 패키지. `can_manager`·`sllidar_ros2` 와 같은 워크스페이스에
나란히 들어간다.

```
 sllidar_ros2 ──/scan──────> drain_lidar_avoidance ──/cmd_vel──> can_manager ──CAN──> 하위 ECU
                                                                      ▲
 drain_vision ──/image_raw──> drain_detector ──/detections/image──    ?
                                                          아직 연결 안 됨 (아래 '미해결' 3번)
```

현재 비전 스택은 **탐지 결과를 그림으로만 내보낸다.** 주행 로직에 물리지 않았다.

## 구성

```
~/ros2_ws/src/drain_vision/
├── drain_vision/
│   ├── camera_publisher.py     ← USB 웹캠 → /image_raw          (Hailo 없어도 동작)
│   └── drain_detector.py       ← /image_raw → /detections/image (Hailo 필요)
├── config/
│   ├── camera_params.yaml
│   └── detector_params.yaml
├── launch/
│   ├── camera.launch.py        ← 카메라만
│   └── detection.launch.py     ← 카메라 + 탐지
├── models/yolov11s_v2.hef      ← 18.6MB, 1클래스(drain), 640x640, NMS 온칩 (현재 모델 · 2026-08-29)
├── models/yolov11s_drain.hef   ← 19MB, 옛 모델. 롤백용으로 남겨 둠
├── deploy/check_hailo.sh       ← Hailo 장착 후 점검 (읽기 전용)
├── package.xml / setup.py / setup.cfg
└── README.md                   ← 이 파일
```

### 원본에서 바꾼 것

| 원본 | 여기 | 이유 |
|---|---|---|
| 패키지 `my_opencv_tutorials` | `drain_vision` | 임시 이름이었음. `drain_lidar_avoidance` 와 짝 |
| 노드 `img_publisher` | `camera_publisher` | 하는 일이 이름에 드러나게 |
| 노드 `yolo_detector` | `drain_detector` | 위와 같음 |
| `*.launch.xml` | `*.launch.py` | `can_manager` 와 형식 통일. 런치 인자 추가 |
| `cv_params.yaml` | `camera_params.yaml` | 이름만 |
| `yolo_params.yaml` | `detector_params.yaml` | 이름만 |
| 토픽 하드코딩 | 파라미터 | `/image_raw`, `/detections/image` 기본값은 그대로 |
| `hailo_platform` 맨몸 import | try/except 로 감쌈 | 모듈 없을 때 설치 안내가 나오게 |

**가져오지 않은 것** — `build/`·`install/`·`log/`·`hailort*.log` (빌드 산출물),
`test/test_{copyright,flake8,pep257}.py` (기본 템플릿. `can_manager` 도 안 갖고 있음),
`bringup_usb_cam.launch.xml` (`usb_cam` 패키지를 쓰는 대안 경로. `camera_publisher` 와
역할이 겹치고 `usb_cam` 은 이 Pi에 설치도 안 돼 있어 제외).

## 설치가 필요한 것

### ROS·OpenCV 쪽 — **추가 설치 없음**

`cv_bridge`, `python3-opencv`(4.6.0), `numpy`(1.26.4), `rqt_image_view` 전부 이 Pi에
이미 있다. `camera_publisher` 는 지금 당장 돈다 (2026-08-20 실측 확인).

있으면 편한 것 (필수 아님):

```bash
sudo apt install v4l-utils                          # v4l2-ctl 로 카메라 포맷 확인
sudo apt install ros-jazzy-image-transport-plugins  # 원격에서 영상 볼 때 대역폭 절약
```

### Hailo 쪽 — **전부 미설치. 아래 순서대로**

> `deploy/check_hailo.sh` 를 돌리면 어디까지 됐는지 항목별로 나온다.
> 2026-08-20 기준 7개 항목 전부 FAIL (모듈 미장착 상태).

**① PCIe 슬롯 켜기** — `/boot/firmware/config.txt` 에 추가 후 재부팅

```
dtparam=pciex1
pciex1_gen=3        # Hailo-8 권장. 링크가 불안정하면 이 줄만 빼서 gen2로 떨어뜨린다
```

**② 드라이버 빌드 전제조건** (둘 다 현재 없음)

```bash
sudo apt install dkms linux-headers-$(uname -r)     # → linux-headers-6.8.0-1060-raspi
```

**③ HailoRT — Hailo Developer Zone 에서 직접 받아야 한다** (로그인 필요, apt 저장소 없음)

| 받을 것 | 설치 |
|---|---|
| `hailort-pcie-driver_<VER>_all.deb` | `sudo dpkg -i` → dkms 로 `hailo_pci` 빌드 |
| `hailort_<VER>_arm64.deb` | `sudo dpkg -i` → `hailortcli`, libhailort |
| `hailort-<VER>-cp312-cp312-linux_aarch64.whl` | `pip install --user <whl>` |

- **`<VER>` 는 `4.23.0`** 으로 맞출 것. 넘겨받은 `.hef` 를 컴파일한 펌웨어가 4.23.0 이다
  (원본 Pi의 `hailort.log` 에서 확인: `firmware_version is: 4.23.0`).
  런타임과 `.hef` 버전이 어긋나면 로드가 깨진다.
- 파이썬 휠은 **cp312** (이 Pi는 Python 3.12.3).
- `sudo apt install hailo-all` 은 **Raspberry Pi OS 전용**이다. 이 Pi는 Ubuntu 24.04라 안 통한다.
  원본 Pi도 같은 이유로 `pip --user` 로 깔았고, 그래서 `drain_detector.py` 가
  `site.getusersitepackages()` 를 `sys.path` 에 직접 넣는다.

**④ 확인**

```bash
bash ~/ros2_ws/src/drain_vision/deploy/check_hailo.sh
```

### ⚠ 하드웨어 간섭 — 먼저 확인할 것

이 Pi의 CAN은 **MCP2515 SPI HAT** 으로 40핀 헤더에 물려 있다
(`config.txt`: `dtoverlay=mcp2515,spi0-1,oscillator=16000000,interrupt=25`).
Hailo M.2 HAT 도 같은 헤더 자리에 얹힌다. **적층 순서와 GPIO 패스스루가 되는지
물리적으로 먼저 맞춰볼 것.** CAN이 죽으면 모터 제어가 통째로 멈춘다.

## 실행

```bash
cd ~/ros2_ws && colcon build --packages-select drain_vision && source install/setup.bash

# 카메라만 (Hailo 없어도 됨)
ros2 launch drain_vision camera.launch.py
ros2 launch drain_vision camera.launch.py camera_device:=2

# 카메라 + 탐지 (Hailo 필요)
ros2 launch drain_vision detection.launch.py
ros2 launch drain_vision detection.launch.py conf_threshold:=0.5

# 영상 보기
ros2 run rqt_image_view rqt_image_view /detections/image
```

카메라 장치 번호: 현재 붙은 Logitech C270 은 `/dev/video0`.
`video1` 도 같은 카메라가 만들지만 메타데이터 노드라 영상이 안 나온다. **0을 쓸 것.**

## 검증 완료 (2026-08-20, 이 Pi)

- `colcon build` 성공, 실행파일 2개 등록, `.hef` 19MB가 share/models 로 설치됨
- `camera.launch.py` 기동 → `/image_raw` 발행. **10.001 Hz** (min 0.098 / max 0.102s),
  `640x480 bgr8`, `frame_id: camera`
- Hailo 없이 `drain_detector` 실행 → 설치 안내 메시지 출력 후 정상 종료 (의도된 동작)

## 미해결

| # | 내용 | 상태 |
|---|---|---|
| 1 | **Hailo 모듈 미장착** — 탐지 노드는 이 Pi에서 한 번도 안 돌았다. 원본 Pi의 성능(FPS·지연)도 넘겨받은 자료에 없음 | 모듈 장착 대기 |
| 2 | **웹캠 버퍼 지연** — 카메라는 30fps로 도는데 노드는 10Hz로 `read()` 한다. V4L2 내부 버퍼에 프레임이 쌓여 **최신 프레임이 아닐 수 있다.** 움직이는 로봇에서는 그만큼 탐지가 늦는다. `CAP_PROP_BUFFERSIZE=1` 또는 별도 grab 스레드로 고쳐야 함 | 미수정 (원본 동작 보존) |
| 3 | **탐지 결과가 주행에 안 물림** — 그려진 그림(`/detections/image`)만 나가고 좌표를 담은 메시지가 없다. `robot_interfaces` 에 `Detection` msg를 추가해 `drain_lidar_avoidance` 나 상위 로직이 받게 해야 함 | 설계 필요 |
| 4 | **TF 없음** — `base_link → camera` 프레임이 없다. 라이다(`base_link → laser`)와 같은 문제 | 카메라 장착 위치 실측 필요 |
| 5 | 모델 학습 데이터·클래스 정의·정확도 지표를 넘겨받지 못함. `.hef` 바이너리만 있음 | 비전팀 문의 |
