# 한 프레임의 탐지 결과 전부.
#
# header.stamp 는 원본 영상의 stamp 를 그대로 물려준다.
# 서보잉 노드는 이 stamp 로 신선도를 판단해야 한다.
# 탐지가 없으면 detections 가 빈 배열로 발행된다. 발행 자체를 거르지 않는다.
# (거르면 소비자가 "탐지 없음"과 "노드 죽음"을 구별할 수 없다)

std_msgs/Header header

uint32 image_width
uint32 image_height

DrainDetection[] detections
