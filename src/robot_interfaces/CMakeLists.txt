cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 메시지/서비스를 추가하면 아래 목록에도 반드시 등록할 것
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorStatus.msg"
  "msg/MotorStatusArray.msg"
  "msg/TofStatus.msg"
  "msg/CanHealth.msg"
  "msg/DrainDetection.msg"
  "msg/DrainDetectionArray.msg"
  "msg/TaskState.msg"
  "srv/ClearMotorFault.srv"
  "srv/AssessDrain.srv"
  "srv/CaptureDrain.srv"
  "srv/ReportDrain.srv"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_package()
