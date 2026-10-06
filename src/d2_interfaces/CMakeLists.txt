cmake_minimum_required(VERSION 3.8)
project(d2_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# 정본은 이 파일들이다 — IRD 5장과 다르면 여기가 맞다. 칸을 바꾸면 받는 파트 확인 뒤 merge
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PickPlace.action"
  "srv/GripperCommand.srv"
  "srv/CheckProgress.srv"
  "srv/HmiCommand.srv"
  "srv/SceneAttach.srv"
  "srv/StopRequest.srv"
  "srv/MoveTo.srv"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
