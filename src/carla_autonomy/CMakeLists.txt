cmake_minimum_required(VERSION 3.8)
project(carla_autonomy)

# C++ 표준 설정
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 1. 필요한 패키지들 불러오기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(carla_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 2. 실행 파일 생성 및 의존성 연결
# (1) Lattice 플래너 연동 노드
add_executable(master_control_node src/control/master_control_node.cpp)
ament_target_dependencies(master_control_node
  rclcpp
  nav_msgs
  std_msgs
  carla_msgs
  tf2
  tf2_geometry_msgs
  geometry_msgs
)

# 3. 설치 설정
# (1) C++ 실행 파일들 설치 (ros2 run 명령어로 실행하기 위해 필요)
install(TARGETS
  master_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# (2) 파이썬 스크립트 폴더 전체 설치 (인지, 판단, 기타 유틸리티 파일들)
install(DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

# (3) launch 파일과 스크립트 리소스 설치
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  USE_SOURCE_PERMISSIONS
)

# (4) 모델 파일 설치 (share/carla_autonomy/models)
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# 5. 패키지 구성 마무리
ament_package()
