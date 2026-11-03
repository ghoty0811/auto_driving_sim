# 기반 오픈소스 및 로컬 변경 기록
원본 Git 저장소의 현재 HEAD를 기록했습니다. 이 커밋은 팀 개발 시작점을 보장하지 않습니다. 아래 변경 목록은 각 HEAD 대비 현재 파일의 차이이며, 개인 기여도를 의미하지 않습니다.
각 디렉터리의 LICENSE 및 기존 저작권 고지는 그대로 유지합니다.
## DynamixelSDK
- 원본: https://github.com/ROBOTIS-GIT/DynamixelSDK.git
- 기준 HEAD: `b5dd448848fabea636bea559653b1da0e671385a`
- 라이선스: [DynamixelSDK/LICENSE](../src/DynamixelSDK/LICENSE)

HEAD 대비 변경 파일 없음.
## turtlebot3
- 원본: https://github.com/ROBOTIS-GIT/turtlebot3.git
- 기준 HEAD: `6b7a53d57a9901c06114710d8c710bb2529c938d`
- 라이선스: [turtlebot3/LICENSE](../src/turtlebot3/LICENSE)

HEAD 대비 변경 파일 없음.
## turtlebot3_autorace
- 원본: https://github.com/ROBOTIS-GIT/turtlebot3_autorace.git
- 기준 HEAD: `9d56c944d0d39940f54914e3f53ea37472206154`
- 라이선스: [turtlebot3_autorace/LICENSE](../src/turtlebot3_autorace/LICENSE)

로컬 변경·추가 파일:

- `turtlebot3_autorace_camera/calibration/extrinsic_calibration/projection.yaml`
- `turtlebot3_autorace_detect/launch/detect_lane.launch.py`
- `turtlebot3_autorace_detect/launch/detect_sign.launch.py`
- `turtlebot3_autorace_detect/package.xml`
- `turtlebot3_autorace_detect/param/traffic_light/traffic_light.yaml`
- `turtlebot3_autorace_detect/setup.py`
- `turtlebot3_autorace_detect/turtlebot3_autorace_detect/detect_construction_sign.py`
- `turtlebot3_autorace_detect/turtlebot3_autorace_detect/detect_intersection_sign.py`
- `turtlebot3_autorace_detect/turtlebot3_autorace_detect/detect_lane.py`
- `turtlebot3_autorace_detect/turtlebot3_autorace_detect/detect_parking_sign.py`
- `turtlebot3_autorace_detect/turtlebot3_autorace_detect/detect_traffic_light.py`
- `turtlebot3_autorace_mission/launch/control_lane.launch.py`
- `turtlebot3_autorace_mission/launch/mission_construction.launch.py`
- `turtlebot3_autorace_mission/setup.py`
- `turtlebot3_autorace_mission/turtlebot3_autorace_mission/avoid_construction.py`
- `turtlebot3_autorace_mission/turtlebot3_autorace_mission/control_lane.py`
- `turtlebot3_autorace_detect/resource/best.pt`
- `turtlebot3_autorace_detect/turtlebot3_autorace_detect/pub_model.py`
- `turtlebot3_autorace_mission/turtlebot3_autorace_mission/crosswalk_stopper.py`
## turtlebot3_msgs
- 원본: https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
- 기준 HEAD: `cdee60f7618b9ed2f85f65259c0f96e3f02b1c55`
- 라이선스: [turtlebot3_msgs/LICENSE](../src/turtlebot3_msgs/LICENSE)

HEAD 대비 변경 파일 없음.
## turtlebot3_simulations
- 원본: https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
- 기준 HEAD: `a35a56c8b04877dc89772b598084d8ce648a9023`
- 라이선스: [turtlebot3_simulations/LICENSE](../src/turtlebot3_simulations/LICENSE)

로컬 변경·추가 파일:

- `turtlebot3_gazebo/models/turtlebot3_autorace_2020/course/materials/textures/course.png`
- `turtlebot3_gazebo/worlds/turtlebot3_autorace_2020.world`
- `turtlebot3_gazebo/launch/custom_track_world.launch.py`
- `turtlebot3_gazebo/launch/turtlebot3_autorace_2025.launch.py`
- `turtlebot3_gazebo/models/turtlebot3_autorace_2020/course/materials/textures/first_course.png`
- `turtlebot3_gazebo/worlds/custom_track.world`
- `turtlebot3_gazebo/worlds/turtlebot3_autorace_2025.world`

## 포트폴리오 정리 시 변경

YOLO 모델의 사용자 홈 절대 경로를 설치된 패키지의 resource 경로로 바꾸고, `model_path` ROS 파라미터로 덮어쓸 수 있도록 했습니다. 루트 README, 공개·실행 가이드, 발표 자료 사본 및 소스 내보내기 도구를 추가했습니다.
