# 실행 가이드

아래는 현재 launch 파일 기준의 실행 예시입니다. 전체 시뮬레이션 실행을 검증한 절차는 아닙니다.

## 준비 및 빌드

ROS 2 Humble, colcon, rosdep과 Gazebo Classic용 gazebo_ros 환경을 준비합니다. 별도 Python 환경을 쓸 경우 ROS의 cv_bridge와 Python 버전 호환을 확인해야 합니다. Ultralytics 의존성 버전은 원본에 고정되어 있지 않습니다.

```bash
cd turtlebot3_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install ultralytics
colcon build --symlink-install
source install/setup.bash
```

`rosdep`만으로 Python 외부 라이브러리까지 모두 설치되지는 않습니다. 전체 워크스페이스에는 navigation 및 manipulation 관련 패키지도 포함되어 있으므로 빌드 오류의 누락 의존성을 확인해야 합니다.

## 시뮬레이터

```bash
export TURTLEBOT3_MODEL=burger_cam
export GAZEBO_MODEL_PATH="$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/models:${GAZEBO_MODEL_PATH}"
ros2 launch turtlebot3_gazebo custom_track_world.launch.py
```

현재 `worlds/custom_track.world`에 `model://custom_track` 참조가 있으나 해당 모델이 저장소에 없습니다. 원본 트랙 모델을 확보해 GAZEBO_MODEL_PATH에 추가해야 합니다. 위 모델 선택은 포함된 카메라 모델 기준 예시이며, 원래 데모의 모델과 시작 좌표는 추가 확인이 필요합니다.

## 인식 및 제어

아래 명령은 각각 별도 터미널에서 실행합니다. 모든 터미널에서 ROS와 워크스페이스의 setup.bash를 먼저 불러오세요.

```bash
ros2 launch turtlebot3_autorace_camera intrinsic_camera_calibration.launch.py
ros2 launch turtlebot3_autorace_camera extrinsic_camera_calibration.launch.py
ros2 launch turtlebot3_autorace_detect detect_lane.launch.py
ros2 run turtlebot3_autorace_detect pub_model
ros2 launch turtlebot3_autorace_detect detect_sign.launch.py
ros2 launch turtlebot3_autorace_mission control_lane.launch.py
```

`detect_sign.launch.py`는 교차로·공사·주차 노드를 실행합니다. `control_lane.launch.py`는 주행 제어·횡단보도·신호등 노드를 실행합니다. 터널 감속 상태 처리는 `control_lane.py`에 포함되어 있습니다. `mission_tunnel`은 별도의 navigation 기반 코드이므로 위 조합에 임의로 추가하지 않습니다.

YOLO 가중치는 패키지의 `resource/best.pt`를 기본으로 사용합니다. 별도 파일 지정:

```bash
ros2 run turtlebot3_autorace_detect pub_model --ros-args -p model_path:=/absolute/path/to/best.pt
```

## 실행 확인과 알려진 한계

```bash
ros2 topic list
ros2 topic hz /camera/image_compensated
ros2 topic echo /detect/model_results
ros2 topic echo /emergency_stop
ros2 topic echo /cmd_vel
```

- 카메라 입력 `/camera/image_raw/compressed`, 보정 이미지 `/camera/image_rect_color`, 투영 이미지 `/camera/image_projected`의 연결을 먼저 확인합니다.
- 카메라 정보 및 영상 transport가 맞지 않으면 intrinsic/extrinsic launch를 실행해도 영상이 전달되지 않을 수 있습니다.
- 신호등 미검출을 초록불로 취급하는 동작과 고정 시간 기반 터널 제어는 시뮬레이션 구현의 한계입니다.
- YOLO 학습 코드·원본 데이터셋·학습 설정·정량 평가 자료는 이 폴더에서 확인되지 않았습니다.
- 이번 포트폴리오 정리에서는 Python 구문, 패키징 경로, 공개용 묶음 구조를 점검했습니다. colcon 전체 빌드와 Gazebo 실행, 주행 성능 검증은 수행하지 않았습니다.
