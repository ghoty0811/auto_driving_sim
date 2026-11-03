# GitHub 공개 가이드

추천 저장소 이름: `turtlebot3-autonomous-driving`

추천 소개: `ROS 2와 Gazebo 기반 TurtleBot3 자율주행 — YOLO 표지판 인식, 차선 추종, 공사 구간 회피, 주차 및 신호등 정지`

## 소스 묶음 생성

현재 `src`의 다섯 프로젝트는 각각 별도 Git 저장소입니다. 원본 워크스페이스에서 곧바로 `git add src`를 하면 내부 소스 대신 gitlink로 등록될 수 있습니다. 제공 도구는 기존 폴더를 수정하지 않고 `.git`, 빌드 산출물, 캐시, 로컬 설정을 제외한 소스 스냅샷을 생성합니다. 수정 파일과 추적되지 않은 소스·모델도 포함합니다.

```bash
python3 tools/export_portfolio.py
```

결과: `dist/turtlebot3-portfolio.zip`. 새 폴더에 압축을 풀면 README, docs, tools, src와 각 원본 LICENSE가 포함됩니다. 원본 워크스페이스의 Git 이력은 그대로 보존됩니다.

## 공개 전 보완

- README에 본인이 담당한 파일·기능·문제 해결 범위를 추가합니다. 현재는 팀 결과물로만 작성했습니다.
- 데모 영상 링크 또는 GIF를 추가합니다. 발표 자료의 영상 화면만으로는 GitHub에서 주행을 바로 확인하기 어렵습니다.
- 누락된 `custom_track` 모델과 당시 실행 환경을 확보하면 재현성이 높아집니다.
- 발표 자료의 팀원 정보 및 이미지·미디어, YOLO 가중치와 학습 데이터의 공개 권한을 확인합니다. 가중치를 일괄 Apache-2.0으로 표시하지 않습니다.

## 업로드

압축을 푼 **새 폴더**에서 실행하세요. GitHub에 빈 저장소를 만든 후 실제 주소를 사용합니다.

```bash
git init -b main
git add .
git diff --cached --stat
git commit -m "Add TurtleBot3 autonomous driving portfolio"
git remote add origin https://github.com/YOUR_USERNAME/turtlebot3-autonomous-driving.git
git push -u origin main
```

업로드 대상 계정·저장소 주소가 정해지지 않아 이 작업에서는 GitHub 저장소 생성과 push를 수행하지 않았습니다.
