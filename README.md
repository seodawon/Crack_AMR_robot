# Crack: 균열 탐지 및 대피 경로 안내 로봇

### SLAM 기반 자율주행 로봇 시스템 구현 프로젝트
- 프로젝트 기간: **2025.05.18 ~ 2025.05.22** (주요 개발 및 시연 기간)  
- 참여인원: **8명**

<br>

## 🎥 프로젝트 소개  
[![Crack Demo](https://img.youtube.com/vi/RlI6yj5Fucw/0.jpg)](https://youtu.be/RlI6yj5Fucw)  
➡ 영상 클릭 시, YouTube 재생  

** Crack! **은 SLAM 기반 자율주행 로봇(AMR) 두 대가 협력하여 건물의 균열을 탐지하고, 위험 상황 발생 시 내부 인원에게 **최적의 대피 경로**를 안내하는 지능형 로봇 시스템입니다.  

- **TurtleBot4 기반 AMR**, ROS2 Humble, NAV2, SLAM 활용  
- **YOLOv8n + OAK-D Pro 3D 카메라**로 균열/사람 객체 탐지  
- **Flask 기반 웹 UI**로 실시간 원격 모니터링 및 로그 관리  
- **2대 로봇 협력 시스템**: AMR1(순찰) + AMR2(대피 안내)  

<br>

## 🔧 주요 기능
- 🤖 **협력적 균열 탐지 및 순찰**  
  AMR1이 SLAM 맵 기반 경로를 순찰하며 균열·사람을 탐지, 위험 균열 발견 시 AMR2와 관제에 경고 전송  
- 🚶 **음성 기반 대피 경로 안내**  
  AMR2가 출구 안전 여부 확인 후, 위험 신호 수신 시 사람을 가장 가까운 안전 출구까지 **음성 안내 + 추종 확인**  
- 📏 **3D 카메라 기반 정밀 균열 분석**  
  OAK-D Pro Depth 정보로 균열 3D 좌표 및 길이 측정, 기준치(25cm↑) 시 위험 판정  
- 🖥️ **Flask 원격 모니터링**  
  로봇 위치, 카메라 화면, 균열/사람 탐지를 지도에 시각화, 로그 기록 및 저장  

<br>

## 🚀 전체 실행 순서

### ✅ (1) TurtleBot4 Navigation 실행
각 로봇(`robot6`, `robot7`)에 대해 터미널 3개에서 Navigation 관련 launch 파일 실행  
(※ robot name은 환경에 맞게 수정 필요)

```bash
# for robot6
ros2 launch turtlebot4_navigation localization.launch.py namespace:=robot6 map:=$HOME/rokey_c1_mini/src/crack/map/map.yaml
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot6
ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/robot6

# for robot7
ros2 launch turtlebot4_navigation localization.launch.py namespace:=robot7 map:=$HOME/rokey_c1_mini/src/crack/map/map.yaml
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot7
ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/robot7
