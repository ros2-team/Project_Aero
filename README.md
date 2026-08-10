# 🤖 Aero: 자율주행 공항 안내 로봇 (Autonomous Airport Guide Robot)

> **ROS 2 Humble** 및 **Behavior Tree** 기반으로 동작하며, 공항 내부에서 목적지 길안내, YOLO+LiDAR 센서 퓨전 기반 사용자 추종, ArUco 마커 기반 자동 도킹 충전 서비스를 제공하는 자율주행 로봇 시스템입니다.

---

## 1. 프로젝트 개요
* **프로젝트명**: 자율주행형 공항 안내 로봇 **Aero**
* **개발 기간**: 2026.03 ~ 2026.07
* **개발 환경**: ROS 2 Humble (Ubuntu 22.04 LTS), Python 3.10, C++, MySQL, Flask, JavaScript

---

## 2. 개발 배경 및 목적
* **공항 이용객 길찾기 지원**: 복잡하고 넓은 공항 내부에서 초행 이용객이나 탑승 시간이 촉박한 승객을 위해 자율주행 기반의 길안내 서비스 제공.
* **사용자 친화적 인터페이스**: 복잡한 지도를 직접 보지 않고도 로봇을 따라 목적지에 도착할 수 있도록 직관적인 시각화 UI 및 추종 기능 구현.
* **안전 및 예외 처리 강화**: 혼잡한 환경에서의 장애물 회피, 배터리 부족, 센서 오류 등 다양한 예외 상황에 유연하게 대응하는 Behavior Tree 제어 구조 적용.

---

## 3. 주요 기능
* **자율 길안내 (Navigation)**: Nav2 및 AMCL 기반으로 장애물을 회피하며 목적지까지 안정적 주행.
* **경유지 및 경로 변경**: 안내 중 사용자 요청에 따라 경유 순서 변경 및 목적지 실시간 재설정.
* **사용자 추종 (Follow Me)**: 후방 카메라(YOLO)와 LiDAR 센서 퓨전을 통해 승객 이탈 여부를 감지하고 실시간 정지 및 추종.
* **자동 도킹 충전 (Auto Docking)**: Nav2 대략적 접근 후, ArUco 마커 기반 상대 위치 계산을 통해 충전 단자로 정밀 곡선 접근 도킹.
* **QR 코드 로봇 호출**: 공항 내 주요 구역의 QR 코드를 스캔하여 로봇을 현재 호출 위치로 이동시키는 원격 호출 서비스.
* **실시간 관제 & 웹 UI**: HTML5 Canvas 및 Socket.IO를 활용하여 로봇 위치, 주행 경로, 배터리 잔량 및 상태를 실시간 시각화.

---

## 4. 기술 스택

### Robot & System
* **OS / Middleware**: Linux (Ubuntu 22.04 LTS), ROS 2 Humble
* **Platform & Hardware**: TurtleBot3, 2D LiDAR, RGB-D Camera

### Software & Algorithm
* **Navigation & Control**: Navigation2, AMCL, Behavior Tree (BehaviorTree.CPP / py_trees)
* **Vision & Sensor Fusion**: OpenCV, YOLOv8, ArUco Marker, LiDAR-Camera Fusion
* **Database**: MySQL

### Web & Integration
* **Frontend**: HTML5, CSS3, JavaScript (ES6+), HTML5 Canvas, Socket.IO Client
* **Backend**: Python 3.10, Flask, Flask-SocketIO
* **Communication**: REST API, ROS 2 Topic/Action/Service, Web Bridge Node

---

## 5. 시스템 아키텍처 (System Architecture)

```text
 [ Web UI (User / Admin) ]
            │ (REST API / WebSockets)
            ▼
   [ Flask Web Server ] ◄───► [ Web Bridge Node ]
                                     │ (ROS 2 Topics/Actions)
                                     ▼
                      ┌─────────────────────────────┐
                      │    Behavior Tree Engine     │  (우선순위 제어 및 상태 관리)
                      └──────────────┬──────────────┘
                                     │
      ┌──────────────────────────────┼──────────────────────────────┐
      ▼                              ▼                              ▼
[ Nav2 Stack ]              [ Sensor Fusion ]              [ Auto Docking ]
 ├─ AMCL (위치 추정)          ├─ YOLO (사용자 인지)          ├─ ArUco Detection
 └─ Controller/Planner       └─ LiDAR (거리 및 추종)         └─ Precise Pure Pursuit
```


## 6. 핵심 구현 내용

### ① 행동 트리 기반 상태 제어 (Behavior Tree Engine)
* 기존 FSM 대비 상태 관리 및 예외 처리가 용이한 **Behavior Tree**를 적용하여 로봇의 제어 우선순위를 체계적으로 관리.
* **Root Selector 제어 우선순위**:
  1. `Battery Check`: 배터리 임계치 미만 시 기존 미션 중단 후 충전소 복귀 수행
  2. `Sensor Check`: 센서 데이터 이탈 또는 HW 에러 시 긴급 정지
  3. `Pause Check`: 승객의 웹 UI 일시정지 요청 처리
  4. `Distance Check`: 전방/후방 안전거리 확보 및 승객 이탈 감지
  5. `Goal Check`: 목적지 유무 및 변경 요청 확인
  6. `Nav2 Execution`: Nav2 Stack을 통한 목적지 자율주행 실행
  7. `QR Service`: 원격 QR 호출 미션 처리
  8. `Idle State`: 대기 모드 전환 및 시스템 상태 발행

### ② ArUco 마커 기반 정밀 자동 도킹 (Auto Docking)
* **위치 오차 극복**: Nav2 글로벌 주행의 도킹 오차를 해결하기 위해 2단계 도킹 전략 도입.
* **곡선 접근 알고리즘**: Nav2로 충전소 근처 접근 후, 카메라로 ArUco 마커 인식 ➔ 상대 3D Pose($x, y, yaw$)를 계산하여 Smooth Curve 곡선 접근 알고리즘으로 단자 정밀 결합.

### ③ YOLO + LiDAR 센서 퓨전 기반 사용자 추종 (Follow Me)
* **전방 안전거리 감지**: 2D LiDAR 스캔 데이터를 분석하여 전방 장애물 접근 시 즉시 감속 및 정지.
* **후방 사용자 추종**: 후방 카메라의 YOLOv8 Bounding Box 영역과 LiDAR Point Cloud 데이터를 매핑하여 사용자의 3D 위치 추적. 승객 이탈 시 안내 일시정지 및 안내 음성/UI 출력.

### ④ Web Bridge Node 및 실시간 UI 연동
* ROS 2와 Web 간의 실시간 비동기 양방향 통신 구현.
* **Web Bridge Node**: `/tf`, `/amcl_pose`, `/plan` 등 ROS 2 토픽을 JSON 규격으로 가공하여 WebSocket으로 브로드캐스팅.

---

## 7. 주요 ROS 2 인터페이스 (Interfaces)

| 분류 | Interface Name | Type | 설명 |
| :--- | :--- | :--- | :--- |
| **Topic** | `/aero/battery_status` | `sensor_msgs/msg/BatteryState` | 로봇 배터리 잔량 및 충전 상태 |
| **Topic** | `/aero/user_pose` | `geometry_msgs/msg/PoseStamped` | YOLO+LiDAR 퓨전으로 계산된 승객 위치 |
| **Topic** | `/aero/web_status` | `std_msgs/msg/String` | 웹 UI로 전달되는 로봇 제어 상태 메시지 |
| **Action** | `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Nav2 자율주행 목표 지점 전달 |
| **Service** | `/aero/trigger_docking` | `std_srvs/srv/Trigger` | 자동 도킹 시퀀스 시작 요청 |

---

## 8. 주요 문제 해결 (Troubleshooting)

### 1. Nav2 도착 오차로 인한 충전 단자 결합 실패
* **문제**: Nav2 자율주행만으로는 2~3cm 내외의 정밀도 오차가 발생하여 충전 단자에 물리적으로 정확히 도킹하지 못하는 현상 발생.
* **해결**: Nav2 목적지를 충전소 1m 전방으로 설정하고, 이후 ArUco 마커를 인식하여 카메라-마커 간 상대 좌표 기반의 Pure Pursuit 제어로 전환하는 **2단계 도킹(Two-stage Docking) 알고리즘**을 구현하여 도킹 성공률 98% 달성.

### 2. 동적 장애물 및 승객 이탈 시 Behavior Tree 교착 상태 (Deadlock)
* **문제**: 승객 추종 주행 중 승객이 시야에서 벗어났을 때 Nav2 주행 명령과 추종 정지 명령이 충돌하는 현상.
* **해결**: Behavior Tree에 `Distance Check` 데코레이터 노드를 삽입하여 승객과의 거리가 2m 이상 벌어지면 Nav2 Action을 즉시 `PAUSE` 상태로 전환하고 UI에 경고 메시지를 띄우도록 제어 로직 구조화.

---

## 9. 저장소 구조 (Directory Structure)  수정필요 
```text
Aero/
├── src
│   ├── robot_bringup           # 시스템 일괄 실행 Launch 파일 및 파라미터
│   │   ├── launch
│   │   └── maps
│   ├── ar_interfaces           # Aero에 사용되는 커스텀 메세지 타입 정의  
│   │   └── msg
│   ├── behavior_tree           # Behavior Tree 및 커스텀 Control Nodes
│   │   ├── behavior_tree
│   │   └── launch
│   ├── perception              # YOLO + LiDAR 센서 퓨전 및 ArUco 도킹 노드
│   │   ├── launch
│   │   └── perception
│   └── web_bridge              # ROS 2 - Web Socket Bridge 노드
│       ├── launch
│       └── web_bridge
└── web                         # Flask 기반 대시보드 및 관제 웹페이지
    └── flask_server
        ├── app.py
        ├── database
        ├── static
        │   ├── css
        │   ├── img
        │   └── js
        └── templates

