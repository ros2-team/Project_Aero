# 🤖 Aero: 자율주행 공항 안내 로봇 (Autonomous Airport Guide Robot)

> **ROS 2 Humble** 및 **Behavior Tree** 기반으로 동작하며, 공항 내부에서 목적지 길안내, YOLO+LiDAR 센서 퓨전 기반 사용자 추종, ArUco 마커 기반 자동 도킹 충전 서비스를 제공하는 자율주행 로봇 시스템입니다.

---

## 1. 프로젝트명
* **프로젝트명**: 자율주행형 공항 안내 로봇 **Aero**
* **개발 환경**: ROS 2 Humble (Ubuntu 22.04 LTS), Python, MySQL, Web (HTML/CSS/JS, Flask)

---

## 2. 개발 배경
* **공항 이용객 길찾기 지원**: 복잡하고 넓은 공항 내부에서 초행 이용객이나 탑승 시간이 촉박한 승객을 위해 자율주행 기반의 길안내 서비스 제공.
* **사용자 친화적 인터페이스**: 복잡한 지도를 직접 보지 않고도 로봇을 따라 목적지에 도착할 수 있도록 직관적인 시각화 UI 및 추종 기능 구현.
* **안전 및 예외 처리 강화**: 혼잡한 환경에서의 장애물 회피, 배터리 부족, 센서 오류 등 다양한 예외 상황에 유연하게 대응하는 제어 구조 필요.

---

## 3. 주요 기능
* **자율 길안내 (Navigation)**: Nav2 및 AMCL 기반으로 장애물을 회피하며 목적지까지 주행.
* **경유지 및 경로 변경**: 안내 중 경유 순서 변경 및 목적지 재설정 가능.
* **사용자 추종 (Follow Me)**: 후방 카메라(YOLO)와 LiDAR 센서 퓨전을 통해 승객 이탈 여부를 감지하고 실시간 정지 및 추종.
* **자동 도킹 충전 (Auto Docking)**: Nav2 접근 후 ArUco 마커 기반 상대 위치 계산을 통해 충전소로 정밀 곡선 접근 도킹.
* **QR 코드 로봇 호출**: 지정 장소의 QR 코드를 스캔하여 로봇을 호출 위치로 이동시키는 기능.
* **실시간 웹 UI 시각화**: HTML5 Canvas를 이용해 경로, 로봇 위치, 진행 상태, 배터리 잔량을 실시간 시각화.

---

## 4. 기술 스택

### Robot & System
* **OS / Middleware**: Linux (Ubuntu 22.04 LTS), ROS 2 Humble
* **Platform & Hardware**: TurtleBot3, 2D LiDAR, RGB Camera

### Software & Algorithm
* **Navigation & SLAM**: Navigation2, AMCL, Behavior Tree
* **Vision & Sensor Fusion**: OpenCV, YOLO, ArUco Marker
### Web & Integration
* **Frontend**: HTML, CSS, JavaScript, HTML5 Canvas, Socket.IO Client
* **Backend**: Python, Flask, Flask-SocketIO, MySQL
* **Communication**: REST API, ROS 2 Topic/Action, Web Bridge Node

---

## 5. 시스템 구조도

```mermaid
graph TD
    A[Sensing: LiDAR/Camera] --> B(Perception: Object Detection)
    B --> C{Obstacle Detected?}
    C -->|Yes| D[Behavior Tree: Abort & E-Brake]
    C -->|No| E[Nav2: Path Planning]
    D --> F[Control: Motor Commands]
    E --> F

## 6. 핵심 구현 내용

### ① 행동 트리 (Behavior Tree Engine)
* **관리 방식**: FSM 대비 상태 관리 및 예외 처리가 용이한 Behavior Tree를 적용하여 로봇의 제어 우선순위 관리.
* **Root Selector 우선순위**:
  1. `Battery Check` (배터리 부족 시 충전소 복귀)
  2. `Sensor Check` (센서 오류 시 긴급 정지)
  3. `Pause Check` (사용자 일시정지 요청)
  4. `Distance Check` (전방/후방 안전거리 확인)
  5. `Goal Check` (목적지 유무 확인)
  6. `Nav2 Execution` (Navigation 경로 주행)
  7. `QR Service` (QR 원격 호출 처리)
  8. `Idle State` (기본 대기)

### ② 자동 도킹 (Auto Docking)
* **오차 극복**: Nav2 좌표 이동의 위치 오차를 극복하기 위해 ArUco 마커 기반 상대 위치 계산 적용.
* **곡선 접근 알고리즘**: 마커 인식 후 정밀 곡선 접근 알고리즘을 통해 충전 단자에 안정적으로 도킹.

### ③ 센서 퓨전 (YOLO + LiDAR)
* **전방 안전거리 감지**: LiDAR 스캔을 통해 정방 장애물 거리를 측정하여 안전거리 이탈 시 정지 및 재개.
* **후방 사용자 추종**: YOLO(사람 인식)와 LiDAR 센서 퓨전을 활용해 사용자를 추종하며, 사용자가 화면을 벗어나거나 일정 거리 이상 이탈 시 자동 정지.

### ④ 웹 인터페이스 (Web UI & Integration)
* **페이지 구성**: Idle(대기), Welcome(시작), Destination(목적지 선택/순서 변경), Navigation(실시간 시각화/일시정지), Finish(완료), QR Call(원격 호출)[cite: 1].
* **Web Bridge Node**: ROS 2 내 상태 및 경로 데이터를 웹 규격으로 변환하여 실시간 양방향 통신 제공[cite: 1].
