import time

class Blackboard:
    def __init__(self):
        # ----------------------------------------------------------------------
        # 1. 배터리 및 무결성 시스템 플래그
        # ----------------------------------------------------------------------
        self.battery_level = 100.0        # 실시간 배터리 잔량 (%)
        self.battery_low = False          # 30% 이하일 때 True로 켜지는 스위치
        self.charging_started = False     # 충전소로 이동 명령이 이미 날아갔는지 잠그는 스위치

        self.last_sensor_time = time.time() # 센서(오도메트리 등)가 살아있는지 체크하는 시간 기록
        self.sensor_timeout = False       # 1초 이상 센서가 먹통이면 True가 되는 스위치
        
        # 🎯 [추가] 웹 인터페이스 제어 플래그 (1.5번 웹 일시정지 브랜치와 연동)
        self.is_paused = False            # 웹에서 일시정지를 누르면 True가 되는 스위치
        
        # ----------------------------------------------------------------------
        # 2. 장애물 센싱 플래그 (라이다 및 AI 카메라 연동)
        # ----------------------------------------------------------------------
        self.obstacle_distance = 10.0     # 라이다 물리 거리 데이터 (미터 단위)
        self.obstacle_detected = False    # 충돌 직전(급정거 필요) 스위치
        self.obstacle_warning = False     # 감속이나 우회 주행이 필요한 영역 감지 스위치
        
        self.front_obstacle_distance = 10.0 # 전방 YOLO 물체 거리
        self.rear_obstacle_distance = 10.0  # 후방 YOLO 물체 거리
        self.is_front_human = False         # 전방 카메라에 사람이 잡혔는가?
        self.is_rear_human = False          # 후방 카메라에 사람이 잡혔는가?

        # 🎯 [추가] 카메라 + 라이다 융합 판단 결과 플래그 (트리 2번, 4번과 연동)
        self.is_dynamic_obstacle = False  # 전방에 움직이는 장애물(사람) 존재 여부
        self.is_static_obstacle = False   # 전방에 멈춰있는 고정 장애물 존재 여부
        
        # ----------------------------------------------------------------------
        # 3. 후방 사용자 추적(Human Tracking) 플래그
        # ----------------------------------------------------------------------
        self.human_distance = 1.2         # 주인이 나와 떨어진 거리
        self.human_tracked = False        # 현재 주인을 락온(Lock-on)하여 추적 중인가? (초기값 False)
        self.human_far = False            # 주인이 너무 멀어져서 멈춰 서서 기다려야 하는가?
        self.human_lost = False           # 주인을 완전히 놓쳐서 탐색 모드로 가야 하는가?
        self.rear_target_id = -1          # ByteTrack이 부여한 주인의 고유 번호 (-1은 지정 안 됨)
        self.rear_cam_angle = 90.0        # 후방 서보 모터의 현재 각도 (0~180도)
        self.human_lost_timer = 0.0

        # ----------------------------------------------------------------------
        # 4. 내비게이션(주행 제어) 및 경유지 플래그 시스템
        # ----------------------------------------------------------------------
        self.current_x = 0.0              # 로봇 현재 위치 X
        self.current_y = 0.0              # 로봇 현재 위치 Y

        self.waypoint_list = [
            {"name": "WayPoint_1", "x": 2.8, "y": 0.6},
            {"name": "WayPoint_2", "x": 2.9, "y": -1.4},
            {"name": "Gate_A3", "x": 1.4, "y": -0.5} 
        ]
        self.has_goal = False             # 아직 안내해야 할 목적지가 남아있는가?
        self.goal_sent = False            # [핵심] Nav2에 목표 좌표가 발사되었는가? (중복 발사 차단 락)
        self.goal_failed = False          # 이동 중 주행이 취소되거나 실패했는가?
        self.is_arrived = False           # 목적지 반경 0.5m 안에 도착했는가? (ArrivalNode가 제어)

        self.wait_started = False         # 도착 후 5초 세기 타이머가 켜졌는가?
        self.wait_start_time = 0.0        # 5초 측정을 시작한 시점 기록

        # 5. 첫 번째 경유지를 안전하게 꺼내 초기화하는 로직 호출
        self._init_first_waypoint()

    def _init_first_waypoint(self):
        """프로그램 실행 시 리스트에서 첫 경유지를 가져와 하단 주행 플래그를 장전합니다."""
        if len(self.waypoint_list) > 0:
            current_wp = self.waypoint_list.pop(0)
            self.has_goal = True
            self.goal_name = current_wp["name"]
            self.goal_x = current_wp["x"]
            self.goal_y = current_wp["y"]



# 일시정지 선택 시 
# shared_blackboard.is_paused = True
# 경유지 유지 선택 시
# shared_blackboard.is_paused = False
# 경유지 수정 후 재개 선택 시 
# shared_blackboard.goal_x = 새_X_좌표
# shared_blackboard.goal_y = 새_Y_좌표
# shared_blackboard.goal_name = "새목적지"
# shared_blackboard.goal_sent = False  # 락 가드를 풀어야 6번 브랜치가 새 좌표를 Nav2로 쏩니다.
# shared_blackboard.is_paused = False  # 일시정지 해제
# 길찾기 종료 시 
# shared_blackboard.has_goal = False
# shared_blackboard.goal_sent = False
# shared_blackboard.is_paused = False
# 현업 가이드: 주행 중이던 Nav2 액션을 즉시 캔슬하기 위해 아래 명령을 웹 노드나 트리 노드에 연동하면 안전합니다.
# bt_node.nav_client.cancel_goal_async()