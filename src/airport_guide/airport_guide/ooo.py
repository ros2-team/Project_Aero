# #!/usr/bin/env python3
# import time
# from enum import Enum

# # 상태 제어를 단일 통로로 캡슐화하기위한 enum
# class GoalState(Enum):
#     IDLE = 1         # 대기 상태 / 목표 없음
#     SENT = 2         # Nav2 액션 서버 요청 발행 완료
#     RUNNING = 3      # Nav2 주행 엔진 정상 구동 중
#     CANCELING = 4    # 정지 명령으로 인한 자율주행 취소 절차 밟는 중
#     DONE = 5         # 목적지 도착 완료 및 대기 세션 진입

# class ChargingState(Enum):
#     IDLE = 1
#     MOVING = 2       # 충전소 이동 중
#     CHARGING = 3     # 충전 패드 도달 완료

# class Blackboard:
#     def __init__(self):
#         # 1) FACTS: 읽기 전용 센서 데이터
#         self.battery_level = 100.0        
#         self.last_sensor_time = time.time() 
#         self.obstacle_distance = 10.0     
#         self.front_obstacle_distance = None
#         self.rear_obstacle_distance = None
#         self.is_front_human = False         
#         self.is_rear_human = False          
#         self.human_distance = 1.2         
#         self.current_x = 0.0              
#         self.current_y = 0.0              

#         # 2) DERIVED: 추론 및 판단 결과 데이터
#         self.battery_low = False          
#         self.obstacle_warning = False     
#         self.obstacle_detected = False    
#         self.is_dynamic_obstacle = False  
#         self.is_static_obstacle = False   
#         self.human_far = False            
#         self.human_lost = False           
#         self.sensor_timeout = False       # 시간 연산 결과물
        

#         # 3) EXEC_STATE: 단일 책임 행동 실행 상태 구조
#         self.goal_state = GoalState.IDLE            # 기존 goal_sent, goal_faiiled 통합
#         self.charging_state = ChargingState.IDLE    # 기존 charging_started 통합
#         self.is_paused = False            
        
#         self.goal_name = ""
#         self.goal_x = 0.0
#         self.goal_y = 0.0

#         # 4) 웹 데이터 처리용 변수
#         self.web_action = ""
      
#         self.web_route_list = []
#         self.web_last_update_time = time.time()
#         self.current_waypoint_index = 0
#         self.navigation_active = False
#         self.navigation_finished = False

#         #qr 변수 
#         self.qr_route_backup = None