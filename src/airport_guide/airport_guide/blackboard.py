class Blackboard:
    def __init__(self):
        # 1. 시스템 및 배터리 상태
        self.battery_level = 100.0
        self.charging_started = False
        
        # 2. 하드웨어 센서 무결성 데이터
        self.last_sensor_time = 0.0      
        self.sensor_timeout = False      
        
        # 3. 물리 장애물 센싱 데이터
        self.obstacle_distance = 10.0
        self.obstacle_direction = "CENTER"
        
        # 4. 휴먼 트래커 데이터
        self.human_tracked = True
        self.human_distance = 1.2
        self.human_lost_timer = 0.0      
        
        # 5. 내비게이션 상태 변수
        self.nav_status = "IDLE"          # IDLE, EXECUTING, RECOVERY, FAILED
        self.current_x = 0.0
        self.current_y = 0.0

        # 6. 경유지 큐(Queue) 데이터 시스템
        self.waypoint_list = [
            {"name": "WayPoint_1", "x": 2.8, "y": 0.6},
            {"name": "WayPoint_2", "x": 2.9, "y": -1.4},
            {"name": "Gate_A3", "x": 1.4, "y": -0.5} 
        ]

        # 첫 번째 경유지 자동 장전
        current_wp = self.waypoint_list.pop(0)
        self.has_goal = True
        self.goal_name = current_wp["name"]
        self.goal_x = current_wp["x"]
        self.goal_y = current_wp["y"]
        self.is_arrived = False
        self.wait_started = False
        self.wait_start_time = 0.0      
        