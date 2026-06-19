class Blackboard:
    def __init__(self):
        self.battery_level = 100.0
        self.charging_started = False
        
        # 다른 노드들과 연동할 기본 상태값들 정의
        self.obstacle_distance = 10.0
        self.obstacle_direction = "CENTER"
        self.human_tracked = True
        self.human_distance = 1.2
        self.has_goal = True
        self.goal_name = "Gate_A3"
        self.is_arrived = False