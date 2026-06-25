import threading
from flask import Flask, render_template, jsonify
import rclpy
from rclpy.node import Node
from flask import request
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
import os
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from rclpy.time import Time

from geometry_msgs.msg import PoseWithCovarianceStamped

from ament_index_python.packages import (
    get_package_share_directory
)

# -----------------------------
# Flask
# -----------------------------

package_share = get_package_share_directory(
    'my_package'
)

app = Flask(
    __name__,
    template_folder=os.path.join(
        package_share,
        'templates'
    ),
    static_folder=os.path.join(
        package_share,
        'static'
    )
)


@app.route('/')
def home():
    return render_template('index.html')

@app.route('/robot_pose')
def robot_pose():

    return jsonify({
        "x": ros_node.robot_x,
        "y": ros_node.robot_y
    })


@app.route('/plan')
def plan():

    return jsonify(
        ros_node.plan
    )

@app.route('/get_goal')
def get_goal():
    return jsonify({
        "x": ros_node.goal_x,
        "y": ros_node.goal_y
    })

# -----------------------------
# ROS2 Node
# -----------------------------

ros_node = None

class WebServer(Node):

    def __init__(self):
        super().__init__('web_server')

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
            )
        
        self.create_subscription(
            Path,
            '/plan',
            self.plan_callback,
            10
            )

        self.get_logger().info('Web Server Node Started')

        # 
        self.robot_x = 0.0
        self.robot_y = 0.0

        self.goal_x = None
        self.goal_y = None

        # 나중에 Path 저장
        self.plan = []

        
    def plan_callback(self, msg):
        self.plan = []
        # print('msg = ')
        for pose in msg.poses:
            
            self.plan.append({
            "x": pose.pose.position.x,
            "y": pose.pose.position.y
            })

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y


    def send_goal(self, x, y):

        self.goal_x = x
        self.goal_y = y

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = Time().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self.nav_client.wait_for_server()
        self.nav_client.send_goal_async(goal_msg)
        self.get_logger().info(f"Goal Sent : ({x}, {y})")


# -----------------------------
# Flask Thread
# -----------------------------

def run_flask():

    app.run(
        host='0.0.0.0',
        port=5000,
        debug=False
    )

# -----------------------------
# Main
# -----------------------------

def main(args=None):

    global ros_node

    rclpy.init(args=args)

    ros_node = WebServer()
    
    # # 목적지 좌표 직접 지정
    # target_x = 3.0
    # target_y = 2.0
    # ros_node.send_goal(target_x, target_y)

    flask_thread = threading.Thread(
        target=run_flask,
        daemon=True
    )

    flask_thread.start()

    def delayed_goal():

        ros_x = 2.8
        ros_y = -1.4
        ros_node.send_goal(ros_x, ros_y)

    #연결 안정화
    goal_thread = threading.Thread(target=delayed_goal, daemon=True)
    goal_thread.start()

    try:
        rclpy.spin(ros_node)

    except KeyboardInterrupt:
        pass

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()