import time
import math
import requests


# ---------------------------------------------------------
# Flask server address
# ---------------------------------------------------------

BASE_URL = "http://127.0.0.1:5000"


# ---------------------------------------------------------
# State
# ---------------------------------------------------------

last_command_id = None


# ---------------------------------------------------------
# API functions
# ---------------------------------------------------------

def get_robot_command():
    try:
        response = requests.get(
            f"{BASE_URL}/api/robot/command",
            timeout=3
        )

        if response.status_code != 200:
            print(
                "command request failed:",
                response.status_code
            )
            return None

        data = response.json()

        if data.get("status") != "success":
            return None

        return data.get("command")

    except Exception as error:
        print("get_robot_command error:", error)
        return None


def mark_command_handled(command_id):
    try:
        response = requests.post(
            f"{BASE_URL}/api/robot/command/handled",
            json={
                "command_id": command_id
            },
            timeout=3
        )

        print(
            "command handled:",
            response.status_code,
            response.text
        )

    except Exception as error:
        print("mark_command_handled error:", error)


def send_robot_status(
    battery,
    x,
    y,
    yaw,
    robot_status,
    network="connected"
):
    try:
        response = requests.post(
            f"{BASE_URL}/api/robot/status",
            json={
                "battery": battery,
                "x": x,
                "y": y,
                "yaw": yaw,
                "robot_status": robot_status,
                "network": network
            },
            timeout=3
        )

        if response.status_code != 200:
            print(
                "robot status failed:",
                response.status_code,
                response.text
            )

    except Exception as error:
        print("send_robot_status error:", error)


def send_navigation_update(status, current_index):
    try:
        response = requests.post(
            f"{BASE_URL}/api/navigation/update",
            json={
                "status": status,
                "current_index": current_index
            },
            timeout=3
        )

        print(
            "navigation update:",
            status,
            current_index,
            response.status_code
        )

    except Exception as error:
        print("send_navigation_update error:", error)

def send_navigation_path(path):
    try:
        response = requests.post(
            f"{BASE_URL}/api/navigation/path",
            json={
                "path": path
            },
            timeout=3
        )

        print(
            "navigation path:",
            len(path),
            "points",
            response.status_code
        )

    except Exception as error:
        print("send_navigation_path error:", error)
# ---------------------------------------------------------
# Movement simulation
# ---------------------------------------------------------

def interpolate(start_x, start_y, target_x, target_y, steps):
    points = []

    for i in range(steps + 1):
        ratio = i / steps

        x = start_x + (target_x - start_x) * ratio
        y = start_y + (target_y - start_y) * ratio

        points.append(
            {
                "x": x,
                "y": y
            }
        )

    return points

def make_test_path(start_x, start_y, target_x, target_y):
    path = []

    # 현재 위치에서 목적지까지 가짜 경로 생성
    points = interpolate(
        start_x,
        start_y,
        target_x,
        target_y,
        steps=12
    )

    for point in points:
        path.append({
            "x": point["x"],
            "y": point["y"]
        })

    return path

def simulate_navigation(route):
    if not route:
        print("empty route")
        return

    print("simulate route start")
    print("route count:", len(route))

    # 시작 위치는 일단 첫 번째 목적지에서 조금 떨어진 위치로 가정
    current_x = route[0]["x"] - 0.5
    current_y = route[0]["y"] - 0.5
    current_yaw = 0.0
    battery = 80

    send_robot_status(
        battery=battery,
        x=current_x,
        y=current_y,
        yaw=current_yaw,
        robot_status="moving"
    )

    time.sleep(1)

    for index, target in enumerate(route):
        target_x = float(target["x"])
        target_y = float(target["y"])
        target_yaw = float(target.get("yaw", 0.0))

        print(
            f"moving to {index + 1}/{len(route)}:",
            target.get("location_name"),
            target_x,
            target_y
        )

        send_navigation_update(
            status="moving",
            current_index=index
        )

        test_path = make_test_path(
            current_x,
            current_y,
            target_x,
            target_y
        )

        send_navigation_path(test_path)

        points = interpolate(
            current_x,
            current_y,
            target_x,
            target_y,
            steps=30
        )

        for point in points:
            current_x = point["x"]
            current_y = point["y"]

            send_robot_status(
                battery=battery,
                x=current_x,
                y=current_y,
                yaw=current_yaw,
                robot_status="moving"
            )

            time.sleep(0.12)

        # 도착 처리
        current_x = target_x
        current_y = target_y
        current_yaw = target_yaw
        battery -= 2

        send_robot_status(
            battery=battery,
            x=current_x,
            y=current_y,
            yaw=current_yaw,
            robot_status="idle"
        )

        print(
            "arrived:",
            target.get("location_name")
        )

        time.sleep(0.8)

    send_navigation_update(
        status="finished",
        current_index=len(route) - 1
    )

    send_robot_status(
        battery=battery,
        x=current_x,
        y=current_y,
        yaw=current_yaw,
        robot_status="finished"
    )

    print("simulate route finished")


# ---------------------------------------------------------
# Main loop
# ---------------------------------------------------------

def main():
    global last_command_id

    print("Dummy Robot Bridge Started")
    print("Flask:", BASE_URL)

    while True:
        command = get_robot_command()

        if not command:
            time.sleep(1)
            continue

        has_command = command.get("has_command")
        command_id = command.get("command_id")
        command_type = command.get("type")
        is_handled = command.get("is_handled")

        if not has_command:
            time.sleep(1)
            continue

        if is_handled:
            time.sleep(1)
            continue

        if command_id == last_command_id:
            time.sleep(1)
            continue

        print("--------------------------------")
        print("new command detected")
        print("command_id:", command_id)
        print("type:", command_type)

        last_command_id = command_id

        mark_command_handled(command_id)

        if command_type == "navigation_route":
            route = command.get("route", [])
            simulate_navigation(route)

        elif command_type == "robot_call":
            route = command.get("route", [])
            simulate_navigation(route)

        elif command_type == "pause_navigation":
            send_robot_status(
                battery=80,
                x=0.0,
                y=0.0,
                yaw=0.0,
                robot_status="paused"
            )

        elif command_type == "resume_navigation":
            send_robot_status(
                battery=80,
                x=0.0,
                y=0.0,
                yaw=0.0,
                robot_status="moving"
            )

        elif command_type == "stop_navigation":
            send_navigation_update(
                status="stopped",
                current_index=0
            )

            send_robot_status(
                battery=80,
                x=0.0,
                y=0.0,
                yaw=0.0,
                robot_status="stopped"
            )

        else:
            print("unknown command type:", command_type)

        time.sleep(1)


if __name__ == "__main__":
    main()