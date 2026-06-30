
#   library     ---------------------------------------------------------------------------------------------

from flask import Flask
from flask import render_template
from flask import request
from flask import jsonify
from flask_socketio import SocketIO
from flask_socketio import emit
from database.database import get_db_connection
from database.database import get_locations
from database.location import get_location_by_code

#   flask init     ---------------------------------------------------------------------------------------------    
app = Flask(__name__)
socketio = SocketIO(
    app,
    cors_allowed_origins = "*"
)

navigation_state = {
    "status" : "idle",
    "type" : None,
    "route" : [],
    "current_index" : 0,
    "is_paused" : False
}

#   function    ---------------------------------------------------------------------------------------------
def send_route_to(route_type, navigation_route):
    payload = {
        "type": route_type,
        "route": navigation_route
    }

    navigation_state["status"] = "moving"
    navigation_state["type"] = route_type
    navigation_state["route"] = navigation_route
    navigation_state["current_index"] = 0
    navigation_state["is_paused"] = False

    print("\n로봇 행동트리로 전달할 payload")
    print(f"type: {payload['type']}")

    for target in payload["route"]:
        print(
            f"{target['order']}. "
            f"{target['location_name']} "
            f"({target['location_code']}) "
            f"x={target['x']}, "
            f"y={target['y']}, "
            f"yaw={target['yaw']}"
        )

    return payload

    # 팀원 행동트리 연동 방식 확정 후 이곳에 연결 검색용)extention_port



#   flask socketio    ---------------------------------------------------------------------------------------------    
@socketio.on("connect")
def handle_connect():
    print("Websocket connected")
    emit("navigation_status",
        {"status" : "connected",
        "message" : "WebSocket 연결완료"})

@socketio.on("disconnect")
def handle_disconnect():
    print("WebSocket disconnected")
    
#   flask route    ---------------------------------------------------------------------------------------------
# 로밍 화면
@app.route("/")
@app.route("/idle")
def idle():
    return render_template("idle.html")

# 안내 여부 확인
@app.route("/welcome")
def welcome():
    return render_template("welcome.html")

# 목적지 선택
@app.route("/destination")
def destination():
    locations = get_locations()
    
    return render_template(
        "destination.html",
        locations = locations
    )

# 안내 중
@app.route("/navigation")
def navigation():

    destination = request.args.get(
        "destination",
        "목적지"
    )

    location_code = request.args.get(
        "location_code",
        "unknown"
    )

    return render_template(
        "navigation.html",
        destination=destination,
        location_code=location_code
    )

# 안내 완료 #
@app.route("/finish")
def finish():
    return render_template("finish.html")

# QR 
@app.route("/qrcall")
def qrcall():

    location_code = request.args.get(
        "location",
        "unknown"
    )

    location = get_location_by_code(location_code)

    if location:
        location_name = location["location_name"]
    else:
        location_name = "알 수 없는 위치"

    return render_template(
        "qrcall.html",
        location_code=location_code,
        location_name=location_name
    )

#   api     ---------------------------------------------------------------------------------------------
@app.route("/api/locations")
def api_locations():
    locations = get_locations()
    return jsonify(locations)

@app.route("/api/navigation/state")
def get_navigation_state():
    return jsonify({
        "status" : "success",
        "navigation_state" : navigation_state
    })

@app.route("/api/navigation/start", methods = ["POST"])
def start_navigation():
    
    request_data = request.get_json()
    if not request_data:
        return jsonify({
            "status" : "error",
            "message" : "경로 데이터가 없습니다"
        }),400

    print("수신 데이터")
    print(request_data)
    
    navigation_route = []

    for item in request_data:
        location = get_location_by_code(item["location_code"])
        if not location:
            return jsonify({
                "status" : "error",
                "message" : f"{item['location_code']} 조회 실패"
            }), 404
        navigation_route.append({
            "order" : item["order"],
            "location_code" : location["location_code"],
            "location_name" : location["location_name"],
            "x" : location["pos_x"],
            "y" : location["pos_y"],
            "yaw" : location["yaw"]
        })
    
    print("최종 경로")
    for route in navigation_route:
        print(route)

    send_route_to("navigation_route", navigation_route)

    return jsonify({
        "status" : "success",
        "message" : "Ros2 navigation route sent",
        "route" : navigation_route
    })
@app.route("/api/navigation/pause", methods=["POST"])
def pause_navigation():
    print("navigation pause request")
    
    navigation_state["status"] = "paused"
    navigation_state["is_paused"] = True

    # Ros2 Nav2 정지 명령 연결
    return jsonify({
        "status" : "success",
        "message" : "navigation paused",
        "navigation_state" : navigation_state
    })

@app.route("/api/navigation/resume", methods=["POST"])
def resume_navigation():
    print("navigation resume request")

    navigation_state["status"] = "moving"
    navigation_state["is_paused"] = False

    # Ros2 Nav2 계속 명령 연결
    return jsonify({
        "status" : "success",
        "message" : "navigation resumed",
        "navigation_state" : navigation_state
    })

@app.route("/api/navigation/stop", methods=["POST"])
def stop_navigation():
    print("navigation stop request")
    
    navigation_state["status"] = "stopped"
    navigation_state["type"] = None
    navigation_state["route"] = []
    navigation_state["current_index"] = 0
    navigation_state["is_paused"] = False

    #Ros2 Nav2 취소 명령 연결
    return jsonify({
        "status" : "success",
        "message" : "navigation stopped",
        "navigation_state" : navigation_state
    })

@app.route("/api/qrcall/callrobot", methods = ["POST"])
def callrobot_qrcall():
    try:
        data = request.get_json()

        location_code = data.get("location")

        if not location_code:
            return jsonify({
                "success" : False,
                "message" : "location 값이 없습니다."
            }), 400

        location = get_location_by_code(location_code)

        if not location:
            return jsonify({
                "success" : False,
                "message" : "등록되지 않은 위치입니다."
            }), 404

        conn = get_db_connection()
        cursor = conn.cursor()

        cursor.execute(
            """
            insert into call_history(location_code)
            values(%s)
            """,
            (location_code,)
        )

        conn.commit()

        cursor.close()
        conn.close()

        qr_route = [
            {
                "order" : 1,
                "location_code" : location["location_code"],
                "location_name" : location["location_name"],
                "x" : location["pos_x"],
                "y" : location["pos_y"],
                "yaw" : location["yaw"]
            }
        ]

        send_route_to("qrcall", qr_route)

        return jsonify({
            "success" : True,
            "message" : "robot call saved"
        })

    except Exception as e:
        print(e)

        return jsonify({
            "success" : False,
            "message" : "server error"
        }), 500
    



if __name__ == "__main__":
    socketio.run(
        app,
        host="0.0.0.0",
        port=5000,
        debug=True,
        use_reloader=False
    )
