from flask import Flask
from flask import render_template
from flask import request
from flask import jsonify
from database.database import get_db_connection
from database.database import get_locations
from database.location import get_location_by_code


app = Flask(__name__)

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
        "화장실"
    )

    return render_template(
        "navigation.html",
        destination=destination
    )

# 안내 완료
@app.route("/finish")
def finish():
    return render_template("finish.html")

# QR 
@app.route("/qrcall")
def qrcall():

    location = request.args.get("location", "unknown")
    
    return render_template(
        "qrcall.html",
        location=location
    )
# QR 로봇 호출
@app.route("/call_robot", methods = ["POST"])
def call_robot():
    try:
        data = request.get_json()
        location = data["location"]

        conn = get_db_connection()
        cursor = conn.cursor()
        cursor.execute(
            """ 
            insert into call_history(location_code)
            values(%s)
            """, 
            (location,)
        )
        conn.commit()
        cursor.close()
        conn.close()
        
        return jsonify({
            "success" : True
        })
    except Exception as e:
        print(e)
        return jsonify({
            "success" : False
        }), 500
#api
@app.route("/api/locations")
def api_locations():
    locations = get_locations()
    return jsonify(locations)

@app.route("/api/navigation/start", methods = ["POST"])
def start_navigation():
    request_data = request.get_json()
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
    return jsonify({
        "status" : "success",
        "route" : navigation_route
    })
    
if __name__ == "__main__":
    app.run(
        host="0.0.0.0",
        port=5000,
        debug=True
    )