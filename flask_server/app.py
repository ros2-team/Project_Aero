from flask import Flask
from flask import render_template
from flask import request
from flask import jsonify
from database.database import get_db_connection
from database.database import get_location


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
    return render_template("destination.html")

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
    
# api test
@app.route("/api/location/<location_code>")
def location_api(location_code):
    location = get_location(location_code)
    if location is None:
        return jsonify({
            "status" : "error"
        }), 404
    return jsonify(location)

if __name__ == "__main__":
    app.run(
        host="0.0.0.0",
        port=5000,
        debug=True
    )