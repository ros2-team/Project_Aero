from flask import Flask, render_template, request , jsonify
import mysql.connector

# 데이터베이스 연결
def get_db_connection():
    return mysql.connector.connect(
        host = "localhost",
        user = "projectAR",
        password = "1234",
        database = "projectAR"
    )

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
# QR 로봇 호출 api ?????????????
@app.route("/qrcalling", methods = ["POST"])
def qrcalling():
    
    data = request.get_json() # json -> dict
    location = data["location"]
    
    conn = get_db_connection()
    cursor = conn.cursor()


if __name__ == "__main__":
    app.run(
        host="0.0.0.0",
        port=5000,
        debug=True
    )