import json
from flask import Flask, render_template, jsonify, request
import paho.mqtt.client as mqtt

app = Flask(__name__)

# Flask 전용 MQTT 클라이언트 생성 (기본 TCP 1883 포트 사용)
mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)

try:
    mqtt_client.connect("localhost", 1883, 60)
    mqtt_client.loop_start()
    print("✔ [Flask] MQTT 브로커 연결 성공 (Port: 1883)")
except Exception as e:
    print(f"[Flask] MQTT 브로커 연결 실패: {e}")

@app.route('/')
def home():
    return render_template('index.html')

@app.route('/api/navigation/start', methods=['POST'])
def handle_navigation():
    request_data = request.get_json()  
    print("\n================ [Flask 수신 경로 데이터] ================")
    print(request_data)
    print("=========================================================")
    
    # 수신 데이터를 JSON 문자열로 인코딩하여 test_sub.py 가 구독하는 토픽으로 중계(Publish)
    json_payload = json.dumps(request_data, ensure_ascii=False)
    mqtt_client.publish("robot/navigation/path", json_payload, qos=1)
    print("🚀 [Flask ➔ MQTT] 'robot/navigation/path' 토픽으로 경로 데이터 중계 완료")
    
    result = {"status": "success", "message": "서버가 경로 수신 및 로봇 명령 중계를 완료했습니다."}
    return jsonify(result)

if __name__ == '__main__':
    app.run(debug=True)