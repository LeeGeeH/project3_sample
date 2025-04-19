# app.py
# Flask 애플리케이션으로 HTTP 요청 처리

from flask import Flask, request, jsonify
from navigation_core import Navigation

app = Flask(__name__)
navigator = Navigation()

@app.route('/init', methods=['GET'])  # 새 엔드포인트 추가
def init_simulation():
    result = navigator.init_simulation()
    return jsonify(result)

@app.route('/update_position', methods=['POST'])
def update_position():
    data = request.get_json()
    if not data or "position" not in data:
        return jsonify({"status": "ERROR", "message": "위치 데이터 누락"}), 400

    result = navigator.position_handler.update_position(data["position"])
    if result["status"] == "ERROR":
        return jsonify(result), 400
    return jsonify(result)

@app.route('/set_destination', methods=['POST'])
def set_destination():
    data = request.get_json()
    if not data or "destination" not in data:
        return jsonify({"status": "ERROR", "message": "목적지 데이터 누락"}), 400

    result = navigator.set_destination(data["destination"])
    if result["status"] == "ERROR":
        return jsonify(result), 400
    return jsonify(result)

@app.route('/get_move', methods=['GET'])
def get_move():
    return jsonify(navigator.get_move())

@app.route('/update_obstacle', methods=['POST'])
def update_obstacle():
    """장애물 데이터를 받아 Navigation 클래스에 반영."""
    data = request.get_json()
    if not data or "obstacle" not in data:
        return jsonify({"status": "ERROR", "message": "장애물 데이터 누락"}), 400

    result = navigator.update_obstacle(data["obstacle"])
    if result["status"] == "ERROR":
        return jsonify(result), 400
    return jsonify(result), 200

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5002)