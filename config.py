# config.py
# 내비게이션 시스템의 구성 파라미터

# 이동 파라미터
MOVE_STEP = 0.1  # 기본 이동 단위 (미터/스텝)
TOLERANCE = 15.0  # 목적지 도착 허용 오차 (미터)
LOOKAHEAD_MIN = 1.0  # 최소 전방 주시 거리 (미터)
LOOKAHEAD_MAX = 10.0  # 최대 전방 주시 거리 (미터)
HEADING_SMOOTHING = 0.8  # 헤딩 평활화 계수 (0~1, 1에 가까울수록 평활화 강함)
STEERING_SMOOTHING = 0.9  # 조향 평활화 계수 (0~1, 1에 가까울수록 평활화 강함)
GOAL_WEIGHT = 2.0  # 목표 방향 가중치 (목표 벡터의 영향력)
SPEED_FACTOR = 0.8  # 조향에 따른 속도 감소 계수 (0~1, 1에 가까울수록 속도 감소 큼)

# 명령별 가중치
WEIGHT_FACTORS = {
    "D": 0.5,  # 오른쪽 조향 가중치
    "A": 0.5,  # 왼쪽 조향 가중치
    "W": 0.5,  # 직진 가중치
    "S": 1.0   # 후진 가중치
}

# P제어 및 I제어 파라미터
CONTROL_PARAMS = {
    "target_val_kh": 30.0,      # 목표 속도 (km/h)
    "kp_val": 0.18,             # 비례 게인 (P 제어)
    "ki_val": 0.0,             # 적분 게인 (I 제어)
    "speed_smoothing": 0.9,     # 속도 평활화 계수 (0~1)
    "integral_limit": 100.0     # 적분항 상한 (Wind-up 방지, 단위: 임의)
}

# 장애물 감지 및 회피 관련 설정
OBSTACLE_RADIUS = 1.0           # 장애물 반경 (미터, 충돌 검사 기준)
OBSTACLE_DETECTION_RANGE = 10.0 # 장애물 감지 최대 거리 (미터)
OBSTACLE_SAFE_DISTANCE = 2.0    # 장애물로부터 안전 거리 (미터)
OBSTACLE_INFLUENCE_WEIGHT = 1.5 # 장애물 회피 영향력 가중치 (회피 강도 조절)
OBSTACLE_DECAY_FACTOR = 0.8     # 거리에 따른 장애물 영향력 감소 계수 (0~1)