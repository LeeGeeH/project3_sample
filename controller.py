import time
from config import CONTROL_PARAMS

class PIController:
    def __init__(self):
        self.integral_error = 0.0  # 적분 오차
        self.last_speed_update_time = time.time()  # 마지막 속도 업데이트 시간
        self.prev_speed_ms = 0.0  # 이전 속도 (평활화를 위해 추가)

    def reset_integral(self):
        """적분항 초기화."""
        self.integral_error = 0.0

    def compute_speed(self, current_speed_kh):
        """P/I 제어를 사용하여 속도 계산."""
        target_val_kh = CONTROL_PARAMS["target_val_kh"]
        kp_val = CONTROL_PARAMS["kp_val"]
        ki_val = CONTROL_PARAMS["ki_val"]
        integral_limit = CONTROL_PARAMS["integral_limit"]
        speed_smoothing = CONTROL_PARAMS["speed_smoothing"]

        # 현재 속도 오차
        error_kh = target_val_kh - current_speed_kh

        # 적분항 업데이트
        now = time.time()
        dt = now - self.last_speed_update_time if now > self.last_speed_update_time else 0.01
        self.last_speed_update_time = now
        self.integral_error += error_kh * dt
        # Wind-up 방지: 적분항 상한/하한 설정
        self.integral_error = max(min(self.integral_error, integral_limit), -integral_limit)

        # P제어 + I제어
        controller_output = kp_val * error_kh + ki_val * self.integral_error
        # 속도를 km/h에서 m/s로 변환
        speed_ms = controller_output / 3.6  # km/h -> m/s

        # 최대 속도 제한 (target_val_kh 기준)
        speed_ms = min(speed_ms, target_val_kh / 3.6)

        # 속도 평활화 적용
        speed_ms = speed_smoothing * self.prev_speed_ms + (1 - speed_smoothing) * speed_ms
        self.prev_speed_ms = speed_ms

        # 음수 속도 방지
        return max(0.0, speed_ms)