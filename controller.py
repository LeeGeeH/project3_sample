# controller.py
# P/I 제어 로직 처리

import time
from config import CONTROL_PARAMS

class PIController:
    def __init__(self):
        self.integral_error = 0.0  # 적분 오차
        self.last_speed_update_time = time.time()  # 마지막 속도 업데이트 시간

    def reset_integral(self):
        """적분항 초기화."""
        self.integral_error = 0.0

    def compute_speed(self, current_speed_kh):
        """P/I 제어를 사용하여 속도 계산."""
        target_val_kh = CONTROL_PARAMS["target_val_kh"]
        kp_val = CONTROL_PARAMS["kp_val"]
        ki_val = CONTROL_PARAMS["ki_val"]
        integral_limit = CONTROL_PARAMS["integral_limit"]

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
        # 음수 속도 방지
        return max(0.0, speed_ms)