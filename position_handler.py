# position_handler.py
# 위치 및 방향 업데이트 처리

import math
import time
from config import HEADING_SMOOTHING, CONTROL_PARAMS

class PositionHandler:
    def __init__(self):
        self.current_position = None  # (x, z)
        self.current_heading = 0.0  # 현재 방향 (라디안)
        self.current_speed_kh = 0.0  # 현재 속도 (km/h)
        self.smoothed_speed_kh = 0.0  # 평활화된 속도
        self.last_update_time = time.time()  # 마지막 업데이트 시간

    def update_position(self, position_str):
        """새 위치 데이터를 기반으로 현재 위치, 방향, 속도를 업데이트."""
        try:
            now = time.time()
            dt = now - self.last_update_time
            self.last_update_time = now

            x, y, z = map(float, position_str.split(","))
            new_position = (x, z)

            if self.current_position:
                prev_x, prev_z = self.current_position
                dx = x - prev_x
                dz = z - prev_z

                distance_moved = math.sqrt(dx**2 + dz**2)
                if distance_moved > 0.01:
                    new_heading = math.atan2(dx, dz)
                    self.current_heading = (
                        HEADING_SMOOTHING * self.current_heading + 
                        (1 - HEADING_SMOOTHING) * new_heading
                    )
                    self.current_heading = math.atan2(
                        math.sin(self.current_heading), math.cos(self.current_heading)
                    )
                    # 속도 계산 (km/h) 및 평활화
                    if dt > 0:
                        raw_speed_kh = (distance_moved / dt) * 3.6
                        smoothing = CONTROL_PARAMS["speed_smoothing"]
                        self.smoothed_speed_kh = (
                            smoothing * self.smoothed_speed_kh + 
                            (1 - smoothing) * raw_speed_kh
                        )
                        self.current_speed_kh = self.smoothed_speed_kh

            self.current_position = new_position
            return {
                "status": "OK",
                "current_position": self.current_position,
                "heading": math.degrees(self.current_heading),
                "speed_kh": self.current_speed_kh
            }
        except Exception as e:
            return {"status": "ERROR", "message": str(e)}