import math
import random
import numpy as np
from position_handler import PositionHandler
from controller import PIController
from config import (
    MOVE_STEP, TOLERANCE, LOOKAHEAD_MIN, LOOKAHEAD_MAX,
    GOAL_WEIGHT, WEIGHT_FACTORS, SPEED_FACTOR, STEERING_SMOOTHING,
    OBSTACLE_RADIUS, CONTROL_PARAMS
)

class Navigation:
    def __init__(self):
        self.position_handler = PositionHandler()
        self.controller = PIController()
        self.destination = None
        self.initial_distance = None
        self.last_command = None
        self.last_steering = 0.0
        self.start_mode = "start"
        self.blue_tank_position = None
        self.red_tank_position = None
        self.obstacles = []

    def set_destination(self, destination_str):
        """목적지를 설정하고 초기 거리를 계산."""
        try:
            x, y, z = map(float, destination_str.split(","))
            self.destination = (x, z)
            self.controller.reset_integral()

            if self.position_handler.current_position:
                curr_x, curr_z = self.position_handler.current_position
                self.initial_distance = math.sqrt((x - curr_x) ** 2 + (z - curr_z) ** 2)

            return {
                "status": "OK",
                "destination": {"x": x, "y": y, "z": z},
                "initial_distance": self.initial_distance
            }
        except Exception as e:
            return {"status": "ERROR", "message": str(e)}

    def add_obstacle(self, x, z):
        """장애물 추가."""
        try:
            x, z = float(x), float(z)
            self.obstacles.append((x, z))
            return {"status": "OK", "message": f"Obstacle added at ({x}, {z})"}
        except ValueError:
            return {"status": "ERROR", "message": "Invalid obstacle coordinates"}

    def update_obstacle(self, obstacle_data):
        """Flask 라우트에서 호출되는 장애물 업데이트 메서드."""
        try:
            x = obstacle_data.get("x")
            z = obstacle_data.get("z")
            if x is None or z is None:
                return {"status": "ERROR", "message": "Missing x or z coordinates"}
            return self.add_obstacle(x, z)
        except Exception as e:
            return {"status": "ERROR", "message": str(e)}

    def is_obstacle_in_path(self, curr_x, curr_z, target_x, target_z):
        """장애물이 경로에 있는지 확인 (선-원 충돌 검사)."""
        for obs_x, obs_z in self.obstacles:
            dist_to_line = abs((target_z - curr_z) * obs_x - (target_x - curr_x) * obs_z + target_x * curr_z - target_z * curr_x) / math.sqrt((target_z - curr_z) ** 2 + (target_x - curr_x) ** 2)
            if dist_to_line < OBSTACLE_RADIUS:
                return True
        return False

    def get_move(self):
        """Pure Pursuit 알고리즘과 P/I 제어를 사용하여 이동 명령 계산."""
        if self.start_mode == "pause":
            return {"move": "STOP", "weight": 1.0}

        if self.position_handler.current_position is None or self.destination is None:
            return {"move": "STOP", "weight": 1.0}

        curr_x, curr_z = self.position_handler.current_position
        dest_x, dest_z = self.destination
        distance = math.sqrt((dest_x - curr_x) ** 2 + (dest_z - curr_z) ** 2)

        if distance < TOLERANCE:
            self.initial_distance = None
            self.controller.reset_integral()
            return {"move": "STOP", "weight": 1.0}

        lookahead_distance = min(LOOKAHEAD_MAX, max(LOOKAHEAD_MIN, distance * 0.5))
        goal_vector = np.array([dest_x - curr_x, dest_z - curr_z])
        goal_distance = np.linalg.norm(goal_vector)

        if goal_distance > 0:
            goal_vector = goal_vector / goal_distance

        target_vector = goal_vector * GOAL_WEIGHT
        target_vector_norm = np.linalg.norm(target_vector)

        if target_vector_norm > 0:
            target_vector = target_vector / target_vector_norm
            target_heading = math.atan2(target_vector[0], target_vector[1])
        else:
            target_heading = math.atan2(goal_vector[0], goal_vector[1])

        lookahead_x = curr_x + target_vector[0] * lookahead_distance
        lookahead_z = curr_z + target_vector[1] * lookahead_distance

        if self.is_obstacle_in_path(curr_x, curr_z, lookahead_x, lookahead_z):
            return {"move": "STOP", "weight": 1.0, "message": "Obstacle detected in path"}

        dx = lookahead_x - curr_x
        dz = lookahead_z - curr_z
        target_heading = math.atan2(dx, dz)

        heading_error = target_heading - self.position_handler.current_heading
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        curvature = 2.0 * math.sin(heading_error) / max(lookahead_distance, 0.01)
        steering = STEERING_SMOOTHING * self.last_steering + (1 - STEERING_SMOOTHING) * curvature
        self.last_steering = steering

        speed_ms = self.controller.compute_speed(self.position_handler.current_speed_kh)

        abs_steering = abs(steering)
        speed_ms = speed_ms * (1.0 - abs_steering * SPEED_FACTOR)

        # 최대 속도 제한 (target_val_kh 기준)
        speed_ms = min(speed_ms, CONTROL_PARAMS["target_val_kh"] / 3.6)

        progress = max(0, 1 - distance / self.initial_distance) if self.initial_distance and distance > 0 else 0.0

        dynamic_weights = {
            "D": WEIGHT_FACTORS["D"] * (1 + abs_steering * 2) if steering > 0 else 0.0,
            "A": WEIGHT_FACTORS["A"] * (1 + abs_steering * 2) if steering < 0 else 0.0,
            "W": WEIGHT_FACTORS["W"] * speed_ms,
            "S": WEIGHT_FACTORS["S"] if heading_error > math.pi * 0.6 else 0.0
        }

        for cmd in dynamic_weights:
            if dynamic_weights[cmd] > 0:
                dynamic_weights[cmd] *= (1 + progress * 0.5)

        commands = [cmd for cmd, w in dynamic_weights.items() if w > 0]
        if not commands:
            command = {"move": "STOP"}
        else:
            weights = [dynamic_weights[cmd] for cmd in commands]
            chosen_cmd = random.choices(commands, weights=weights, k=1)[0]
            command = {"move": chosen_cmd, "weight": dynamic_weights[chosen_cmd]}
            self.last_command = chosen_cmd

        if self.last_command:
            move_distance = MOVE_STEP * speed_ms
            new_x, new_z = curr_x, curr_z

            if self.last_command == "D":
                new_x += move_distance * math.cos(self.position_handler.current_heading + math.pi/2)
                new_z += move_distance * math.sin(self.position_handler.current_heading + math.pi/2)
            elif self.last_command == "A":
                new_x += move_distance * math.cos(self.position_handler.current_heading - math.pi/2)
                new_z += move_distance * math.sin(self.position_handler.current_heading - math.pi/2)
            elif self.last_command == "W":
                new_x += move_distance * math.sin(self.position_handler.current_heading)
                new_z += move_distance * math.cos(self.position_handler.current_heading)
            elif self.last_command == "S":
                new_x -= move_distance * math.sin(self.position_handler.current_heading)
                new_z -= move_distance * math.cos(self.position_handler.current_heading)

            self.position_handler.current_position = (new_x, new_z)

        return command