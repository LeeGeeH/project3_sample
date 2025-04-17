# navigation_core.py
# 핵심 내비게이션 로직 및 Pure Pursuit 알고리즘 처리

import math
import random
import numpy as np
from position_handler import PositionHandler
from controller import PIController
from config import (
    MOVE_STEP, TOLERANCE, LOOKAHEAD_MIN, LOOKAHEAD_MAX,
    GOAL_WEIGHT, WEIGHT_FACTORS, SPEED_FACTOR, STEERING_SMOOTHING
)

class Navigation:
    def __init__(self):
        self.position_handler = PositionHandler()
        self.controller = PIController()
        self.destination = None
        self.initial_distance = None
        self.last_command = None
        self.last_steering = 0.0
        # 초기화 정보 추가
        self.start_mode = "start"  # 기본값: 시뮬레이션 시작
        self.blue_tank_position = None  # 아군 전차 위치 (x, y, z)
        self.red_tank_position = None  # 적 전차 위치 (x, y, z)

    def init_simulation(self):
        """시뮬레이션 초기화 정보를 설정하고 반환."""
        # 기본 초기화 값 (config.py에서 가져오거나 하드코딩)
        default_init = {
            "startMode": self.start_mode,
            "blStartX": 100.0,
            "blStartY": 100.0,
            "blStartZ": 100.23,
            "rdStartX": 59.0,
            "rdStartY": 10.0,
            "rdStartZ": 280.0
        }

        # 아군 전차 위치 설정
        self.blue_tank_position = (
            default_init["blStartX"],
            default_init["blStartY"],
            default_init["blStartZ"]
        )
        # 아군 전차의 초기 위치를 position_handler에 설정
        self.position_handler.current_position = (
            default_init["blStartX"],
            default_init["blStartZ"]
        )

        # 적 전차 위치 설정
        self.red_tank_position = (
            default_init["rdStartX"],
            default_init["rdStartY"],
            default_init["rdStartZ"]
        )

        # startMode에 따라 동작 설정 (예: pause일 경우 이동 중지)
        self.start_mode = default_init["startMode"]

        return default_init

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

    def get_move(self):
        """Pure Pursuit 알고리즘과 P/I 제어를 사용하여 이동 명령 계산."""
        # startMode가 pause일 경우 이동 중지
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

        # Pure Pursuit 알고리즘
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
        dx = lookahead_x - curr_x
        dz = lookahead_z - curr_z
        target_heading = math.atan2(dx, dz)

        heading_error = target_heading - self.position_handler.current_heading
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        curvature = 2.0 * math.sin(heading_error) / max(lookahead_distance, 0.01)
        steering = STEERING_SMOOTHING * self.last_steering + (1 - STEERING_SMOOTHING) * curvature
        self.last_steering = steering

        # P/I 제어로 속도 계산
        speed_ms = self.controller.compute_speed(self.position_handler.current_speed_kh)

        # 조향에 따른 속도 감소
        abs_steering = abs(steering)
        speed_ms = speed_ms * (1.0 - abs_steering * SPEED_FACTOR)

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