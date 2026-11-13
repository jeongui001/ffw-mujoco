"""Independent thumb/four-finger targets through the existing position servos."""

import json

import numpy as np

from ffw_control.robots.model import PROJECT_ROOT


class FingerController:
    def __init__(self, robot, data):
        self.robot = robot
        config = json.loads((PROJECT_ROOT / "configs/fingers.json").read_text())
        self.velocity_limit = float(config["joint_velocity_limit"])
        if not np.isfinite(self.velocity_limit) or self.velocity_limit <= 0:
            raise ValueError("Finger joint velocity limit must be positive and finite")
        self.targets = {side: np.zeros(2) for side in ("right", "left")}
        self.qpos_ids = {}
        self.actuator_ids = {}
        self.limits = {}
        self.opened = {}
        self.closed = {}
        self.commanded_positions = {}
        for side, prefix in (("right", "r"), ("left", "l")):
            names = [f"finger_{prefix}_joint{index}" for index in range(1, 21)]
            joint_ids = np.array([robot.model.joint(name).id for name in names])
            self.qpos_ids[side] = robot.model.jnt_qposadr[joint_ids]
            self.actuator_ids[side] = np.array([robot.model.actuator(name).id for name in names])
            self.limits[side] = robot.model.jnt_range[joint_ids].copy()
            self.opened[side] = np.array(config["thumb_open"] + config["four_fingers_open"] * 4)
            self.closed[side] = np.array(config["thumb_closed"][side] + config["four_fingers_closed"] * 4)
            for endpoint in (self.opened[side], self.closed[side]):
                if (endpoint.shape != (20,) or not np.isfinite(endpoint).all()
                        or np.any(endpoint < self.limits[side][:, 0])
                        or np.any(endpoint > self.limits[side][:, 1])):
                    raise ValueError(f"Invalid finger posture for {side}")
            self.commanded_positions[side] = data.qpos[self.qpos_ids[side]].copy()

    def set_targets(self, side, thumb, four_fingers):
        if side not in self.targets:
            raise ValueError(f"Unknown hand: {side}")
        values = np.asarray([thumb, four_fingers], dtype=float)
        if values.shape != (2,) or not np.isfinite(values).all() or np.any((values < 0) | (values > 1)):
            raise ValueError("Finger targets must be finite values from 0 (open) to 1 (closed)")
        self.targets[side] = values.copy()

    def apply(self, data, dt):
        for side, values in self.targets.items():
            # Thumb joints 1-4; four fingers each have four joints (5-20).
            fraction = np.repeat(values, [4, 16])
            goal = self.opened[side] + fraction * (self.closed[side] - self.opened[side])
            command = self.commanded_positions[side]
            command += np.clip(goal - command, -self.velocity_limit * dt, self.velocity_limit * dt)
            data.ctrl[self.actuator_ids[side]] = command
