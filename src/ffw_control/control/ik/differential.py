"""Bounded dual-arm differential IK with shared position and pose priorities."""

import numpy as np

from ffw_control.control.hierarchy.qp import QPFailure, solve_hierarchy
from ffw_control.robots.model import pose_error


class DifferentialIK:
    def __init__(self, robot):
        self.robot = robot
        self.config = robot.config
        # Translate radian errors into a comparable task length, not Euler deltas.
        self.task_scale = np.diag([1, 1, 1, 0.2, 0.2, 0.2])

    def errors(self, data, side, target):
        error = pose_error(self.robot.pose(data, side), target)
        return np.linalg.norm(error[:3]), np.linalg.norm(error[3:])

    def reached(self, errors, factor=1.0):
        return (errors[0] <= factor * self.config["position_tolerance"]
                and errors[1] <= factor * np.deg2rad(self.config["orientation_tolerance_degrees"]))

    def task_velocity(self, error):
        """Limit a position-only or full-pose correction velocity."""
        velocity = self.config["task_gain"] * error
        for section, limit in ((slice(0, 3), self.config["hand_linear_velocity_limit"]),
                               (slice(3, 6), self.config["hand_angular_velocity_limit"])):
            norm = np.linalg.norm(velocity[section])
            if norm > limit:
                velocity[section] *= limit / norm
        return velocity

    def bounds(self, data, dt, previous=None):
        robot = self.robot
        position = data.qpos[robot.qpos_ids]
        lower = np.maximum(-robot.velocity_limits, (robot.limits[:, 0] - position) / dt)
        upper = np.minimum(robot.velocity_limits, (robot.limits[:, 1] - position) / dt)
        if previous is not None:
            # Leave enough distance to brake before a joint limit.
            acceleration = robot.acceleration_limits
            below = np.maximum(0, position - robot.limits[:, 0])
            above = np.maximum(0, robot.limits[:, 1] - position)
            braking_step = acceleration * dt
            lower = np.maximum(lower, braking_step - np.sqrt(braking_step**2 + 2 * acceleration * below))
            upper = np.minimum(upper, -braking_step + np.sqrt(braking_step**2 + 2 * acceleration * above))
            lower = np.maximum(lower, previous - acceleration * dt)
            upper = np.minimum(upper, previous + acceleration * dt)
        if np.any(lower > upper + 1e-9):
            raise QPFailure("Joint braking/acceleration constraints conflict")
        return lower / robot.velocity_limits, upper / robot.velocity_limits

    def velocity(self, data, side, target, held_position, dt, previous=None):
        robot = self.robot
        speed = robot.velocity_limits
        passive = "left" if side == "right" else "right"
        lower, upper = self.bounds(data, dt, previous)
        constraints = np.eye(15)

        jacobian = robot.jacobian(data, side) * speed
        desired = self.task_velocity(pose_error(robot.pose(data, side), target))
        passive_jacobian = robot.jacobian(data, passive)[:3] * speed
        passive_desired = self.task_velocity(held_position - robot.pose(data, passive).position)
        # Both positions have equal weight; passive-hand orientation is free.
        primary = (np.vstack((self.task_scale @ jacobian, passive_jacobian)),
                   np.concatenate((self.task_scale @ desired, passive_desired)))

        position = data.qpos[robot.qpos_ids]
        # The lift can settle at any height instead of returning to its start.
        posture_matrix = np.eye(15)[:14]
        posture_velocity = 0.15 * (robot.nominal[:14] - position[:14]) / speed[:14]
        previous_scaled = np.zeros(15) if previous is None else previous / speed
        secondary_matrix = np.vstack((np.eye(15), 0.5 * np.eye(15),
                                      0.1 * posture_matrix))
        secondary_target = np.concatenate((np.zeros(15), 0.5 * previous_scaled,
                                           0.1 * posture_velocity))
        tasks = [primary, (secondary_matrix, secondary_target)]
        return solve_hierarchy(tasks, constraints, lower, upper) * speed
