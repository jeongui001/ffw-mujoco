"""Bounded dual-arm differential IK and fixed-lift feasibility search."""

import numpy as np

from ffw_control.control.hierarchy.qp import QPFailure, solve_hierarchy, solve_qp
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

    def arm_feasible(self, source, side, target):
        """Test kinematic residual, independently of physical tracking delay."""
        robot = self.robot
        data = robot.new_data()
        data.qpos[:] = source.qpos
        robot.update_kinematics(data)
        section = robot.arm_slice(side)
        indices = robot.qpos_ids[section]
        speed = robot.velocity_limits[section]
        dt = 0.05
        for _ in range(self.config["feasibility_iterations"]):
            if self.reached(self.errors(data, side, target)):
                return True
            jacobian = robot.jacobian(data, side)[:, section] * speed
            error = pose_error(robot.pose(data, side), target)
            task = self.task_scale @ jacobian
            desired = self.task_scale @ self.task_velocity(error)
            lower, upper = self.bounds(data, dt)
            # Tiny damping selects a bounded minimum-motion solution near singularities.
            matrix = np.vstack((task, 1e-3 * np.eye(7)))
            velocity = solve_qp(matrix, np.concatenate((desired, np.zeros(7))),
                                np.eye(7), lower[section], upper[section]) * speed
            data.qpos[indices] += dt * velocity
            robot.update_kinematics(data)
        return self.reached(self.errors(data, side, target))

    def velocity(self, data, side, target, held_pose, allow_lift, dt, previous=None):
        robot = self.robot
        speed = robot.velocity_limits
        passive = "left" if side == "right" else "right"
        lower, upper = self.bounds(data, dt, previous)
        if not allow_lift:
            lower[robot.arm_slice(passive)] = 0
            upper[robot.arm_slice(passive)] = 0
            lower[-1] = upper[-1] = 0
        constraints = np.eye(15)

        passive_jacobian = robot.jacobian(data, passive) * speed
        height_error = held_pose.position[2] - robot.pose(data, passive).position[2]
        tolerance = self.config["held_height_tolerance"]
        constraints = np.vstack((constraints, passive_jacobian[2]))
        lower = np.append(lower, (height_error - tolerance) / dt)
        upper = np.append(upper, (height_error + tolerance) / dt)

        jacobian = robot.jacobian(data, side) * speed
        desired = self.task_velocity(pose_error(robot.pose(data, side), target))
        primary = (self.task_scale @ jacobian, self.task_scale @ desired)
        lift = np.zeros((1, 15))
        lift[0, -1] = 1

        position = data.qpos[robot.qpos_ids]
        posture_velocity = 0.15 * (robot.nominal - position) / speed
        previous_scaled = np.zeros(15) if previous is None else previous / speed
        # Full passive-hand pose is a soft preference; only height is mandatory.
        passive_desired = self.task_velocity(pose_error(robot.pose(data, passive), held_pose))
        secondary_matrix = np.vstack((np.eye(15), 0.5 * np.eye(15),
                                      0.1 * np.eye(15), passive_jacobian))
        secondary_target = np.concatenate((np.zeros(15), 0.5 * previous_scaled,
                                           0.1 * posture_velocity, passive_desired))
        tasks = [primary]
        if allow_lift:
            tasks.append((lift, np.zeros(1)))
        tasks.append((secondary_matrix, secondary_target))
        return solve_hierarchy(tasks, constraints, lower, upper) * speed
