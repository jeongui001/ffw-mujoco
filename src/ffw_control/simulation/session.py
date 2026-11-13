"""Shared two-panel targets, arm-first policy, and physical tracking."""

import time

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from ffw_control.control.dynamics.servo import JointServo
from ffw_control.control.hierarchy.qp import QPFailure
from ffw_control.control.ik.differential import DifferentialIK
from ffw_control.robots.model import Pose, Robot


class ControlSession:
    def __init__(self, robot=None):
        self.robot = robot or Robot()
        self.data = self.robot.new_data()
        self.reference = self.robot.new_data()
        self.servo = JointServo(self.robot, self.data)
        self.ik = DifferentialIK(self.robot)
        self.initial_poses = {s: self.robot.pose(self.data, s) for s in ("right", "left")}
        self.targets = {s: p.copy() for s, p in self.initial_poses.items()}
        self.rpy = {s: np.zeros(3) for s in self.targets}
        self.active_side = "right"
        self.held_pose = self.targets["left"].copy()
        self.mode = "ARM"
        self.arm_solution_available = True
        self.status = "Ready"
        self.previous_velocity = np.zeros(15)
        self.position_target = self.robot.nominal.copy()
        self.started_at = time.monotonic()
        self.steps = 0
        self.last_loop_seconds = 0.0

    def set_target(self, side, position, rpy_degrees):
        self.robot.arm_slice(side)
        position = np.asarray(position, dtype=float)
        angles = np.asarray(rpy_degrees, dtype=float)
        if position.shape != (3,) or angles.shape != (3,) or not np.isfinite(np.r_[position, angles]).all():
            raise ValueError("Target must contain six finite position/angle values")
        if side != self.active_side:
            passive = self.active_side
            self.held_pose = self.robot.pose(self.data, passive)
            self.targets[passive] = self.held_pose.copy()
            relative = self.held_pose.rotation @ self.initial_poses[passive].rotation.T
            self.rpy[passive] = Rotation.from_matrix(relative).as_euler("xyz", degrees=True)
            self.active_side = side
        self.targets[side] = Pose(position.copy(), Rotation.from_euler(
            "xyz", angles, degrees=True).as_matrix() @ self.initial_poses[side].rotation)
        self.rpy[side] = angles.copy()
        try:
            feasible = self.ik.arm_feasible(self.reference, side, self.targets[side])
            # Latch the decision for this target: no per-tick mode chattering.
            self.arm_solution_available = feasible
            passive = "left" if side == "right" else "right"
            moving_passive = np.r_[self.previous_velocity[self.robot.arm_slice(passive)],
                                    self.previous_velocity[-1]]
            self.mode = "ARM" if feasible and np.max(np.abs(moving_passive)) < 1e-7 else "LIFT"
            self.status = "Tracking" if feasible else "Lift enabled (arm residual)"
        except QPFailure as error:
            self.status = f"Feasibility check failed: {error}"
            self.mode = "STOP"

    def tick(self):
        started = time.monotonic()
        dt = self.robot.config["control_dt"]
        if self.mode != "STOP":
            try:
                velocity = self.ik.velocity(
                    self.reference, self.active_side, self.targets[self.active_side],
                    self.held_pose, self.mode == "LIFT", dt, self.previous_velocity,
                )
                self.position_target += dt * velocity
                self.reference.qpos[self.robot.qpos_ids] = self.position_target
                self.robot.update_kinematics(self.reference)
                self.previous_velocity = velocity
                errors = self.ik.errors(self.reference, self.active_side, self.targets[self.active_side])
                self.status = "IK reached / settling" if self.ik.reached(errors) else "Tracking / limited"
                passive = "left" if self.active_side == "right" else "right"
                passive_speed = np.r_[velocity[self.robot.arm_slice(passive)], velocity[-1]]
                if self.arm_solution_available and np.max(np.abs(passive_speed)) < 1e-7:
                    self.mode = "ARM"
            except QPFailure as error:
                # Hold the last valid target; never integrate an infeasible QP result.
                self.previous_velocity[:] = 0
                self.mode = "STOP"
                self.status = f"Stopped: {error}"
        substeps = round(dt / self.robot.model.opt.timestep)
        for _ in range(substeps):
            mujoco.mj_step1(self.robot.model, self.data)
            self.servo.apply(self.data, self.position_target, self.previous_velocity)
            mujoco.mj_step2(self.robot.model, self.data)
        mujoco.mj_forward(self.robot.model, self.data)
        if self.mode != "STOP":
            actual_errors = self.ik.errors(self.data, self.active_side, self.targets[self.active_side])
            passive = "left" if self.active_side == "right" else "right"
            held_error = abs(self.robot.pose(self.data, passive).position[2] - self.held_pose.position[2])
            if self.ik.reached(actual_errors) and held_error <= 0.002:
                self.status = "Reached"
        self.steps += 1
        self.last_loop_seconds = time.monotonic() - started

    def metrics(self):
        passive = "left" if self.active_side == "right" else "right"
        errors = self.ik.errors(self.data, self.active_side, self.targets[self.active_side])
        predicted = self.ik.errors(self.reference, self.active_side, self.targets[self.active_side])
        return {
            "active_hand": self.active_side, "mode": self.mode, "status": self.status,
            "position_error_m": float(errors[0]), "orientation_error_deg": float(np.rad2deg(errors[1])),
            "ik_position_error_m": float(predicted[0]),
            "ik_orientation_error_deg": float(np.rad2deg(predicted[1])),
            "held_height_error_m": float(self.robot.pose(self.data, passive).position[2]
                                           - self.held_pose.position[2]),
            "lift_m": float(self.data.qpos[self.robot.qpos_ids[-1]]),
            "sim_time": float(self.data.time), "wall_time": time.monotonic() - self.started_at,
            "compute_ms": self.last_loop_seconds * 1000, "saturated_actuators": self.servo.saturated,
        }
