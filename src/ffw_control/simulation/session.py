"""Shared two-panel targets, arm-first policy, and physical tracking."""

import time
from concurrent.futures import ThreadPoolExecutor

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from ffw_control.control.dynamics.servo import JointServo
from ffw_control.control.fingers import FingerController
from ffw_control.control.joints import JointController
from ffw_control.control.hierarchy.qp import QPFailure
from ffw_control.control.ik.differential import DifferentialIK
from ffw_control.robots.model import Pose, Robot
from ffw_control.simulation.timing import LoopTiming


class ControlSession:
    def __init__(self, robot=None):
        self.robot = robot or Robot()
        self.data = self.robot.new_data()
        self.reference = self.robot.new_data()
        self.servo = JointServo(self.robot, self.data)
        self.fingers = FingerController(self.robot, self.data)
        self.joints = JointController(self.robot, self.data, self.fingers.velocity_limit)
        self.ik = DifferentialIK(self.robot)
        self.initial_poses = {s: self.robot.pose(self.data, s) for s in ("right", "left")}
        self.targets = {s: p.copy() for s, p in self.initial_poses.items()}
        self.rpy = {s: np.zeros(3) for s in self.targets}
        self.active_side = "right"
        self.held_pose = self.targets["left"].copy()
        self.mode = "ARM"
        self.control_mode = "IK"
        self.paused = False
        self.timing = LoopTiming(self.robot.config["loop_frequency_ema_alpha"])
        self.arm_solution_available = True
        self.status = "Ready"
        self.previous_velocity = np.zeros(15)
        self.position_target = self.robot.nominal.copy()
        self.started_at = time.monotonic()
        self.steps = 0
        self.last_loop_seconds = 0.0
        self._planner = None
        self._future = None
        self._pending_plan = None
        self._plan_revision = 0
        self._future_revision = 0

    def set_finger_targets(self, side, thumb, four_fingers):
        if self.control_mode != "IK":
            raise ValueError("Return to IK mode before using finger presets")
        # Finger input does not change the active hand or replan its pose.
        self.fingers.set_targets(side, thumb, four_fingers)

    def set_target(self, side, position, rpy_degrees, asynchronous=False):
        if self.control_mode != "IK":
            raise ValueError("Return to IK mode before editing hand targets")
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
        self._plan_revision += 1
        if asynchronous:
            self._pending_plan = (self._plan_revision, side, self.targets[side].copy())
            self.mode = "ARM" if self._passive_can_stop() else "LIFT"
            self.status = "Checking arm reach"
            self._start_pending_plan()
            return
        try:
            feasible = self.ik.arm_feasible(self.reference, side, self.targets[side])
            self._choose_mode(feasible)
        except QPFailure as error:
            self.status = f"Feasibility check failed: {error}"
            self.mode = "STOP"

    def set_control_mode(self, mode):
        if mode not in ("IK", "JOINT"):
            raise ValueError(f"Unknown control mode: {mode}")
        if mode == self.control_mode:
            return
        self._plan_revision += 1
        self._pending_plan = None
        self.joints.capture(self.data)
        self.control_mode = mode
        self.position_target = np.clip(self.data.qpos[self.robot.qpos_ids],
                                       self.robot.limits[:, 0], self.robot.limits[:, 1])
        self.previous_velocity[:] = 0
        if mode == "JOINT":
            self.mode = "JOINT"
            self.status = "Direct joints / Cartesian holds off"
            return
        self.reference.qpos[:] = self.data.qpos
        self.reference.qpos[self.robot.qpos_ids] = self.position_target
        self.robot.update_kinematics(self.reference)
        self.targets = {side: self.robot.pose(self.data, side) for side in self.targets}
        for side, pose in self.targets.items():
            relative = pose.rotation @ self.initial_poses[side].rotation.T
            self.rpy[side] = Rotation.from_matrix(relative).as_euler("xyz", degrees=True)
        passive = "left" if self.active_side == "right" else "right"
        self.held_pose = self.targets[passive].copy()
        self.fingers.capture(self.data)
        self.arm_solution_available = True
        self.mode = "ARM"
        self.status = "Holding measured pose"

    def set_joint_target(self, name, value):
        if self.control_mode != "JOINT":
            raise ValueError("Enable direct joint control before editing joints")
        self.joints.set_target(name, value)

    def set_paused(self, paused):
        self.paused = bool(paused)
        self.timing.reset()

    def joint_states(self):
        if self.control_mode == "JOINT":
            targets = self.joints.targets.copy()
        else:
            targets = self.data.ctrl[self.joints.actuator_ids].copy()
            targets[self.robot.joint_ids] = self.position_target
        return self.joints.states(self.data, targets)

    def _passive_can_stop(self):
        passive = "left" if self.active_side == "right" else "right"
        first_joint = 0 if passive == "right" else 7
        indices = list(range(first_joint, first_joint + 7)) + [14]
        stopping_step = self.robot.acceleration_limits[indices] * self.robot.config["control_dt"]
        return np.all(np.abs(self.previous_velocity[indices]) <= stopping_step + 1e-8)

    def _choose_mode(self, feasible):
        # Keep the decision for this target. An active lift must brake before
        # switching to the fixed-lift mode, rather than stopping instantaneously.
        self.arm_solution_available = feasible
        self.mode = "ARM" if feasible and self._passive_can_stop() else "LIFT"
        self.status = "Tracking" if feasible else "Lift enabled (arm residual)"

    def _start_pending_plan(self):
        if self._pending_plan is None or self._future is not None:
            return
        if self._planner is None:
            self._planner = ThreadPoolExecutor(max_workers=1, thread_name_prefix="ffw-arm-check")
        revision, side, target = self._pending_plan
        self._pending_plan = None
        snapshot = self.robot.new_data()
        snapshot.qpos[:] = self.reference.qpos
        self.robot.update_kinematics(snapshot)
        self._future_revision = revision
        self._future = self._planner.submit(self.ik.arm_feasible, snapshot, side, target)

    def _poll_plan(self):
        if self._future is None or not self._future.done():
            return
        try:
            feasible = self._future.result()
            if self._future_revision == self._plan_revision:
                self._choose_mode(feasible)
        except QPFailure as error:
            if self._future_revision == self._plan_revision:
                self.mode = "STOP"
                self.status = f"Feasibility check failed: {error}"
        self._future = None
        self._start_pending_plan()

    def close(self):
        if self._planner is not None:
            self._planner.shutdown(wait=True, cancel_futures=True)

    def tick(self):
        if self.paused:
            return
        started = time.monotonic()
        self.timing.record(started)
        self._poll_plan()
        dt = self.robot.config["control_dt"]
        self.joints.advance(dt)
        if self.control_mode == "JOINT":
            self.position_target = self.joints.commands[self.robot.joint_ids].copy()
            self.previous_velocity = self.joints.command_velocities[self.robot.joint_ids].copy()
        elif self.mode != "STOP":
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
                if self.arm_solution_available and self._passive_can_stop():
                    self.mode = "ARM"
            except QPFailure as error:
                # Hold the last valid target; never integrate an infeasible QP result.
                self.previous_velocity[:] = 0
                self.mode = "STOP"
                self.status = f"Stopped: {error}"
                self._plan_revision += 1
                self._pending_plan = None
        substeps = round(dt / self.robot.model.opt.timestep)
        for _ in range(substeps):
            mujoco.mj_step1(self.robot.model, self.data)
            self.servo.apply(self.data, self.position_target, self.previous_velocity)
            self.joints.apply_native(self.data, include_fingers=self.control_mode == "JOINT")
            if self.control_mode == "IK":
                self.fingers.apply(self.data, self.robot.model.opt.timestep)
            mujoco.mj_step2(self.robot.model, self.data)
        mujoco.mj_forward(self.robot.model, self.data)
        if self.control_mode == "IK" and self.mode != "STOP":
            actual_errors = self.ik.errors(self.data, self.active_side, self.targets[self.active_side])
            passive = "left" if self.active_side == "right" else "right"
            held_error = abs(self.robot.pose(self.data, passive).position[2] - self.held_pose.position[2])
            if self.ik.reached(actual_errors) and held_error <= self.robot.config["held_height_tracking_tolerance"]:
                self.status = "Reached"
        self.steps += 1
        self.last_loop_seconds = time.monotonic() - started

    def metrics(self):
        passive = "left" if self.active_side == "right" else "right"
        errors = self.ik.errors(self.data, self.active_side, self.targets[self.active_side])
        predicted = self.ik.errors(self.reference, self.active_side, self.targets[self.active_side])
        right_errors = self.ik.errors(self.reference, "right", self.targets["right"])
        return {
            "active_hand": self.active_side, "mode": self.mode, "status": self.status,
            "control_mode": self.control_mode, "paused": self.paused,
            "ik_err": float(right_errors[0]) if self.control_mode == "IK" else None,
            "right_ik_orientation_error_deg": float(np.rad2deg(right_errors[1])) if self.control_mode == "IK" else None,
            "loop_freq": self.timing.frequency, "loop_freq_ema": self.timing.ema,
            "position_error_m": float(errors[0]) if self.control_mode == "IK" else None,
            "orientation_error_deg": float(np.rad2deg(errors[1])) if self.control_mode == "IK" else None,
            "ik_position_error_m": float(predicted[0]) if self.control_mode == "IK" else None,
            "ik_orientation_error_deg": float(np.rad2deg(predicted[1])) if self.control_mode == "IK" else None,
            "held_height_error_m": float(self.robot.pose(self.data, passive).position[2]
                                           - self.held_pose.position[2]) if self.control_mode == "IK" else None,
            "lift_m": float(self.data.qpos[self.robot.qpos_ids[-1]]),
            "sim_time": float(self.data.time), "wall_time": time.monotonic() - self.started_at,
            "compute_ms": self.last_loop_seconds * 1000, "saturated_actuators": self.servo.saturated,
            "planning": self._future is not None and self._future_revision == self._plan_revision,
            "finger_targets": {
                side: {"thumb": float(values[0]), "four_fingers": float(values[1])}
                for side, values in self.fingers.targets.items()
            },
        }
