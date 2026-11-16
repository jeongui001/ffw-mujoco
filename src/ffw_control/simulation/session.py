"""Shared hand targets, two-level HQP, and physical tracking."""

import time

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
from ffw_control.simulation.objects import ObjectMonitor
from ffw_control.simulation.grasp import GraspSequence


class ControlSession:
    def __init__(self, robot=None):
        self.robot = robot or Robot()
        self.data = self.robot.new_data()
        self.reference = self.robot.new_data()
        self.servo = JointServo(self.robot, self.data)
        self.fingers = FingerController(self.robot, self.data)
        self.joints = JointController(self.robot, self.data, self.fingers.velocity_limit)
        self.objects = ObjectMonitor(self.robot.model)
        self.grasp_demo = None
        self.ik = DifferentialIK(self.robot)
        self.initial_poses = {s: self.robot.pose(self.data, s) for s in ("right", "left")}
        self.targets = {s: p.copy() for s, p in self.initial_poses.items()}
        self.rpy = {s: np.zeros(3) for s in self.targets}
        self.active_side = "right"
        self.held_position = self.targets["left"].position.copy()
        self.mode = "HQP"
        self.control_mode = "IK"
        self.paused = False
        self.timing = LoopTiming(self.robot.config["loop_frequency_ema_alpha"])
        self.status = "Ready"
        self.previous_velocity = np.zeros(15)
        self.position_target = self.robot.nominal.copy()
        self.started_at = time.monotonic()
        self.steps = 0
        self.last_loop_seconds = 0.0
        self._target_revision = 0

    def set_finger_targets(self, side, thumb, four_fingers):
        if self.control_mode != "IK":
            raise ValueError("Return to IK mode before using finger presets")
        # Finger input does not change the active hand or its pose target.
        self.fingers.set_targets(side, thumb, four_fingers)

    def start_grasp_demo(self, side):
        sequence = GraspSequence(self, side)
        self.cancel_grasp_demo()
        self.grasp_demo = sequence
        sequence.start()

    def cancel_grasp_demo(self):
        if self.grasp_demo is not None:
            self.grasp_demo.cancel()

    def set_target(self, side, position, rpy_degrees):
        if self.control_mode != "IK":
            raise ValueError("Return to IK mode before editing hand targets")
        self.robot.arm_slice(side)
        position = np.asarray(position, dtype=float)
        angles = np.asarray(rpy_degrees, dtype=float)
        if position.shape != (3,) or angles.shape != (3,) or not np.isfinite(np.r_[position, angles]).all():
            raise ValueError("Target must contain six finite position/angle values")
        if side != self.active_side:
            passive = self.active_side
            measured = self.robot.pose(self.data, passive)
            self.held_position = measured.position.copy()
            self.targets[passive] = measured
            relative = measured.rotation @ self.initial_poses[passive].rotation.T
            self.rpy[passive] = Rotation.from_matrix(relative).as_euler("xyz", degrees=True)
            self.active_side = side
        self.targets[side] = Pose(position.copy(), Rotation.from_euler(
            "xyz", angles, degrees=True).as_matrix() @ self.initial_poses[side].rotation)
        self.rpy[side] = angles.copy()
        self._target_revision += 1
        self.mode = "HQP"
        self.status = "Tracking"

    def set_control_mode(self, mode):
        if mode not in ("IK", "JOINT"):
            raise ValueError(f"Unknown control mode: {mode}")
        if mode == self.control_mode:
            return
        self.cancel_grasp_demo()
        self._target_revision += 1
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
        self.held_position = self.targets[passive].position.copy()
        self.fingers.capture(self.data)
        self.mode = "HQP"
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
            targets[self.joints.arm_rows] = self.position_target
        return self.joints.states(self.data, targets)

    def tick(self):
        if self.paused:
            return
        started = time.monotonic()
        self.timing.record(started)
        if self.grasp_demo is not None:
            self.grasp_demo.advance()
        dt = self.robot.config["control_dt"]
        self.joints.advance(dt)
        if self.control_mode == "JOINT":
            self.position_target = self.joints.commands[self.joints.arm_rows].copy()
            self.previous_velocity = self.joints.command_velocities[self.joints.arm_rows].copy()
        elif self.mode != "STOP":
            try:
                velocity = self.ik.velocity(
                    self.reference, self.active_side, self.targets[self.active_side],
                    self.held_position, dt, self.previous_velocity,
                )
                self.position_target += dt * velocity
                self.reference.qpos[self.robot.qpos_ids] = self.position_target
                self.robot.update_kinematics(self.reference)
                self.previous_velocity = velocity
                errors = self.ik.errors(self.reference, self.active_side, self.targets[self.active_side])
                held_error = self.held_position_error(self.reference)
                reached = (self.ik.reached(errors)
                           and held_error <= self.robot.config["held_position_tolerance"])
                self.status = "IK reached / settling" if reached else "Tracking / limited"
            except QPFailure as error:
                # Hold the last valid target; never integrate an infeasible QP result.
                self.previous_velocity[:] = 0
                self.mode = "STOP"
                self.status = f"Stopped: {error}"
                self._target_revision += 1
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
            held_error = self.held_position_error(self.data)
            if self.ik.reached(actual_errors) and held_error <= self.robot.config["held_position_tolerance"]:
                self.status = "Reached"
        self.steps += 1
        self.last_loop_seconds = time.monotonic() - started

    def held_position_error(self, data):
        passive = "left" if self.active_side == "right" else "right"
        return float(np.linalg.norm(self.robot.pose(data, passive).position - self.held_position))

    def metrics(self):
        errors = self.ik.errors(self.data, self.active_side, self.targets[self.active_side])
        predicted = self.ik.errors(self.reference, self.active_side, self.targets[self.active_side])
        right_errors = self.ik.errors(self.reference, "right", self.targets["right"])
        return {
            "active_hand": self.active_side, "mode": self.mode, "status": self.status,
            "control_mode": self.control_mode, "paused": self.paused,
            "ik_err": float(right_errors[0]) if self.control_mode == "IK" else None,
            "right_ik_orientation_error_deg": (float(np.rad2deg(right_errors[1]))
                                               if self.control_mode == "IK" and self.active_side == "right"
                                               else None),
            "loop_freq": self.timing.frequency, "loop_freq_ema": self.timing.ema,
            "position_error_m": float(errors[0]) if self.control_mode == "IK" else None,
            "orientation_error_deg": float(np.rad2deg(errors[1])) if self.control_mode == "IK" else None,
            "ik_position_error_m": float(predicted[0]) if self.control_mode == "IK" else None,
            "ik_orientation_error_deg": float(np.rad2deg(predicted[1])) if self.control_mode == "IK" else None,
            "held_position_error_m": self.held_position_error(self.data) if self.control_mode == "IK" else None,
            "ik_held_position_error_m": self.held_position_error(self.reference) if self.control_mode == "IK" else None,
            "lift_m": float(self.data.qpos[self.robot.qpos_ids[-1]]),
            "sim_time": float(self.data.time), "wall_time": time.monotonic() - self.started_at,
            "compute_ms": self.last_loop_seconds * 1000, "saturated_actuators": self.servo.saturated,
            "objects": self.objects.states(self.data),
            "grasp_demo": None if self.grasp_demo is None else {
                "side": self.grasp_demo.side, "phase": self.grasp_demo.phase,
                "message": self.grasp_demo.message,
            },
            "finger_targets": {
                side: {"thumb": float(values[0]), "four_fingers": float(values[1])}
                for side, values in self.fingers.targets.items()
            },
        }
