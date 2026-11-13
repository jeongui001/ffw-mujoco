"""All-joint monitoring and rate-limited manual commands."""

import mujoco
import numpy as np


class JointController:
    def __init__(self, robot, data, finger_velocity_limit):
        self.robot = robot
        model = robot.model
        self.names = [model.joint(index).name for index in range(model.njnt)]
        self.joint_by_name = {name: index for index, name in enumerate(self.names)}
        self.actuator_ids = np.full(model.njnt, -1, dtype=int)
        for actuator, (joint, _) in enumerate(model.actuator_trnid):
            if model.actuator_trntype[actuator] != mujoco.mjtTrn.mjTRN_JOINT:
                raise ValueError("Joint panel requires joint transmissions")
            self.actuator_ids[joint] = actuator
        if np.any(self.actuator_ids < 0) or model.nq != model.njnt:
            raise ValueError("Joint panel requires one actuated scalar coordinate per joint")
        self.qpos_ids = model.jnt_qposadr.copy()
        self.dof_ids = model.jnt_dofadr.copy()
        self.velocity_controlled = model.jnt_limited == 0
        self.limits = model.jnt_range.copy()
        wheel_limit = robot.config["wheel_speed_limit"]
        for joint in np.flatnonzero(self.velocity_controlled):
            original = model.actuator_ctrlrange[self.actuator_ids[joint]]
            self.limits[joint] = [max(-wheel_limit, original[0]), min(wheel_limit, original[1])]
        self.position_speeds = np.full(model.njnt, robot.config["auxiliary_joint_velocity_limit"])
        self.position_speeds[robot.joint_ids] = robot.velocity_limits
        for joint, name in enumerate(self.names):
            if name.startswith("finger_"):
                self.position_speeds[joint] = finger_velocity_limit
        manual_arm = set(robot.joint_ids)
        self.native_joints = np.array([j for j in range(model.njnt) if j not in manual_arm])
        self.auxiliary_joints = np.array([j for j in self.native_joints
                                         if not self.names[j].startswith("finger_")])
        self.capture(data)

    def capture(self, data):
        """Start a mode transition from measured state, without modifying qpos."""
        self.commands = np.clip(data.qpos[self.qpos_ids], self.limits[:, 0], self.limits[:, 1])
        self.targets = self.commands.copy()
        drives = self.velocity_controlled
        self.commands[drives] = np.clip(data.qvel[self.dof_ids[drives]],
                                       self.limits[drives, 0], self.limits[drives, 1])
        self.targets[drives] = 0
        self.command_velocities = np.zeros_like(self.commands)

    def set_target(self, name, value):
        if name not in self.joint_by_name:
            raise ValueError(f"Unknown joint: {name}")
        joint = self.joint_by_name[name]
        value = float(value)
        if not np.isfinite(value) or not self.limits[joint, 0] <= value <= self.limits[joint, 1]:
            raise ValueError(f"Target outside the joint panel range for {name}")
        self.targets[joint] = value

    def advance(self, dt):
        previous = self.commands.copy()
        maximum_step = self.position_speeds * dt
        maximum_step[self.velocity_controlled] = self.robot.config["wheel_acceleration_limit"] * dt
        self.commands += np.clip(self.targets - self.commands, -maximum_step, maximum_step)
        self.command_velocities = (self.commands - previous) / dt

    def apply_native(self, data, include_fingers=False):
        joints = self.native_joints if include_fingers else self.auxiliary_joints
        data.ctrl[self.actuator_ids[joints]] = self.commands[joints]

    def states(self, data, targets):
        model = self.robot.model
        return [
            {
                "name": name,
                "position": float(data.qpos[self.qpos_ids[joint]]),
                "velocity": float(data.qvel[self.dof_ids[joint]]),
                "force": float(data.qfrc_actuator[self.dof_ids[joint]]),
                "target": float(targets[joint]),
                "lower": float(self.limits[joint, 0]), "upper": float(self.limits[joint, 1]),
                "unit": "m" if model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_SLIDE else "rad",
                "velocity_controlled": bool(self.velocity_controlled[joint]),
            }
            for joint, name in enumerate(self.names)
        ]
