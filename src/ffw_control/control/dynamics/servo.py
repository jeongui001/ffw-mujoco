"""Joint PD plus bias feedforward, through force-limited actuators."""

import mujoco
import numpy as np


class JointServo:
    def __init__(self, robot, data):
        self.robot = robot
        model = robot.model
        ids = robot.actuator_ids
        self.kp = robot.position_gains.copy()
        self.kd = robot.damping_gains.copy()
        # The lift's large frictionloss needs more stiffness than the XML demo.
        self.kp[-1] = robot.config["lift_position_gain"]
        self.kd[-1] = robot.config["lift_velocity_gain"]
        self.force_limits = model.actuator_forcerange[ids].copy()
        # Convert only these 15 project actuators to direct force commands.
        # The original XML and other position/velocity servos are untouched.
        model.actuator_gainprm[ids] = 0
        model.actuator_gainprm[ids, 0] = 1
        model.actuator_biastype[ids] = mujoco.mjtBias.mjBIAS_NONE
        model.actuator_biasprm[ids] = 0
        model.actuator_ctrllimited[ids] = False
        for actuator in range(model.nu):
            joint = model.actuator_trnid[actuator, 0]
            if actuator not in ids and model.actuator_biastype[actuator] != mujoco.mjtBias.mjBIAS_NONE:
                # Position servos hold their initial pose; wheel velocity stays zero.
                if model.actuator_biasprm[actuator, 1] != 0:
                    data.ctrl[actuator] = data.qpos[model.jnt_qposadr[joint]]
        self.saturated = 0

    def apply(self, data, position_target, velocity_target):
        robot = self.robot
        dofs = robot.dof_ids
        torque = (self.kp * (position_target - data.qpos[robot.qpos_ids])
                  + self.kd * (velocity_target - data.qvel[dofs])
                  + data.qfrc_bias[dofs]
                  + robot.model.dof_damping[dofs] * velocity_target)
        clipped = np.clip(torque, self.force_limits[:, 0], self.force_limits[:, 1])
        self.saturated = int(np.count_nonzero(np.abs(torque - clipped) > 1e-8))
        data.ctrl[robot.actuator_ids] = clipped
