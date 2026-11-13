"""Project model, joint indexing, and hand-frame kinematics."""

import json
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation


PROJECT_ROOT = Path(__file__).resolve().parents[3]


@dataclass
class Pose:
    position: np.ndarray
    rotation: np.ndarray

    def copy(self):
        return Pose(self.position.copy(), self.rotation.copy())


def pose_error(current: Pose, target: Pose) -> np.ndarray:
    """World-frame translation and SO(3) rotation-vector error."""
    angular = Rotation.from_matrix(target.rotation @ current.rotation.T).as_rotvec()
    return np.concatenate((target.position - current.position, angular))


class Robot:
    def __init__(self):
        self.config = json.loads((PROJECT_ROOT / "configs/control.json").read_text())
        mapping = json.loads((PROJECT_ROOT / "configs/robot.json").read_text())
        spec = mujoco.MjSpec.from_file(str(PROJECT_ROOT / mapping["scene_path"]))
        # This experiment isolates arm control: no floating base or wheel motion.
        spec.delete(spec.joint("floating_base"))
        spec.body("base_link").pos = [0, 0, 0.0035]
        for side in ("right", "left"):
            body = mapping["hands"][side]["reference_body"]
            spec.body(body).add_site(
                name=f"{side}_palm", pos=[0, 0, 0.06],
                size=[0.012, 0, 0], rgba=[0.1, 0.7, 0.9, 1],
            )
            # Convex palm/root meshes overlap even in the open pose. Exclude
            # only these structural neighbours; distal and object contacts remain.
            prefix = "r" if side == "right" else "l"
            for link in (2, 6, 10, 14, 18):
                spec.add_exclude(bodyname1=body, bodyname2=f"finger_{prefix}_link{link}")
        self.model = spec.compile()
        self.model.opt.timestep = self.config["physics_dt"]
        names = (mapping["hands"]["right"]["arm_joints"]
                 + mapping["hands"]["left"]["arm_joints"]
                 + [mapping["shared_lift_joint"]])
        self.joint_ids = np.array([self.model.joint(n).id for n in names])
        self.qpos_ids = self.model.jnt_qposadr[self.joint_ids]
        self.dof_ids = self.model.jnt_dofadr[self.joint_ids]
        self.actuator_ids = np.array([self.model.actuator(n).id for n in names])
        self.position_gains = self.model.actuator_gainprm[self.actuator_ids, 0].copy()
        self.damping_gains = -self.model.actuator_biasprm[self.actuator_ids, 2].copy()
        self.limits = self.model.jnt_range[self.joint_ids].copy()
        self.site_ids = {s: self.model.site(f"{s}_palm").id for s in ("right", "left")}
        self.nominal = np.array(self.config["initial_right_arm"]
                                + self.config["initial_left_arm"]
                                + [self.config["initial_lift"]])
        self.velocity_limits = np.array([self.config["arm_velocity_limit"]] * 14
                                        + [self.config["lift_velocity_limit"]])
        self.acceleration_limits = np.array([self.config["arm_acceleration_limit"]] * 14
                                            + [self.config["lift_acceleration_limit"]])

    @staticmethod
    def arm_slice(side):
        if side not in ("right", "left"):
            raise ValueError(f"Unknown hand: {side}")
        return slice(0, 7) if side == "right" else slice(7, 14)

    def new_data(self):
        data = mujoco.MjData(self.model)
        data.qpos[self.qpos_ids] = self.nominal
        mujoco.mj_forward(self.model, data)
        return data

    def pose(self, data, side):
        site = self.site_ids[side]
        return Pose(data.site_xpos[site].copy(), data.site_xmat[site].reshape(3, 3).copy())

    def jacobian(self, data, side):
        translation = np.zeros((3, self.model.nv))
        rotation = np.zeros_like(translation)
        mujoco.mj_jacSite(self.model, data, translation, rotation, self.site_ids[side])
        return np.vstack((translation[:, self.dof_ids], rotation[:, self.dof_ids]))

    def update_kinematics(self, data):
        mujoco.mj_kinematics(self.model, data)
        mujoco.mj_comPos(self.model, data)
