"""Check the configured FFW model without opening a window or advancing time."""

import json
from pathlib import Path

import mujoco
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    config = json.loads((PROJECT_ROOT / "configs/robot.json").read_text())
    model = mujoco.MjModel.from_xml_path(str(PROJECT_ROOT / config["scene_path"]))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    def require_id(kind: mujoco.mjtObj, name: str) -> int:
        object_id = mujoco.mj_name2id(model, kind, name)
        if object_id < 0:
            raise ValueError(f"Missing model object: {name} ({kind})")
        return object_id

    require_id(mujoco.mjtObj.mjOBJ_BODY, config["base_body"])
    require_id(mujoco.mjtObj.mjOBJ_JOINT, config["shared_lift_joint"])
    for side, hand in config["hands"].items():
        require_id(mujoco.mjtObj.mjOBJ_BODY, hand["reference_body"])
        if len(hand["arm_joints"]) != 7 or len(hand["arm_actuators"]) != 7:
            raise ValueError(f"Expected seven arm joints and actuators for {side}")
        for joint, actuator in zip(hand["arm_joints"], hand["arm_actuators"]):
            joint_id = require_id(mujoco.mjtObj.mjOBJ_JOINT, joint)
            actuator_id = require_id(mujoco.mjtObj.mjOBJ_ACTUATOR, actuator)
            if (
                model.actuator_trntype[actuator_id] != mujoco.mjtTrn.mjTRN_JOINT
                or model.actuator_trnid[actuator_id, 0] != joint_id
            ):
                raise ValueError(f"Actuator {actuator} does not drive joint {joint}")
        print(f"{side}: 7 arm joints/actuators OK; hand body={hand['reference_body']}")

    if not np.isfinite(data.qpos).all() or not np.isfinite(data.xpos).all():
        raise ValueError("Non-finite initial model state")
    print(f"MuJoCo {mujoco.__version__}: model loaded; nq={model.nq}, nv={model.nv}, nu={model.nu}")
    print("Model check passed (no simulation steps or GUI).")


if __name__ == "__main__":
    main()
