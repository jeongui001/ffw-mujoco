"""Observe physical cans and hand contacts without changing their dynamics."""

import mujoco
import numpy as np


class ObjectMonitor:
    def __init__(self, model):
        self.model = model
        self.can_bodies = {side: model.body(f"can_{side}").id for side in ("right", "left")}
        self.can_geoms = {side: model.geom(f"can_{side}_collision").id for side in self.can_bodies}
        self.table_geom = model.geom("workbench_top").id
        self.table_height = (model.body_pos[model.geom_bodyid[self.table_geom], 2]
                             + model.geom_size[self.table_geom, 2])
        self.hand_groups = {}
        for body in range(model.nbody):
            name = model.body(body).name or ""
            for side, prefix in (("right", "r"), ("left", "l")):
                if name == f"hx5_{prefix}_base":
                    self.hand_groups[body] = (side, "palm")
                elif name.startswith(f"finger_{prefix}_link"):
                    number = int(name.rsplit("link", 1)[1])
                    self.hand_groups[body] = (side, "thumb" if number <= 4 else "fingers")

    def states(self, data):
        result = {}
        for side, body in self.can_bodies.items():
            geom = self.can_geoms[side]
            radius, half_height = self.model.geom_size[geom, :2]
            axis_z = abs(data.geom_xmat[geom].reshape(3, 3)[2, 2])
            extent_z = half_height * axis_z + radius * np.sqrt(max(0, 1 - axis_z**2))
            clearance = float(data.geom_xpos[geom, 2] - extent_z - self.table_height)
            contacts = {hand: {group: 0.0 for group in ("thumb", "fingers", "palm")}
                        for hand in ("right", "left")}
            supported = False
            for index, contact in enumerate(data.contact):
                if contact.geom1 == geom:
                    other = contact.geom2
                elif contact.geom2 == geom:
                    other = contact.geom1
                else:
                    continue
                force = np.zeros(6)
                mujoco.mj_contactForce(self.model, data, index, force)
                if force[0] <= 0.01:
                    continue
                supported |= other == self.table_geom
                hand_group = self.hand_groups.get(self.model.geom_bodyid[other])
                if hand_group:
                    hand, group = hand_group
                    contacts[hand][group] += float(force[0])
            grasped_by = [hand for hand, force in contacts.items()
                          if force["fingers"] > 0.05 and force["thumb"] + force["palm"] > 0.05]
            lifted = not supported and clearance > 0.02
            touching = any(sum(force.values()) > 0 for force in contacts.values())
            state = ("held" if lifted and grasped_by else "lifted" if lifted else
                     "contact" if touching else "on table" if supported else "free")
            result[side] = {
                "position_m": data.xpos[body].tolist(), "clearance_m": clearance,
                "on_table": bool(supported), "lifted": bool(lifted), "state": state,
                "grasped_by": grasped_by, "hand_contact_force_n": contacts,
            }
        return result
