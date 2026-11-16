"""A repeatable pickup sequence using the same pose and finger controls as the UI."""

import numpy as np
from scipy.spatial.transform import Rotation


class GraspSequence:
    MOVING = ("APPROACH", "LOWER", "LIFT")

    def __init__(self, session, side):
        session.robot.arm_slice(side)
        if session.control_mode != "IK":
            raise ValueError("Return to IK mode before starting a pickup")
        state = session.objects.states(session.data)[side]
        geom = session.robot.model.geom(f"can_{side}_collision").id
        upright = abs(session.data.geom_xmat[geom].reshape(3, 3)[2, 2])
        if abs(state["clearance_m"]) > 0.01 or upright < 0.95:
            raise ValueError("Pickup requires an upright can on the table. Reset the scene first.")
        self.session = session
        self.side = side
        # Fingers run vertically along an upright can; the palm faces diagonally
        # forward to keep the wrist away from its rotation limit.
        rotation = np.array([[0.5, 0, np.sqrt(0.75)],
                             [np.sqrt(0.75), 0, -0.5], [0, 1, 0]])
        if side == "left":
            reflection = np.diag([1, -1, 1])
            rotation = reflection @ rotation @ reflection
        self.angles = Rotation.from_matrix(
            rotation @ session.initial_poses[side].rotation.T).as_euler("xyz", degrees=True)
        # Palm site is 6cm along local Z. This offset leaves space for the open
        # proximal fingers, before closing pushes the can against the palm.
        self.grasp_position = np.array(state["position_m"]) - rotation @ [0.055, 0, 0.07]
        self.grasp_position[2] = session.objects.table_height + session.robot.model.geom_size[geom, 1]
        self.phase = "READY"
        self.message = "Ready"
        self.phase_started = session.data.time
        self.command_revision = session._target_revision
        self.finger_target = np.zeros(2)

    @property
    def active(self):
        return self.phase in (*self.MOVING, "CLOSE", "HOLD")

    def start(self):
        self.session.set_finger_targets(self.side, 0, 0)
        self._move("APPROACH", self.grasp_position + [0, 0, 0.21])

    def cancel(self):
        if self.active:
            self.phase = "CANCELLED"
            self.message = "Cancelled / last commands retained"

    def _move(self, phase, position):
        self.session.set_target(self.side, position, self.angles)
        self.command_revision = self.session._target_revision
        self.phase = phase
        self.message = phase.title()
        self.phase_started = self.session.data.time

    def _fail(self, reason):
        self.phase = "FAILED"
        self.message = reason

    def advance(self):
        if not self.active:
            return
        session = self.session
        # Manual input takes over before the sequence can issue another command.
        if (session.control_mode != "IK" or self.command_revision != session._target_revision
                or not np.array_equal(session.fingers.targets[self.side], self.finger_target)):
            self.cancel()
            return
        if session.mode == "STOP":
            self._fail(session.status)
            return
        elapsed = session.data.time - self.phase_started
        if self.phase in self.MOVING:
            error = session.ik.errors(session.data, self.side, session.targets[self.side])
            reached = error[0] < 0.003 and error[1] < np.deg2rad(1)
            if elapsed > 16:
                self._fail(f"{self.phase.title()} timed out / target not reached")
            elif elapsed >= 1 and reached:
                if self.phase == "APPROACH":
                    self._move("LOWER", self.grasp_position)
                elif self.phase == "LOWER":
                    session.set_finger_targets(self.side, 1, 1)
                    self.finger_target[:] = 1
                    self.phase = "CLOSE"
                    self.message = "Closing fingers"
                    self.phase_started = session.data.time
                else:
                    self.phase = "HOLD"
                    self.message = "Checking physical hold"
                    self.phase_started = session.data.time
        elif self.phase == "CLOSE" and elapsed >= 3.6:
            state = session.objects.states(session.data)[self.side]
            if self.side not in state["grasped_by"]:
                self._fail("No opposing hand contacts / lift skipped")
            else:
                self._move("LIFT", self.grasp_position + [0, 0, 0.14])
        elif self.phase == "HOLD" and elapsed >= 3:
            state = session.objects.states(session.data)[self.side]
            if state["clearance_m"] > 0.08 and self.side in state["grasped_by"]:
                self.phase = "DONE"
                self.message = "Pickup held / use finger sliders to release"
            else:
                self._fail("Can slipped / physical hold not confirmed")
