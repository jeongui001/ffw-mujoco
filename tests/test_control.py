"""Regression checks for task priority, arm-first policy, and physical tracking."""

import unittest
from threading import Event
from unittest.mock import patch

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from ffw_control.control.hierarchy.qp import QPFailure, solve_hierarchy
from ffw_control.robots.model import Pose, Robot, pose_error
from ffw_control.simulation.session import ControlSession
from ffw_control.simulation.timing import LoopTiming


class HierarchyTests(unittest.TestCase):
    def test_lower_tasks_cannot_trade_away_primary_task(self):
        tasks = [(np.array([[1., 1.]]), np.array([1.])),
                 (np.array([[1., 0.]]), np.array([0.])),
                 (np.array([[0., 1.]]), np.array([-100.]))]
        result = solve_hierarchy(tasks, np.eye(2), np.zeros(2), np.ones(2))
        np.testing.assert_allclose(result, [0, 1], atol=2e-6)

    def test_unreachable_task_respects_joint_bound(self):
        result = solve_hierarchy([(np.eye(1), np.array([5.]))],
                                 np.eye(1), np.array([-1.]), np.array([1.]))
        np.testing.assert_allclose(result, [1], atol=2e-6)

    def test_conflicting_hard_constraints_report_failure(self):
        with self.assertRaises(QPFailure):
            solve_hierarchy([(np.eye(1), np.zeros(1))],
                            np.ones((2, 1)), np.array([0., 1.]), np.array([0., 1.]))

    def test_rotation_error_wraps_across_180_degrees(self):
        current = Pose(np.zeros(3), Rotation.from_euler("z", 179, degrees=True).as_matrix())
        target = Pose(np.zeros(3), Rotation.from_euler("z", -179, degrees=True).as_matrix())
        self.assertAlmostEqual(np.rad2deg(np.linalg.norm(pose_error(current, target)[3:])), 2.0)


class TimingTests(unittest.TestCase):
    def test_frequency_uses_start_intervals_and_ema(self):
        timing = LoopTiming(0.1)
        timing.record(10.0)
        self.assertEqual(timing.frequency, 0)
        timing.record(10.02)
        self.assertAlmostEqual(timing.frequency, 50)
        timing.record(10.06)
        self.assertAlmostEqual(timing.frequency, 25)
        self.assertAlmostEqual(timing.ema, 47.5)
        timing.reset()
        timing.record(100.0)
        self.assertEqual(timing.frequency, 0)
        timing.record(100.04)
        self.assertAlmostEqual(timing.frequency, 25)
        self.assertAlmostEqual(timing.ema, 25)


class RobotControlTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = Robot()

    def run_ticks(self, session, count, check_rates=False):
        old_velocity = session.previous_velocity.copy()
        max_held_error = 0.0
        for _ in range(count):
            session.tick()
            self.assertNotEqual(session.mode, "STOP", session.status)
            self.assertTrue(np.isfinite(session.data.qpos).all())
            q = session.position_target
            self.assertTrue(np.all(q >= self.robot.limits[:, 0] - 2e-6))
            self.assertTrue(np.all(q <= self.robot.limits[:, 1] + 2e-6))
            if check_rates:
                velocity = session.previous_velocity
                self.assertTrue(np.all(np.abs(velocity) <= self.robot.velocity_limits + 2e-6))
                acceleration = np.abs(velocity - old_velocity) / self.robot.config["control_dt"]
                self.assertTrue(np.all(acceleration <= self.robot.acceleration_limits + 2e-4))
                old_velocity = velocity.copy()
            max_held_error = max(max_held_error, abs(session.metrics()["held_height_error_m"]))
        return max_held_error

    def assert_reached(self, session):
        metrics = session.metrics()
        self.assertLess(metrics["position_error_m"], 0.002, metrics)
        self.assertLess(metrics["orientation_error_deg"], 1.0, metrics)
        self.assertLess(abs(metrics["held_height_error_m"]), 0.002, metrics)

    def test_fixed_base_and_shared_lift_jacobian(self):
        self.assertEqual(self.robot.model.body("base_link").jntnum[0], 0)
        free = np.flatnonzero(self.robot.model.jnt_type == mujoco.mjtJoint.mjJNT_FREE)
        self.assertEqual({self.robot.model.joint(j).name for j in free},
                         {"can_right_free", "can_left_free"})
        data = self.robot.new_data()
        for side in ("right", "left"):
            jacobian = self.robot.jacobian(data, side)
            before = self.robot.pose(data, side).position
            data.qpos[self.robot.qpos_ids[-1]] += 1e-5
            self.robot.update_kinematics(data)
            derivative = (self.robot.pose(data, side).position - before) / 1e-5
            np.testing.assert_allclose(derivative, jacobian[:3, -1], atol=1e-7)

    def test_free_cans_settle_on_table_without_external_forces(self):
        session = ControlSession(self.robot)
        for _ in range(150):
            session.tick()
        before = session.data.qpos.copy()
        states = session.objects.states(session.data)
        np.testing.assert_array_equal(session.data.qpos, before)
        np.testing.assert_array_equal(session.data.qfrc_applied, 0)
        np.testing.assert_array_equal(session.data.xfrc_applied, 0)
        self.assertEqual(self.robot.model.neq, 0)
        for side, state in states.items():
            self.assertTrue(state["on_table"], state)
            self.assertFalse(state["lifted"], state)
            self.assertEqual(state["grasped_by"], [])
            self.assertLess(abs(state["clearance_m"]), 0.002)
            expected_y = -0.22 if side == "right" else 0.22
            np.testing.assert_allclose(state["position_m"], [0.56, expected_y, 0.86], atol=0.002)

    def test_both_hands_pick_up_hold_and_release_physical_cans(self):
        for side in ("right", "left"):
            with self.subTest(side=side):
                session = ControlSession(self.robot)
                before = session.data.qpos.copy()
                session.start_grasp_demo(side)
                np.testing.assert_array_equal(session.data.qpos, before)
                for _ in range(1200):
                    session.tick()
                    self.assertNotEqual(session.mode, "STOP", session.status)
                    if not session.grasp_demo.active:
                        break
                self.assertEqual(session.grasp_demo.phase, "DONE", session.metrics())
                state = session.objects.states(session.data)[side]
                self.assertEqual(state["state"], "held", state)
                self.assertGreater(state["clearance_m"], 0.10)
                self.assertIn(side, state["grasped_by"])
                self.assertFalse(state["on_table"])
                np.testing.assert_array_equal(session.data.qfrc_applied, 0)
                np.testing.assert_array_equal(session.data.xfrc_applied, 0)
                held_height = state["position_m"][2]
                session.set_finger_targets(side, 0, 0)
                for _ in range(200):
                    session.tick()
                released = session.objects.states(session.data)[side]
                self.assertEqual(released["grasped_by"], [], released)
                self.assertLess(released["position_m"][2], held_height - 0.08)

    def test_pickup_pause_manual_takeover_and_mode_switch(self):
        session = ControlSession(self.robot)
        session.start_grasp_demo("right")
        session.set_paused(True)
        before = session.data.qpos.copy()
        for _ in range(20):
            session.tick()
        self.assertEqual(session.grasp_demo.phase, "APPROACH")
        self.assertEqual(session.data.time, 0)
        np.testing.assert_array_equal(session.data.qpos, before)
        session.set_paused(False)
        session.set_finger_targets("right", 0.3, 0.4)
        session.tick()
        self.assertEqual(session.grasp_demo.phase, "CANCELLED")
        np.testing.assert_array_equal(session.fingers.targets["right"], [0.3, 0.4])
        session.start_grasp_demo("right")
        session.set_control_mode("JOINT")
        self.assertEqual(session.grasp_demo.phase, "CANCELLED")
        with self.assertRaises(ValueError):
            session.start_grasp_demo("left")

    def test_each_panel_tracks_translation_and_rotation_without_lift(self):
        for side in ("right", "left"):
            with self.subTest(side=side):
                session = ControlSession(self.robot)
                lift_start = session.position_target[-1]
                session.set_finger_targets(side, 0.7, 0.8)
                other = "left" if side == "right" else "right"
                session.set_finger_targets(other, 0.4, 0.6)
                target = session.targets[side].position + [0.03, 0.01, 0.03]
                session.set_target(side, target, [5, -5, 5])
                self.assertEqual(session.mode, "ARM")
                self.run_ticks(session, 180, check_rates=True)
                self.assert_reached(session)
                self.assertAlmostEqual(session.position_target[-1], lift_start, places=6)
                curls = session.data.qpos[session.fingers.qpos_ids[side]][5::4]
                self.assertTrue(np.all(curls > 0.5))

    def test_finger_groups_close_and_reopen_independently_on_both_hands(self):
        for side in ("right", "left"):
            with self.subTest(side=side):
                session = ControlSession(self.robot)
                active_before = session.active_side
                pose_targets = {s: p.copy() for s, p in session.targets.items()}
                revision_before = session._plan_revision
                session.set_finger_targets(side, 1, 0)
                self.assertEqual(session.active_side, active_before)
                self.assertEqual(session._plan_revision, revision_before)
                self.run_ticks(session, 120)
                positions = session.data.qpos[session.fingers.qpos_ids[side]]
                # Thumb yaw is mirrored, while the other four digits stay open.
                yaw_sign = -1 if side == "right" else 1
                self.assertGreater(yaw_sign * positions[1], 0.5)
                self.assertLess(np.max(np.abs(positions[4:])), 0.08)
                other = "left" if side == "right" else "right"
                other_positions = session.data.qpos[session.fingers.qpos_ids[other]]
                self.assertLess(np.max(np.abs(other_positions)), 0.08)

                session.set_finger_targets(side, 1, 1)
                self.run_ticks(session, 150)
                curled = session.data.qpos[session.fingers.qpos_ids[side]].copy()
                self.assertTrue(np.all(curled[5::4] > 0.8))
                forces = session.data.actuator_force[session.fingers.actuator_ids[side]]
                limits = self.robot.model.actuator_forcerange[session.fingers.actuator_ids[side]]
                self.assertTrue(np.all(forces >= limits[:, 0] - 1e-8))
                self.assertTrue(np.all(forces <= limits[:, 1] + 1e-8))

                session.set_finger_targets(side, 0, 0)
                self.run_ticks(session, 150)
                reopened = session.data.qpos[session.fingers.qpos_ids[side]]
                self.assertLess(np.max(np.abs(reopened)), 0.15)
                self.assertEqual(session.active_side, active_before)
                for hand, target in pose_targets.items():
                    np.testing.assert_array_equal(session.targets[hand].position, target.position)
                    np.testing.assert_array_equal(session.targets[hand].rotation, target.rotation)
                self.assert_reached(session)

    def test_finger_commands_are_rate_limited_and_do_not_teleport_joints(self):
        session = ControlSession(self.robot)
        dt = self.robot.model.opt.timestep
        actual_before = session.data.qpos.copy()
        for side in ("right", "left"):
            session.set_finger_targets(side, 1, 1)
        for _ in range(40):
            old = {s: q.copy() for s, q in session.fingers.commanded_positions.items()}
            session.fingers.apply(session.data, dt)
            for side, command in session.fingers.commanded_positions.items():
                self.assertLessEqual(np.max(np.abs(command - old[side])),
                                     session.fingers.velocity_limit * dt + 1e-12)
                limits = session.fingers.limits[side]
                self.assertTrue(np.all(command >= limits[:, 0]))
                self.assertTrue(np.all(command <= limits[:, 1]))
        np.testing.assert_array_equal(session.data.qpos, actual_before)

    def test_invalid_finger_inputs_leave_previous_targets_unchanged(self):
        session = ControlSession(self.robot)
        session.set_finger_targets("right", 0.3, 0.7)
        for side, thumb, fingers in (("right", -0.1, 0), ("right", 0, 1.1),
                                     ("left", np.nan, 0), ("left", 0, np.inf),
                                     ("unknown", 0, 0)):
            with self.subTest(side=side, thumb=thumb, fingers=fingers):
                with self.assertRaises(ValueError):
                    session.set_finger_targets(side, thumb, fingers)
        np.testing.assert_array_equal(session.fingers.targets["right"], [0.3, 0.7])
        np.testing.assert_array_equal(session.fingers.targets["left"], [0, 0])

    def test_each_lift_mode_holds_opposite_hand_height(self):
        for side in ("right", "left"):
            with self.subTest(side=side):
                session = ControlSession(self.robot)
                start = session.position_target.copy()
                session.set_target(side, session.targets[side].position + [0, 0, -0.10], [0, 0, 0])
                self.assertEqual(session.mode, "LIFT")
                worst_height_error = self.run_ticks(session, 300, check_rates=True)
                self.assert_reached(session)
                self.assertLess(worst_height_error, 0.003)
                self.assertLess(session.position_target[-1], start[-1] - 0.01)
                passive = "left" if side == "right" else "right"
                section = self.robot.arm_slice(passive)
                self.assertGreater(np.linalg.norm(session.position_target[section] - start[section]), 0.01)

    def test_switching_panels_preserves_the_new_passive_height(self):
        session = ControlSession(self.robot)
        session.set_target("right", session.targets["right"].position + [0, 0, -0.10], [0, 0, 0])
        self.run_ticks(session, 250)
        right_height = self.robot.pose(session.data, "right").position[2]
        left_target = self.robot.pose(session.data, "left").position + [0.02, 0, 0.02]
        session.set_target("left", left_target, [0, 0, 0])
        self.run_ticks(session, 180)
        self.assert_reached(session)
        self.assertAlmostEqual(session.held_pose.position[2], right_height, places=7)

    def test_large_unreachable_target_stays_finite_and_bounded(self):
        session = ControlSession(self.robot)
        session.set_target("right", [2.0, -2.0, 2.0], [0, 0, 0])
        self.run_ticks(session, 150, check_rates=True)
        self.assertGreater(session.metrics()["position_error_m"], 0.1)
        self.assertNotEqual(session.status, "Reached")

    def test_force_commands_respect_actuator_limits(self):
        session = ControlSession(self.robot)
        session.servo.apply(session.data, session.position_target + 10, np.zeros(15))
        force = session.data.ctrl[self.robot.actuator_ids]
        self.assertTrue(np.all(force >= session.servo.force_limits[:, 0]))
        self.assertTrue(np.all(force <= session.servo.force_limits[:, 1]))
        self.assertGreater(session.servo.saturated, 0)
        np.testing.assert_array_equal(session.data.qfrc_applied, 0)

    def test_nonfinite_target_is_rejected(self):
        session = ControlSession(self.robot)
        with self.assertRaises(ValueError):
            session.set_target("right", [np.nan, 0, 0], [0, 0, 0])

    def test_joint_monitor_covers_robot_joints_and_uses_measured_state(self):
        session = ControlSession(self.robot)
        rows = session.joint_states()
        self.assertEqual(len(rows), self.robot.model.nu)
        self.assertEqual(len({row["name"] for row in rows}), self.robot.model.nu)
        self.assertFalse(any(row["name"].startswith("can_") for row in rows))
        with self.assertRaises(ValueError):
            session.joints.set_target("can_right_free", 0)
        for row in rows:
            joint = self.robot.model.joint(row["name"])
            self.assertEqual(row["position"], session.data.qpos[joint.qposadr[0]])
            self.assertEqual(row["velocity"], session.data.qvel[joint.dofadr[0]])
            self.assertEqual(row["force"], session.data.qfrc_actuator[joint.dofadr[0]])
        drives = [row for row in rows if row["velocity_controlled"]]
        self.assertEqual(len(drives), 3)
        self.assertTrue(all("wheel_drive" in row["name"] for row in drives))

    def test_manual_joints_move_and_return_to_ik_without_restoring_old_goals(self):
        session = ControlSession(self.robot)
        before = session.data.qpos.copy()
        session.set_control_mode("JOINT")
        np.testing.assert_array_equal(session.data.qpos, before)
        targets = {"head_joint1": 0.2, "arm_r_joint4": -0.7,
                   "finger_r_joint6": 0.8, "left_wheel_drive_joint": 1.5}
        for name, value in targets.items():
            session.set_joint_target(name, value)
        old_commands = session.joints.commands.copy()
        with patch.object(session.ik, "velocity", side_effect=AssertionError("IK ran in joint mode")), \
                patch.object(session.fingers, "apply", side_effect=AssertionError("Presets ran in joint mode")):
            for _ in range(130):
                session.tick()
                change = np.abs(session.joints.commands - old_commands)
                allowed = session.joints.position_speeds * self.robot.config["control_dt"]
                allowed[session.joints.velocity_controlled] = (self.robot.config["wheel_acceleration_limit"]
                                                               * self.robot.config["control_dt"])
                self.assertTrue(np.all(change <= allowed + 1e-10))
                old_commands = session.joints.commands.copy()
        rows = {row["name"]: row for row in session.joint_states()}
        self.assertGreater(rows["head_joint1"]["position"], 0.15)
        self.assertGreater(rows["arm_r_joint4"]["position"], -0.75)
        self.assertGreater(rows["finger_r_joint6"]["position"], 0.6)
        self.assertGreater(rows["left_wheel_drive_joint"]["velocity"], 0.05)
        self.assertIsNone(session.metrics()["ik_err"])
        measured = session.data.qpos.copy()
        hand_poses = {s: self.robot.pose(session.data, s) for s in session.targets}
        session.set_control_mode("IK")
        np.testing.assert_array_equal(session.data.qpos, measured)
        for side in hand_poses:
            np.testing.assert_array_equal(session.targets[side].position, hand_poses[side].position)
            self.assertIsNotNone(session.fingers.held_positions[side])
        self.run_ticks(session, 100)
        self.assert_reached(session)
        finger = self.robot.model.joint("finger_r_joint6").qposadr[0]
        self.assertLess(abs(session.data.qpos[finger] - measured[finger]), 0.04)
        drive = session.joints.joint_by_name["left_wheel_drive_joint"]
        self.assertEqual(session.joints.commands[drive], 0)
        session.set_finger_targets("right", 0.5, 0.5)
        self.assertIsNone(session.fingers.held_positions["right"])
        self.assertIsNotNone(session.fingers.held_positions["left"])

    def test_joint_mode_blocks_conflicting_inputs_and_rejects_invalid_values(self):
        session = ControlSession(self.robot)
        with self.assertRaises(ValueError):
            session.set_joint_target("head_joint1", 0.1)
        session.set_control_mode("JOINT")
        for name, value in (("head_joint1", 10), ("lift_joint", np.nan),
                            ("left_wheel_drive_joint", 50), ("unknown", 0)):
            with self.assertRaises(ValueError):
                session.set_joint_target(name, value)
        with self.assertRaises(ValueError):
            session.set_target("right", [0, 0, 0.5], [0, 0, 0])
        with self.assertRaises(ValueError):
            session.set_finger_targets("right", 1, 1)
        with self.assertRaises(ValueError):
            session.set_control_mode("unknown")
        self.assertEqual(session.control_mode, "JOINT")

    def test_right_ik_metric_does_not_follow_active_left_hand(self):
        session = ControlSession(self.robot)
        session.set_target("left", session.targets["left"].position + [0.02, 0, 0.03], [0, 0, 0])
        metrics = session.metrics()
        self.assertEqual(metrics["active_hand"], "left")
        self.assertGreater(metrics["ik_position_error_m"], 0.03)
        self.assertLess(metrics["ik_err"], 1e-8)

    def test_pause_freezes_simulation_but_wall_time_continues(self):
        session = ControlSession(self.robot)
        self.run_ticks(session, 2)
        frozen_time = session.data.time
        frozen_state = session.data.qpos.copy()
        session.set_paused(True)
        with patch("ffw_control.simulation.session.time.monotonic", return_value=session.started_at + 10):
            session.tick()
            metrics = session.metrics()
        self.assertEqual(session.data.time, frozen_time)
        np.testing.assert_array_equal(session.data.qpos, frozen_state)
        self.assertEqual(metrics["wall_time"], 10)
        self.assertEqual(metrics["loop_freq"], 0)
        self.assertEqual(metrics["loop_freq_ema"], 0)
        session.set_paused(False)
        session.tick()
        self.assertGreater(session.data.time, frozen_time)
        self.assertEqual(session.timing.frequency, 0)

    def test_pending_ik_plan_cannot_override_manual_mode(self):
        session = ControlSession(self.robot)
        release = Event()

        def feasibility(*args):
            release.wait(5)
            return False

        try:
            with patch.object(session.ik, "arm_feasible", side_effect=feasibility):
                session.set_target("right", session.targets["right"].position, [0, 0, 0], asynchronous=True)
                session.set_control_mode("JOINT")
                session.set_joint_target("head_joint1", 0.2)
                release.set()
                session._future.result(timeout=5)
                session.tick()
                self.assertEqual(session.mode, "JOINT")
                self.assertEqual(session.control_mode, "JOINT")
                self.assertFalse(session.metrics()["planning"])
        finally:
            release.set()
            session.close()

    def test_stale_background_plan_cannot_enable_lift_for_a_new_target(self):
        session = ControlSession(self.robot)
        started, release = Event(), Event()
        calls = []

        def feasibility(*args):
            calls.append(args)
            if len(calls) == 1:
                started.set()
                release.wait(5)
                return False
            return True

        try:
            with patch.object(session.ik, "arm_feasible", side_effect=feasibility):
                target = session.targets["right"].position.copy()
                session.set_target("right", target + [0, 0, -0.1], [0, 0, 0], asynchronous=True)
                self.assertTrue(started.wait(5))
                session.set_target("right", target + [0.01, 0, 0], [0, 0, 0], asynchronous=True)
                release.set()
                session._future.result(timeout=5)
                session.tick()
                self.assertEqual(session.mode, "ARM")
                session._future.result(timeout=5)
                session.tick()
                self.assertTrue(session.arm_solution_available)
                self.assertEqual(session.mode, "ARM")
                self.assertEqual(len(calls), 2)
        finally:
            release.set()
            session.close()

    def test_pending_plan_cannot_restart_a_stopped_controller(self):
        session = ControlSession(self.robot)
        release = Event()

        def feasibility(*args):
            release.wait(5)
            return True

        try:
            with patch.object(session.ik, "arm_feasible", side_effect=feasibility):
                session.set_target("right", session.targets["right"].position, [0, 0, 0], asynchronous=True)
                with patch.object(session.ik, "velocity", side_effect=QPFailure("test failure")):
                    session.tick()
                self.assertEqual(session.mode, "STOP")
                release.set()
                session._future.result(timeout=5)
                session.tick()
                self.assertEqual(session.mode, "STOP")
        finally:
            release.set()
            session.close()


if __name__ == "__main__":
    unittest.main()
