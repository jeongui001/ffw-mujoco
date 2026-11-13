"""Regression checks for task priority, arm-first policy, and physical tracking."""

import unittest

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from ffw_control.control.hierarchy.qp import QPFailure, solve_hierarchy
from ffw_control.robots.model import Pose, Robot, pose_error
from ffw_control.simulation.session import ControlSession


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
        self.assertNotIn(mujoco.mjtJoint.mjJNT_FREE, self.robot.model.jnt_type)
        data = self.robot.new_data()
        for side in ("right", "left"):
            jacobian = self.robot.jacobian(data, side)
            before = self.robot.pose(data, side).position
            data.qpos[self.robot.qpos_ids[-1]] += 1e-5
            self.robot.update_kinematics(data)
            derivative = (self.robot.pose(data, side).position - before) / 1e-5
            np.testing.assert_allclose(derivative, jacobian[:3, -1], atol=1e-7)

    def test_each_panel_tracks_translation_and_rotation_without_lift(self):
        for side in ("right", "left"):
            with self.subTest(side=side):
                session = ControlSession(self.robot)
                lift_start = session.position_target[-1]
                target = session.targets[side].position + [0.03, 0.01, 0.03]
                session.set_target(side, target, [5, -5, 5])
                self.assertEqual(session.mode, "ARM")
                self.run_ticks(session, 180, check_rates=True)
                self.assert_reached(session)
                self.assertAlmostEqual(session.position_target[-1], lift_start, places=6)

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


if __name__ == "__main__":
    unittest.main()
