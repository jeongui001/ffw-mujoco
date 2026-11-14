"""GLFW/MuJoCo view with hand, joint, and simulation panels 1-6."""

import struct
import time
import zlib
from pathlib import Path

import glfw
import imgui
import mujoco
import numpy as np
from imgui.integrations.glfw import GlfwRenderer
from OpenGL import GL


def save_screenshot(path, width, height):
    """Write the complete OpenGL framebuffer, including panels, as PNG."""
    GL.glPixelStorei(GL.GL_PACK_ALIGNMENT, 1)
    pixels = GL.glReadPixels(0, 0, width, height, GL.GL_RGB, GL.GL_UNSIGNED_BYTE)
    rows = np.frombuffer(pixels, dtype=np.uint8).reshape(height, width * 3)[::-1]
    raw = b"".join(b"\x00" + row.tobytes() for row in rows)

    def chunk(kind, payload):
        return (struct.pack("!I", len(payload)) + kind + payload
                + struct.pack("!I", zlib.crc32(kind + payload) & 0xffffffff))

    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(b"\x89PNG\r\n\x1a\n"
                       + chunk(b"IHDR", struct.pack("!2I5B", width, height, 8, 2, 0, 0, 0))
                       + chunk(b"IDAT", zlib.compress(raw)) + chunk(b"IEND", b""))


class ControlWindow:
    SIDEBAR = 330

    def __init__(self, session):
        if not glfw.init():
            raise RuntimeError("Cannot open display. Use --headless for model/controller checks.")
        glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 2)
        glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 1)
        self.window = glfw.create_window(1400, 850, "FFW-SH5 | Dual-hand HQP control", None, None)
        if not self.window:
            glfw.terminate()
            raise RuntimeError("Cannot create an OpenGL window")
        glfw.make_context_current(self.window)
        glfw.swap_interval(1)
        self.imgui_context = imgui.create_context()
        imgui.get_io().ini_file_name = None
        self.renderer = GlfwRenderer(self.window)
        self.session = session
        self.scene = mujoco.MjvScene(session.robot.model, maxgeom=4000)
        self.context = mujoco.MjrContext(session.robot.model, mujoco.mjtFontScale.mjFONTSCALE_150)
        self.option = mujoco.MjvOption()
        self.option.geomgroup[3] = 0
        self.camera = mujoco.MjvCamera()
        self.camera.lookat[:] = [0.4, 0, 0.85]
        self.camera.distance = 3.0
        self.camera.azimuth = 135
        self.camera.elevation = -20
        self.cursor = glfw.get_cursor_pos(self.window)
        self.paused = False
        self.joint_filter = ""
        self.pickup_error = ""
        glfw.set_scroll_callback(self.window, self.scroll)
        style = imgui.get_style()
        style.window_rounding = 5
        style.frame_rounding = 3
        style.colors[imgui.COLOR_WINDOW_BACKGROUND] = (0.09, 0.105, 0.12, 0.97)
        style.colors[imgui.COLOR_FRAME_BACKGROUND] = (0.16, 0.19, 0.22, 1)
        style.colors[imgui.COLOR_SLIDER_GRAB] = (0.2, 0.6, 0.95, 1)
        style.colors[imgui.COLOR_SLIDER_GRAB_ACTIVE] = (0.4, 0.75, 1, 1)
        style.colors[imgui.COLOR_PLOT_HISTOGRAM] = (0.2, 0.6, 0.95, 1)

    def scroll(self, window, x, y):
        self.renderer.scroll_callback(window, x, y)
        if not imgui.get_io().want_capture_mouse:
            mujoco.mjv_moveCamera(self.session.robot.model, mujoco.mjtMouse.mjMOUSE_ZOOM,
                                  0, -0.05 * y, self.scene, self.camera)

    def move_camera(self):
        cursor = glfw.get_cursor_pos(self.window)
        dx, dy = np.array(cursor) - self.cursor
        self.cursor = cursor
        if imgui.get_io().want_capture_mouse:
            return
        height = max(1, glfw.get_window_size(self.window)[1])
        action = None
        if glfw.get_mouse_button(self.window, glfw.MOUSE_BUTTON_LEFT):
            action = mujoco.mjtMouse.mjMOUSE_ROTATE_V
        elif glfw.get_mouse_button(self.window, glfw.MOUSE_BUTTON_RIGHT):
            action = mujoco.mjtMouse.mjMOUSE_MOVE_V
        if action is not None:
            mujoco.mjv_moveCamera(self.session.robot.model, action,
                                  dx / height, dy / height, self.scene, self.camera)

    def draw_hand_frames(self):
        """Draw measured palm frames as decorations, without physical geometry."""
        colors = ((1.0, 0.15, 0.15, 1.0), (0.15, 1.0, 0.15, 1.0),
                  (0.2, 0.5, 1.0, 1.0))
        for side, prefix in (("right", "R"), ("left", "L")):
            pose = self.session.robot.pose(self.session.data, side)
            for axis, (label, color) in enumerate(zip("XYZ", colors)):
                geom = self.scene.geoms[self.scene.ngeom]
                mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_ARROW,
                                    np.zeros(3), pose.position, np.eye(3).ravel(),
                                    np.array(color, dtype=np.float32))
                endpoint = pose.position + 0.15 * pose.rotation[:, axis]
                mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_ARROW,
                                     0.007, pose.position, endpoint)
                # MuJoCo 3.3.7 renders the shaft and tip over half the Z size.
                geom.size[2] *= 2
                geom.category = mujoco.mjtCatBit.mjCAT_DECOR
                geom.emission = 0.8
                self.scene.ngeom += 1
                # Connector labels sit at their origin, so place separate labels
                # at the axis tips to keep X/Y/Z from overlapping at the palm.
                geom = self.scene.geoms[self.scene.ngeom]
                mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_LABEL,
                                    np.zeros(3), endpoint, np.eye(3).ravel(),
                                    np.array(color, dtype=np.float32))
                geom.category = mujoco.mjtCatBit.mjCAT_DECOR
                geom.label = f"{prefix}:{label}"
                self.scene.ngeom += 1

    def hand_panel(self, side, number, top):
        session = self.session
        flags = imgui.WINDOW_NO_MOVE | imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_COLLAPSE
        imgui.set_next_window_position(0, top)
        imgui.set_next_window_size(self.SIDEBAR, 205)
        imgui.begin(f"Panel {number} | {side.title()} hand", flags=flags)
        if session.control_mode == "JOINT":
            imgui.text_wrapped("Hand targets paused. Disable Direct joints in panel 5 to resume.")
            measured = session.robot.pose(session.data, side).position
            for label, value in zip(("X", "Y", "Z"), measured):
                imgui.text(f"Actual {label}: {value:.3f} m")
            imgui.end()
            return
        imgui.text_colored("ACTIVE" if session.active_side == side else "Height held",
                           0.35, 0.8, 1.0)
        position = session.targets[side].position.copy()
        angles = session.rpy[side].copy()
        changed = False
        imgui.push_item_width(215)
        for index, label in enumerate(("X (m)", "Y (m)", "Z (m)")):
            minimum, maximum = ((-0.8, 0.9), (-1.0, 1.0), (0.15, 1.65))[index]
            edited, position[index] = imgui.slider_float(label, float(position[index]),
                                                        minimum, maximum, format="%.3f")
            position[index] = np.clip(position[index], minimum, maximum)
            changed |= edited
        for index, label in enumerate(("Roll (deg)", "Pitch (deg)", "Yaw (deg)")):
            edited, angles[index] = imgui.slider_float(label, float(angles[index]),
                                                      -180, 180, format="%.1f")
            angles[index] = np.clip(angles[index], -180, 180)
            changed |= edited
        imgui.pop_item_width()
        imgui.text("World XYZ / rotation from initial hand")
        imgui.end()
        if changed:
            session.set_target(side, position, angles, asynchronous=True)

    def finger_panel(self, side, number, top):
        imgui.set_next_window_position(0, top)
        imgui.set_next_window_size(self.SIDEBAR, 95)
        imgui.begin(f"Panel {number} | {side.title()} fingers", flags=(imgui.WINDOW_NO_MOVE
                    | imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_COLLAPSE))
        if self.session.control_mode == "JOINT":
            imgui.text_wrapped("Finger presets paused. Individual joints are controlled in panel 5.")
            imgui.end()
            return
        held = self.session.fingers.held_positions[side] is not None
        imgui.text("Individual pose held" if held else "Thumb: 0 = outward / 1 = curl")
        values = self.session.fingers.targets[side].copy()
        changed = False
        imgui.push_item_width(215)
        for index, label in enumerate(("Thumb", "Four fingers")):
            edited, values[index] = imgui.slider_float(label, float(values[index]), 0, 1, format="%.2f")
            values[index] = np.clip(values[index], 0, 1)
            changed |= edited
        imgui.pop_item_width()
        imgui.end()
        if changed:
            self.session.set_finger_targets(side, *values)

    def joint_panel(self, top):
        height = glfw.get_window_size(self.window)[1]
        imgui.set_next_window_position(0, top)
        imgui.set_next_window_size(self.SIDEBAR, max(140, height - top))
        imgui.begin("Panel 5 | Joint monitor", flags=(imgui.WINDOW_NO_MOVE
                    | imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_COLLAPSE))
        if imgui.button("Resume" if self.paused else "Pause"):
            self.paused = not self.paused
            self.session.set_paused(self.paused)
        imgui.same_line()
        if imgui.button("Reset"):
            # Recreate from the original model, not the already-converted motors.
            from ffw_control.simulation.session import ControlSession
            self.session.close()
            self.session = ControlSession()
            self.paused = False
            self.pickup_error = ""
        imgui.same_line()
        changed, direct = imgui.checkbox("Direct joints", self.session.control_mode == "JOINT")
        if changed:
            self.session.set_control_mode("JOINT" if direct else "IK")
        direct = self.session.control_mode == "JOINT"
        imgui.text("JOINT: Cartesian holds off" if direct else
                   f"{len(self.session.joints.names)} joints | hover: position / speed / force")
        imgui.push_item_width(245)
        _, self.joint_filter = imgui.input_text("Filter", self.joint_filter, 128)
        imgui.pop_item_width()
        imgui.begin_child("Joint rows", 0, 0)
        count = 0
        for state in self.session.joint_states():
            if self.joint_filter.lower() not in state["name"].lower():
                continue
            count += 1
            imgui.push_id(state["name"])
            color = ((0.9, 0.5, 0.5) if "_r_" in state["name"] else
                     (0.4, 0.8, 0.6) if "_l_" in state["name"] else (0.5, 0.75, 0.95))
            imgui.text_colored(state["name"], *color)
            hovered = imgui.is_item_hovered()
            imgui.same_line(position=166)
            if direct:
                imgui.push_item_width(130)
                edited, value = imgui.slider_float("##target", state["target"], state["lower"],
                                                   state["upper"], format="v=%.2f" if state["velocity_controlled"] else "%.3f")
                imgui.pop_item_width()
                if edited:
                    value = np.clip(value, state["lower"], state["upper"])
                    self.session.set_joint_target(state["name"], value)
            else:
                actual = state["velocity"] if state["velocity_controlled"] else state["position"]
                fraction = np.clip((actual - state["lower"]) / (state["upper"] - state["lower"]), 0, 1)
                prefix = "v=" if state["velocity_controlled"] else ""
                imgui.progress_bar(float(fraction), (130, 16), f"{prefix}{actual:.3f}")
            if hovered or imgui.is_item_hovered():
                input_unit = "rad/s" if state["velocity_controlled"] else state["unit"]
                force_unit = "N" if state["unit"] == "m" else "Nm"
                imgui.set_tooltip(f"{state['name']}\nPosition: {state['position']:.4f} {state['unit']}\n"
                                  f"Speed: {state['velocity']:.4f} {state['unit']}/s\n"
                                  f"Actuator force: {state['force']:.3f} {force_unit}\n"
                                  f"Target: {state['target']:.4f} {input_unit}")
            imgui.pop_id()
        if not count:
            imgui.text_disabled("No matching joints")
        imgui.end_child()
        imgui.end()

    def status_panel(self):
        metrics = self.session.metrics()
        height = glfw.get_window_size(self.window)[1]
        imgui.set_next_window_position(self.SIDEBAR + 10, max(10, height - 395))
        imgui.set_next_window_size(350, 385)
        imgui.set_next_window_bg_alpha(0.65)
        imgui.begin("Panel 6 | Simulation status", flags=(imgui.WINDOW_NO_MOVE
                    | imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_COLLAPSE))
        if metrics["ik_err"] is None:
            imgui.text("ik_err (R): N/A in joint mode")
        else:
            imgui.text(f"ik_err (R): {metrics['ik_err'] * 1000:.2f} mm")
            imgui.text(f"Right IK rotation: {metrics['right_ik_orientation_error_deg']:.2f} deg")
        imgui.text(f"sim_time: {metrics['sim_time']:.2f} s")
        imgui.text(f"wall_time: {metrics['wall_time']:.2f} s")
        imgui.text(f"loop_freq: {metrics['loop_freq']:.1f} Hz")
        imgui.text(f"loop_freq (ema): {metrics['loop_freq_ema']:.1f} Hz")
        imgui.separator()
        state = "Checking arm reach" if metrics["planning"] else metrics["status"]
        paused = " | PAUSED" if metrics["paused"] else ""
        imgui.text(f"{metrics['control_mode']} / {metrics['mode']}{paused}")
        imgui.text_wrapped(state)
        if metrics["control_mode"] == "IK":
            imgui.text(f"Actual {metrics['active_hand']}: {metrics['position_error_m']*1000:.2f} mm / {metrics['orientation_error_deg']:.2f} deg")
            imgui.text(f"Other hand Z error: {metrics['held_height_error_m']*1000:.2f} mm")
        imgui.text(f"Lift: {metrics['lift_m']:.4f} m | saturated: {metrics['saturated_actuators']}")
        imgui.text(f"Compute: {metrics['compute_ms']:.1f} ms")
        imgui.separator()
        for side, can in metrics["objects"].items():
            imgui.text(f"{side.title()} can: {can['state']} | {can['clearance_m'] * 1000:.0f} mm")
        for side in ("right", "left"):
            if side == "left":
                imgui.same_line()
            if imgui.button(f"Pick {side} can"):
                try:
                    self.session.start_grasp_demo(side)
                    self.pickup_error = ""
                except ValueError as error:
                    self.pickup_error = str(error)
        if metrics["grasp_demo"] is not None:
            imgui.text_wrapped(f"Pickup {metrics['grasp_demo']['side']}: {metrics['grasp_demo']['message']}")
            if self.session.grasp_demo.active and imgui.button("Cancel sequence"):
                self.session.cancel_grasp_demo()
        if self.pickup_error:
            imgui.text_wrapped(self.pickup_error)
        imgui.end()

    def panels(self):
        self.hand_panel("right", 1, 0)
        self.finger_panel("right", 2, 207)
        self.hand_panel("left", 3, 304)
        self.finger_panel("left", 4, 511)
        self.joint_panel(608)
        self.status_panel()

    def run(self, frames=0, screenshot=None):
        previous = time.monotonic()
        accumulator = 0.0
        count = 0
        try:
            while not glfw.window_should_close(self.window):
                glfw.poll_events()
                if glfw.get_key(self.window, glfw.KEY_ESCAPE) == glfw.PRESS:
                    break
                now = time.monotonic()
                accumulator += min(now - previous, 0.1)
                previous = now
                dt = self.session.robot.config["control_dt"]
                while accumulator >= dt:
                    if not self.paused:
                        self.session.tick()
                    accumulator -= dt
                self.renderer.process_inputs()
                imgui.new_frame()
                self.panels()
                self.move_camera()
                width, height = glfw.get_framebuffer_size(self.window)
                window_width = max(1, glfw.get_window_size(self.window)[0])
                sidebar = round(self.SIDEBAR * width / window_width)
                GL.glViewport(0, 0, width, height)
                GL.glClearColor(0.06, 0.07, 0.08, 1)
                GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
                mujoco.mjv_updateScene(self.session.robot.model, self.session.data, self.option,
                                       None, self.camera, mujoco.mjtCatBit.mjCAT_ALL, self.scene)
                self.draw_hand_frames()
                mujoco.mjr_render(mujoco.MjrRect(sidebar, 0, max(1, width-sidebar), height),
                                   self.scene, self.context)
                imgui.render()
                self.renderer.render(imgui.get_draw_data())
                count += 1
                if frames and count >= frames:
                    if screenshot:
                        save_screenshot(screenshot, width, height)
                    break
                glfw.swap_buffers(self.window)
        finally:
            self.session.close()
            self.context.free()
            self.renderer.shutdown()
            imgui.destroy_context(self.imgui_context)
            glfw.destroy_window(self.window)
            glfw.terminate()
