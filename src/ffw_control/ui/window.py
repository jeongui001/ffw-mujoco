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

from ffw_control.ui.theme import BACKGROUND, LEFT, MUTED, RIGHT, TEXT, apply_theme


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
    SIDEBAR = 400
    MARGIN = 12
    PANEL_FLAGS = (imgui.WINDOW_NO_MOVE | imgui.WINDOW_NO_RESIZE
                   | imgui.WINDOW_NO_COLLAPSE | imgui.WINDOW_NO_TITLE_BAR)

    def __init__(self, session):
        if not glfw.init():
            raise RuntimeError("Cannot open display. Use --headless for model/controller checks.")
        glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 2)
        glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 1)
        self.window = glfw.create_window(1500, 960, "FFW-SH5 | Control Studio", None, None)
        if not self.window:
            glfw.terminate()
            raise RuntimeError("Cannot create an OpenGL window")
        glfw.make_context_current(self.window)
        glfw.set_window_size_limits(self.window, 1120, 850, glfw.DONT_CARE, glfw.DONT_CARE)
        glfw.swap_interval(1)
        self.imgui_context = imgui.create_context()
        imgui.get_io().ini_file_name = None
        self.heading_font = apply_theme()
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
        glfw.set_scroll_callback(self.window, self.scroll)

    def begin_panel(self, number, title, top, height, accent=RIGHT, left=None, width=None):
        """Give every panel the same inset, heading, and quiet number marker."""
        imgui.set_next_window_position(self.MARGIN if left is None else left, top)
        imgui.set_next_window_size(self.SIDEBAR - 2 * self.MARGIN if width is None else width,
                                   height)
        imgui.begin(f"Panel {number} | {title}", flags=self.PANEL_FLAGS)
        imgui.text_colored(f"{number:02d}", *accent)
        imgui.same_line()
        imgui.push_font(self.heading_font)
        imgui.text(title)
        imgui.pop_font()

    def sidebar_header(self):
        imgui.set_next_window_position(self.MARGIN, 8)
        imgui.set_next_window_size(self.SIDEBAR - 2 * self.MARGIN, 36)
        imgui.set_next_window_bg_alpha(0)
        imgui.begin("Control Studio", flags=self.PANEL_FLAGS | imgui.WINDOW_NO_INPUTS)
        imgui.push_font(self.heading_font)
        imgui.text("FFW-SH5")
        imgui.pop_font()
        imgui.same_line()
        imgui.text_colored("/  CONTROL STUDIO", *MUTED)
        imgui.end()

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
        accent = RIGHT if side == "right" else LEFT
        self.begin_panel(number, f"{side.title()} hand", top, 180, accent)
        if imgui.is_item_hovered():
            imgui.set_tooltip("World XYZ / rotation from initial hand")
        imgui.same_line(position=imgui.get_window_width() - 96)
        state = ("JOINT MODE" if session.control_mode == "JOINT" else
                 "ACTIVE" if session.active_side == side else "XYZ HOLD")
        imgui.text_colored(state, *(accent if state == "ACTIVE" else MUTED))
        imgui.spacing()
        if session.control_mode == "JOINT":
            imgui.text_wrapped("Hand targets paused. Disable Direct joints in panel 5 to resume.")
            measured = session.robot.pose(session.data, side).position
            for label, value in zip(("X", "Y", "Z"), measured):
                imgui.text(f"Actual {label}: {value:.3f} m")
            imgui.end()
            return
        position = session.targets[side].position.copy()
        angles = session.rpy[side].copy()
        changed = False
        imgui.push_item_width(248)
        imgui.push_style_color(imgui.COLOR_SLIDER_GRAB, *accent, 0.85)
        imgui.push_style_color(imgui.COLOR_SLIDER_GRAB_ACTIVE, *accent, 1)
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
        imgui.pop_style_color(2)
        imgui.end()
        if changed:
            session.set_target(side, position, angles)

    def finger_panel(self, side, number, top):
        accent = RIGHT if side == "right" else LEFT
        self.begin_panel(number, f"{side.title()} fingers", top, 104, accent)
        if self.session.control_mode == "JOINT":
            imgui.text_wrapped("Finger presets paused. Individual joints are controlled in panel 5.")
            imgui.end()
            return
        held = self.session.fingers.held_positions[side] is not None
        imgui.text_disabled("Individual pose held" if held else "0  Open / outward     1  Curl")
        values = self.session.fingers.targets[side].copy()
        changed = False
        imgui.push_item_width(248)
        imgui.push_style_color(imgui.COLOR_SLIDER_GRAB, *accent, 0.85)
        imgui.push_style_color(imgui.COLOR_SLIDER_GRAB_ACTIVE, *accent, 1)
        for index, label in enumerate(("Thumb", "Four fingers")):
            edited, values[index] = imgui.slider_float(label, float(values[index]), 0, 1, format="%.2f")
            values[index] = np.clip(values[index], 0, 1)
            changed |= edited
        imgui.pop_item_width()
        imgui.pop_style_color(2)
        imgui.end()
        if changed:
            self.session.set_finger_targets(side, *values)

    def joint_panel(self, top):
        height = glfw.get_window_size(self.window)[1]
        self.begin_panel(5, "Joint monitor", top, height - top - self.MARGIN, MUTED)
        imgui.spacing()
        if imgui.button("Resume" if self.paused else "Pause"):
            self.paused = not self.paused
            self.session.set_paused(self.paused)
        imgui.same_line()
        if imgui.button("Reset"):
            # Recreate from the original model, not the already-converted motors.
            from ffw_control.simulation.session import ControlSession
            self.session = ControlSession()
            self.paused = False
        imgui.same_line()
        changed, direct = imgui.checkbox("Direct joints", self.session.control_mode == "JOINT")
        if changed:
            self.session.set_control_mode("JOINT" if direct else "IK")
        direct = self.session.control_mode == "JOINT"
        imgui.text_disabled("JOINT: Cartesian holds off" if direct else
                            f"{len(self.session.joints.names)} joints / hover for details")
        imgui.push_item_width(290)
        _, self.joint_filter = imgui.input_text("Filter", self.joint_filter, 128)
        imgui.pop_item_width()
        imgui.begin_child("Joint rows", 0, 0)
        count = 0
        for state in self.session.joint_states():
            if self.joint_filter.lower() not in state["name"].lower():
                continue
            count += 1
            imgui.push_id(state["name"])
            color = RIGHT if "_r_" in state["name"] else LEFT if "_l_" in state["name"] else TEXT
            imgui.text_colored(state["name"], *color)
            hovered = imgui.is_item_hovered()
            imgui.same_line(position=190)
            if direct:
                imgui.push_item_width(140)
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
                imgui.progress_bar(float(fraction), (140, 19), f"{prefix}{actual:.3f}")
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
        self.begin_panel(6, "Simulation status", height - 194, 182,
                         left=self.SIDEBAR + 20, width=330)
        imgui.spacing()
        if metrics["ik_err"] is None:
            self.metric_row("ik_err (R)", "N/A in joint mode")
        else:
            self.metric_row("ik_err (R)", f"{metrics['ik_err'] * 1000:.2f} mm", RIGHT)
            rotation_error = metrics["right_ik_orientation_error_deg"]
            self.metric_row("Right rotation" if rotation_error is None else "Right IK rotation",
                            "free" if rotation_error is None else f"{rotation_error:.2f} deg", RIGHT)
        self.metric_row("sim_time", f"{metrics['sim_time']:.2f} s")
        self.metric_row("wall_time", f"{metrics['wall_time']:.2f} s")
        self.metric_row("loop_freq", f"{metrics['loop_freq']:.1f} Hz")
        self.metric_row("loop_freq (ema)", f"{metrics['loop_freq_ema']:.1f} Hz")
        imgui.end()

    def metric_row(self, label, value, color=TEXT):
        imgui.text_colored(label, *MUTED)
        value_left = imgui.get_window_width() - 12 - imgui.calc_text_size(value)[0]
        imgui.same_line(position=value_left)
        imgui.text_colored(value, *color)
        imgui.spacing()

    def panels(self):
        self.sidebar_header()
        self.hand_panel("right", 1, 48)
        self.finger_panel("right", 2, 236)
        self.hand_panel("left", 3, 348)
        self.finger_panel("left", 4, 536)
        self.joint_panel(648)
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
                GL.glClearColor(*BACKGROUND)
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
            self.context.free()
            self.renderer.shutdown()
            imgui.destroy_context(self.imgui_context)
            glfw.destroy_window(self.window)
            glfw.terminate()
