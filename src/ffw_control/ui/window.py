"""GLFW/MuJoCo view with hand pose and finger panels 1-4."""

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
        self.camera.lookat[:] = [0.1, 0, 0.85]
        self.camera.distance = 2.6
        self.camera.azimuth = 135
        self.camera.elevation = -20
        self.cursor = glfw.get_cursor_pos(self.window)
        self.paused = False
        glfw.set_scroll_callback(self.window, self.scroll)
        style = imgui.get_style()
        style.window_rounding = 5
        style.frame_rounding = 3
        style.colors[imgui.COLOR_WINDOW_BACKGROUND] = (0.09, 0.105, 0.12, 0.97)
        style.colors[imgui.COLOR_FRAME_BACKGROUND] = (0.16, 0.19, 0.22, 1)
        style.colors[imgui.COLOR_SLIDER_GRAB] = (0.2, 0.6, 0.95, 1)
        style.colors[imgui.COLOR_SLIDER_GRAB_ACTIVE] = (0.4, 0.75, 1, 1)

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

    def hand_panel(self, side, number, top):
        session = self.session
        flags = imgui.WINDOW_NO_MOVE | imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_COLLAPSE
        imgui.set_next_window_position(0, top)
        imgui.set_next_window_size(self.SIDEBAR, 220)
        imgui.begin(f"Panel {number} | {side.title()} hand", flags=flags)
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
        imgui.set_next_window_size(self.SIDEBAR, 105)
        imgui.begin(f"Panel {number} | {side.title()} fingers", flags=(imgui.WINDOW_NO_MOVE
                    | imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_COLLAPSE))
        imgui.text("0 = open / 1 = closed")
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

    def panels(self):
        self.hand_panel("right", 1, 0)
        self.finger_panel("right", 2, 222)
        self.hand_panel("left", 3, 329)
        self.finger_panel("left", 4, 551)
        imgui.set_next_window_position(0, 658)
        imgui.set_next_window_size(self.SIDEBAR, 150)
        imgui.begin("Controls", flags=imgui.WINDOW_NO_MOVE | imgui.WINDOW_NO_RESIZE)
        if imgui.button("Resume" if self.paused else "Pause"):
            self.paused = not self.paused
        imgui.same_line()
        if imgui.button("Reset"):
            # Recreate from the original model, not the already-converted motors.
            from ffw_control.simulation.session import ControlSession
            self.session.close()
            self.session = ControlSession()
        imgui.text_wrapped("Last edited hand follows its target. The other hand keeps its height.")
        imgui.text("Drag: orbit | Right drag: pan | Wheel: zoom")
        imgui.end()

        metrics = self.session.metrics()
        height = glfw.get_window_size(self.window)[1]
        imgui.set_next_window_position(self.SIDEBAR + 10, max(10, height - 205))
        imgui.set_next_window_size(345, 195)
        imgui.set_next_window_bg_alpha(0.65)
        imgui.begin("Tracking status", flags=imgui.WINDOW_NO_MOVE | imgui.WINDOW_NO_RESIZE)
        state = "Checking arm reach" if metrics["planning"] else metrics["status"]
        imgui.text(f"{metrics['active_hand'].title()} / {metrics['mode']} / {state}")
        imgui.text(f"Actual error: {metrics['position_error_m']*1000:.2f} mm / {metrics['orientation_error_deg']:.2f} deg")
        imgui.text(f"IK error: {metrics['ik_position_error_m']*1000:.2f} mm / {metrics['ik_orientation_error_deg']:.2f} deg")
        imgui.text(f"Other hand Z error: {metrics['held_height_error_m']*1000:.2f} mm")
        imgui.text(f"Lift: {metrics['lift_m']:.4f} m | saturated: {metrics['saturated_actuators']}")
        imgui.text(f"Sim: {metrics['sim_time']:.2f} s | compute: {metrics['compute_ms']:.1f} ms")
        imgui.end()

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
