"""Run hand pose/finger panels 1-4 or a bounded headless controller session."""

import argparse
import json
import os
import sys

# The bundled GLFW selects Wayland from the desktop environment by default.
# Use X11/XWayland for this Linux OpenGL application, without changing the desktop.
if sys.platform.startswith("linux"):
    os.environ.setdefault("PYGLFW_LIBRARY_VARIANT", "x11")
    os.environ.setdefault("PYOPENGL_PLATFORM", "glx")

from ffw_control.simulation.session import ControlSession


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--seconds", type=float, default=5.0, help="Headless simulation duration")
    parser.add_argument("--side", choices=("right", "left"), default="right")
    parser.add_argument("--offset", nargs=3, type=float, metavar=("X", "Y", "Z"))
    parser.add_argument("--rpy", nargs=3, type=float, default=[0, 0, 0], metavar=("ROLL", "PITCH", "YAW"))
    parser.add_argument("--right-fingers", nargs=2, type=float, default=[0, 0], metavar=("THUMB", "FOUR_FINGERS"),
                        help="Right finger targets: 0=open, 1=closed")
    parser.add_argument("--left-fingers", nargs=2, type=float, default=[0, 0], metavar=("THUMB", "FOUR_FINGERS"),
                        help="Left finger targets: 0=open, 1=closed")
    parser.add_argument("--frames", type=int, default=0, help="Close GUI after this many frames; 0 keeps it open")
    parser.add_argument("--screenshot", help="Save PNG on the final frame; requires --frames")
    args = parser.parse_args()
    if args.seconds <= 0 or args.frames < 0:
        parser.error("--seconds must be positive and --frames must be nonnegative")
    if args.screenshot and not args.frames:
        parser.error("--screenshot requires --frames")
    session = ControlSession()
    try:
        session.set_finger_targets("right", *args.right_fingers)
        session.set_finger_targets("left", *args.left_fingers)
    except ValueError as error:
        parser.error(str(error))
    if args.offset is not None or any(args.rpy):
        target = session.targets[args.side].position + (args.offset or [0, 0, 0])
        session.set_target(args.side, target, args.rpy)
    if args.headless:
        for _ in range(round(args.seconds / session.robot.config["control_dt"])):
            session.tick()
        print(json.dumps(session.metrics(), indent=2))
        if session.mode == "STOP":
            raise SystemExit(1)
    else:
        from ffw_control.ui.window import ControlWindow
        ControlWindow(session).run(args.frames, args.screenshot)


if __name__ == "__main__":
    main()
