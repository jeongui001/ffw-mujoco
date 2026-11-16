"""Shared typography and colors for the desktop control panels."""

from pathlib import Path

import imgui


RIGHT = (0.35, 0.80, 0.91)
LEFT = (0.72, 0.66, 0.96)
TEXT = (0.89, 0.92, 0.96)
MUTED = (0.53, 0.60, 0.69)
BACKGROUND = (0.055, 0.071, 0.094, 1.0)


def apply_theme():
    """Load bundled fonts before the renderer builds its font texture."""
    fonts = Path(__file__).with_name("fonts")
    atlas = imgui.get_io().fonts
    atlas.add_font_from_file_ttf(str(fonts / "Lato-Regular.ttf"), 15)
    heading_font = atlas.add_font_from_file_ttf(str(fonts / "Lato-Semibold.ttf"), 16)

    style = imgui.get_style()
    style.window_padding = (12, 8)
    style.frame_padding = (8, 2)
    style.item_spacing = (8, 3)
    style.item_inner_spacing = (8, 4)
    style.window_rounding = 9
    style.child_rounding = 5
    style.frame_rounding = 4
    style.grab_rounding = 4
    style.scrollbar_rounding = 5
    style.window_border_size = 1
    style.frame_border_size = 0
    style.scrollbar_size = 7
    style.grab_min_size = 12

    colors = {
        imgui.COLOR_TEXT: (*TEXT, 1),
        imgui.COLOR_TEXT_DISABLED: (*MUTED, 1),
        imgui.COLOR_WINDOW_BACKGROUND: (0.085, 0.106, 0.137, 0.98),
        imgui.COLOR_CHILD_BACKGROUND: (0, 0, 0, 0),
        imgui.COLOR_POPUP_BACKGROUND: (0.10, 0.13, 0.17, 0.99),
        imgui.COLOR_BORDER: (0.19, 0.24, 0.30, 0.65),
        imgui.COLOR_BORDER_SHADOW: (0, 0, 0, 0),
        imgui.COLOR_FRAME_BACKGROUND: (0.13, 0.17, 0.22, 1),
        imgui.COLOR_FRAME_BACKGROUND_HOVERED: (0.18, 0.24, 0.30, 1),
        imgui.COLOR_FRAME_BACKGROUND_ACTIVE: (0.20, 0.29, 0.36, 1),
        imgui.COLOR_BUTTON: (0.15, 0.21, 0.27, 1),
        imgui.COLOR_BUTTON_HOVERED: (0.21, 0.31, 0.39, 1),
        imgui.COLOR_BUTTON_ACTIVE: (0.25, 0.40, 0.48, 1),
        imgui.COLOR_CHECK_MARK: (*RIGHT, 1),
        imgui.COLOR_SLIDER_GRAB: (*RIGHT, 0.85),
        imgui.COLOR_SLIDER_GRAB_ACTIVE: (0.60, 0.91, 0.98, 1),
        imgui.COLOR_PLOT_HISTOGRAM: (0.28, 0.54, 0.65, 1),
        imgui.COLOR_PLOT_HISTOGRAM_HOVERED: (*RIGHT, 1),
        imgui.COLOR_SEPARATOR: (0.19, 0.24, 0.30, 1),
        imgui.COLOR_SCROLLBAR_BACKGROUND: (0, 0, 0, 0),
        imgui.COLOR_SCROLLBAR_GRAB: (0.23, 0.29, 0.36, 1),
        imgui.COLOR_SCROLLBAR_GRAB_HOVERED: (0.32, 0.40, 0.48, 1),
        imgui.COLOR_SCROLLBAR_GRAB_ACTIVE: (0.40, 0.50, 0.60, 1),
        imgui.COLOR_HEADER: (0.16, 0.25, 0.31, 1),
        imgui.COLOR_HEADER_HOVERED: (0.21, 0.33, 0.40, 1),
        imgui.COLOR_HEADER_ACTIVE: (0.25, 0.40, 0.48, 1),
        imgui.COLOR_TEXT_SELECTED_BACKGROUND: (0.27, 0.61, 0.74, 0.35),
    }
    for color, value in colors.items():
        style.colors[color] = value
    return heading_font
