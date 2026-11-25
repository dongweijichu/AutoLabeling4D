"""Project a replayed action onto the unchanged scale and axes of the before-action LiDAR JPEG."""
from math import cos, isfinite, pi, sin


AXES = ("x", "y", "z", "length", "width", "height", "yaw")
PANELS = ((6, 57, 476, 510), (489, 57, 954, 259), (489, 297, 954, 510))
SIGNS = ((-1, -1), (1, -1), (1, 1), (-1, 1))


def panel_points(panel, extent_x, extent_y, points, round_pixels=True):
    left, top, right, bottom = panel
    cx, cy = (left + right) / 2, (top + bottom) / 2
    scale = min((right - left - 28) / (2 * extent_x), (bottom - top - 28) / (2 * extent_y))
    coords = [[cx + scale * x, cy - scale * y] for x, y in points]
    return [[round(x), round(y)] for x, y in coords] if round_pixels else coords


def overlays(box, action):
    """Return top/side/end after-action polygons and heading segment in 960×525 source pixels."""
    if len(box) != 7 or not all(map(isfinite, box)) or min(box[3:6]) <= 0:
        raise ValueError("Invalid input box")
    after = list(box)
    if action["type"] != "stop":
        i = AXES.index(action["axis"])
        expected = "translate" if i < 3 else "resize" if i < 6 else "rotate"
        if action["type"] != expected or not isfinite(action["value"]):
            raise ValueError("Invalid action")
        after[i] += action["value"]
        if i == 6:
            after[i] = (after[i] + pi) % (2*pi) - pi
    if min(after[3:6]) <= 0:
        raise ValueError("Replayed box has a nonpositive dimension")
    dx, dy, dz = [a-b for a, b in zip(after[:3], box[:3])]
    l0, w0, h0 = (d/2 for d in box[3:6])
    l, w, h = (d/2 for d in after[3:6])
    c0, s0 = cos(box[6]), sin(box[6])
    c, s = cos(after[6]), sin(after[6])
    ex = abs(c0)*l0 + abs(s0)*w0 + 1.65
    ey = abs(s0)*l0 + abs(c0)*w0 + 1.65
    footprint = [(dx+c*i*l-s*j*w, dy+s*i*l+c*j*w) for i, j in SIGNS]
    top = panel_points(PANELS[0], ex, ey, footprint)
    heading = panel_points(PANELS[0], ex, ey, [(dx,dy), (dx+c*l*.75,dy+s*l*.75)], round_pixels=False)

    # Orthographic silhouettes in the original box's two vertical planes.
    # A new yaw changes the projected half-spans even though l, w and h stay fixed.
    angle = after[6] - box[6]
    a, b = abs(cos(angle)), abs(sin(angle))
    silhouettes = []
    for panel, horizontal, half_span, old_half in (
            (PANELS[1], dx*c0+dy*s0, a*l+b*w, l0),
            (PANELS[2], -dx*s0+dy*c0, b*l+a*w, w0)):
        rect = [(horizontal+i*half_span, dz+j*h) for i, j in SIGNS]
        silhouettes.append(panel_points(panel, old_half+1.65, h0+1.15, rect))
    return {"polygons": [top, *silhouettes], "heading": heading}
