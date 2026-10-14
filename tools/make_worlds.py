"""Generate the bundled worlds in ../worlds/.  Run: python tools/make_worlds.py"""
import json
import math
import os

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "worlds")
TAPE = 0.019        # 3/4 inch electrical tape


def arc(cx, cy, r, a0, a1, step_deg=3.0):
    n = max(2, int(abs(a1 - a0) / step_deg))
    return [[round(cx + r * math.cos(math.radians(a0 + (a1 - a0) * i / n)), 4),
             round(cy + r * math.sin(math.radians(a0 + (a1 - a0) * i / n)), 4)] for i in range(n + 1)]


def box(x0, y0, x1, y1):
    return [[x0, y0, x1, y0], [x1, y0, x1, y1], [x1, y1, x0, y1], [x0, y1, x0, y0]]


def save(name, data):
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, name + ".json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump(data, f, indent=1)
    print("wrote", name)


def main():
    # 1. oval line track ----------------------------------------------------
    pts = [[0.55, 0.35], [1.45, 0.35]] + arc(1.45, 0.75, 0.40, -90, 90)[1:] + [[0.55, 1.15]] + arc(0.55, 0.75, 0.40, 90, 270)[1:-1]
    save("line_oval", {
        "name": "Oval line track",
        "description": "Black tape oval (0.9 m straights, 0.4 m radius turns) - line following",
        "size": [2.0, 1.5], "background": 1.0,
        "lines": [{"points": pts, "closed": True, "width": TAPE}],
        "start": [0.75, 0.35, 0.0],
    })

    # 2. line course with sharp turns, a gap and a dead end -----------------------------
    course = ([[0.2, 0.3], [1.1, 0.3]] + arc(1.1, 0.6, 0.3, -90, 0)[1:] + [[1.4, 1.0], [1.0, 1.0]])
    part2 = [[0.95, 1.0], [0.6, 1.0]] + arc(0.6, 1.2, 0.2, -90, -270, )[1:] + [[0.6, 1.4], [0.25, 1.4]]
    save("line_course", {
        "name": "Line course",
        "description": "Curves, a sharp 90 degree corner, a 5 cm gap and a dead end - FSM line following",
        "size": [1.8, 1.6], "background": 1.0,
        "lines": [{"points": course, "width": TAPE}, {"points": part2, "width": TAPE}],
        "start": [0.3, 0.3, 0.0],
    })

    # 3. straight line for speed / odometry tests ------------------------------------
    save("line_straight", {
        "name": "Straight line",
        "description": "A 2.5 m straight line - speed, odometry and steering tests",
        "size": [3.0, 0.8], "background": 1.0,
        "lines": [{"points": [[0.25, 0.4], [2.75, 0.4]], "width": TAPE}],
        "start": [0.3, 0.4, 0.0],
    })

    # 4. walled arena with obstacles -------------------------------------------------
    walls = box(0.0, 0.0, 1.6, 1.6) + box(1.0, 1.0, 1.25, 1.25) + [[0.2, 1.1, 0.55, 1.45]]
    save("wall_arena", {
        "name": "Walled arena",
        "description": "1.6 m box with obstacles - bump switches and IR distance sensors",
        "size": [1.6, 1.6], "background": 0.92,
        "walls": walls,
        "start": [0.8, 0.5, 90.0],
    })

    # 5. corridor loop with a centre line (wall following / IR control) ---------------
    walls = box(0.0, 0.0, 2.2, 1.6) + box(0.45, 0.45, 1.75, 1.15)
    centre = [[0.225, 0.225], [1.975, 0.225], [1.975, 1.375], [0.225, 1.375]]
    save("corridor_loop", {
        "name": "Corridor loop",
        "description": "45 cm wide corridor around a block, with a centre line - wall following (Lab 17)",
        "size": [2.2, 1.6], "background": 0.92,
        "walls": walls,
        "lines": [{"points": centre, "closed": True, "width": TAPE}],
        "start": [0.8, 0.225, 0.0],
    })

    # 6. maze ---------------------------------------------------------------
    c = 0.4
    walls = box(0.0, 0.0, 4 * c, 4 * c)
    segs = [(1, 0, 1, 2), (2, 1, 2, 3), (3, 0, 3, 1), (0, 3, 1, 3), (3, 2, 4, 2), (2, 3, 2, 4), (1, 2, 2, 2)]
    walls += [[x0 * c, y0 * c, x1 * c, y1 * c] for x0, y0, x1, y1 in segs]
    save("maze", {
        "name": "Small maze",
        "description": "4x4 maze with 40 cm cells - wall following and bump recovery",
        "size": [4 * c, 4 * c], "background": 0.92,
        "walls": walls,
        "start": [0.5 * c, 0.5 * c, 90.0],
    })


if __name__ == "__main__":
    main()
