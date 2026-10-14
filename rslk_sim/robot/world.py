"""World description: floor (line track) and walls.  Coordinates in metres, y up."""
import json
import math
import os

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
import pygame  # noqa: E402

MM_PER_PX = 1.0     # floor raster resolution


class World:
    def __init__(self, data, base_dir="."):
        self.name = data.get("name", "world")
        self.width, self.height = data.get("size", [2.0, 1.5])
        self.background = float(data.get("background", 1.0))
        self.walls = [tuple(w) for w in data.get("walls", [])]
        sx, sy, sh = data.get("start", [self.width / 2, self.height / 2, 90.0])
        self.start = (sx, sy, math.radians(sh))
        self.description = data.get("description", "")
        self._build_floor(data, base_dir)

    @classmethod
    def load(cls, path):
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        return cls(data, os.path.dirname(os.path.abspath(path)))

    @classmethod
    def empty(cls):
        return cls({"name": "empty floor", "size": [2.0, 2.0], "start": [1.0, 1.0, 90.0]})

    # -- floor ---------------------------------------------------------------
    def _px(self, x, y):
        return (x * 1000.0 / MM_PER_PX, (self.height - y) * 1000.0 / MM_PER_PX)

    def _build_floor(self, data, base_dir):
        w = int(round(self.width * 1000 / MM_PER_PX))
        h = int(round(self.height * 1000 / MM_PER_PX))
        surf = pygame.Surface((w, h))
        g = int(round(self.background * 255))
        surf.fill((g, g, g))
        img = data.get("image")
        if img:
            pic = pygame.image.load(os.path.join(base_dir, img["file"]))
            scale = img.get("meters_per_pixel", 0.001) * 1000 / MM_PER_PX
            pic = pygame.transform.smoothscale(pic, (int(pic.get_width() * scale), int(pic.get_height() * scale)))
            ox, oy = img.get("origin", [0.0, 0.0])
            px, py = self._px(ox, oy + pic.get_height() * MM_PER_PX / 1000)
            surf.blit(pic, (px, py))
        for area in data.get("areas", []):
            g = int(round(area.get("gray", 0.5) * 255))
            pts = [self._px(x, y) for x, y in area["points"]]
            pygame.draw.polygon(surf, (g, g, g), pts)
        for line in data.get("lines", []):
            g = int(round(line.get("gray", 0.0) * 255))
            color = (g, g, g)
            width_px = max(1, int(round(line.get("width", 0.019) * 1000 / MM_PER_PX)))
            pts = [self._px(x, y) for x, y in line["points"]]
            if line.get("closed"):
                pts = pts + [pts[0]]
            for a, b in zip(pts, pts[1:]):
                pygame.draw.line(surf, color, a, b, width_px)
            for p in pts:
                pygame.draw.circle(surf, color, (int(p[0]), int(p[1])), width_px // 2)
        self.floor = surf
        self._fw, self._fh = w, h

    def reflectance(self, x, y):
        """Floor brightness 0 (black) .. 1 (white) at world point (x, y)."""
        px = int(x * 1000.0 / MM_PER_PX)
        py = int((self.height - y) * 1000.0 / MM_PER_PX)
        if 0 <= px < self._fw and 0 <= py < self._fh:
            return self.floor.get_at((px, py))[0] / 255.0
        return self.background

    # -- walls ---------------------------------------------------------------
    def raycast(self, x, y, angle, max_dist=1.5):
        """Distance from (x, y) along `angle` to the nearest wall (or max_dist)."""
        dx, dy = math.cos(angle), math.sin(angle)
        best = max_dist
        for x1, y1, x2, y2 in self.walls:
            ex, ey = x2 - x1, y2 - y1
            den = dx * ey - dy * ex
            if abs(den) < 1e-12:
                continue
            wx, wy = x1 - x, y1 - y
            t = (wx * ey - wy * ex) / den
            u = (wx * dy - wy * dx) / den
            if 0 <= u <= 1 and 0 < t < best:
                best = t
        return best

    def closest_wall_points(self, x, y, radius):
        """[(distance, angle)] of wall points within radius of (x, y)."""
        hits = []
        for x1, y1, x2, y2 in self.walls:
            ex, ey = x2 - x1, y2 - y1
            L2 = ex * ex + ey * ey
            u = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - x1) * ex + (y - y1) * ey) / L2))
            cx, cy = x1 + u * ex, y1 + u * ey
            d = math.hypot(cx - x, cy - y)
            if d <= radius:
                hits.append((d, math.atan2(cy - y, cx - x)))
        return hits

    def collides(self, x, y, radius):
        return bool(self.closest_wall_points(x, y, radius))
