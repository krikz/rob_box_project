"""Угол врезки «картинка в картинке»: уходит от робота на виде сверху.

Решение принимает код, а не глаз монтажёра: по положению робота в пикселях кадра выбирается
угол, дальний от него. Угол держится, пока робот от врезки далеко (в центре кадра не мельтешим);
уходит от робота с гистерезисом и плавным переездом за TRANSITION_S секунд ролика. Если прямой путь врезки
в новый угол прошёл бы по роботу, врезка не едет через него, а гаснет в старом углу и проявляется в новом.
Чистая функция состояния без PIL и numpy: тест гоняется без контейнера rob_box_sim:hil.
"""

import math

W, H = 1280, 720
PIP_W, PIP_H = 480, 270
MARGIN = 12  # от края кадра до рамки врезки
BAR_H = 48  # нижняя плашка со статусом
ROBOT_HALF = 30  # робот на виде сверху ~0.75 м * 40 пикс/м, берём коробку 60x60 пикс вокруг точки позы
LEAVE_DIST = 90  # робот ближе этого к рамке врезки: пора уезжать (из центра кадра врезка не уходит)
TRANSITION_S = 0.5
GOOD_GAP = 120  # новый угол с таким зазором считаем хорошим: среди хороших берём тот, куда можно доехать
DRIFT = 30  # на сколько пикселей робот может уйти за время переезда (≈2.5 с sim при ×5)
SWEEP_STEPS = 24

# верхний левый угол рамки врезки (без белой окантовки в 2 пикс.) для каждого угла
CORNERS = {
    "br": (W - PIP_W - MARGIN, H - PIP_H - BAR_H - MARGIN),
    "bl": (MARGIN, H - PIP_H - BAR_H - MARGIN),
    "tr": (W - PIP_W - MARGIN, MARGIN),
    "tl": (MARGIN, MARGIN),
}
DEFAULT_CORNER = "br"


def rect_at(x, y):
    """Рамка врезки с окантовкой: (x0, y0, x1, y1)."""
    return (x - 2, y - 2, x + PIP_W + 2, y + PIP_H + 2)


def robot_box(rx, ry, half=ROBOT_HALF):
    return (rx - half, ry - half, rx + half, ry + half)


def gap(a, b):
    """Зазор между двумя прямоугольниками в пикселях (0 и меньше — пересекаются, отрицательное — глубина)."""
    dx = max(b[0] - a[2], a[0] - b[2])
    dy = max(b[1] - a[3], a[1] - b[3])
    if dx > 0 and dy > 0:
        return math.hypot(dx, dy)
    return max(dx, dy)


def corner_gap(corner, rx, ry):
    return gap(rect_at(*CORNERS[corner]), robot_box(rx, ry))


def smoothstep(p):
    return p * p * (3 - 2 * p)


def slide_is_clear(from_xy, corner, rx, ry):
    """Врезка, едущая из from_xy в угол, нигде по пути не заденет робота (с запасом на его дрейф)."""
    box = robot_box(rx, ry, ROBOT_HALF + DRIFT)
    x1, y1 = CORNERS[corner]
    for i in range(SWEEP_STEPS + 1):
        s = smoothstep(i / SWEEP_STEPS)
        pos = (from_xy[0] + (x1 - from_xy[0]) * s, from_xy[1] + (y1 - from_xy[1]) * s)
        if gap(rect_at(*pos), box) < 0:
            return False
    return True


def choose_corner(current, from_xy, rx, ry):
    """Угол врезки по положению робота: (угол, способ переезда 'slide' | 'fade'). Остаться — (current, None)."""
    if corner_gap(current, rx, ry) >= LEAVE_DIST:
        return current, None

    def score(c):
        g = corner_gap(c, rx, ry)
        return (g >= GOOD_GAP and slide_is_clear(from_xy, c, rx, ry), g)

    best = max((c for c in CORNERS if c != current), key=score)
    return best, "slide" if score(best)[0] else "fade"


class PipPlacer:
    """Положение врезки по кадрам: переключение с гистерезисом, переезд за TRANSITION_S.

    update(rx, ry) -> (x, y, alpha): левый верхний угол врезки и её непрозрачность (1 — обычная).
    """

    def __init__(self, fps, corner=DEFAULT_CORNER):
        self.fps = fps
        self.corner = corner
        self.mode = None
        self.from_xy = CORNERS[corner]
        self.progress = 1.0  # 1 — переезд закончен
        self.pos = CORNERS[corner]

    def update(self, rx, ry):
        if self.progress >= 1.0:
            nxt, mode = choose_corner(self.corner, self.pos, rx, ry)
            if mode:
                self.from_xy, self.corner, self.mode, self.progress = self.pos, nxt, mode, 0.0
        alpha = 1.0
        if self.progress < 1.0:
            self.progress = min(1.0, self.progress + 1.0 / (self.fps * TRANSITION_S))
            p = self.progress
            if self.mode == "slide":
                s = smoothstep(p)
                (x0, y0), (x1, y1) = self.from_xy, CORNERS[self.corner]
                self.pos = (x0 + (x1 - x0) * s, y0 + (y1 - y0) * s)
            elif p < 0.5:  # fade: гаснет там, где был
                self.pos, alpha = self.from_xy, 1.0 - smoothstep(p * 2)
            else:  # и проявляется в новом углу
                self.pos, alpha = CORNERS[self.corner], smoothstep((p - 0.5) * 2)
        else:
            self.pos = CORNERS[self.corner]
        return (round(self.pos[0]), round(self.pos[1]), alpha)
