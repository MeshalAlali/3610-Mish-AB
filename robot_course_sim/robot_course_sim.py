import argparse
import math
import os
import sys
import tkinter as tk
from dataclasses import dataclass, field


WHITE_THRESH = 200
BLACK_THRESH = 800
MAX_DUTY = 12.0
MOTOR_SCALE = 1.2
DT = 0.02


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def lerp(a, b, t):
    return a + (b - a) * t


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def point_segment_distance(px, py, ax, ay, bx, by):
    dx = bx - ax
    dy = by - ay
    seg2 = dx * dx + dy * dy
    if seg2 <= 1e-9:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / seg2
    t = clamp(t, 0.0, 1.0)
    qx = ax + t * dx
    qy = ay + t * dy
    return math.hypot(px - qx, py - qy)


def polyline_distance(px, py, pts):
    best = 1e9
    for i in range(len(pts) - 1):
        ax, ay = pts[i]
        bx, by = pts[i + 1]
        d = point_segment_distance(px, py, ax, ay, bx, by)
        if d < best:
            best = d
    return best


def arc_points(cx, cy, r, a0_deg, a1_deg, n):
    pts = []
    for i in range(n + 1):
        t = i / n
        a = math.radians(lerp(a0_deg, a1_deg, t))
        pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


@dataclass
class TapeRect:
    x1: float
    y1: float
    x2: float
    y2: float

    def contains(self, x, y):
        return self.x1 <= x <= self.x2 and self.y1 <= y <= self.y2

    def distance(self, x, y):
        dx = max(self.x1 - x, 0.0, x - self.x2)
        dy = max(self.y1 - y, 0.0, y - self.y2)
        return math.hypot(dx, dy)


@dataclass
class TapePath:
    points: list
    width: float = 7.0
    color: str = "#111111"
    active: bool = True

    def distance(self, x, y):
        return polyline_distance(x, y, self.points)


@dataclass
class CircleWall:
    cx: float
    cy: float
    r: float

    def contains(self, x, y):
        return math.hypot(x - self.cx, y - self.cy) <= self.r


@dataclass
class WallRect:
    x1: float
    y1: float
    x2: float
    y2: float

    def contains(self, x, y):
        return self.x1 <= x <= self.x2 and self.y1 <= y <= self.y2


@dataclass
class SensorRead:
    one: float
    two: float
    three: float
    four: float
    five: float
    six: float


@dataclass
class Course:
    width: float = 320.0
    height: float = 220.0
    tape_paths: list = field(default_factory=list)
    tape_rects: list = field(default_factory=list)
    walls: list = field(default_factory=list)
    decor: dict = field(default_factory=dict)

    @staticmethod
    def default():
        c = Course()

        # figure-1 style layout
        home = (190.0, 35.0)
        line_pts = [home, (230, 35), (270, 35), (288, 36), (298, 41), (304, 52), (306, 68), (304, 82), (298, 92),
                   (284, 96), (258, 96), (246, 102), (241, 114), (241, 138), (246, 149), (258, 154), (288, 154)]
        wall_pts = [home, (160, 35), (148, 38), (136, 47), (124, 60), (112, 75)]
        color_pts = [home, (190, 72), (190, 112)]

        c.tape_paths.append(TapePath(line_pts, width=7.0, color="#111111", active=True))
        c.tape_paths.append(TapePath(wall_pts, width=7.0, color="#111111", active=True))
        c.tape_paths.append(TapePath(color_pts, width=6.5, color="#111111", active=True))

        # stop bars / markers from the board view
        c.tape_rects.append(TapeRect(284, 148, 294, 160))  # line end cap
        c.tape_rects.append(TapeRect(183, 108, 197, 114))  # color/home bar
        c.tape_rects.append(TapeRect(118, 55, 128, 69))    # wall branch marker

        # circle obstacle on left side
        c.walls.append(CircleWall(66, 138, 28))

        # hidden home trigger wall so the second line-follow can stop at HOME like the real board flow
        c.walls.append(WallRect(182, 52, 198, 66))

        c.decor = {
            "home": home,
            "line_end": (289, 154),
            "wall_start": (112, 75),
            "wall_start_theta": math.radians(-128),
            "wall_circle": (66, 138, 28),
            "color_left": (147, 150),
            "color_right": (213, 148),
        }
        return c

    def reflectance_value(self, x, y):
        best = 100.0
        fade = 4.5

        for rect in self.tape_rects:
            if rect.contains(x, y):
                return 900.0
            dist = rect.distance(x, y)
            val = 100.0 + 800.0 * math.exp(-((dist / fade) ** 2))
            if val > best:
                best = val

        for path in self.tape_paths:
            if not path.active:
                continue
            dist = path.distance(x, y)
            if dist <= path.width / 2.0:
                return 900.0
            edge_dist = max(0.0, dist - path.width / 2.0)
            val = 100.0 + 800.0 * math.exp(-((edge_dist / fade) ** 2))
            if val > best:
                best = val

        return best

    def nearest_wall_distance(self, ox, oy, dx, dy, max_dist=60.0):
        step = 0.4
        dist = 0.0
        while dist <= max_dist:
            px = ox + dx * dist
            py = oy + dy * dist
            if px < 0 or px > self.width or py < 0 or py > self.height:
                return dist
            for wall in self.walls:
                if wall.contains(px, py):
                    return dist
            dist += step
        return max_dist


@dataclass
class Robot:
    x: float = 190.0
    y: float = 35.0
    theta: float = 0.0
    length: float = 14.0
    width: float = 12.0
    wheelbase: float = 12.0
    left_duty: float = 0.0
    right_duty: float = 0.0

    def forward(self):
        return math.cos(self.theta), math.sin(self.theta)

    def right(self):
        return math.sin(self.theta), -math.cos(self.theta)

    def left(self):
        rx, ry = self.right()
        return -rx, -ry

    def sensor_points(self):
        fx, fy = self.forward()
        rx, ry = self.right()
        front_offset = 8.0
        laterals = [-7.5, -4.5, -1.5, 1.5, 4.5, 7.5]
        pts = []
        for lat in laterals:
            px = self.x + fx * front_offset + rx * lat
            py = self.y + fy * front_offset + ry * lat
            pts.append((px, py))
        return pts

    def ultrasonic_front_origin(self):
        fx, fy = self.forward()
        return self.x + fx * 6.0, self.y + fy * 6.0

    def ultrasonic_side_origin(self):
        rx, ry = self.right()
        return self.x + rx * 6.0, self.y + ry * 6.0

    def step(self, dt):
        vl = self.left_duty / MAX_DUTY * 18.0
        vr = self.right_duty / MAX_DUTY * 18.0
        v = 0.5 * (vl + vr)
        omega = (vr - vl) / self.wheelbase
        self.x += math.cos(self.theta) * v * dt
        self.y += math.sin(self.theta) * v * dt
        self.theta = wrap_angle(self.theta + omega * dt)


class Simulator:
    def __init__(self):
        self.course = Course.default()
        self.robot = Robot()
        self.reset()

    def reset(self):
        hx, hy = self.course.decor["home"]
        self.robot = Robot(x=hx, y=hy, theta=0.0)
        self.phase = "followLine1"
        self.phase_time = 0.0
        self.sim_time = 0.0
        self.paused = True
        self.done = False
        self.log = ["ready"]

        self.line_integral = 0.0
        self.line_prev_error = 0.0
        self.wall_integral = 0.0
        self.wall_prev_error = 0.0
        self.turn180_start_theta = self.robot.theta
        self.turn_target_theta = self.robot.theta
        self.wall_angle_start = None
        self.move_from = None
        self.move_to = None
        self.move_theta_from = 0.0
        self.move_theta_to = 0.0

        self.last_reflectance = SensorRead(100, 100, 100, 100, 100, 100)
        self.last_front_cm = 60.0
        self.last_side_cm = 60.0
        self.last_control = 0.0
        self.last_error = 0.0
        self.last_reason = ""

    def set_phase(self, phase, reason=""):
        self.phase = phase
        self.phase_time = 0.0
        self.last_reason = reason
        if reason:
            self.log.append(f"{phase}: {reason}")
        else:
            self.log.append(phase)

        if phase.startswith("followLine"):
            self.line_integral = 0.0
            self.line_prev_error = 0.0
        if phase == "turn180":
            self.turn180_start_theta = self.robot.theta
            self.turn_target_theta = wrap_angle(self.robot.theta + math.pi)
        if phase == "moveToWallStart":
            self.move_from = (self.robot.x, self.robot.y)
            self.move_to = self.course.decor["wall_start"]
            self.move_theta_from = self.robot.theta
            self.move_theta_to = self.course.decor["wall_start_theta"]
        if phase == "followWall":
            self.wall_integral = 0.0
            self.wall_prev_error = 0.0
            cx, cy, _ = self.course.decor["wall_circle"]
            self.wall_angle_start = math.atan2(self.robot.y - cy, self.robot.x - cx)

    def reflectance_read(self):
        vals = []
        for px, py in self.robot.sensor_points():
            vals.append(self.course.reflectance_value(px, py))
        self.last_reflectance = SensorRead(*vals)
        return vals

    def front_ultrasonic_cm(self):
        ox, oy = self.robot.ultrasonic_front_origin()
        dx, dy = self.robot.forward()
        self.last_front_cm = self.course.nearest_wall_distance(ox, oy, dx, dy)
        return self.last_front_cm

    def side_ultrasonic_cm(self):
        ox, oy = self.robot.ultrasonic_side_origin()
        dx, dy = self.robot.right()  # keep side sensor on robot right to match existing sim behavior
        self.last_side_cm = self.course.nearest_wall_distance(ox, oy, dx, dy)
        return self.last_side_cm

    def drive_percent(self, left_percent, right_percent):
        self.robot.left_duty = clamp(left_percent, -100, 100) / 100.0 * MAX_DUTY
        self.robot.right_duty = clamp(right_percent, -100, 100) / 100.0 * MAX_DUTY

    def stop(self):
        self.robot.left_duty = 0.0
        self.robot.right_duty = 0.0

    def all_white(self, vals):
        return all(v < WHITE_THRESH for v in vals)

    def all_black(self, vals):
        return all(v > BLACK_THRESH for v in vals)

    def near_point(self, p, r):
        return math.hypot(self.robot.x - p[0], self.robot.y - p[1]) <= r

    def follow_line_step(self, dt):
        vals = self.reflectance_read()
        front_cm = self.front_ultrasonic_cm()

        if self.phase == "followLine1" and self.near_point(self.course.decor["line_end"], 10.0):
            self.stop()
            self.set_phase("pause1", "line end")
            return

        if self.phase == "followLine2" and self.near_point(self.course.decor["home"], 10.0) and self.phase_time > 0.8:
            self.stop()
            self.set_phase("pause3", "home reached")
            return

        if self.phase == "followLine2" and front_cm <= 7.5 and self.phase_time > 0.8:
            self.stop()
            self.set_phase("pause3", f"front ultrasonic {front_cm:.1f} cm")
            return

        if self.all_black(vals):
            self.stop()
            if self.phase == "followLine1":
                self.set_phase("pause1", "all black")
            else:
                self.set_phase("pause3", "all black")
            return

        if self.all_white(vals) and self.phase_time > 0.4:
            self.stop()
            if self.phase == "followLine1":
                self.set_phase("pause1", "all white")
            else:
                self.set_phase("pause3", "all white")
            return

        calibrated = [(v - WHITE_THRESH) / (BLACK_THRESH - WHITE_THRESH) for v in vals]
        s = sum(calibrated)
        if s <= 0:
            self.stop()
            return

        error = sum(w * c for w, c in zip([-3, -2, -1, 1, 2, 3], calibrated)) / s
        self.line_integral += error * dt
        derivative = (error - self.line_prev_error) / dt
        control = 9.0 * error + 0.0 * self.line_integral + 0.55 * derivative
        self.line_prev_error = error

        left = clamp(9.0 + control, -12.0, 12.0)
        right = clamp(9.0 - control, -12.0, 12.0)
        self.drive_percent(left / 12.0 * 100.0, MOTOR_SCALE * right / 12.0 * 100.0)

        self.last_error = error
        self.last_control = control

    def turn180_step(self, dt):
        self.robot.left_duty = -8.0
        self.robot.right_duty = 8.0
        err = wrap_angle(self.turn_target_theta - self.robot.theta)
        if abs(math.degrees(err)) < 4.0 or self.phase_time > 2.6:
            self.stop()
            self.set_phase("pause2", "turn180 done")

    def move_to_wall_start_step(self, dt):
        self.stop()
        t = clamp(self.phase_time / 1.0, 0.0, 1.0)
        self.robot.x = lerp(self.move_from[0], self.move_to[0], t)
        self.robot.y = lerp(self.move_from[1], self.move_to[1], t)
        dth = wrap_angle(self.move_theta_to - self.move_theta_from)
        self.robot.theta = wrap_angle(self.move_theta_from + dth * t)
        if t >= 1.0:
            self.set_phase("wallWake", "at wall start")

    def follow_wall_step(self, dt):
        cx, cy, r = self.course.decor["wall_circle"]
        side_cm = self.side_ultrasonic_cm()
        front_cm = self.front_ultrasonic_cm()

        error = 8.0 - side_cm
        self.wall_integral += error * dt
        derivative = (error - self.wall_prev_error) / dt
        control = 16.0 * error + 0.0 * self.wall_integral + 3.5 * derivative
        control = clamp(control, -5.5, 5.5)
        self.wall_prev_error = error

        # keep moving around the circle instead of pinching into it
        if front_cm < 12.0:
            control -= 2.0

        left = clamp(8.5 - control, -11.0, 11.0)
        right = clamp(8.5 + control, -11.0, 11.0)
        self.drive_percent(left / 12.0 * 100.0, MOTOR_SCALE * right / 12.0 * 100.0)

        self.last_error = error
        self.last_control = control

        ang = math.atan2(self.robot.y - cy, self.robot.x - cx)
        swept = wrap_angle(ang - self.wall_angle_start)
        swept = swept if swept >= 0 else swept + 2.0 * math.pi
        if swept > math.radians(315):
            self.stop()
            self.set_phase("done", "wall lap complete")
            self.done = True

    def pause_step(self):
        self.stop()
        if self.phase_time >= 0.10:
            if self.phase == "pause1":
                self.set_phase("turn180")
            elif self.phase == "pause2":
                self.set_phase("lineWake2", "wake motors")
            elif self.phase == "pause3":
                self.set_phase("moveToWallStart", "switch to wall task")

    def wake_step(self):
        self.drive_percent(83.0, 83.0)
        if self.phase_time >= 0.03:
            if self.phase == "lineWake1":
                self.set_phase("followLine1")
            elif self.phase == "lineWake2":
                self.set_phase("followLine2")
            elif self.phase == "wallWake":
                self.set_phase("followWall")

    def turn_motion_step(self, dt):
        err = wrap_angle(self.turn_target_theta - self.robot.theta)
        max_turn = math.radians(160.0) * dt
        step = clamp(err, -max_turn, max_turn)
        self.robot.theta = wrap_angle(self.robot.theta + step)

    def update(self, dt=DT):
        if self.paused or self.done:
            return

        self.phase_time += dt
        self.sim_time += dt

        if self.phase == "lineWake1":
            self.wake_step()
        elif self.phase == "followLine1":
            self.follow_line_step(dt)
        elif self.phase == "pause1":
            self.pause_step()
        elif self.phase == "turn180":
            self.turn180_step(dt)
        elif self.phase == "pause2":
            self.pause_step()
        elif self.phase == "lineWake2":
            self.wake_step()
        elif self.phase == "followLine2":
            self.follow_line_step(dt)
        elif self.phase == "pause3":
            self.pause_step()
        elif self.phase == "moveToWallStart":
            self.move_to_wall_start_step(dt)
        elif self.phase == "wallWake":
            self.wake_step()
        elif self.phase == "followWall":
            self.follow_wall_step(dt)
        elif self.phase == "done":
            self.done = True
            self.stop()

        if self.phase == "turn180":
            self.turn_motion_step(dt)
        elif self.phase not in {"moveToWallStart", "done"}:
            self.robot.step(dt)

        self.robot.x = clamp(self.robot.x, 0, self.course.width)
        self.robot.y = clamp(self.robot.y, 0, self.course.height)

    def start(self):
        self.paused = False
        if self.phase == "followLine1" and self.sim_time == 0.0 and self.phase_time == 0.0:
            self.set_phase("lineWake1", "wake motors")

    def toggle_pause(self):
        self.paused = not self.paused
        if not self.paused and self.phase == "followLine1" and self.sim_time == 0.0 and self.phase_time == 0.0:
            self.set_phase("lineWake1", "wake motors")

    def status_text(self):
        vals = [
            self.last_reflectance.one,
            self.last_reflectance.two,
            self.last_reflectance.three,
            self.last_reflectance.four,
            self.last_reflectance.five,
            self.last_reflectance.six,
        ]
        return (
            f"phase: {self.phase}\n"
            f"time: {self.sim_time:5.2f} s\n"
            f"pose: ({self.robot.x:5.1f}, {self.robot.y:5.1f})  heading {math.degrees(self.robot.theta):6.1f}°\n"
            f"motor duty: L {self.robot.left_duty:5.2f}  R {self.robot.right_duty:5.2f}\n"
            f"error: {self.last_error:6.2f}  control: {self.last_control:7.2f}\n"
            f"front ultrasonic: {self.last_front_cm:5.1f} cm\n"
            f"side ultrasonic:  {self.last_side_cm:5.1f} cm\n"
            f"reflectance: {[int(v) for v in vals]}\n"
            f"last reason: {self.last_reason or '-'}"
        )


class App:
    def __init__(self):
        self.sim = Simulator()
        self.root = tk.Tk()
        self.root.title("nanobot course simulator - ece 3610 board")
        self.scale = 3.4
        self.canvas = tk.Canvas(
            self.root,
            width=int(self.sim.course.width * self.scale),
            height=int(self.sim.course.height * self.scale),
            bg="#f8f8f8",
            highlightthickness=0,
        )
        self.canvas.grid(row=0, column=0, rowspan=8, padx=12, pady=12)

        self.status = tk.Label(self.root, text="", justify="left", font=("Menlo", 11))
        self.status.grid(row=0, column=1, sticky="nw", padx=8, pady=(12, 6))

        self.log_box = tk.Text(self.root, width=40, height=14, font=("Menlo", 10))
        self.log_box.grid(row=1, column=1, sticky="nw", padx=8)
        self.log_box.insert("1.0", "ready\n")
        self.log_box.config(state="disabled")

        btns = tk.Frame(self.root)
        btns.grid(row=2, column=1, sticky="nw", padx=8, pady=8)
        tk.Button(btns, text="start / pause", command=self.toggle).grid(row=0, column=0, padx=4)
        tk.Button(btns, text="step", command=self.step_once).grid(row=0, column=1, padx=4)
        tk.Button(btns, text="reset", command=self.reset).grid(row=0, column=2, padx=4)

        self.speed = tk.DoubleVar(value=1.0)
        tk.Label(self.root, text="speed").grid(row=3, column=1, sticky="nw", padx=8)
        tk.Scale(
            self.root,
            from_=0.25,
            to=5.0,
            resolution=0.25,
            orient="horizontal",
            variable=self.speed,
            length=240,
        ).grid(row=4, column=1, sticky="nw", padx=8)

        self.draw()
        self.tick()

    def world(self, x, y):
        return x * self.scale, y * self.scale

    def reset(self):
        self.sim.reset()
        self.sync_log()
        self.draw()

    def toggle(self):
        self.sim.toggle_pause()

    def step_once(self):
        if self.sim.done:
            return
        self.sim.update(DT)
        self.sync_log()
        self.draw()

    def sync_log(self):
        self.log_box.config(state="normal")
        self.log_box.delete("1.0", "end")
        self.log_box.insert("1.0", "\n".join(self.sim.log[-16:]))
        self.log_box.config(state="disabled")

    def draw_polyline(self, pts, width, color, dash=None):
        flat = []
        for x, y in pts:
            flat.extend(self.world(x, y))
        self.canvas.create_line(*flat, fill=color, width=width * self.scale / 2, capstyle="round", joinstyle="round", dash=dash)

    def draw_course(self):
        self.canvas.delete("all")

        # active black tape
        for path in self.sim.course.tape_paths:
            self.draw_polyline(path.points, path.width, path.color)

        for rect in self.sim.course.tape_rects:
            x1, y1 = self.world(rect.x1, rect.y1)
            x2, y2 = self.world(rect.x2, rect.y2)
            self.canvas.create_rectangle(x1, y1, x2, y2, fill="#111111", outline="#111111")

        # colored pads and guide lines from the board image
        hx, hy = self.sim.course.decor["home"]
        lx, ly = self.sim.course.decor["color_left"]
        rx, ry = self.sim.course.decor["color_right"]
        self.draw_polyline([(hx, 104), (lx, ly)], 1.0, "#89a7a5", dash=(5, 6))
        self.draw_polyline([(hx, 104), (rx, ry)], 1.0, "#a89a9a", dash=(5, 6))

        for x, y, color in [(lx, ly, "#3aa8ff"), (rx, ry, "#ef4b4b")]:
            s = 9
            pts = [(x, y - s), (x + s, y), (x, y + s), (x - s, y)]
            flat = []
            for px, py in pts:
                flat.extend(self.world(px, py))
            self.canvas.create_polygon(flat, fill=color, outline="#111111", width=1)

        # circle wall obstacle on left
        for wall in self.sim.course.walls:
            if isinstance(wall, CircleWall):
                x1, y1 = self.world(wall.cx - wall.r, wall.cy - wall.r)
                x2, y2 = self.world(wall.cx + wall.r, wall.cy + wall.r)
                self.canvas.create_oval(x1, y1, x2, y2, fill="#fafafa", outline="#b5b5b5", width=1)
                self.canvas.create_arc(x1 + 8, y1 + 10, x2 - 8, y2 - 8, start=20, extent=280, style="arc", outline="#d4d4d4", width=1)

    def draw_robot(self):
        r = self.sim.robot
        fx, fy = r.forward()
        rx, ry = r.right()
        half_l = r.length / 2
        half_w = r.width / 2
        pts = [
            (r.x + fx * half_l - rx * half_w, r.y + fy * half_l - ry * half_w),
            (r.x + fx * half_l + rx * half_w, r.y + fy * half_l + ry * half_w),
            (r.x - fx * half_l + rx * half_w, r.y - fy * half_l + ry * half_w),
            (r.x - fx * half_l - rx * half_w, r.y - fy * half_l - ry * half_w),
        ]
        flat = []
        for x, y in pts:
            flat.extend(self.world(x, y))
        self.canvas.create_polygon(flat, fill="#4b89ff", outline="#1a3f9c", width=1)

        nose_x, nose_y = self.world(r.x + fx * (half_l + 3), r.y + fy * (half_l + 3))
        cx, cy = self.world(r.x, r.y)
        self.canvas.create_line(cx, cy, nose_x, nose_y, fill="#ffda44", width=2)

        vals = [
            self.sim.last_reflectance.one,
            self.sim.last_reflectance.two,
            self.sim.last_reflectance.three,
            self.sim.last_reflectance.four,
            self.sim.last_reflectance.five,
            self.sim.last_reflectance.six,
        ]
        for (px, py), val in zip(r.sensor_points(), vals):
            sx, sy = self.world(px, py)
            color = "#111111" if val > 500 else "#ffffff"
            self.canvas.create_oval(sx - 4, sy - 4, sx + 4, sy + 4, fill=color, outline="#ff4f4f")

        fx0, fy0 = r.ultrasonic_front_origin()
        sx0, sy0 = r.ultrasonic_side_origin()
        fdx, fdy = r.forward()
        rdx, rdy = r.right()
        fx1, fy1 = self.world(fx0 + fdx * self.sim.last_front_cm, fy0 + fdy * self.sim.last_front_cm)
        sx1, sy1 = self.world(sx0 + rdx * self.sim.last_side_cm, sy0 + rdy * self.sim.last_side_cm)
        fx0s, fy0s = self.world(fx0, fy0)
        sx0s, sy0s = self.world(sx0, sy0)
        self.canvas.create_line(fx0s, fy0s, fx1, fy1, fill="#ff8b3d", dash=(6, 4), width=1)
        self.canvas.create_line(sx0s, sy0s, sx1, sy1, fill="#23b26d", dash=(6, 4), width=1)

    def draw(self):
        self.draw_course()
        self.sim.reflectance_read()
        self.sim.front_ultrasonic_cm()
        self.sim.side_ultrasonic_cm()
        self.draw_robot()
        self.status.config(text=self.sim.status_text())

    def tick(self):
        steps = max(1, int(self.speed.get()))
        frac = self.speed.get() / steps
        for _ in range(steps):
            self.sim.update(DT * frac)
        self.sync_log()
        self.draw()
        self.root.after(16, self.tick)

    def run(self):
        self.root.mainloop()


def run_headless(max_time=50.0):
    sim = Simulator()
    sim.start()
    while not sim.done and sim.sim_time < max_time:
        sim.update(DT)
    print(sim.status_text())
    print("\nlog:")
    for line in sim.log:
        print(f"- {line}")
    return 0


def main():
    parser = argparse.ArgumentParser(description="nanobot course simulator")
    parser.add_argument("--headless", action="store_true", help="run without gui")
    args = parser.parse_args()

    if args.headless:
        return run_headless()

    if sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
        return run_headless()

    try:
        app = App()
        app.run()
        return 0
    except tk.TclError:
        return run_headless()


if __name__ == "__main__":
    raise SystemExit(main())
