#!/usr/bin/env python3
"""Pure pursuit path follower for the AgileX Scout Mini.

Input : a path (nav_msgs/Path, frame "odom") + odometry (/odom)
Output: velocity commands v [m/s] and omega [rad/s] on /cmd_vel

Run ON THE ROBOT while `scoutlaunch` is running in another terminal.

Test mode (built-in path, relative to where the robot stands at start):
    python3 pure_pursuit.py --test line
    python3 pure_pursuit.py --test circle
    python3 pure_pursuit.py --test scurve
    python3 pure_pursuit.py --test sharp_right

Path mode (wait for paths, e.g. from flow matching, on /fm_path):
    python3 pure_pursuit.py
  Every new path replaces the current one. The robot stops at the end of
  each path and waits for the next one.

Optional: --log run.csv writes pose, command and tracking error per step.
The path being followed is republished on /pp_path for RViz.
Ctrl+C always stops the robot.
"""

import argparse
import csv
import math
import time

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry, Path
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from rclpy.signals import SignalHandlerOptions
except ImportError:
    # No ROS 2 here (e.g. on a Windows PC). The geometry helpers and
    # PurePursuitCore still work, so plot_paths.py can import this file.
    rclpy = None
    Node = object
    PoseStamped = Twist = Odometry = Path = None
    DurabilityPolicy = QoSProfile = SignalHandlerOptions = None


# --------------------------------------------------------------------------
# Geometry helpers (pure Python, no ROS) - easy to unit test / reuse
# --------------------------------------------------------------------------

def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def resample(points, spacing):
    """Return points spaced `spacing` metres apart along the polyline."""
    if len(points) < 2:
        return list(points)
    out = [points[0]]
    carry = 0.0  # distance already travelled since last output point
    for (x0, y0), (x1, y1) in zip(points[:-1], points[1:]):
        seg = math.hypot(x1 - x0, y1 - y0)
        if seg < 1e-9:
            continue
        d = spacing - carry
        while d <= seg:
            t = d / seg
            out.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
            d += spacing
        carry = seg - (d - spacing)
    if math.hypot(out[-1][0] - points[-1][0], out[-1][1] - points[-1][1]) > 1e-6:
        out.append(points[-1])
    return out


TEST_PATHS = ["line", "circle", "scurve", "sharp_right"]


def make_test_path(kind):
    """Test paths in the ROBOT frame: x forward, y left, start at (0, 0)."""
    if kind == "line":
        return [(0.0, 0.0), (2.0, 0.0)]
    if kind == "circle":  # full left circle, radius 0.75 m
        r = 0.75
        return [(r * math.sin(t), r * (1 - math.cos(t)))
                for t in (2 * math.pi * i / 200 for i in range(201))]
    if kind == "scurve":  # 2.5 m forward, 0.8 m to the left, zero slope at both ends
        length, amp = 2.5, 0.4
        return [(x, amp * (1 - math.cos(math.pi * x / length)))
                for x in (length * i / 100 for i in range(101))]
    if kind == "sharp_right":  # 1.5 m straight, then a 90 degree corner, 1.0 m to the right
        return [(0.0, 0.0), (1.5, 0.0), (1.5, -1.0)]
    raise ValueError(kind)


def to_world(points, x0, y0, yaw0):
    c, s = math.cos(yaw0), math.sin(yaw0)
    return [(x0 + c * px - s * py, y0 + s * px + c * py) for px, py in points]


class PurePursuitCore:
    """The control law, independent of ROS. compute() -> (v, w, info)."""

    def __init__(self, v_max=0.3, v_min=0.05, w_max=1.0, lookahead=0.4,
                 r_min=0.5, slow_dist=0.5, goal_tol=0.1, rotate_thresh=1.0,
                 spacing=0.05):
        self.v_max, self.v_min, self.w_max = v_max, v_min, w_max
        self.lookahead, self.r_min, self.slow_dist = lookahead, r_min, slow_dist
        self.goal_tol, self.rotate_thresh, self.spacing = goal_tol, rotate_thresh, spacing
        self.path = []
        self.idx = 0

    def set_path(self, points):
        self.path = resample(points, self.spacing)
        self.idx = 0

    def compute(self, x, y, yaw):
        path, n = self.path, len(self.path)

        # 1) Closest point, searched only forward in a ~1 m window so the
        #    robot never jumps to a later/earlier part of a crossing path.
        window = max(5, int(1.0 / self.spacing))
        best, best_d = self.idx, float("inf")
        for i in range(self.idx, min(n, self.idx + window)):
            d = math.hypot(path[i][0] - x, path[i][1] - y)
            if d < best_d:
                best, best_d = i, d
        self.idx = best
        cross_track = best_d

        # 2) Goal check: near the end of the path and close to / past the goal.
        gx, gy = path[-1]
        goal_dist = math.hypot(gx - x, gy - y)
        remaining = (n - 1 - self.idx) * self.spacing
        goal_xr = math.cos(yaw) * (gx - x) + math.sin(yaw) * (gy - y)
        if remaining < self.lookahead and (goal_dist < self.goal_tol or goal_xr < 0.0):
            return 0.0, 0.0, {"done": True, "cte": cross_track, "look": (gx, gy)}

        # 3) Lookahead point: first point at least `lookahead` away.
        j = self.idx
        while j < n - 1 and math.hypot(path[j][0] - x, path[j][1] - y) < self.lookahead:
            j += 1
        lx, ly = path[j]

        # 4) Lookahead point in robot frame.
        dx, dy = lx - x, ly - y
        xr = math.cos(yaw) * dx + math.sin(yaw) * dy
        yr = -math.sin(yaw) * dx + math.cos(yaw) * dy
        heading_err = math.atan2(yr, xr)

        if abs(heading_err) > self.rotate_thresh:
            # Target is far to the side / behind: turn on the spot first.
            v = 0.0
            w = math.copysign(min(self.w_max, max(0.3, 1.5 * abs(heading_err))), heading_err)
        else:
            ld2 = xr * xr + yr * yr
            curvature = 2.0 * yr / ld2 if ld2 > 1e-9 else 0.0
            v = self.v_max
            # Regulation: slow down in tight curves ...
            if abs(curvature) > 1.0 / self.r_min:
                v *= (1.0 / abs(curvature)) / self.r_min
            # ... and when approaching the goal.
            v = min(v, self.v_max * goal_dist / self.slow_dist)
            v = max(v, self.v_min)
            w = v * curvature
            if abs(w) > self.w_max:  # keep curvature, scale both down
                scale = self.w_max / abs(w)
                v, w = v * scale, w * scale

        return v, w, {"done": False, "cte": cross_track, "look": (lx, ly)}


# --------------------------------------------------------------------------
# ROS 2 node
# --------------------------------------------------------------------------

class PurePursuitNode(Node):
    def __init__(self, args):
        super().__init__("pure_pursuit")
        self.args = args
        self.core = PurePursuitCore(
            v_max=args.v_max, v_min=args.v_min, w_max=args.w_max,
            lookahead=args.lookahead, r_min=args.r_min, slow_dist=args.slow_dist,
            goal_tol=args.goal_tol, spacing=args.spacing)

        self.cmd_pub = self.create_publisher(Twist, args.cmd_topic, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.viz_pub = self.create_publisher(Path, "/pp_path", latched)
        self.create_subscription(Odometry, args.odom_topic, self.on_odom, 10)
        self.create_subscription(Path, args.path_topic, self.on_path, 10)
        self.create_timer(1.0 / args.rate, self.step)

        self.pose = None
        self.last_odom = 0.0
        self.active = False
        self.finished = False  # test mode: exit after the goal
        self.t0 = time.time()

        self.log_file = None
        if args.log:
            self.log_file = open(args.log, "w", newline="")
            self.log = csv.writer(self.log_file)
            self.log.writerow(["t", "x", "y", "yaw", "v", "w", "look_x", "look_y", "cte", "idx"])

        if args.test:
            self.get_logger().info(f"Test mode '{args.test}': waiting for odometry ...")
        else:
            self.get_logger().info(f"Waiting for a nav_msgs/Path on {args.path_topic} ...")

    # ---- callbacks -------------------------------------------------------

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        self.last_odom = time.time()
        if self.args.test and not self.active and not self.finished:
            local = make_test_path(self.args.test)
            self.start_path(to_world(local, *self.pose), "odom")

    def on_path(self, msg: Path):
        if self.args.test:
            return
        frame = msg.header.frame_id
        if frame and frame != "odom":
            self.get_logger().warn(
                f"Path frame is '{frame}', but this controller works in 'odom'. "
                "Points are used as-is.")
        pts = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        if not pts:
            self.get_logger().warn("Empty path received - stopping.")
            self.active = False
            self.send(0.0, 0.0)
            return
        self.start_path(pts, frame or "odom")

    def start_path(self, pts, frame):
        self.core.set_path(pts)
        self.active = True
        viz = Path()
        viz.header.frame_id = frame
        viz.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.core.path:
            ps = PoseStamped()
            ps.header = viz.header
            ps.pose.position.x, ps.pose.position.y = x, y
            ps.pose.orientation.w = 1.0
            viz.poses.append(ps)
        self.viz_pub.publish(viz)
        self.get_logger().info(f"Following new path with {len(self.core.path)} points.")

    # ---- control loop ----------------------------------------------------

    def step(self):
        if not self.active:
            return
        if self.pose is None or time.time() - self.last_odom > self.args.odom_timeout:
            self.send(0.0, 0.0)
            self.get_logger().warn("No fresh odometry - holding still.",
                                   throttle_duration_sec=2.0)
            return

        x, y, yaw = self.pose
        v, w, info = self.core.compute(x, y, yaw)
        self.send(v, w)

        if self.log_file:
            self.log.writerow([f"{time.time() - self.t0:.3f}", f"{x:.4f}", f"{y:.4f}",
                               f"{yaw:.4f}", f"{v:.4f}", f"{w:.4f}",
                               f"{info['look'][0]:.4f}", f"{info['look'][1]:.4f}",
                               f"{info['cte']:.4f}", self.core.idx])

        if info["done"]:
            self.active = False
            self.stop()
            self.get_logger().info("Goal reached.")
            if self.args.test:
                self.finished = True

    def send(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def stop(self):
        for _ in range(5):
            self.send(0.0, 0.0)
            time.sleep(0.02)

    def close(self):
        if self.log_file:
            self.log_file.close()


def main():
    p = argparse.ArgumentParser(description="Pure pursuit path follower for the Scout Mini.")
    p.add_argument("--test", choices=TEST_PATHS,
                   help="drive a built-in test path instead of waiting for one")
    p.add_argument("--v-max", type=float, default=0.3, help="max forward speed [m/s]")
    p.add_argument("--v-min", type=float, default=0.05, help="min forward speed while driving [m/s]")
    p.add_argument("--w-max", type=float, default=1.0, help="max turn speed [rad/s]")
    p.add_argument("--lookahead", type=float, default=0.4, help="lookahead distance [m]")
    p.add_argument("--r-min", type=float, default=0.5,
                   help="slow down when the curve radius is below this [m]")
    p.add_argument("--slow-dist", type=float, default=0.5,
                   help="start slowing down this far from the goal [m]")
    p.add_argument("--goal-tol", type=float, default=0.1, help="goal tolerance [m]")
    p.add_argument("--spacing", type=float, default=0.05, help="path resampling distance [m]")
    p.add_argument("--rate", type=float, default=20.0, help="control rate [Hz]")
    p.add_argument("--odom-timeout", type=float, default=0.5, help="stop if odom is older [s]")
    p.add_argument("--cmd-topic", default="/cmd_vel")
    p.add_argument("--odom-topic", default="/odom")
    p.add_argument("--path-topic", default="/fm_path")
    p.add_argument("--log", help="write a CSV log to this file")
    args = p.parse_args()

    if rclpy is None:
        raise SystemExit("ROS 2 (rclpy) not found. Run this on the robot, "
                         "or source /opt/ros/<distro>/setup.bash first.")

    # Own SIGINT handling so we can still publish a stop command on Ctrl+C.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = PurePursuitNode(args)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        node.stop()
        print("Robot stopped.")
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
