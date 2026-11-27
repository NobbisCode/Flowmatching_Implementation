#!/usr/bin/env python3
"""Control the AgileX Scout Mini via /cmd_vel (geometry_msgs/Twist).

Run this ON THE ROBOT (or any machine that sees the robot's ROS 2 graph)
while `scoutlaunch` is running in another terminal.

Two modes:

  1) Keyboard control (default):
        python3 scout_control.py

        w / s   : increase / decrease forward speed
        a / d   : increase / decrease turn speed (a = left, d = right)
        space   : stop (set both speeds to 0)
        q       : quit (the robot is stopped on exit)

  2) Fixed command for a given time, then stop:
        python3 scout_control.py --linear 0.2 --angular 0.3 --duration 3

The script publishes continuously at --rate Hz (the base stops when commands
stop arriving) and always sends a zero command when it exits, including on
Ctrl+C.
"""

import argparse
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.signals import SignalHandlerOptions


class ScoutController:
    def __init__(self, topic: str, max_linear: float, max_angular: float):
        self.node = rclpy.create_node("scout_control")
        self.pub = self.node.create_publisher(Twist, topic, 10)
        self.max_linear = max_linear
        self.max_angular = max_angular

    def send(self, linear: float, angular: float) -> tuple[float, float]:
        """Publish a velocity command, clamped to the configured limits."""
        linear = max(-self.max_linear, min(self.max_linear, linear))
        angular = max(-self.max_angular, min(self.max_angular, angular))
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.pub.publish(msg)
        return linear, angular

    def stop(self, repeats: int = 5) -> None:
        """Send zero velocity a few times so it is not lost."""
        for _ in range(repeats):
            self.send(0.0, 0.0)
            time.sleep(0.02)

    def wait_for_subscriber(self, timeout: float = 3.0) -> None:
        start = time.time()
        while self.pub.get_subscription_count() == 0 and time.time() - start < timeout:
            time.sleep(0.1)
        if self.pub.get_subscription_count() == 0:
            print(
                f"WARNING: nobody is subscribed to {self.pub.topic_name}. "
                "Is scoutlaunch running?"
            )

    def destroy(self) -> None:
        self.node.destroy_node()


def run_timed(ctrl: ScoutController, linear: float, angular: float,
              duration: float, rate: float) -> None:
    period = 1.0 / rate
    end = time.time() + duration
    lin, ang = ctrl.send(linear, angular)
    print(f"Driving: linear={lin:.2f} m/s, angular={ang:.2f} rad/s for {duration:.1f} s")
    while time.time() < end:
        ctrl.send(linear, angular)
        time.sleep(period)
    print("Done.")


def run_keyboard(ctrl: ScoutController, lin_step: float, ang_step: float,
                 rate: float) -> None:
    if not sys.stdin.isatty():
        raise SystemExit("Keyboard mode needs an interactive terminal.")

    print(__doc__.split("Two modes:")[1].split("2)")[0].strip())
    print()

    period = 1.0 / rate
    linear, angular = 0.0, 0.0
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())  # single key presses, Ctrl+C still works
        while True:
            ready, _, _ = select.select([sys.stdin], [], [], period)
            if ready:
                key = sys.stdin.read(1).lower()
                if key == "w":
                    linear += lin_step
                elif key == "s":
                    linear -= lin_step
                elif key == "a":
                    angular += ang_step
                elif key == "d":
                    angular -= ang_step
                elif key == " ":
                    linear, angular = 0.0, 0.0
                elif key == "q":
                    break
            linear, angular = ctrl.send(linear, angular)
            # Snap tiny float drift to zero for display
            linear = round(linear, 3)
            angular = round(angular, 3)
            sys.stdout.write(
                f"\rlinear: {linear:+.2f} m/s   angular: {angular:+.2f} rad/s   "
            )
            sys.stdout.flush()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print()


def main() -> None:
    p = argparse.ArgumentParser(description="Control the Scout Mini via /cmd_vel.")
    p.add_argument("--linear", type=float, help="forward speed in m/s (timed mode)")
    p.add_argument("--angular", type=float, default=0.0,
                   help="turn speed in rad/s, positive = left (timed mode)")
    p.add_argument("--duration", type=float, default=2.0,
                   help="seconds to drive in timed mode (default 2)")
    p.add_argument("--max-linear", type=float, default=0.5, help="speed limit in m/s")
    p.add_argument("--max-angular", type=float, default=1.0, help="turn limit in rad/s")
    p.add_argument("--linear-step", type=float, default=0.05, help="keyboard step in m/s")
    p.add_argument("--angular-step", type=float, default=0.1, help="keyboard step in rad/s")
    p.add_argument("--rate", type=float, default=20.0, help="publish rate in Hz")
    p.add_argument("--topic", default="/cmd_vel")
    args = p.parse_args()

    # Disable rclpy's own SIGINT handling so we can still publish a stop on Ctrl+C.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    ctrl = ScoutController(args.topic, args.max_linear, args.max_angular)
    try:
        ctrl.wait_for_subscriber()
        if args.linear is not None or args.angular != 0.0:
            run_timed(ctrl, args.linear or 0.0, args.angular, args.duration, args.rate)
        else:
            run_keyboard(ctrl, args.linear_step, args.angular_step, args.rate)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        ctrl.stop()
        print("Robot stopped.")
        ctrl.destroy()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
