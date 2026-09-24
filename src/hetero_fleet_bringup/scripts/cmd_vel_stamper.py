#!/usr/bin/python3
"""Bridge Nav2's Twist command to Jazzy ros2_control's TwistStamped input.

Also the last place a robot can escape a standoff, which is why it listens to
two topics rather than one. See `unstick()` below.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


def moving(twist: Twist, eps: float = 0.02) -> bool:
    return abs(twist.linear.x) > eps or abs(twist.linear.y) > eps \
        or abs(twist.angular.z) > eps


class CmdVelStamper(Node):
    def __init__(self):
        super().__init__("cmd_vel_stamper")
        robot_name = self.declare_parameter("robot_name", "").value
        controller_name = self.declare_parameter("controller_name", "").value
        if not robot_name or not controller_name:
            raise ValueError("robot_name and controller_name are required")

        # Default is the velocity smoother's output. nav2.launch.py points this
        # at <ns>/cmd_vel instead, so the collision monitor sits in the chain:
        # smoother -> cmd_vel_smoothed -> collision_monitor -> cmd_vel -> here.
        default_input = f"/{robot_name}/cmd_vel_smoothed"
        input_topic = self.declare_parameter("input_topic", default_input).value
        if not input_topic:
            input_topic = default_input
        # Jazzy's diff_drive_controller listens on <controller>/cmd_vel, but
        # mecanum_drive_controller listens on <controller>/reference - a
        # hardcoded name with no parameter to change it. Publishing to the
        # wrong one leaves the robot with zero subscribers on its velocity
        # command and it simply never moves. Default keeps the diff-drive
        # topic; nav2.launch.py passes the right one per robot type.
        default_output = f"/{robot_name}/{controller_name}/cmd_vel"
        output_topic = self.declare_parameter("output_topic", default_output).value
        if not output_topic:
            output_topic = default_output
        self.publisher = self.create_publisher(TwistStamped, output_topic, 10)
        # The simulated mecanum plant mirrors the lateral axis (commanding +y
        # moves the robot -y), so Nav2's holonomic commands have to be mirrored
        # on the way out or the controller fights its own feedback loop. The
        # diff-drive robots have no lateral axis and leave this off.
        mirror = self.declare_parameter("mirror_y", False).value
        if isinstance(mirror, str):
            self.mirror_y = mirror.strip().lower() in ("1", "true", "yes", "on")
        else:
            self.mirror_y = bool(mirror)
        self.create_subscription(Twist, input_topic, self.on_command, 10)

        # --- standoff escape valve -----------------------------------------
        # The collision monitor is a pure position filter: while a teammate
        # sits inside the stop polygon EVERY command is zeroed, including the
        # reverse that would open the gap. A robot that ends up inside that
        # polygon therefore cannot move again in any direction, and since the
        # polygon travels with the robot, nothing on the Nav2 side can change
        # that. Measured on a live run: three robots mutually blocked in the
        # east connector, positions identical to the millimetre for over 60 s,
        # fleet_manager's own back-off goals producing no motion at all.
        #
        # Worse, the monitor does not keep saying "stop": with
        # stop_pub_timeout 2.0 s it zeroes cmd_vel for two seconds and then
        # goes SILENT, and the controller's own cmd_vel_timeout then freezes
        # the wheels. At that point nothing at all is arriving here, so a valve
        # that only reacts to "monitor keeps publishing zero" never fires - as
        # measured, six firings in a run that needed it constantly.
        #
        # So this watches for silence as well: if the request upstream
        # (cmd_vel_smoothed, i.e. what the controller wants) has been asking to
        # move for `unstick_after_sec` while the monitor has said nothing or
        # nothing but zeros, creep at `unstick_ratio` of the request instead of
        # sitting frozen. Deliberately narrow: only after a continuous
        # override, capped at the same proportion the monitor's own slowdown
        # uses, and it stands down the instant the monitor lets a real command
        # through.
        self.upstream = None
        self.last_command = None
        self.overridden_since = None
        self.unstick_after_sec = float(
            self.declare_parameter("unstick_after_sec", 3.0).value)
        self.unstick_ratio = float(
            self.declare_parameter("unstick_ratio", 0.4).value)
        self.create_subscription(
            Twist, f"/{robot_name}/cmd_vel_smoothed", self.on_request, 10)
        self.create_timer(0.1, self.check_override)
        self.get_logger().info(
            f"stamping {input_topic} -> {output_topic} "
            f"(standoff escape after {self.unstick_after_sec:.0f}s)"
        )

    def on_request(self, twist: Twist) -> None:
        """What Nav2's controller asked for, before the collision monitor."""
        self.upstream = twist

    def _seconds(self, since) -> float:
        return (self.get_clock().now() - since).nanoseconds * 1e-9

    def _publish(self, command: Twist, now) -> None:
        stamped = TwistStamped()
        stamped.header.stamp = now.to_msg()
        stamped.twist = command
        if self.mirror_y:
            stamped.twist.linear.y = -command.linear.y
        self.publisher.publish(stamped)

    def check_override(self) -> None:
        """Creep while the safety layer has been overriding us for too long."""
        now = self.get_clock().now()
        if self.upstream is None or not moving(self.upstream):
            self.overridden_since = None
            return
        if self.last_command is not None and moving(self.last_command):
            self.overridden_since = None
            return
        if self.overridden_since is None:
            self.overridden_since = now
            return
        held = self._seconds(self.overridden_since)
        if held < self.unstick_after_sec:
            return
        creep = Twist()
        creep.linear.x = self.upstream.linear.x * self.unstick_ratio
        creep.linear.y = self.upstream.linear.y * self.unstick_ratio
        creep.angular.z = self.upstream.angular.z * self.unstick_ratio
        self._publish(creep, now)
        self.get_logger().warn(
            f"held by the collision monitor for {held:.0f}s - creeping at "
            f"{self.unstick_ratio:.0%} to break the standoff",
            throttle_duration_sec=5.0,
        )

    def on_command(self, command: Twist) -> None:
        now = self.get_clock().now()
        self.last_command = command
        if moving(command):
            self.overridden_since = None
        self._publish(command, now)


def main():
    rclpy.init()
    node = CmdVelStamper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
