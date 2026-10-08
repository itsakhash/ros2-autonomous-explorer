"""Autonomous frontier exploration on top of SLAM Toolbox and Nav2.

Repeatedly: read the SLAM map, find the nearest frontier (edge of explored space),
send it to Nav2 as a NavigateToPose goal, wait for the result. Goals that are rejected,
fail, or time out are blacklisted. After each goal the node waits briefly so SLAM can
publish its map update; a frontier that is still next to a place the robot has already
reached is stale and is blacklisted instead of revisited. When no usable frontier remains
for several consecutive checks, exploration is finished: a summary is logged, the map is
optionally saved, and the node exits.

At startup the robot can be boxed in by obstacles, so its first map is tiny and may have
no usable frontier. A 360-degree lidar learns nothing by turning in place, so in that case
the robot drives a short way to a known-free cell (a "seed" move) to widen its view.
"""

import math
import subprocess

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener

from autonomous_explorer.frontier import choose_frontier, find_frontiers


def pick_seed_cell(
    grid, robot_cell, free_max, min_dist, max_dist, blacklist=(), blacklist_radius=0.0
):
    """Return a known-free (x, y) cell a short drive from the robot, or None.

    Distances are in cells. Picks the farthest free cell within max_dist (and at
    least min_dist away), skipping cells near blacklisted ones.
    """
    ys, xs = np.nonzero((grid >= 0) & (grid <= free_max))
    if xs.size == 0:
        return None
    dist = np.hypot(xs - robot_cell[0], ys - robot_cell[1])
    ok = (dist >= min_dist) & (dist <= max_dist)
    for bx, by in blacklist:
        ok &= np.hypot(xs - bx, ys - by) > blacklist_radius
    if not ok.any():
        return None
    best = int(np.argmax(np.where(ok, dist, -1.0)))
    return int(xs[best]), int(ys[best])


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")
        self.declare_parameter("free_max", 25)
        self.declare_parameter("min_frontier_size", 8)  # cells
        self.declare_parameter("min_goal_distance", 0.4)  # metres from the robot
        self.declare_parameter("blacklist_radius", 0.5)  # metres around failed goals
        self.declare_parameter("revisit_radius", 0.4)  # metres around reached goals
        self.declare_parameter("settle_time", 6.0)  # s to wait for SLAM after a goal
        self.declare_parameter("goal_timeout", 120.0)  # seconds
        self.declare_parameter("idle_ticks_to_finish", 3)  # checks with no frontier
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("save_map_prefix", "")  # e.g. /home/me/maps/explored
        self.declare_parameter(
            "min_known_area", 10.0
        )  # m^2, below this the map is "tiny"
        self.declare_parameter("max_seed_goals", 4)  # startup moves allowed
        self.declare_parameter("seed_min_dist", 0.5)  # metres
        self.declare_parameter("seed_max_dist", 1.0)  # metres

        self.map = None
        self.busy = False
        self.cancelling = False
        self.goal_handle = None
        self.goal_sent_at = None
        self.goal_cell = None
        self.settle_until = 0.0
        self.blacklist = []
        self.reached = []
        self.idle_ticks = 0
        self.seed_goals = 0
        self.goals_sent = 0
        self.goals_succeeded = 0
        self.goals_failed = 0
        self.goals_skipped = 0
        self.finished = False
        self.start_time = None

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "map", self.on_map, map_qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.create_timer(1.0, self.tick)
        self.get_logger().info("frontier_explorer started, waiting for map and Nav2")

    # ----------------------------------------------------------------- helpers
    def param(self, name):
        return self.get_parameter(name).value

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_map(self, msg):
        self.map = msg

    def robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", self.param("base_frame"), rclpy.time.Time()
            )
        except TransformException:
            return None
        return tf.transform.translation.x, tf.transform.translation.y

    def is_revisit(self, cell, res):
        """Return True if this frontier is next to a place the robot already reached."""
        limit = self.param("revisit_radius")
        return any(
            math.hypot(cell[0] - c[0], cell[1] - c[1]) * res < limit
            for c in self.reached
        )

    # -------------------------------------------------------------- main loop
    def tick(self):
        if self.finished:
            return
        if self.busy:
            self.check_timeout()
            return
        if self.now_s() < self.settle_until:
            return  # give SLAM time to publish the map update from the last goal
        if self.map is None:
            self.get_logger().info("waiting for /map ...", throttle_duration_sec=5.0)
            return
        if not self.nav.server_is_ready():
            self.get_logger().info("waiting for Nav2 ...", throttle_duration_sec=5.0)
            return
        xy = self.robot_xy()
        if xy is None:
            self.get_logger().info(
                "waiting for map->base_link transform ...", throttle_duration_sec=5.0
            )
            return
        if self.start_time is None:
            self.start_time = self.now_s()

        info = self.map.info
        res = info.resolution
        grid = np.asarray(self.map.data, dtype=np.int16).reshape(
            info.height, info.width
        )
        robot_cell = (
            int((xy[0] - info.origin.position.x) / res),
            int((xy[1] - info.origin.position.y) / res),
        )

        clusters = find_frontiers(
            grid, self.param("free_max"), self.param("min_frontier_size")
        )
        target = choose_frontier(
            clusters,
            robot_cell,
            self.blacklist,
            blacklist_radius=self.param("blacklist_radius") / res,
            min_distance=self.param("min_goal_distance") / res,
        )

        known_area = float(np.count_nonzero(grid >= 0)) * res * res
        self.get_logger().info(
            f"map {info.width}x{info.height}, known {known_area:.1f} m^2, "
            f"{len(clusters)} frontier clusters, "
            f'{"target found" if target is not None else "no usable target"}',
            throttle_duration_sec=3.0,
        )

        if target is not None and self.is_revisit(target, res):
            # Stale frontier: we already stood here and it did not go away.
            self.goals_skipped += 1
            self.blacklist.append(target)
            self.get_logger().info(
                f"skipping frontier at cell {target}: already reached, no new map there"
            )
            return

        if target is None:
            # A tiny map with no usable frontier means the robot is boxed in at the
            # start. Move to a nearby free cell so the lidar sees more.
            if known_area < self.param(
                "min_known_area"
            ) and self.seed_goals < self.param("max_seed_goals"):
                seed = pick_seed_cell(
                    grid,
                    robot_cell,
                    self.param("free_max"),
                    self.param("seed_min_dist") / res,
                    self.param("seed_max_dist") / res,
                    self.blacklist,
                    self.param("blacklist_radius") / res,
                )
                if seed is not None:
                    self.seed_goals += 1
                    self.idle_ticks = 0
                    self.get_logger().info(
                        "map is tiny with no usable frontier: "
                        f"seed move {self.seed_goals} to a nearby free cell"
                    )
                    self.send_goal(seed, xy, info)
                    return
            self.idle_ticks += 1
            if self.idle_ticks >= self.param("idle_ticks_to_finish"):
                self.finish(grid, res)
            return

        self.idle_ticks = 0
        self.send_goal(target, xy, info)

    def send_goal(self, cell, robot_xy, info):
        res = info.resolution
        wx = info.origin.position.x + (cell[0] + 0.5) * res
        wy = info.origin.position.y + (cell[1] + 0.5) * res
        yaw = math.atan2(wy - robot_xy[1], wx - robot_xy[0])

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = wx
        goal.pose.pose.position.y = wy
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.goals_sent += 1
        self.busy = True
        self.cancelling = False
        self.goal_cell = cell
        self.goal_sent_at = self.now_s()
        self.get_logger().info(
            f"goal {self.goals_sent}: target at ({wx:.2f}, {wy:.2f}) m"
        )
        self.nav.send_goal_async(goal).add_done_callback(self.on_goal_response)

    # --------------------------------------------------------------- callbacks
    def on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("goal rejected by Nav2, blacklisting it")
            self.end_goal(success=False)
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("goal reached")
            self.end_goal(success=True)
        else:
            self.get_logger().warn(f"goal failed (status {status}), blacklisting it")
            self.end_goal(success=False)

    def end_goal(self, success):
        if success:
            self.goals_succeeded += 1
            self.reached.append(self.goal_cell)
        else:
            self.goals_failed += 1
            self.blacklist.append(self.goal_cell)
        self.goal_handle = None
        self.busy = False
        self.settle_until = self.now_s() + float(self.param("settle_time"))

    def check_timeout(self):
        if self.goal_handle is None or self.cancelling:
            return
        if self.now_s() - self.goal_sent_at > self.param("goal_timeout"):
            self.get_logger().warn("goal timed out, cancelling it")
            self.cancelling = True
            self.goal_handle.cancel_goal_async()

    # ------------------------------------------------------------------ finish
    def finish(self, grid, res):
        elapsed = self.now_s() - self.start_time
        known_area = float(np.count_nonzero(grid >= 0)) * res * res
        self.get_logger().info(
            "EXPLORATION COMPLETE: "
            f"{elapsed:.0f} s (simulation time), "
            f"{self.goals_sent} goals ({self.goals_succeeded} reached, "
            f"{self.goals_failed} failed), {self.goals_skipped} stale frontiers skipped, "
            f"{self.seed_goals} startup seed moves, mapped area {known_area:.1f} m^2"
        )
        prefix = self.param("save_map_prefix")
        if prefix:
            subprocess.run(
                [
                    "ros2",
                    "run",
                    "nav2_map_server",
                    "map_saver_cli",
                    "-f",
                    prefix,
                    "--ros-args",
                    "-p",
                    "use_sim_time:=true",
                ],
                check=False,
                timeout=60,
            )
        self.finished = True


def main():
    rclpy.init()
    node = FrontierExplorer()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
