#!/usr/bin/env python3
"""
Frontier-based autonomous exploration.

All tuning values are ROS 2 parameters — override at launch:
  ros2 run diff_drive_robot frontier_explorer.py --ros-args \
      -p min_frontier_size:=15 -p revisit_radius:=1.0

Run alongside slam.launch.py (mapping mode):
  ros2 launch diff_drive_robot slam.launch.py
  ros2 run diff_drive_robot frontier_explorer.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import tf2_ros

from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus

import numpy as np
import math
from collections import deque
import subprocess
import os


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        self.declare_parameter('min_frontier_size', 5)
        self.declare_parameter('revisit_radius',    0.3)
        self.declare_parameter('poll_period',       1.5)
        self.declare_parameter('map_topic',         '/map')
        self.declare_parameter('action_name',       'navigate_to_pose')
        self.declare_parameter('goal_frame',        'map')
        self.declare_parameter('base_frame',        'base_link')
        self.declare_parameter('min_goal_distance', 0.35)
        self.declare_parameter('map_save_path',     '')

        self._min_size      = self.get_parameter('min_frontier_size').value
        self._revisit_r     = self.get_parameter('revisit_radius').value
        self._goal_frame    = self.get_parameter('goal_frame').value
        self._base_frame    = self.get_parameter('base_frame').value
        self._min_goal_dist = self.get_parameter('min_goal_distance').value
        self._map_save_path = self.get_parameter('map_save_path').value.strip()
        map_topic           = self.get_parameter('map_topic').value
        action_name         = self.get_parameter('action_name').value
        poll_period         = self.get_parameter('poll_period').value

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._map: OccupancyGrid | None = None
        self._navigating = False
        self._visited: list[tuple[float, float]] = []
        self._iteration = 0
        self._map_saved = False

        self._nav_client = ActionClient(self, NavigateToPose, action_name)
        self._map_sub = self.create_subscription(
            OccupancyGrid, map_topic, self._map_callback, 1)

        self.get_logger().info('Waiting for Nav2 action server...')
        self._nav_client.wait_for_server()
        self.get_logger().info('Ready. Waiting for map...')

        self.create_timer(poll_period, self._explore)

    # ------------------------------------------------------------------
    # Callbacks — do not modify
    # ------------------------------------------------------------------
    def _map_callback(self, msg: OccupancyGrid):
        self._map = msg

    def _already_visited(self, fx, fy):
        return any(
            math.hypot(fx - vx, fy - vy) < self._revisit_r
            for vx, vy in self._visited)

    def _robot_position(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                self._goal_frame, self._base_frame, rclpy.time.Time())
            return (
                tf.transform.translation.x,
                tf.transform.translation.y,
            )
        except Exception:
            self.get_logger().debug(
                f'Waiting for TF {self._goal_frame} -> {self._base_frame}')
            return None

    def _send_goal(self, x: float, y: float):
        self._navigating = True
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self._goal_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.w = 1.0
        future = self._nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Goal rejected. Trying next frontier.')
            self._navigating = False
            return
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Frontier reached. Searching for next...')
        else:
            self.get_logger().warn(f'Navigation failed (status={status}).')
        self._navigating = False

    def _save_map_once(self):
        if self._map_saved or not self._map_save_path:
            return
        save_prefix = os.path.expanduser(self._map_save_path)
        save_dir = os.path.dirname(save_prefix)
        if save_dir:
            os.makedirs(save_dir, exist_ok=True)
        self.get_logger().info(f'Auto-saving map to: {save_prefix}')
        try:
            subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', save_prefix],
                check=True)
            self._map_saved = True
            self.get_logger().info('Map saved successfully.')
        except Exception as exc:
            self.get_logger().error(f'Failed to save map: {exc}')

    def _finish_exploration(self):
        if self._map_save_path:
            self.get_logger().info(f'Final map save to {self._map_save_path} ...')
            try:
                subprocess.run(
                    ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', self._map_save_path],
                    check=True)
                self.get_logger().info('Map saved successfully.')
            except subprocess.CalledProcessError as e:
                self.get_logger().error(f'Failed to save map: {e}')
        self.get_logger().info('Shutting down explorer.')
        raise SystemExit(0)

    # ------------------------------------------------------------------
    # Main exploration loop — do not modify
    # ------------------------------------------------------------------
    def _explore(self):
        if self._map is None or self._navigating:
            return

        frontiers = self._find_frontiers()
        if not frontiers:
            self.get_logger().info('No frontiers — exploration complete.')
            self._save_map_once()
            return

        goal = self._best_frontier(frontiers)
        if goal is None:
            self.get_logger().info('All frontiers already visited. Exploration complete!')
            self._finish_exploration()
            return

        self._iteration += 1
        if self._map_save_path and self._iteration % 10 == 0:
            subprocess.Popen(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', self._map_save_path],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        self.get_logger().info(
            f'Navigating to frontier ({goal[0]:.2f}, {goal[1]:.2f})')
        self._visited.append(goal)
        self._send_goal(*goal)

    # ------------------------------------------------------------------
    # TODO 1 — Frontier detection
    # ------------------------------------------------------------------
    def _find_frontiers(self) -> list[tuple[float, float]]:
        """
        Read self._map (an OccupancyGrid) and return a list of (x, y)
        world-coordinate centroids, one per frontier cluster.

        A frontier cell is a free cell (value 0) that is directly adjacent
        (4-connected) to at least one unknown cell (value -1).

        Steps:
          1. Reshape msg.data into a 2-D numpy array.
          2. Build a boolean mask of frontier cells.
          3. BFS-cluster the frontier cells. Discard clusters smaller
             than self._min_size.
          4. Convert each surviving cluster's mean grid position to world
             coordinates using msg.info.resolution and msg.info.origin.
          5. Return the list of (x, y) centroids.
        """
        raise NotImplementedError

    # ------------------------------------------------------------------
    # TODO 2 — Frontier selection
    # ------------------------------------------------------------------
    def _best_frontier(self, frontiers):
        """
        Given a list of (x, y) frontier centroids, return the one closest
        to the robot that has not already been visited and is at least
        self._min_goal_dist away.

        Use self._robot_position() to get the current (x, y) of the robot.
        Use self._already_visited(fx, fy) to skip visited frontiers.
        Return None if no valid frontier exists.
        """
        raise NotImplementedError


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()