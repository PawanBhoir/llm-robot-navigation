#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import tf2_ros

from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue

import numpy as np
import math
from collections import deque
import subprocess
import os


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        # Parameters tuned for early map generation and steady movement
        self.declare_parameter('min_frontier_size', 3)       # Lowered to capture small initial clusters
        self.declare_parameter('revisit_radius',    0.4)       # Smaller revisit radius to avoid premature "Complete"
        self.declare_parameter('poll_period',       1.5)
        self.declare_parameter('map_topic',         '/map')
        self.declare_parameter('action_name',       'navigate_to_pose')
        self.declare_parameter('goal_frame',        'map')
        self.declare_parameter('base_frame',        'base_link')
        self.declare_parameter('min_goal_distance', 0.2)       # Keeps robot from ignoring nearby frontiers
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
        self._map_saved = False

        self._nav_client = ActionClient(self, NavigateToPose, action_name)

        # QoS configuration ensuring instant map subscription
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self._map_sub = self.create_subscription(
            OccupancyGrid, map_topic, self._map_callback, map_qos)

        self.get_logger().info('Waiting for Nav2 action server...')
        self._nav_client.wait_for_server()
        self.get_logger().info('Ready. Autonomous exploration started.')

        self.create_timer(poll_period, self._explore)

        # Update RegulatedPurePursuitController parameters dynamically
        self._param_client = self.create_client(SetParameters, '/controller_server/set_parameters')
        if self._param_client.wait_for_service(timeout_sec=2.0):
            req = SetParameters.Request()
            
            p1 = Parameter(name='FollowPath.desired_linear_vel', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=0.25))
            p2 = Parameter(name='FollowPath.min_desired_linear_velocity', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=0.08))
            
            req.parameters = [p1, p2]
            self._param_client.call_async(req)
            self.get_logger().info("Successfully updated RPP velocity thresholds!")

    def _map_callback(self, msg: OccupancyGrid):
        self._map = msg

    def _get_robot_pose(self) -> tuple[float, float] | None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._goal_frame, self._base_frame, rclpy.time.Time())
            return (transform.transform.translation.x, transform.transform.translation.y)
        except tf2_ros.TransformException:
            return None

    def _explore(self):
        if self._map is None or self._navigating:
            return

        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            return

        grid = np.array(self._map.data, dtype=np.int8).reshape(
            (self._map.info.height, self._map.info.width))
        height, width = grid.shape
        resolution = self._map.info.resolution
        origin_x = self._map.info.origin.position.x
        origin_y = self._map.info.origin.position.y

        # Detect frontier cells: Free space (0) adjacent to unknown (-1)
        free_cells = (grid == 0)
        unknown_cells = (grid == -1)
        frontier_mask = np.zeros_like(grid, dtype=bool)

        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:
            shifted = np.zeros_like(grid, dtype=bool)
            if dx >= 0 and dy >= 0:
                shifted[dx:, dy:] = unknown_cells[:height-dx, :width-dy]
            elif dx >= 0 and dy < 0:
                shifted[dx:, :width+dy] = unknown_cells[:height-dx, -dy:]
            elif dx < 0 and dy >= 0:
                shifted[:height+dx, dy:] = unknown_cells[-dx:, :width-dy]
            else:
                shifted[:height+dx, :width+dy] = unknown_cells[-dx:, -dy:]

            frontier_mask |= (free_cells & shifted)

        # BFS Clustering
        visited_cells = np.zeros_like(grid, dtype=bool)
        frontier_clusters = []

        frontier_indices = np.argwhere(frontier_mask)
        for r, c in frontier_indices:
            if visited_cells[r, c]:
                continue

            cluster = []
            queue = deque([(r, c)])
            visited_cells[r, c] = True

            while queue:
                curr_r, curr_c = queue.popleft()
                cluster.append((curr_r, curr_c))

                for dr, dc in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:
                    nr, nc = curr_r + dr, curr_c + dc
                    if 0 <= nr < height and 0 <= nc < width:
                        if frontier_mask[nr, nc] and not visited_cells[nr, nc]:
                            visited_cells[nr, nc] = True
                            queue.append((nr, nc))

            if len(cluster) >= self._min_size:
                avg_r = sum(cell[0] for cell in cluster) / len(cluster)
                avg_c = sum(cell[1] for cell in cluster) / len(cluster)

                wx = origin_x + (avg_c + 0.5) * resolution
                wy = origin_y + (avg_r + 0.5) * resolution
                frontier_clusters.append((wx, wy))

        # Select Best Frontier
        rx, ry = robot_pose
        best_frontier = None
        min_dist = float('inf')

        for fx, fy in frontier_clusters:
            if any(math.hypot(fx - vx, fy - vy) < self._revisit_r for vx, vy in self._visited):
                continue

            dist = math.hypot(fx - rx, fy - ry)
            if dist < self._min_goal_dist:
                continue

            if dist < min_dist:
                min_dist = dist
                best_frontier = (fx, fy)

        if best_frontier is None:
            self.get_logger().info("Exploration complete! No valid frontiers remaining.")
            if self._map_save_path and not self._map_saved:
                self.get_logger().info(f"Saving finalized map to {self._map_save_path}...")
                try:
                    os.makedirs(os.path.dirname(self._map_save_path), exist_ok=True)
                    subprocess.Popen(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', self._map_save_path])
                    self._map_saved = True
                except Exception as e:
                    self.get_logger().error(f"Failed to save map: {e}")
            return

        # Pullback Math: Step 0.50m from Frontier back toward Robot
        fx, fy = best_frontier
        angle = math.atan2(ry - fy, rx - fx)
        safe_x = fx + 0.50 * math.cos(angle)
        safe_y = fy + 0.50 * math.sin(angle)

        # Mark original frontier target as visited so we don't repeat the loop
        self._visited.append((fx, fy))

        self._send_goal(safe_x, safe_y)

    def _send_goal(self, x: float, y: float):
        self._navigating = True
        self.get_logger().info(f"Navigating to safe open goal: ({x:.2f}, {y:.2f})")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self._goal_frame
        
        # Zero-timestamp allows Nav2 to accept goal immediately in sim time
        goal_msg.pose.header.stamp = rclpy.time.Time().to_msg()
        
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        send_goal_future = self._nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2 server.')
            self._navigating = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Reached goal successfully.')
        else:
            self.get_logger().warn(f'Goal finished with status code: {status}')

        self._navigating = False


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()