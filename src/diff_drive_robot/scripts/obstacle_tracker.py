#!/usr/bin/env python3
import json
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import tf2_ros
from geometry_msgs.msg import Point, TransformStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class ObstacleTracker(Node):
    def __init__(self):
        super().__init__('obstacle_tracker')

        self.declare_parameter('robot_ns', '')
        self.declare_parameter('min_speed', 0.08)
        self.declare_parameter('history_len', 10)
        self.declare_parameter('lookback', 5)
        self.declare_parameter('cluster_radius', 0.4)
        self.declare_parameter('marker_lifetime', 0.5)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')

        ns = self.get_parameter('robot_ns').value
        self._min_speed = self.get_parameter('min_speed').value
        history_len = self.get_parameter('history_len').value
        self._lookback = self.get_parameter('lookback').value
        self._cluster_r = self.get_parameter('cluster_radius').value
        self._marker_life = self.get_parameter('marker_lifetime').value
        self._base_frame = self.get_parameter('base_frame').value
        self._map_frame = self.get_parameter('map_frame').value

        if ns:
            self._base_frame = f'{ns}/{self._base_frame}'

        pre = f'/{ns}' if ns else ''

        self._buf: deque = deque(maxlen=history_len)

        self._tf_buf = tf2_ros.Buffer()
        self._tf_lis = tf2_ros.TransformListener(self._tf_buf, self)

        self._marker_pub = self.create_publisher(
            MarkerArray, f'{pre}/obstacle_tracker/markers', 10)
        self._state_pub = self.create_publisher(
            String, f'{pre}/obstacle_tracker/state', 10)

        self.create_subscription(LaserScan, f'{pre}/scan', self._scan_cb, 10)

        self.get_logger().info(
            f'ObstacleTracker ns={ns or "/"} '
            f'min_speed={self._min_speed} m/s '
            f'lookback={self._lookback} frames')

    # ── TODO 1 — Closing-ray detection and TF transform ───────────────────────

    def _scan_cb(self, msg: LaserScan):
        self._buf.append(msg)
        if len(self._buf) <= self._lookback:
            return

        prev_msg = self._buf[-self._lookback - 1]
        t_now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        t_prev = prev_msg.header.stamp.sec + prev_msg.header.stamp.nanosec * 1e-9
        dt = t_now - t_prev

        if dt <= 0:
            return

        closing_points_robot_frame = []
        num_rays = min(len(msg.ranges), len(prev_msg.ranges))

        for i in range(num_rays):
            r_now = msg.ranges[i]
            r_prev = prev_msg.ranges[i]
            if (msg.range_min < r_now < msg.range_max) and (prev_msg.range_min < r_prev < prev_msg.range_max):
                closing_speed = (r_prev - r_now) / dt
                if closing_speed > self._min_speed:
                    angle = msg.angle_min + i * msg.angle_increment
                    x = r_now * math.cos(angle)
                    y = r_now * math.sin(angle)
                    closing_points_robot_frame.append((x, y))

        if not closing_points_robot_frame:
            self._publish([], msg.header.stamp)
            return

        # Lookup transform from laser sensor frame to map frame
        sensor_frame = msg.header.frame_id if msg.header.frame_id else self._base_frame
        try:
            t = self._tf_buf.lookup_transform(
                self._map_frame, sensor_frame, rclpy.time.Time(), timeout=Duration(seconds=0.1))
        except Exception:
            try:
                t = self._tf_buf.lookup_transform(
                    self._map_frame, sensor_frame, msg.header.stamp, timeout=Duration(seconds=0.1))
            except Exception as e:
                self.get_logger().warn(f"TF Lookup failed: {e}", throttle_duration_sec=2.0)
                return

        tx, ty = t.transform.translation.x, t.transform.translation.y
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        map_points = []
        for rx, ry in closing_points_robot_frame:
            mx = tx + (rx * math.cos(yaw) - ry * math.sin(yaw))
            my = ty + (rx * math.sin(yaw) + ry * math.cos(yaw))
            map_points.append((mx, my))

        clusters = self._cluster(map_points)
        self._publish(clusters, msg.header.stamp)

    # ── TODO 2 — Single-linkage clustering ───────────────────────────────────

    def _cluster(self, pts: list[tuple[float, float]]) -> list[dict]:
        if not pts:
            return []
        clusters = []
        visited = set()
        for i, p1 in enumerate(pts):
            if i in visited:
                continue
            current_cluster = [p1]
            visited.add(i)
            queue = deque([p1])
            while queue:
                current_p = queue.popleft()
                for j, p2 in enumerate(pts):
                    if j not in visited and math.hypot(current_p[0] - p2[0], current_p[1] - p2[1]) <= self._cluster_r:
                        visited.add(j)
                        current_cluster.append(p2)
                        queue.append(p2)
            clusters.append({
                'x': sum(p[0] for p in current_cluster) / len(current_cluster),
                'y': sum(p[1] for p in current_cluster) / len(current_cluster),
                'count': len(current_cluster)
            })
        return clusters

    # ── Publish — do not modify ───────────────────────────────────────────────

    def _publish(self, clusters: list[dict], stamp):
        markers = MarkerArray()

        del_marker = Marker()
        del_marker.action = Marker.DELETEALL
        del_marker.header.frame_id = self._map_frame
        del_marker.header.stamp = stamp
        markers.markers.append(del_marker)

        for i, c in enumerate(clusters):
            m = Marker()
            m.header.frame_id = self._map_frame
            m.header.stamp = stamp
            m.ns = 'moving_obstacles'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = c['x']
            m.pose.position.y = c['y']
            m.pose.position.z = 0.3
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.35
            m.color.r = 1.0
            m.color.g = 0.2
            m.color.b = 0.0
            m.color.a = 0.85
            m.lifetime.sec = int(self._marker_life)
            m.lifetime.nanosec = int((self._marker_life % 1) * 1e9)
            markers.markers.append(m)

        self._marker_pub.publish(markers)

        state_msg = String()
        state_msg.data = json.dumps({
            'moving_obstacles': [
                {'x': round(c['x'], 2), 'y': round(c['y'], 2), 'points': c['count']}
                for c in clusters
            ]
        })
        self._state_pub.publish(state_msg)

        if clusters:
            self.get_logger().info(
                f'Moving obstacles: {len(clusters)} cluster(s) — '
                + ', '.join(f'({c["x"]:.2f},{c["y"]:.2f})' for c in clusters))


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()