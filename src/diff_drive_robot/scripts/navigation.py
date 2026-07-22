#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np

class NavigationFSM(Node):
    def __init__(self):
        super().__init__('navigation_fsm')
        
        # ROS Parameters
        self.declare_parameter('goal_x', 3.0)
        self.declare_parameter('goal_y', 2.0)
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        self.timer = self.create_timer(0.05, self.control_loop) # 20Hz execution loop
        
        # State Initialization
        self.states = ['GOAL_SEEK', 'FIND_CLEAR', 'MOVE_CLEAR', 'REALIGN']
        self.current_state = 'GOAL_SEEK'
        
        # Odometry state track variables
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        # Scan data track variables
        self.scan_ranges = []
        self.angle_min = 0.0
        self.angle_increment = 0.0
        
        # Configuration parameters
        self.obstacle_threshold = 0.6  # meters
        self.detection_angle = 0.5     # ~30 degrees wide arc
        self.clear_heading = 0.0
        self.move_clear_start_x = 0.0
        self.move_clear_start_y = 0.0
        
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Quaternions to Euler Yaw conversion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment

    def get_front_obstacle_distance(self):
        if not self.scan_ranges:
            return float('inf')
        
        min_dist = float('inf')
        for i, r in enumerate(self.scan_ranges):
            angle = self.angle_min + i * self.angle_increment
            if abs(angle) <= self.detection_angle:
                if 0.05 < r < min_dist:
                    min_dist = r
        return min_dist

    def find_clear_direction(self):
        """Scans local headings and selects the candidate that has enough 
        clearance while prioritizing headings close to the goal orientation."""
        if not self.scan_ranges:
            return self.yaw
        
        angle_to_goal = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
        
        best_heading = self.yaw
        best_score = float('-inf')
        
        # Evaluate 36 alternate local directions relative to the robot body frame (-pi to +pi)
        for rel_angle in np.linspace(-math.pi, math.pi, 36):
            eval_global_heading = self.yaw + rel_angle
            clearance = float('inf')
            
            # Check clearance in a small sector (+-0.25 rad) around eval_angle
            for i, r in enumerate(self.scan_ranges):
                ray_angle = self.angle_min + i * self.angle_increment
                diff = math.atan2(math.sin(ray_angle - rel_angle), math.cos(ray_angle - rel_angle))
                
                if abs(diff) < 0.25:
                    if 0.05 < r < clearance:
                        clearance = r
            
            # Penalize headings that deviate too far from the target goal direction
            goal_heading_diff = abs(math.atan2(math.sin(eval_global_heading - angle_to_goal), 
                                               math.cos(eval_global_heading - angle_to_goal)))
            
            # Score balances distance clearance vs. goal alignment
            if clearance >= self.obstacle_threshold:
                score = clearance - (0.5 * goal_heading_diff)
            else:
                score = clearance - 5.0  # heavy penalty for unsafe directions
                
            if score > best_score:
                best_score = score
                best_heading = eval_global_heading
                
        return best_heading

    def control_loop(self):
        if not self.scan_ranges:
            return
            
        dist_to_goal = math.hypot(self.goal_x - self.x, self.goal_y - self.y)
        angle_to_goal = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
        
        twist = Twist()
        
        # Check termination condition
        if dist_to_goal < 0.15:
            self.get_logger().info("Goal position successfully reached!")
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            return

        front_dist = self.get_front_obstacle_distance()

        # FSM State Transitions & Execution
        if self.current_state == 'GOAL_SEEK':
            if front_dist < self.obstacle_threshold:
                self.get_logger().info("Obstacle detected! State -> FIND_CLEAR")
                self.current_state = 'FIND_CLEAR'
            else:
                yaw_diff = math.atan2(math.sin(angle_to_goal - self.yaw), math.cos(angle_to_goal - self.yaw))
                twist.linear.x = 0.25
                twist.angular.z = 1.2 * yaw_diff
                
        elif self.current_state == 'FIND_CLEAR':
            self.clear_heading = self.find_clear_direction()
            self.move_clear_start_x = self.x
            self.move_clear_start_y = self.y
            self.get_logger().info(f"Clear direction found. State -> MOVE_CLEAR (Heading: {self.clear_heading:.2f})")
            self.current_state = 'MOVE_CLEAR'
            
        elif self.current_state == 'MOVE_CLEAR':
            moved_dist = math.hypot(self.x - self.move_clear_start_x, self.y - self.move_clear_start_y)
            yaw_diff = math.atan2(math.sin(self.clear_heading - self.yaw), math.cos(self.clear_heading - self.yaw))
            
            if moved_dist > 0.8 and front_dist > self.obstacle_threshold:
                self.get_logger().info("Obstacle cleared. State -> REALIGN")
                self.current_state = 'REALIGN'
            else:
                twist.linear.x = 0.2
                twist.angular.z = 1.0 * yaw_diff
                
        elif self.current_state == 'REALIGN':
            yaw_diff = math.atan2(math.sin(angle_to_goal - self.yaw), math.cos(angle_to_goal - self.yaw))
            if abs(yaw_diff) < 0.1:
                self.get_logger().info("Realigned with goal. State -> GOAL_SEEK")
                self.current_state = 'GOAL_SEEK'
            else:
                twist.linear.x = 0.0
                twist.angular.z = 1.0 if yaw_diff > 0 else -1.0
                
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationFSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()