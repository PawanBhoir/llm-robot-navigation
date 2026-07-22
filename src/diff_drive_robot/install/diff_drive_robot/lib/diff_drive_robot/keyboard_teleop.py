#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios

# Key mappings for control
move_bindings = {
    'w': (1.0, 0.0),
    's': (-1.0, 0.0),
    'a': (0.0, 1.0),
    'd': (0.0, -1.0),
    'x': (0.0, 0.0)
}

msg = """
SmartBOT Keyboard Teleoperation
---------------------------
Moving around:
        w
   a    s    d
        x

w/s : increase/decrease linear velocity (forward/backward)
a/d : increase/decrease angular velocity (left/right)
x   : force immediate full stop
CTRL-C to quit
"""

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.linear_speed = 0.3   # m/s
        self.angular_speed = 0.8  # rad/s
        
        self.get_logger().info("Keyboard Teleop Node Initialized. Ready for inputs.")
        print(msg)

    def get_key(self):
        """Reads non-blocking raw characters from standard input."""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        twist = Twist()
        try:
            while rclpy.ok():
                key = self.get_key()
                if key in move_bindings.keys():
                    x, th = move_bindings[key]
                    twist.linear.x = x * self.linear_speed
                    twist.angular.z = th * self.angular_speed
                    self.publisher_.publish(twist)
                elif key == '':
                    # No key pressed during the non-blocking read window: Stop safely
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.publisher_.publish(twist)
                
                if (key == '\x03'):  # Ctrl-C
                    break
        except Exception as e:
            self.get_logger().error(f"Error in teleop loop: {e}")
        finally:
            # Clean stop sequence on exit
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()