#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control Your Robot!
---------------------------
Hold key to move:
   w
a  s  d
   x

Release key -> Stops instantly!
CTRL-C to quit
"""

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info("Keyboard Teleop Started — Auto-stop enabled.")

    def getKey(self, timeout=0.05):
        """Non-blocking key read with a short timeout."""
        tty.setraw(sys.stdin.fileno())
        # Check if stdin has data waiting within the timeout window
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
            # Flush any queued/backlogged keys so we only care about the present moment
            termios.tcflush(sys.stdin, termios.TCIFLUSH)
        else:
            key = ''  # Timeout reached -> No key is being actively held down
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def run(self):
        print(msg)
        linear_speed = 0.5
        angular_speed = 1.0

        try:
            while rclpy.ok():
                key = self.getKey(timeout=0.05)
                twist = Twist()

                if key == 'w':
                    twist.linear.x = linear_speed
                elif key == 's' or key == 'x':
                    twist.linear.x = -linear_speed
                elif key == 'a':
                    twist.angular.z = angular_speed
                elif key == 'd':
                    twist.angular.z = -angular_speed
                elif key == '\x03':  # Ctrl+C
                    break
                else:
                    # Key released or no active input -> Send zero velocity immediately
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0

                self.publisher_.publish(twist)

        except Exception as e:
            self.get_logger().error(f"Teleop error: {e}")
        finally:
            self.stop()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()