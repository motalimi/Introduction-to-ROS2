#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, termios, tty, select

class TurtleKeyboardController(Node):
    def __init__(self):
        super().__init__('turtle_keyboard_controller')
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.linear_speed = 1.5
        self.angular_speed = 1.57
        self.run()

    def get_key(self):
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        twist = Twist()
        try:
            while rclpy.ok():
                key = self.get_key().lower()
                if key == 'i':
                    twist.linear.x = self.linear_speed
                    twist.angular.z = 0.0
                elif key == ',':
                    twist.linear.x = -self.linear_speed
                    twist.angular.z = 0.0
                elif key == 'j':
                    twist.linear.x = 0.0
                    twist.angular.z = self.angular_speed
                elif key == 'l':
                    twist.linear.x = 0.0
                    twist.angular.z = -self.angular_speed
                elif key == 'k':
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                elif key == '\x03':
                    break
                self.cmd_pub.publish(twist)
        except KeyboardInterrupt:
            pass
        finally:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleKeyboardController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
