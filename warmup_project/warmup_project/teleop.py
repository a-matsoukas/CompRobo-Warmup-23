import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tty
import select
import sys
import termios
from math import pi


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop')
        self.settings = termios.tcgetattr(sys.stdin)
        self.key = None
        self.velocity_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_timer(.1, self.run_loop)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run_loop(self):
        # How to fix this so you only need to hit Ctrl-c once to exit??
        if self.key != '\x03':
            self.key = self.getKey()
            direction = Twist()

            # based on a wheel diameter of .2m, and a max wheel velocity of .3 m/s,
            # the max angular velocity is 6pi rad/sec
            if self.key in ['i', 'k', ',', 'j', 'l', 'u', 'o', 'm', '.']:
                if self.key == 'i':
                    direction.linear.x = 0.3
                elif self.key == 'k':
                    direction.linear.x = 0.0
                elif self.key == ',':
                    direction.linear.x = -0.3
                elif self.key == 'j':
                    direction.angular.z = 0.5 * pi
                elif self.key == 'l':
                    direction.angular.z = -0.5 * pi
                elif self.key == 'u':
                    direction.linear.x = 0.3
                    direction.angular.z = 0.5 * pi
                elif self.key == 'o':
                    direction.linear.x = 0.3
                    direction.angular.z = -0.5 * pi
                elif self.key == 'm':
                    direction.linear.x = -0.3
                    direction.angular.z = -0.5 * pi
                else:
                    direction.linear.x = -0.3
                    direction.angular.z = 0.5 * pi

                self.velocity_pub.publish(direction)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()


# Given Skeleton Code
"""
def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


settings = termios.tcgetattr(sys.stdin)
key = None

while key != '\x03':
    key = getKey()
    print(key)
"""
