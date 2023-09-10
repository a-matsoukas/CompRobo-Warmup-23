import rclpy
from rclpy.node import Node
import tty
import select
import sys
import termios


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop')
        self.settings = termios.tcgetattr(sys.stdin)
        self.key = None
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
            print(self.key)


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
