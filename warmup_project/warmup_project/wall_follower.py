import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from neato2_interfaces.msg import Bump
from math import pi, atan, degrees
from statistics import mean


class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.velocity_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.process_data, 10)
        self.marker_pub = self.create_publisher(
            Marker, 'marker', 10)
        self.create_subscription(Bump, 'bump', self.process_bump, 10)

        self.estop = False

        self.theta1 = None
        self.base_ang_vel = pi / 4

        self.create_timer(.1, self.run_loop)

    def run_loop(self):
        if not self.estop and self.theta1 is not None:
            msg = Twist()
            msg.linear.x = 0.25
            msg.angular.z = self.base_ang_vel * ((self.theta1 - 45) / 45)
            self.velocity_pub.publish(msg)
        else:
            self.stop_moving()

    def process_data(self, scan_data):
        # pick a few angles to calculate
        angs = [180, 185, 190, 195, 200, 205, 210, 215, 220,
                225, 230, 235, 240, 245, 250, 255, 260, 265, 270]

        res = []
        # iterate through and calculate the angle with the wall
        for ang in angs:
            x = scan_data.ranges[ang]
            y = scan_data.ranges[ang + 90]
            if x != 0 and y != 0:
                theta = degrees(
                    atan(x / y))
                # account for difference in angles
                res.append(theta - 225 + ang)
        print(res)
        self.theta1 = None if len(res) == 0 else mean(res)
        print(self.theta1)

        # publish points along the wall for visual

    def process_bump(self, bump_data):
        if bump_data.left_front or bump_data.right_front or bump_data.left_side or bump_data.right_side:
            self.estop = True

    def stop_moving(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.velocity_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
