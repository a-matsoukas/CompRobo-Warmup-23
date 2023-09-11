import rclpy
from rclpy.node import Node
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
        self.create_subscription(Bump, 'bump', self.process_bump, 10)

        self.estop = False

        self.theta1 = 0.0
        self.base_ang_vel = pi / 4

        self.create_timer(.1, self.run_loop)

    def run_loop(self):
        if not self.estop:
            msg = Twist()
            msg.linear.x = 0.25
            msg.angular.z = self.base_ang_vel * ((self.theta1 - 45) / 45)
            self.velocity_pub.publish(msg)
        else:
            self.stop_moving()

    def process_data(self, scan_data):
        # pick a few angles to calculate
        angs = [215, 220, 225, 230, 235]

        res = []
        # iterate through and calculate the angle with the wall
        for ang in angs:
            theta = degrees(
                atan(scan_data.ranges[ang] / scan_data.ranges[ang + 90]))
            res.append(theta - 225 + ang)  # account for difference in angles
        self.theta1 = mean(res)

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
