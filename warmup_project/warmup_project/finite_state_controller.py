import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from neato2_interfaces.msg import Bump
from sensor_msgs.msg import LaserScan
from .drive_square import DriveSquareNode
from .person_follower import PersonFollowerNode
from math import isinf
from enum import Enum


class FiniteStateControllerNode(Node):
    def __init__(self):
        super().__init__('finite_state_controller')

        self.velocity_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Bump, 'bump', self.process_bump, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.update_state, 10)

        self.drive_square_node = DriveSquareNode()
        self.person_follow_node = PersonFollowerNode()

        self.States = Enum(
            'States', ['INITIALIZE', 'PERSON_FOLLOW', 'DRIVE_SQUARE', 'ESTOP'])

        self.state = self.States.INITIALIZE

        self.estop = False
        self.object_detected = False

        self.create_timer(.1, self.run_loop)

    def run_loop(self):
        if self.state == self.States.INITIALIZE:
            if self.estop:
                self.state = self.States.ESTOP

            if self.object_detected:
                self.state = self.States.PERSON_FOLLOW
            else:
                self.state = self.States.DRIVE_SQUARE

        elif self.state == self.States.PERSON_FOLLOW:
            if self.estop:
                self.state = self.States.ESTOP

            rclpy.spin_once(self.person_follow_node)

            if self.person_follow_node.target_dis is None:
                self.object_detected = False
                self.state = self.States.DRIVE_SQUARE

        elif self.state == self.States.DRIVE_SQUARE:
            if self.estop:
                self.state = self.States.ESTOP

            rclpy.spin_once(self.drive_square_node)

            if self.object_detected:
                self.state = self.States.PERSON_FOLLOW

        elif self.state == self.States.ESTOP:
            self.stop_moving()

        else:
            print("Error State")

    def update_state(self, scan_data):
        dists = scan_data.ranges
        for ang in list(range(0, self.person_follow_node.left_ang + 1, 1)) + list(range(360 - self.person_follow_node.left_ang, 360, 1)):
            if not isinf(dists[ang]) and dists[ang] != 0 and dists[ang] <= self.person_follow_node.max_object_range:
                self.object_detected = True
                break

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
    node = FiniteStateControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
