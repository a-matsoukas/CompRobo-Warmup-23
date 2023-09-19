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

        # create publisher for neatos velocity and subcription to bump (for estop)
        self.velocity_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.bump_sub = self.create_subscription(
            Bump, 'bump', self.process_bump, 10)

        # create subscription to lidar scan (for state switching)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.update_state, 10)

        # initialize nodes for two behaviors
        self.drive_square_node = DriveSquareNode()
        self.person_follow_node = PersonFollowerNode()

        # create a set of states
        self.States = Enum(
            'States', ['INITIALIZE', 'PERSON_FOLLOW', 'DRIVE_SQUARE', 'ESTOP'])

        # set start state
        self.state = self.States.INITIALIZE

        # will stop if true
        self.estop = False

        # used to switch from drive_square to person_following
        self.object_detected = False

        # set up run loop
        self.create_timer(.1, self.run_loop)

    def run_loop(self):
        if self.state == self.States.INITIALIZE:
            # go to estop state if True
            if self.estop:
                self.state = self.States.ESTOP

            # if object go to person following, else go to drive square
            if self.object_detected:
                self.state = self.States.PERSON_FOLLOW
            else:
                self.state = self.States.DRIVE_SQUARE

        elif self.state == self.States.PERSON_FOLLOW:
            # go to estop state if True
            if self.estop:
                self.state = self.States.ESTOP

            # spin person following once -- allows for node not to block program
            rclpy.spin_once(self.person_follow_node)

            # change state only if there is no valid target and update parameters
            if self.person_follow_node.target_dis is None:
                self.object_detected = False
                self.state = self.States.DRIVE_SQUARE

        elif self.state == self.States.DRIVE_SQUARE:
            # go to estop state if True
            if self.estop:
                self.state = self.States.ESTOP

            # spin drive square once -- allows for node not to block program
            rclpy.spin_once(self.drive_square_node)

            # change state only if object is detected
            if self.object_detected:
                self.state = self.States.PERSON_FOLLOW

        elif self.state == self.States.ESTOP:
            # stop moving
            self.stop_moving()

        else:
            # catch all state - should not be used
            print("Error State")

    def update_state(self, scan_data):
        """
        Check scan data within person tracking range and update state
        """
        dists = scan_data.ranges
        # for data point within valid scan angles (set by person follower node)
        for ang in list(range(0, self.person_follow_node.left_ang + 1, 1)) + list(range(360 - self.person_follow_node.left_ang, 360, 1)):
            # if there is a valid data point, set object_detected to True
            if not isinf(dists[ang]) and dists[ang] != 0 and dists[ang] <= self.person_follow_node.max_object_range:
                self.object_detected = True
                break

    def process_bump(self, bump_data):
        """
        Will flag neato to stop if bump sensor is hit
        """
        if bump_data.left_front or bump_data.right_front or bump_data.left_side or bump_data.right_side:
            self.estop = True

    def stop_moving(self):
        """
        Set velocities to 0
        """
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
