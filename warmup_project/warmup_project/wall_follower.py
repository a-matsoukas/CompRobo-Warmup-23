import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from neato2_interfaces.msg import Bump
from math import pi, atan, sin, cos, degrees, radians, isinf
from statistics import mean


class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower')

        # create publisher for neatos velocity and publisher for marker(s) to visualize wall
        # create subscription to neatos scan topic (wall detection) and bump sensor (emergency stop)
        self.velocity_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.process_data, 10)
        self.marker_pub = self.create_publisher(
            Marker, 'marker', 10)
        self.bump_sub = self.create_subscription(
            Bump, 'bump', self.process_bump, 10)

        # will stop if True
        self.estop = False

        # angle with wall - should be 45deg
        self.theta1 = None

        # fastest possible ang vel of neato
        self.base_ang_vel = pi / 4

        # a list of point objects to be published to visualize where the neato is seeing the wall
        self.wall_points_odom = []

        # a marker object to vizualize the wall and its properties
        self.wall_marker = Marker()
        self.wall_marker.header.frame_id = 'base_link'  # with respect to the neatos frame
        self.wall_marker.type = 6  # cube list
        self.wall_marker.scale.x = .25
        self.wall_marker.scale.y = .25
        self.wall_marker.scale.z = .25
        self.wall_marker.color.a = 1.0
        self.wall_marker.color.r = 0.0
        self.wall_marker.color.g = 1.0
        self.wall_marker.color.b = 0.0

        # set up run loop
        self.create_timer(.1, self.run_loop)

    def run_loop(self):
        # if bumper is not pressed and neato senses wall
        if not self.estop and self.theta1 is not None:
            msg = Twist()

            # set constant linear velocity and angular velocity proportional to how far angle is from 45deg
            msg.linear.x = 0.25
            msg.angular.z = self.base_ang_vel * ((self.theta1 - 45) / 45)

            # helper func to visualize wall points
            self.wall_vis()

            # publish velocities
            self.velocity_pub.publish(msg)
        else:
            # else stop moving
            self.stop_moving()

    def process_data(self, scan_data):
        """
        Use lidar data to calculate angle with wall
        """
        # pick a few angles to calculate
        angs = [180, 185, 190, 195, 200, 205, 210, 215, 220,
                225, 230, 235, 240, 245, 250, 255, 260, 265]

        res = []  # stores calculations for angle
        wall_points = []  # stores valid data points to be passed to self.wall_marker
        # iterate through and calculate the angle with the wall
        for ang in angs:
            # get dist at each angle and the angle at 90deg with it
            x = scan_data.ranges[ang]
            y = scan_data.ranges[ang + 90]

            # check both data points are valid
            if x != 0 and y != 0 and not isinf(x) and not isinf(y):
                # calculate angle with wall
                theta = degrees(
                    atan(x / y))

                # account for difference in angles from 225deg
                res.append(theta - 225 + ang)

                # update wall points
                wall_points += [self.create_point(self.polar_to_cart(
                    x, ang)), self.create_point(self.polar_to_cart(y, ang + 90))]

        self.wall_points_odom = wall_points

        # set angle to the mean of the collected measurements for redundancy
        # if no valid data, angle is None, and neato will stop
        self.theta1 = None if len(res) == 0 else mean(res)
        print(self.theta1)

    def process_bump(self, bump_data):
        """
        Flag neato to stop if bump sensor is hit
        """
        if bump_data.left_front or bump_data.right_front or bump_data.left_side or bump_data.right_side:
            self.estop = True

    def stop_moving(self):
        """
        Set neatos velocities to 0
        """
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.velocity_pub.publish(msg)

    def create_point(self, coords):
        """
        Helper function to create point object from ordered pair
        """
        pt = Point()
        pt.x = coords[0]
        pt.y = coords[1]
        pt.z = coords[2]
        return pt

    def wall_vis(self):
        """
        Helper function to publish wall visual
        """
        self.wall_marker.points = self.wall_points_odom
        self.marker_pub.publish(self.wall_marker)

    def polar_to_cart(self, rad, theta):
        """
        Helper function to convert from polar to cartesian coordinates in the base_link frame
        """
        x = rad * cos(radians(theta))
        y = rad * sin(radians(theta))
        z = 0.0
        return [x, y, z]


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
