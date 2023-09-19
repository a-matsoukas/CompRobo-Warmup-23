import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from neato2_interfaces.msg import Bump
from math import pi, sin, cos, radians, isinf


class PersonFollowerNode(Node):
    def __init__(self):
        super().__init__('person_follower')

        # create publisher for neatos velocity and publisher for marker to visualize person
        # create subscription to neatos scan topic (wall detection) and bump sensor (emergency stop)
        self.velocity_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.process_data, 10)
        self.marker_pub = self.create_publisher(
            Marker, 'marker', 10)
        self.bump_sub = self.create_subscription(
            Bump, 'bump', self.process_bump, 10)

        # will stop when True
        self.estop = False

        # angle and distance to target, in base_link frame
        self.target_angle = None
        self.target_dis = None

        # marker object for visualizing person and its attributes
        self.target_pt = Marker()
        self.target_pt.header.frame_id = 'base_link'
        self.target_pt.type = 3
        self.target_pt.scale.x = 0.25
        self.target_pt.scale.y = 0.25
        self.target_pt.scale.z = 1.0
        self.target_pt.color.a = 1.0
        self.target_pt.color.r = 0.0
        self.target_pt.color.g = 1.0
        self.target_pt.color.b = 0.0

        # attributes for customizing the tracking region
        self.left_ang = 30  # angle to left and right of center in which person is tracked
        # neato will stay this far from person (in meters)
        self.follow_dist = 0.75
        # will not follow objects farther than this (in meters)
        self.max_object_range = 1.25

        # max ang / lin velocity possible
        self.base_ang_vel = pi / 4
        self.base_lin_vel = 0.5

        # set up run loop
        self.create_timer(.1, self.run_loop)

    def run_loop(self):
        # check if valid data and estop isn't active
        if not self.estop and self.target_angle is not None and self.target_dis is not None:
            msg = Twist()

            # determine linear velocity proportional to how close / far it is from target
            if self.target_dis >= self.follow_dist:
                msg.linear.x = self.base_lin_vel * \
                    ((self.target_dis - self.follow_dist) /
                     (self.max_object_range - self.follow_dist))
            else:
                msg.linear.x = self.base_lin_vel * \
                    ((self.target_dis - self.follow_dist) / self.follow_dist)

            # determine angular velocity proportional to angle to target
            msg.angular.z = self.base_ang_vel * \
                (self.target_angle / self.left_ang)

            # publish velocity
            self.velocity_pub.publish(msg)

            # helper function to visualize person
            self.pub_marker(self.polar_to_cart(
                self.target_dis, self.target_angle))
        else:
            # else stop moving
            self.stop_moving()

    def process_data(self, scan_data):
        """
        Take scan data, remove values outside of scan region, and pair with list of corresponding angles
        """
        radii = scan_data.ranges
        tracking_radii = radii[360 - self.left_ang:360] + \
            radii[0:self.left_ang + 1]
        tracking_angles = list(range(360 - self.left_ang, 360, 1)) + \
            list(range(0, self.left_ang + 1, 1))
        self.calc_centroid(tracking_radii, tracking_angles)

    def calc_centroid(self, radii, angles):
        """
        Calculate centriod of points in tracking region
        """
        radii_sum = 0
        angle_sum = 0
        num_processed = 0

        for i in range(len(radii)):
            # for each valid data point that is in the tracking distance
            if radii[i] != 0 and not isinf(radii[i]) and radii[i] <= self.max_object_range:

                # add dist to sum
                radii_sum += radii[i]

                # add or subtract angle based on side of neato
                if angles[i] > self.left_ang:
                    angle_sum += angles[i] - 360
                else:
                    angle_sum += angles[i]

                # increment number points seen
                num_processed += 1

        # if at least one valid point, calculate mean
        # else, no target in tracking region
        if num_processed != 0:
            self.target_dis = radii_sum / num_processed
            self.target_angle = angle_sum / num_processed
        else:
            self.target_dis = None
            self.target_angle = None
        print(self.target_dis, self.target_angle)

    def polar_to_cart(self, rad, theta):
        """
        Helper function to convert from polar to cartesian coordinates in the base_link frame
        """
        x = rad * cos(radians(theta))
        y = rad * sin(radians(theta))
        z = 0.0
        return [x, y, z]

    def pub_marker(self, coords):
        """
        Helper function to visualize person using marker
        """
        self.target_pt.pose.position.x = coords[0]
        self.target_pt.pose.position.y = coords[1]
        self.target_pt.pose.position.z = coords[2]
        self.marker_pub.publish(self.target_pt)

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


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
