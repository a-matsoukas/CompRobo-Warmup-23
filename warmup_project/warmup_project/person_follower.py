import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from neato2_interfaces.msg import Bump
from math import pi, atan, sin, cos, degrees, radians, isinf
from statistics import mean


class PersonFollowerNode(Node):
    def __init__(self):
        super().__init__('person_follower')
        self.velocity_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.process_data, 10)
        self.marker_pub = self.create_publisher(
            Marker, 'marker', 10)
        self.create_subscription(Bump, 'bump', self.process_bump, 10)

        self.estop = False

        self.target_angle = None
        self.target_dis = None

        self.left_ang = 45
        self.follow_dist = 0.5
        self.max_object_range = 1.0

        self.base_ang_vel = pi / 4
        self.base_lin_vel = 0.5

        self.create_timer(.1, self.run_loop)

    def run_loop(self):
        if not self.estop and self.target_angle is not None and self.target_dis is not None:
            msg = Twist()

            if self.target_dis >= self.follow_dist:
                msg.linear.x = self.base_lin_vel * \
                    ((self.target_dis - self.follow_dist) /
                     (self.max_object_range - self.follow_dist))
            else:
                msg.linear.x = self.base_lin_vel * \
                    ((self.target_dis - self.follow_dist) / self.follow_dist)

            msg.angular.z = self.base_ang_vel * \
                (self.target_angle / self.left_ang)
            self.velocity_pub.publish(msg)
        else:
            self.stop_moving()

    def process_data(self, scan_data):
        radii = scan_data.ranges
        tracking_radii = radii[360 - self.left_ang:360] + \
            radii[0:self.left_ang + 1]
        tracking_angles = list(range(360 - self.left_ang, 360, 1)) + \
            list(range(0, self.left_ang + 1, 1))
        self.calc_centroid(tracking_radii, tracking_angles)

    def calc_centroid(self, radii, angles):
        radii_sum = 0
        angle_sum = 0
        num_processed = 0

        for i in range(len(radii)):
            if radii[i] != 0 and not isinf(radii[i]) and radii[i] <= self.max_object_range:
                radii_sum += radii[i]

                if angles[i] > self.left_ang:
                    angle_sum += angles[i] - 360
                else:
                    angle_sum += angles[i]
                num_processed += 1

        if num_processed != 0:
            self.target_dis = radii_sum / num_processed
            self.target_angle = angle_sum / num_processed
        else:
            self.target_dis = None
            self.target_angle = None
        print(self.target_dis, self.target_angle)

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
    node = PersonFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
