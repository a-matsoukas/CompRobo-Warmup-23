import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from neato2_interfaces.msg import Bump
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from .angle_helpers import euler_from_quaternion
from math import pi, sin, cos, atan2, degrees, radians, sqrt, isinf


class ObstacleAvoiderNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        self.velocity_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.process_data, 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.update_attract_forces, 10)
        self.goal_pub = self.create_publisher(Marker, 'marker', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.create_subscription(Bump, 'bump', self.process_bump, 10)

        self.estop = False

        # neato target in base_link frame -- set by user
        self.target_base_link = [0.1, -1.5]
        # neato target in world frame -- to be set by program
        self.target_world = None

        self.goal_marker = Marker()
        # this works if the odom frame lines up with world frame. Update with name of world frame
        self.goal_marker.header.frame_id = 'odom'
        self.goal_marker.type = 3
        self.goal_marker.scale.x = 0.25
        self.goal_marker.scale.y = 0.25
        self.goal_marker.scale.z = 1.0
        self.goal_marker.color.a = 1.0
        self.goal_marker.color.r = 0.0
        self.goal_marker.color.g = 1.0
        self.goal_marker.color.b = 0.0

        self.max_field_dist = 2.0
        self.alpha = .75
        self.beta = 0.015
        self.delta_x_repel = 0.0
        self.delta_y_repel = 0.0
        self.delta_x_attract = 0.0
        self.delta_y_attract = 0.0
        self.at_target = False

        self.set_vel = 0.0
        self.set_ang = 0.0

        self.base_ang_vel = pi / 1.5

        self.create_timer(.1, self.run_loop)

    def run_loop(self):
        if not self.estop and not self.at_target:
            self.set_vel, self.set_ang = self.calc_vel_and_ang()

            msg = Twist()
            msg.linear.x = min(self.set_vel, .1)
            msg.angular.z = self.base_ang_vel * (self.set_ang / 180.0)
            self.mark_goal()
            self.velocity_pub.publish(msg)

        else:
            self.stop_moving()

    def calc_vel_and_ang(self):
        delta_x_net = self.delta_x_attract + self.delta_x_repel
        delta_y_net = self.delta_y_attract + self.delta_y_repel
        print("Attractive Forces:", self.delta_x_attract, self.delta_y_attract)
        print("Repulsive Forces:", self.delta_x_repel, self.delta_y_repel)
        print("Net Force:", delta_x_net, delta_y_net)

        vel = sqrt(delta_x_net**2 + delta_y_net**2)
        ang = atan2(delta_y_net, delta_x_net)
        print(vel, degrees(ang))

        return vel, degrees(ang)

    def process_data(self, scan_data):
        radii = scan_data.ranges
        delta_x = 0
        delta_y = 0
        for i in range(0, 360, 1):
            if radii[i] != 0 and not isinf(radii[i]) and radii[i] <= self.max_field_dist:
                delta_x += - self.beta * \
                    (self.max_field_dist - radii[i]) * cos(radians(i))
                delta_y += - self.beta * \
                    (self.max_field_dist - radii[i]) * sin(radians(i))

        self.delta_x_repel = delta_x
        self.delta_y_repel = delta_y

    def update_attract_forces(self, odom_data):
        neato_pos = [odom_data.pose.pose.position.x,
                     odom_data.pose.pose.position.y]
        neato_ang = euler_from_quaternion(odom_data.pose.pose.orientation.x, odom_data.pose.pose.orientation.y,
                                          odom_data.pose.pose.orientation.z, odom_data.pose.pose.orientation.w)[2]

        # if no world target, set based on goal in base_link
        # this will allow the target to stay fixed as neato moves
        if self.target_world is None:
            self.target_world = [(cos(neato_ang) * self.target_base_link[0] - sin(neato_ang) * self.target_base_link[1]) + neato_pos[0],
                                 (sin(neato_ang) * self.target_base_link[0] + cos(neato_ang) * self.target_base_link[1]) + neato_pos[1]]

        # convert world target from world to base_link for calculations
        target_base_link = [cos(-neato_ang) * (self.target_world[0] - neato_pos[0]) - sin(-neato_ang) * (self.target_world[1] - neato_pos[1]),
                            sin(-neato_ang) * (self.target_world[0] - neato_pos[0]) + cos(-neato_ang) * (self.target_world[1] - neato_pos[1])]

        # calculate distance and angle
        target_dis = sqrt(
            (target_base_link[0])**2 + (target_base_link[1])**2)
        target_angle = atan2(target_base_link[1], target_base_link[0])

        print("Polar Coords of Target", target_dis, degrees(target_angle))

        # update delta_x and delta_y
        if target_dis < 0.5:
            self.delta_x_attract = 0
            self.delta_y_attract = 0
            self.at_target = True
        elif target_dis >= 0.5 and target_dis <= self.max_field_dist:
            self.delta_x_attract = self.alpha * target_dis * cos(target_angle)
            self.delta_y_attract = self.alpha * target_dis * sin(target_angle)
        else:
            self.delta_x_attract = self.alpha * \
                self.max_field_dist * cos(target_angle)
            self.delta_y_attract = self.alpha * \
                self.max_field_dist * sin(target_angle)

    def mark_goal(self):
        if self.target_world is not None:
            self.goal_marker.pose.position.x = self.target_world[0]
            self.goal_marker.pose.position.y = self.target_world[1]
            self.goal_marker.pose.position.z = 0.0
            self.goal_pub.publish(self.goal_marker)

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
    node = ObstacleAvoiderNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
