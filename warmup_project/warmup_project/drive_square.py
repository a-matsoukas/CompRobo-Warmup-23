import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import pi


class DriveSquareNode(Node):
    def __init__(self):
        super().__init__('drive_square')

        # create a publisher for neato's velocity
        self.velocity_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # set square parameters and lin / ang velocities
        self.side_len = 1.0                                 # m
        self.lin_vel = 0.1                                  # m/s
        self.travel_time = self.side_len / self.lin_vel     # s

        self.turn_angle = pi / 2                            # rad
        self.ang_vel = 0.25 * pi                             # rad / s
        self.turn_time = self.turn_angle / self.ang_vel     # s

        # track state of the neato as it is driving
        self.start_time = None
        self.turning = False
        self.num_turns = 0

        # set up a run loop
        self.create_timer(.1, self.run_loop)

    def run_loop(self):
        # neato should drive only if it hasn't completed the square (3 corners)
        if self.num_turns <= 3:
            msg = Twist()

            # set start time
            if self.start_time is None:
                self.start_time = self.get_clock().now()

            # set behavior duration based on turning or driving
            if self.turning:
                duration = self.turn_time
            else:
                duration = self.travel_time

            # check if turned / driven long enough and switch behaviors and reset start time
            if self.get_clock().now() - self.start_time > rclpy.time.Duration(seconds=duration):
                if self.turning:
                    self.num_turns += 1
                    self.turning = False
                else:
                    self.turning = True
                self.start_time = None
            # if not, continue current behavior
            else:
                if self.turning:
                    msg.angular.z = self.ang_vel
                else:
                    msg.linear.x = self.lin_vel

            # publish velocity
            self.velocity_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DriveSquareNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
