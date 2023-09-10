import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker


class MarkerNode(Node):
    def __init__(self):
        super().__init__('marker')
        self.marker_pub = self.create_publisher(Marker, 'marker', 10)

        self.create_timer(.1, self.run_loop)

    def run_loop(self):
        marker = Marker()

        marker.header.frame_id = 'odom'

        marker.type = Marker.SPHERE

        marker.pose.position.x = 1.0
        marker.pose.position.y = 2.0
        marker.pose.position.z = 0.0

        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5

        marker.color.a = 1.0  # alpha value (transparency)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
