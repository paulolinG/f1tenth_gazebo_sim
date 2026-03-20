import math
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist


class AckermannToTwist(Node):
    def __init__(self):
        """
        Converts AckermannDriveStamped (/drive) to Twist (/cmd_vel).

        Bicycle model:
        linear.x  = speed
        angular.z = speed * tan(steering_angle) / wheelbase
        """
        super().__init__('ackermann_to_twist')

        self.declare_parameter('wheelbase', 0.3302)
        self.declare_parameter('input_topic', '/drive')
        self.declare_parameter('output_topic', '/cmd_vel')

        self.wheelbase = self.get_parameter('wheelbase').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.pub = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(
            AckermannDriveStamped, input_topic, self._callback, 10
        )

        self.get_logger().info(
            f'Ackermann->Twist: {input_topic} -> {output_topic} '
            f'(wheelbase={self.wheelbase}m)'
        )

    def _callback(self, msg: AckermannDriveStamped):
        twist = Twist()
        speed = msg.drive.speed
        steer = msg.drive.steering_angle

        twist.linear.x = speed
        if abs(self.wheelbase) > 1e-6:
            twist.angular.z = speed * math.tan(steer) / self.wheelbase
        else:
            twist.angular.z = 0.0

        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = AckermannToTwist()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()