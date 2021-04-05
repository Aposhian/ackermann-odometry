import rclpy
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy


class AckermannTeleopJoy(Node):
    """Publishes ackermann control by joystick"""

    def __init__(self):
        super().__init__('ackermann_teleop_joy')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_steering_angle', 3.14 / 2),
                ('max_speed', 1.0)
            ]
        )
        self.max_steering_angle = float(self.get_parameter('max_steering_angle').value)
        self.max_speed = float(self.get_parameter('max_speed').value)

        self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        self.publisher = self.create_publisher(AckermannDriveStamped, 'ackermann_cmd', 10)

    def joy_callback(self, msg: Joy):
        speed = self.max_speed * msg.axes[1]
        steering_direction = 1 if speed > 0 else -1
        self.publisher.publish(AckermannDriveStamped(
            header=msg.header,
            drive=AckermannDrive(
                steering_angle=steering_direction * self.max_steering_angle * msg.axes[2],
                speed=speed
            )
        ))


def main(args=None):
    rclpy.init(args=args)

    ackermann_teleop_joy = AckermannTeleopJoy()

    rclpy.spin(ackermann_teleop_joy)

    ackermann_teleop_joy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
