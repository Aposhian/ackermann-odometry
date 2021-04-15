from typing import NamedTuple

import numpy as np
import rclpy
from ackermann_interfaces.msg import AckermannFeedback
import builtin_interfaces.msg
from geometry_msgs.msg import (Point, Pose, PoseWithCovariance, Quaternion,
                               Twist, TwistWithCovariance, Vector3)
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import Header


class AckermannState(NamedTuple):
    position: np.array
    orientation: Rotation
    left_wheel_speed: float
    right_wheel_speed: float
    steering_angle: float
    time: rclpy.time.Time


class OdomPublisher(Node):
    """Listens to odometry information and publishes Odometry message at regular rate"""

    def __init__(self):
        super().__init__('odom_publisher')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('axle_length', None),
                ('wheelbase_length', None),
                ('wheel_radius', None),
                ('center_of_mass_offset', 0.0),
                ('damping_factor', 0.1)
            ]
        )
        try:
            self.axle_length = float(self.get_parameter('axle_length').value)
            self.wheelbase_length = float(self.get_parameter('wheelbase_length').value)
            self.wheel_radius = float(self.get_parameter('wheel_radius').value)
            self.center_of_mass_offset = float(self.get_parameter('center_of_mass_offset').value)
            self.damping_factor = float(self.get_parameter('damping_factor').value)
        except TypeError as e:
            raise RuntimeError('Not all parameters are set properly') from e

        # Publishers
        queue_size = 10
        self.publisher = self.create_publisher(Odometry, 'odom', queue_size)

        # Subscribers
        self.create_subscription(
            AckermannFeedback,
            'feedback',
            self.feedback_callback,
            10
        )

        self.state = AckermannState(
            position=np.array([0,0,0]),
            orientation=Rotation([0, 0, 0, 1]), # identity
            left_wheel_speed=0.0,
            right_wheel_speed=0.0,
            steering_angle=0.0,
            time=self.get_clock().now()
        )

    def state_update(self, state: AckermannState, feedback: AckermannFeedback) -> AckermannState:
        """Calculate the next state based off the current state"""
        average_wheel_speed = (state.left_wheel_speed + state.right_wheel_speed) / 2
        linear_speed = average_wheel_speed * self.wheel_radius
        turn_radius = self.turn_radius(state.steering_angle)
        angular_speed = linear_speed / turn_radius

        feedback_time = rclpy.time.Time.from_msg(feedback.header.stamp)
        time_delta = (feedback_time - state.time).nanoseconds * 1e-9

        heading_delta = angular_speed * time_delta
        orientation_delta = Rotation.from_euler('xyz', [0, 0, heading_delta])
        if np.isfinite(turn_radius):
            x_delta = turn_radius * (1 - np.cos(heading_delta))
            y_delta = turn_radius * np.sin(heading_delta)
            position_delta = np.array([x_delta, y_delta, 0])
        else:
            position_delta = state.position + time_delta * self.linear_velocity(state.orientation, linear_speed)

        return AckermannState(
            position=state.position + self.damping_factor * position_delta,
            orientation=state.orientation * orientation_delta,
            left_wheel_speed=feedback.left_wheel_speed,
            right_wheel_speed=feedback.right_wheel_speed,
            steering_angle=feedback.left_wheel_speed,
            time=feedback_time
        )

    def output(self, state: AckermannState) -> Odometry:
        """Build Odometry message from state"""
        quaternion = state.orientation.as_quat()
        linear_speed = self.wheel_radius * (state.left_wheel_speed + state.right_wheel_speed) / 2
        linear_velocity = self.linear_velocity(state.orientation, linear_speed)
        angular_speed = linear_speed / self.turn_radius(state.steering_angle)

        return Odometry(
            header=Header(
                stamp=state.time.to_msg(),
                frame_id='odom'
            ),
            child_frame_id='',
            pose=PoseWithCovariance(
                pose=Pose(
                    position=Point(
                        x=state.position[0],
                        y=state.position[1],
                        z=state.position[2]
                    ),
                    orientation=Quaternion(
                        x=quaternion[0],
                        y=quaternion[1],
                        z=quaternion[2],
                        w=quaternion[3]
                    )
                )
            ),
            twist=TwistWithCovariance(
                twist=Twist(
                    linear=Vector3(
                        x=linear_velocity[0],
                        y=linear_velocity[1],
                        z=linear_velocity[2]
                    ),
                    angular=Vector3(
                        z=angular_speed
                    )
                )
            )
        )

    def feedback_callback(self, msg: AckermannFeedback):
        """Update state and publish to Odometry"""
        self.state = self.state_update(self.state, msg)
        self.get_logger().debug(f'state update: {self.state}')
        output = self.output(self.state)
        self.get_logger().debug(f'odometry: {output}')

        self.publisher.publish(output)

    def turn_radius(self, steering_angle: float) -> float:
        """
        Calculate the radius of the circle tracked by a turning vehicle
        For left turns, the radius is positive. For right turns, the radius is negative.
        """
        # If the steering angle is 0, then cot(0) is undefined
        if np.isclose(0, steering_angle):
            return np.inf
        else:
            return np.sign(steering_angle) \
                * np.sqrt(self.center_of_mass_offset**2 + self.wheelbase_length**2 * 1 / np.tan(steering_angle)**2)

    def linear_velocity(self, orientation: Rotation, speed: float) -> np.array:
        return orientation.apply([0, speed, 0]) # rotate from +y direction

def main(args=None):
    rclpy.init(args=args)

    odom_publisher = OdomPublisher()

    rclpy.spin(odom_publisher)

    odom_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
