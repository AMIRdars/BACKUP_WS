#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time_source import TimeSource
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from rclpy.parameter import Parameter


class RoverTwistRelay(Node):
    def __init__(self):
        super().__init__('rover_twist_relay')


        # Force use_sim_time=True
        self.set_parameters([
            Parameter('use_sim_time', Parameter.Type.BOOL, True)
        ])


        # Attach time source
        self._time_source = TimeSource()
        self._time_source.attach_node(self)


        # Declare rover selector
        self.rover = self.declare_parameter('rover', 'mecanum3').value


        # Declare default (fallback) geometry
        self.declare_parameter('wheel_radius', 0.07)
        self.declare_parameter('wheel_separation_width', 0.24)
        self.declare_parameter('wheel_separation_length', 0.175)


        # Declare per-rover parameters (flat names)
        self.declare_parameter('mecanum3.wheel_radius', 0.07)
        self.declare_parameter('mecanum3.wheel_separation_width', 0.24)
        self.declare_parameter('mecanum3.wheel_separation_length', 0.175)

        self.declare_parameter('g120a.wheel_radius', 0.085)
        self.declare_parameter('g120a.wheel_separation_width', 0.30)
        self.declare_parameter('g120a.wheel_separation_length', 0.22)

        self.declare_parameter('g40a_lb.wheel_radius', 0.0762)
        self.declare_parameter('g40a_lb.wheel_separation_width', 0.2358)
        self.declare_parameter('g40a_lb.wheel_separation_length', 0.040)



        # Apply rover-specific geometry
        self.apply_rover_geometry()


        # Publisher
        self.pub_wheel = self.create_publisher(
            Float64MultiArray,
            '/wheel_velocity_controller/commands',
            10
        )


        # Subscriber
        self.sub_twist = self.create_subscription(
            Twist,
            '/rover_twist',
            self.on_twist_received,
            10
        )

        self.last_twist = Twist()

        # 100 Hz timer
        self.timer = self.create_timer(0.01, self.on_timer)

        self.get_logger().info(
            f"rover_twist_relay started (rover={self.rover})"
        )

    # ------------------------------------------------------------
    def apply_rover_geometry(self):
        prefix = f'{self.rover}.'
        params = self._parameters

        def get_param(name, default):
            key = prefix + name
            if key in params:
                return params[key].value
            return default

        wheel_radius = get_param(
            'wheel_radius',
            self.get_parameter('wheel_radius').value
        )
        width = get_param(
            'wheel_separation_width',
            self.get_parameter('wheel_separation_width').value
        )
        length = get_param(
            'wheel_separation_length',
            self.get_parameter('wheel_separation_length').value
        )

        self.set_parameters([
            Parameter('wheel_radius', Parameter.Type.DOUBLE, wheel_radius),
            Parameter('wheel_separation_width', Parameter.Type.DOUBLE, width),
            Parameter('wheel_separation_length', Parameter.Type.DOUBLE, length),
        ])

        self.r = wheel_radius
        self.W = width
        self.L = length
        self.k_geom = self.L + self.W


    # Convert Twist to wheel velocities (skid-steer / 4-wheel differential drive)
    # vy (lateral) is ignored: cylinder wheels cannot strafe.
    # Left wheels share one speed, right wheels share another.
    def twist_to_wheels(self, twist):
        vx = twist.linear.x
        wz = twist.angular.z

        v_l = (vx - wz * self.W / 2.0) / self.r
        v_r = (vx + wz * self.W / 2.0) / self.r

        msg = Float64MultiArray()
        msg.data = [v_l, v_r, v_l, v_r]  # [front_left, front_right, back_left, back_right]
        return msg

    # ------------------------------------------------------------
    def on_twist_received(self, msg):
        self.last_twist = msg
        self.pub_wheel.publish(self.twist_to_wheels(msg))

    # ------------------------------------------------------------
    def on_timer(self):
        self.pub_wheel.publish(self.twist_to_wheels(self.last_twist))


def main():
    rclpy.init()
    node = RoverTwistRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

