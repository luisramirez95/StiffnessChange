import rclpy
import math
import time
from rclpy.node import Node
from motor_srv.turtle_gait_simple import Robot


class OscillateNode(Node):
    def __init__(self):
        super().__init__('oscillate_node')

        self.declare_parameter('amplitude_deg', 10.0)
        self.declare_parameter('frequency_hz', 0.5)
        self.declare_parameter('update_rate_hz', 50.0)
        self.declare_parameter('servo_ids', [10,11,12])
        self.declare_parameter('device', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 4000000)

        self.amplitude_deg = self.get_parameter('amplitude_deg').value
        self.frequency_hz = self.get_parameter('frequency_hz').value
        update_rate = self.get_parameter('update_rate_hz').value
        servo_ids = list(self.get_parameter('servo_ids').value)
        device = self.get_parameter('device').value
        baudrate = self.get_parameter('baudrate').value

        self.get_logger().info(
            f'Connecting to {device} @ {baudrate} baud, servo_ids={servo_ids}'
        )
        self.robot = Robot(
            servo_ids=servo_ids,
            DEVICENAME=device,
            BAUDRATE=baudrate,
        )

        # Enable torque and set gentle PID gains
        self.robot.enable_torque()
        self.robot.set_profile_velocity(100)
        self.robot.set_profile_acc(50)
        self.robot.set_P_gain(300)
        self.robot.set_I_gain(20)
        self.robot.set_D_gain(1)

        # Read and store current positions as center points
        current_positions = self.robot.robot.sync_read(type="position")
        self.center_positions = list(current_positions)
        self.get_logger().info(
            f'Center positions (encoder): {[int(p) for p in self.center_positions]}'
        )
        center_deg = [self.robot.encoder_to_degrees(p) for p in self.center_positions]
        self.get_logger().info(
            f'Center positions (degrees): {[round(d, 1) for d in center_deg]}'
        )

        # Convert amplitude from degrees to encoder ticks
        # 4096 ticks over 360 degrees => ~11.378 ticks per degree
        self.amplitude_ticks = self.amplitude_deg * (4096.0 / 360.0)
        self.get_logger().info(
            f'Oscillating ±{self.amplitude_deg}° ({self.amplitude_ticks:.0f} ticks) '
            f'at {self.frequency_hz} Hz'
        )

        self.time_elapsed = 0.0
        self.dt = 1.0 / update_rate
        self.timer = self.create_timer(self.dt, self.oscillate_callback)

    def oscillate_callback(self):
        offset = self.amplitude_ticks * math.sin(
            2.0 * math.pi * self.frequency_hz * self.time_elapsed
        )

        goal_positions = [
            int(center + offset) for center in self.center_positions
        ]

        self.robot.robot.sync_write(type="position", dataDict=goal_positions)
        self.time_elapsed += self.dt

    def destroy_node(self):
        self.get_logger().info('Stopping oscillation, returning to center positions...')
        self.robot.robot.sync_write(
            type="position", dataDict=[int(p) for p in self.center_positions]
        )
        time.sleep(1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OscillateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
