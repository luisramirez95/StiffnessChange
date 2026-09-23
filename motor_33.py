import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from dynamixel_sdk import *  # PortHandler, PacketHandler, GroupSyncRead/Write, helpers
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# ---- X-series (Protocol 2.0) control table (common addresses) ----
ADDR_TORQUE_ENABLE        = 64
TORQUE_ENABLE             = 1

ADDR_GOAL_POSITION        = 116
LEN_GOAL_POSITION         = 4

ADDR_PRESENT_CURRENT      = 126
LEN_PRESENT_CURRENT       = 2

ADDR_PRESENT_VELOCITY     = 128
LEN_PRESENT_VELOCITY      = 4

ADDR_PRESENT_POSITION     = 132
LEN_PRESENT_POSITION      = 4

ADDR_OPERATING_MODE       = 11  # 3: Position, 4: Extended Position, etc.

ADDR_POS_D_GAIN           = 80
ADDR_POS_I_GAIN           = 82
ADDR_POS_P_GAIN           = 84

class DynamixelControlNode(Node):
    def __init__(self):
        super().__init__('dynamixel_control_node')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Dynamixel settings
        self.DEVICENAME = '/dev/ttyUSB0'
        self.BAUDRATE = 4000000
        self.PROTOCOL_VERSION = 2.0
        self.DXL_IDS = [1, 2, 3, 4, 5, 6]  # Do not use 0 (broadcast)

        # Default positions
        self.goal_positions = [2048, 2048, 2048, 1797, 1888, 2048]
        self.target_positions = [2048, 2048, 2048, 1797, 1888, 2048]

        self.motor_commands_available = False
        self.ready_to_move = True

        # Initialize communication with Dynamixel motors
        self.port_handler = PortHandler(self.DEVICENAME)
        self.packet_handler = PacketHandler(self.PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            self.get_logger().error("Failed to open port")
            rclpy.shutdown()
            return

        if not self.port_handler.setBaudRate(self.BAUDRATE):
            self.get_logger().error("Failed to set baudrate")
            rclpy.shutdown()
            return

        self.get_logger().info("Port opened and baudrate set")

        # Initialize SyncWrite for goal position
        self.goal_position_write = GroupSyncWrite(
            self.port_handler, self.packet_handler,
            ADDR_GOAL_POSITION, LEN_GOAL_POSITION
        )

        self.enable_torque()
        self.set_motor_gains(kp=800, kd=15)

        # SyncRead for position, velocity and current
        self.position_read = GroupSyncRead(
            self.port_handler, self.packet_handler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
        )
        self.velocity_read = GroupSyncRead(
            self.port_handler, self.packet_handler, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY
        )
        self.current_read = GroupSyncRead(
            self.port_handler, self.packet_handler, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT
        )

        for dxl_id in self.DXL_IDS:
            self.position_read.addParam(dxl_id)
            self.velocity_read.addParam(dxl_id)
            self.current_read.addParam(dxl_id)

        # ROS pubs/subs
        self.motor_status_publisher = self.create_publisher(Float32MultiArray, 'dynamixel_status', sensor_qos)
        self.current_publisher = self.create_publisher(Float32MultiArray, 'dynamixel_current', sensor_qos)

        self.goal_position_subscriber = self.create_subscription(
            Float32MultiArray, 'goal_positions', self.goal_position_callback, 10
        )
        self.motor_commands_subscriber = self.create_subscription(
            Float32MultiArray, '/robot/motor_commands', self.motor_commands_callback, 10
        )

        # Timer for control loop (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

    def enable_torque(self):
        for dxl_id in self.DXL_IDS:
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
            )
            if dxl_comm_result == COMM_SUCCESS and dxl_error == 0:
                self.get_logger().info(f"Motor {dxl_id} torque enabled")
            else:
                self.get_logger().error(f"Torque enable failed for {dxl_id}, comm:{dxl_comm_result} err:{dxl_error}")

    def set_motor_gains(self, kp=640, kd=0):
        kp = int(kp)
        kd = int(kd)
        for dxl_id in self.DXL_IDS:
            operating_mode, dxl_comm_result, dxl_error = self.packet_handler.read1ByteTxRx(
                self.port_handler, dxl_id, ADDR_OPERATING_MODE
            )
            if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                self.get_logger().warn(f"Could not read operating mode for {dxl_id} (comm:{dxl_comm_result} err:{dxl_error})")

            if operating_mode not in [3, 4]:
                self.get_logger().error(
                    f"Motor {dxl_id} not in (Position/Ext Position) mode! Current mode: {operating_mode}"
                )
                continue

            # Position P and D gains
            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(
                self.port_handler, dxl_id, ADDR_POS_P_GAIN, kp
            )
            if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                self.get_logger().error(f"Failed to set Kp for {dxl_id}, comm:{dxl_comm_result} err:{dxl_error}")

            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(
                self.port_handler, dxl_id, ADDR_POS_D_GAIN, kd
            )
            if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                self.get_logger().error(f"Failed to set Kd for {dxl_id}, comm:{dxl_comm_result} err:{dxl_error}")

        self.get_logger().info(f"Set Kp={kp}, Kd={kd} for all motors.")

    def move_to_zero_position(self):
        for dxl_id in self.DXL_IDS:
            param_goal_position = [
                DXL_LOBYTE(DXL_LOWORD(2048)),
                DXL_HIBYTE(DXL_LOWORD(2048)),
                DXL_LOBYTE(DXL_HIWORD(2048)),
                DXL_HIBYTE(DXL_HIWORD(2048)),
            ]
            if not self.goal_position_write.addParam(dxl_id, param_goal_position):
                self.get_logger().error(f"addParam failed for DXL:{dxl_id}")

        dxl_comm_result = self.goal_position_write.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f"txPacket failed (comm:{dxl_comm_result})")
        self.goal_position_write.clearParam()

        self.get_logger().info("Motors moved to zero position, waiting 1s...")
        time.sleep(1)
        self.ready_to_move = True
        self.get_logger().info("Ready to move to target positions.")

    def goal_position_callback(self, msg):
        if not self.motor_commands_available:
            if len(msg.data) == len(self.DXL_IDS):
                self.goal_positions = [int(pos) for pos in msg.data]

    def motor_commands_callback(self, msg):
        if len(msg.data) == 6:
            self.motor_commands_available = True
            self.target_positions = [int(pos) for pos in msg.data]
            self.get_logger().info(f"Received 3 motor commands: {msg.data}")
        else:
            self.get_logger().warn(f"Invalid motor commands length: {len(msg.data)}, expected 6")

    def convert_to_signed(self, value, bit_length=32):
        if value >= (1 << (bit_length - 1)):
            value -= (1 << bit_length)
        return value

    def control_loop(self):
        raw_positions = self.target_positions if self.motor_commands_available else self.goal_positions
        motor_mapping = {1: raw_positions[0], 2: raw_positions[1], 3: raw_positions[2], 4: raw_positions[3], 5: raw_positions[4], 6: raw_positions[5]}

        # Send goal positions
        for dxl_id in self.DXL_IDS:
            gp = motor_mapping[dxl_id]
            param_goal_position = [
                DXL_LOBYTE(DXL_LOWORD(gp)),
                DXL_HIBYTE(DXL_LOWORD(gp)),
                DXL_LOBYTE(DXL_HIWORD(gp)),
                DXL_HIBYTE(DXL_HIWORD(gp)),
            ]
            if not self.goal_position_write.addParam(dxl_id, param_goal_position):
                self.get_logger().error(f"addParam failed for DXL:{dxl_id}")

        dxl_comm_result = self.goal_position_write.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f"Goal position txPacket failed (comm:{dxl_comm_result})")
        self.goal_position_write.clearParam()

        # Read position, velocity and current
        self.position_read.txRxPacket()
        self.velocity_read.txRxPacket()
        self.current_read.txRxPacket()

        # Publish motor status (position and velocity)
        status_msg = Float32MultiArray()
        motor_data = []
        for dxl_id in self.DXL_IDS:
            pos = self.position_read.getData(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            vel = self.velocity_read.getData(dxl_id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY)
            vel = self.convert_to_signed(vel, 32)
            motor_data.extend([float(dxl_id), float(pos), float(vel)])
        status_msg.data = motor_data
        self.motor_status_publisher.publish(status_msg)

        # Publish current data
        current_msg = Float32MultiArray()
        current_data = []
        for dxl_id in self.DXL_IDS:
            curr = self.current_read.getData(dxl_id, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT)
            curr = self.convert_to_signed(curr, 16)
            current_data.extend([float(dxl_id), float(curr)])
        current_msg.data = current_data
        self.current_publisher.publish(current_msg)

def main():
    rclpy.init()
    node = DynamixelControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
