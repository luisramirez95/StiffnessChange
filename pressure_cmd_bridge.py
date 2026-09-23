#!/usr/bin/env python3
"""Single serial owner, preserving the original pressure_cmd interface."""
import json
import signal
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Int8, String
import serial

from motor_srv.pneumatic_data import parse_line


class PressureCmdBridge(Node):
    def __init__(self):
        super().__init__('pressure_cmd_bridge')
        for name, default in [('port', '/dev/ttyACM0'), ('baud', 9600),
                              ('write_newline', True), ('reconnect_period', .5),
                              ('serial_timeout', .2), ('experiment_watchdog', 2.)]:
            self.declare_parameter(name, default)
        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.write_newline = self.get_parameter('write_newline').value
        self.serial_timeout = self.get_parameter('serial_timeout').value
        self.cmd_map = {0: '2', 1: '10', 2: 't', 3: 'release', 4: '4'}
        self.ser = None
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.pending_release = False
        self.heartbeat = None
        self.raw_pub = self.create_publisher(String, 'pressure/raw', 100)
        self.telemetry_pub = self.create_publisher(String, 'pressure/telemetry', 100)
        self.status_pub = self.create_publisher(String, 'pressure/bridge_status', 10)
        self.command_pub = self.create_publisher(String, 'pressure/command_sent', 10)
        self.create_subscription(Int8, 'pressure_cmd', self.cmd_callback, 10)
        self.create_subscription(String, 'pressure/experiment_heartbeat', self.heartbeat_callback, 10)
        self.create_timer(.1, self.watchdog)
        self.serial_thread = threading.Thread(target=self.serial_worker, daemon=True)
        self.serial_thread.start()
        self.get_logger().info(f'Pressure bridge: {self.port} @ {self.baud}; manual mappings unchanged')

    def status(self, event):
        self.status_pub.publish(String(data=event))

    def heartbeat_callback(self, msg):
        with self.lock:
            self.heartbeat = time.monotonic() if msg.data == 'active' else None

    def watchdog(self):
        with self.lock:
            if self.heartbeat is not None and time.monotonic() - self.heartbeat > self.get_parameter('experiment_watchdog').value:
                self.heartbeat = None
                self.pending_release = True
                self.send(3)
                self.status('experiment watchdog expired; release requested')

    def disconnect(self):
        if self.ser:
            try:
                self.ser.close()
            except serial.SerialException:
                pass
        self.ser = None

    def send(self, value):
        # Caller holds lock. A failed release is retried after reconnect.
        if value == 3:
            self.pending_release = True
        if not self.ser or not self.ser.is_open:
            self.status('command failed: serial not ready')
            return
        try:
            command = self.cmd_map[value] + ('\n' if self.write_newline else '')
            encoded = command.encode('ascii')
            if self.ser.write(encoded) != len(encoded):
                raise serial.SerialException('Incomplete serial write')
            if value == 3:
                self.pending_release = False
            self.command_pub.publish(String(data=json.dumps({
                'command': value, 'serial_command': self.cmd_map[value],
                'ros_time_s': self.get_clock().now().nanoseconds / 1e9,
                'monotonic_s': time.monotonic()})))
        except (serial.SerialException, OSError) as exc:
            self.status('command failed: ' + str(exc))
            self.disconnect()

    def cmd_callback(self, msg):
        if msg.data not in self.cmd_map:
            self.get_logger().warn(f'Unknown pressure_cmd value: {msg.data}')
            return
        with self.lock:
            self.send(msg.data)

    def serial_worker(self):
        buffer = b''
        while rclpy.ok() and not self.stop_event.is_set():
            if self.ser is None:
                try:
                    # Linux exclusive access also rejects a second copy of this bridge.
                    connection = serial.Serial(self.port, self.baud, timeout=self.serial_timeout,
                                               write_timeout=.2, exclusive=True)
                    if self.stop_event.wait(2.):
                        connection.close()
                        break
                    with self.lock:
                        self.ser = connection
                        buffer = b''
                        if self.pending_release:
                            self.send(3)
                    self.status('connected')
                except (serial.SerialException, OSError) as exc:
                    self.status('disconnected: ' + str(exc))
                    self.stop_event.wait(self.get_parameter('reconnect_period').value)
                    continue
            try:
                with self.lock:
                    chunk = self.ser.read(self.ser.in_waiting) if self.ser else b''
                buffer += chunk
                if len(buffer) > 16384:
                    buffer = b''
                    self.status('invalid telemetry: serial line exceeds 16384 bytes')
                while b'\n' in buffer:
                    raw, buffer = buffer.split(b'\n', 1)
                    self.publish_line(raw)
                self.stop_event.wait(.01)
            except (serial.SerialException, OSError) as exc:
                with self.lock:
                    self.disconnect()
                self.status('disconnected: ' + str(exc))

    def publish_line(self, raw):
        stamp = self.get_clock().now().nanoseconds / 1e9
        received = time.monotonic()
        line = raw.decode('utf-8', errors='replace').strip()
        self.raw_pub.publish(String(data=line))
        try:
            data = parse_line(line)
            if data is None:
                return
            data.update(valid=True, ros_time_s=stamp, monotonic_s=received, raw=line)
        except ValueError as exc:
            data = dict(valid=False, error=str(exc), ros_time_s=stamp,
                        monotonic_s=received, raw=line)
        self.telemetry_pub.publish(String(data=json.dumps(data, allow_nan=False)))

    def destroy_node(self):
        self.stop_event.set()
        self.serial_thread.join(timeout=3.)
        with self.lock:
            if self.heartbeat is not None:
                self.send(3)
            self.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop = threading.Event()
    previous = {sig: signal.signal(sig, lambda signum, frame: stop.set())
                for sig in (signal.SIGINT, signal.SIGTERM)}
    node = None
    try:
        node = PressureCmdBridge()
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    main()
