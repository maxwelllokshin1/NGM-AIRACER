#!/usr/bin/env python3
"""
Motor test script — interactively set drive speed from the terminal.

Usage:
    sudo -E env PYTHONPATH=$PYTHONPATH LD_LIBRARY_PATH=$LD_LIBRARY_PATH \
        python3 ~/scripts/motor_test.py

Publishes AckermannDriveStamped to /ackermann_cmd (straight to
ackermann_to_vesc_node, bypassing the mux). Steering stays at 0.

Enter speeds in m/s (e.g. "1.0", "-0.5", "0").
Ctrl+C or "q" to exit and stop the motor.

Speed is clamped to ±4.0 m/s (matches max_speed in movement.yaml).
"""

import threading
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

SPEED_MAX = 2.5          # m/s
PUBLISH_TOPIC = '/ackermann_cmd'


class MotorTester(Node):
    def __init__(self):
        super().__init__('motor_tester')
        self.pub = self.create_publisher(AckermannDriveStamped, PUBLISH_TOPIC, 10)
        self.current_speed = 0.0
        # Publish at 20 Hz so the VESC doesn't time out on the topic
        self.timer = self.create_timer(0.05, self._publish)
        self.get_logger().info(f'Publishing to {PUBLISH_TOPIC} at 20 Hz (steering=0)')
        self.get_logger().info(f'Speed range: 0 to {SPEED_MAX} m/s')

    def set_speed(self, speed: float):
        clamped = max(-SPEED_MAX, min(SPEED_MAX, speed))
        if clamped != speed:
            self.get_logger().warn(
                f'Clamped {speed:.2f} → {clamped:.2f} m/s (speed limit)')
        self.current_speed = clamped
        self.get_logger().info(f'Speed set to {clamped:.2f} m/s')

    def stop(self):
        self.current_speed = 0.0
        self._publish()

    def _publish(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = self.current_speed
        msg.drive.steering_angle = 0.0
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = MotorTester()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print(f'\nMotor Test — enter speed in m/s (±{SPEED_MAX} m/s), or "q" to quit.')
    print('Examples: "1.0"  "-0.5"  "0"\n')

    try:
        while True:
            try:
                raw = input('speed (m/s): ').strip()
            except EOFError:
                break

            if raw.lower() in ('q', 'quit', 'exit'):
                break

            if raw == '':
                continue

            try:
                speed = float(raw)
            except ValueError:
                print(f'  Invalid input "{raw}" — enter a number.')
                continue

            node.set_speed(speed)

    except KeyboardInterrupt:
        pass
    finally:
        print('\nStopping motor and shutting down...')
        node.stop()
        import time; time.sleep(0.2)   # let the stop command publish
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
