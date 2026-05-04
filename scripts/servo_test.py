#!/usr/bin/env python3
"""
Servo test script — interactively set steering angle from the terminal.

Usage:
    python3 servo_test.py

The script publishes AckermannDriveStamped to /ackermann_cmd (the topic
ackermann_to_vesc_node subscribes to), bypassing the mux entirely.
Speed is always 0 — only the servo moves.

Enter angles in degrees (e.g. "10", "-5", "0").
Ctrl+C to exit and center the servo.

Physical limits:
    max =  15° (full left)
    min =  -5° (full right)
    center = 5° (mechanical zero is not electrical zero)
"""

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

STEER_CENTER_DEG = 1.0  # mechanical center
PUBLISH_TOPIC = '/ackermann_cmd'


class ServoTester(Node):
    def __init__(self):
        super().__init__('servo_tester')
        self.pub = self.create_publisher(AckermannDriveStamped, PUBLISH_TOPIC, 10)
        # Publish at 20 Hz so the VESC doesn't time out on the topic
        self.current_angle_rad = 0.0
        self.timer = self.create_timer(0.05, self._publish)
        self.get_logger().info(f'Publishing to {PUBLISH_TOPIC} at 20 Hz (speed=0)')
        self.get_logger().info(f'Center: {STEER_CENTER_DEG}° — no clamping, VESC enforces limits')

    def set_angle_deg(self, deg: float):
        import math
        self.current_angle_rad = math.radians(deg)
        self.get_logger().info(f'Steering set to {deg:.1f}°')

    def center(self):
        import math
        self.current_angle_rad = math.radians(STEER_CENTER_DEG)
        self._publish()

    def _publish(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.steering_angle = self.current_angle_rad
        msg.drive.speed = 0.0
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ServoTester()

    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print(f'\nServo Test — enter angle in degrees (center={STEER_CENTER_DEG}°), or "q" to quit.')
    print('Examples: "5"  "-9"  "-3"\n')

    try:
        while True:
            try:
                raw = input('angle (deg): ').strip()
            except EOFError:
                break

            if raw.lower() in ('q', 'quit', 'exit'):
                break

            if raw == '':
                continue

            try:
                deg = float(raw)
            except ValueError:
                print(f'  Invalid input "{raw}" — enter a number.')
                continue

            node.set_angle_deg(deg)

    except KeyboardInterrupt:
        pass
    finally:
        print('\nCentering servo and shutting down...')
        node.center()
        import time; time.sleep(0.2)   # let the center command publish
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
