#!/usr/bin/env python3
"""
Dead man's switch — toggle autonomy on/off with the spacebar.

Usage (inside the container, after sourcing ROS):
    sudo -E env PYTHONPATH=$PYTHONPATH LD_LIBRARY_PATH=$LD_LIBRARY_PATH \
        python3 ~/scripts/deadmans_switch.py

How it works:
    ACTIVE   → does nothing. Autonomy runs normally via /drive_nav → /drive.
    PAUSED   → publishes zero speed to /teleop at 20 Hz.
               The ackermann_mux gives /teleop priority 100 vs /drive priority 10,
               so the zero command wins and the car stops instantly.
               Releasing (pressing space again) stops publishing — the mux times
               out /teleop after 0.2 s and falls back to autonomy automatically.

Press SPACE to toggle. Press Q to quit.
"""

import sys
import tty
import termios
import threading
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

PUBLISH_TOPIC = '/teleop'
PUBLISH_HZ = 20


class DeadmansSwitch(Node):
    def __init__(self):
        super().__init__('deadmans_switch')
        self.pub = self.create_publisher(AckermannDriveStamped, PUBLISH_TOPIC, 10)
        self.paused = False
        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self._publish)

    def toggle(self):
        self.paused = not self.paused
        state = 'PAUSED  — car stopped' if self.paused else 'ACTIVE  — autonomy running'
        self.get_logger().info(f'[DEADMAN] {"=" * 10} {state} {"=" * 10}')
        print(f'\n  >>> {"PAUSED  — car stopped" if self.paused else "ACTIVE  — autonomy running"} <<<\n')

    def _publish(self):
        if not self.paused:
            return
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        self.pub.publish(msg)


def get_key():
    """Read a single keypress without needing Enter."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def main():
    rclpy.init()
    node = DeadmansSwitch()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print('\n' + '=' * 50)
    print('  DEAD MAN\'S SWITCH')
    print('  SPACE  →  toggle autonomy on/off')
    print('  Q      →  quit')
    print('=' * 50)
    print('\n  Status: ACTIVE — autonomy running\n')

    try:
        while True:
            key = get_key()
            if key == ' ':
                node.toggle()
            elif key in ('q', 'Q', '\x03'):  # q, Q, or Ctrl+C
                break
    except Exception:
        pass
    finally:
        print('\nShutting down — re-enabling autonomy...')
        node.paused = False   # stop publishing zeros so mux falls through
        import time; time.sleep(0.3)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
