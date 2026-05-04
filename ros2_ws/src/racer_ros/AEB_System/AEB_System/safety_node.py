#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class SafetyNode(Node):
    """
    Emergency braking node using iTTC (inverse Time-To-Collision).
    """
    def __init__(self):
        super().__init__('AEB_System')

        self.declare_parameter('ttc_threshold', 2.0)
        self.declare_parameter('speed_threshold', 0.5)

        self.ttc_threshold = self.get_parameter('ttc_threshold').value
        self.speed_threshold = self.get_parameter('speed_threshold').value

        self.add_on_set_parameters_callback(self._on_parameters_changed)

        self.speed = 0.0
        self.last_odom_time = None           # detect stale odom
        self.latest_nav_cmd = AckermannDriveStamped()

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.nav_sub = self.create_subscription(
            AckermannDriveStamped, '/drive_nav', self.nav_callback, 10)

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)

        # Log speed at 10 Hz regardless of scan rate
        self.create_timer(0.1, self._speed_log_timer)

        self.get_logger().info('=' * 50)
        self.get_logger().info('Safety Node initialized')
        self.get_logger().info(f'TTC Threshold: {self.ttc_threshold} seconds')
        self.get_logger().info(f'Speed Threshold: {self.speed_threshold} m/s')
        self.get_logger().info('=' * 50)

    def _on_parameters_changed(self, params):
        self.ttc_threshold = self.get_parameter('ttc_threshold').value
        self.speed_threshold = self.get_parameter('speed_threshold').value
        return SetParametersResult(successful=True)

    def _ts(self):
        """Human-readable timestamp: HH:MM:SS.ms"""
        now = self.get_clock().now().nanoseconds
        total_s = now / 1e9
        h = int(total_s // 3600) % 24
        m = int(total_s % 3600) // 60
        s = total_s % 60
        return f'{h:02d}:{m:02d}:{s:06.3f}'

    def _speed_log_timer(self):
        """Periodic speed log at 10 Hz so speed is always visible."""
        odom_age = None
        if self.last_odom_time is not None:
            odom_age = (self.get_clock().now() - self.last_odom_time).nanoseconds / 1e9

        if odom_age is None or odom_age > 0.5:
            # Odom is stale — fall back to commanded speed
            fallback = self.latest_nav_cmd.drive.speed
            self.get_logger().warn(
                f'[{self._ts()}] ODOM STALE ({odom_age:.1f}s) | '
                f'Using commanded speed={fallback:.2f} m/s as fallback',
                throttle_duration_sec=1.0)
        else:
            self.get_logger().info(
                f'[{self._ts()}] SPEED | odom={self.speed:.3f} m/s | '
                f'cmd={self.latest_nav_cmd.drive.speed:.2f} m/s')

    def _effective_speed(self):
        """Use odom speed if fresh, else fall back to commanded speed."""
        if self.last_odom_time is None:
            return self.latest_nav_cmd.drive.speed
        age = (self.get_clock().now() - self.last_odom_time).nanoseconds / 1e9
        if age > 0.5:
            return self.latest_nav_cmd.drive.speed
        return self.speed

    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x
        self.last_odom_time = self.get_clock().now()

    def nav_callback(self, drive_msg):
        self.latest_nav_cmd = drive_msg

    def scan_callback(self, scan_msg):
        effective_speed = self._effective_speed()

        if abs(effective_speed) < self.speed_threshold:
            self.drive_pub.publish(self.latest_nav_cmd)
            return

        ranges = np.array(scan_msg.ranges)
        num_beams = len(ranges)
        angles = scan_msg.angle_min + np.arange(num_beams) * scan_msg.angle_increment

        # Front: ±30° (wider than before — was ±10° which missed angled approaches)
        front_beams = np.abs(angles) < np.radians(30.0)

        # Side: 30°–90° — now covers the actual walls the car follows (was 15°–60°)
        left_beams  = (angles >  np.radians(30.0)) & (angles <  np.radians(90.0))
        right_beams = (angles < -np.radians(30.0)) & (angles > -np.radians(90.0))
        side_beams  = left_beams | right_beams

        # iTTC calculation using effective speed
        range_rates = -effective_speed * np.cos(angles)
        ittc = np.full(num_beams, np.inf)
        approaching = range_rates < 0
        valid_ranges = (ranges >= scan_msg.range_min) & (ranges <= scan_msg.range_max)
        mask = approaching & valid_ranges
        ittc[mask] = ranges[mask] / np.abs(range_rates[mask])
        ittc = np.nan_to_num(ittc, nan=np.inf, posinf=np.inf, neginf=np.inf)

        min_front_ittc = np.min(ittc[front_beams]) if front_beams.any() else np.inf
        min_left_ittc  = np.min(ittc[left_beams])  if left_beams.any()  else np.inf
        min_right_ittc = np.min(ittc[right_beams]) if right_beams.any() else np.inf
        min_side_ittc  = min(min_left_ittc, min_right_ittc)

        threat_side = 'LEFT' if min_left_ittc < min_right_ittc else 'RIGHT'

        # ── Status line every scan ──────────────────────────────────────────
        self.get_logger().info(
            f'[{self._ts()}] AEB | speed={effective_speed:.2f} m/s | '
            f'front_iTTC={min_front_ittc:.2f}s | '
            f'left_iTTC={min_left_ittc:.2f}s | right_iTTC={min_right_ittc:.2f}s | '
            f'threshold={self.ttc_threshold:.2f}s',
            throttle_duration_sec=0.25)

        # ── Warnings ────────────────────────────────────────────────────────
        if min_front_ittc < self.ttc_threshold:
            front_ittc_copy = ittc.copy()
            front_ittc_copy[~front_beams] = np.inf
            min_idx = np.argmin(front_ittc_copy)
            self.get_logger().warn(
                f'[{self._ts()}] *** FRONT COLLISION *** | '
                f'iTTC={min_front_ittc:.3f}s | '
                f'range={ranges[min_idx]:.2f}m | angle={np.degrees(angles[min_idx]):.1f}°')

        side_threshold = self.ttc_threshold * 0.5
        if min_side_ittc < side_threshold:
            side_ittc_copy = ittc.copy()
            side_ittc_copy[~side_beams] = np.inf
            min_idx = np.argmin(side_ittc_copy)
            self.get_logger().warn(
                f'[{self._ts()}] *** {threat_side} SIDE COLLISION *** | '
                f'iTTC={min_side_ittc:.3f}s | '
                f'range={ranges[min_idx]:.2f}m | angle={np.degrees(angles[min_idx]):.1f}°')

        # ── Action ──────────────────────────────────────────────────────────
        if min_front_ittc < self.ttc_threshold:
            self.publish_brake(min_front_ittc, self.ttc_threshold,
                               emergency=True, direction='FRONT')
        elif min_side_ittc < side_threshold:
            self.publish_brake(min_side_ittc, side_threshold,
                               emergency=False, direction=threat_side)
        else:
            self.drive_pub.publish(self.latest_nav_cmd)

    def publish_brake(self, min_ittc, threshold, emergency=True, direction='FRONT'):
        brake_msg = AckermannDriveStamped()
        brake_msg.header.stamp = self.get_clock().now().to_msg()
        brake_msg.header.frame_id = 'base_link'

        if emergency:
            brake_msg.drive.speed = 0.0
            brake_msg.drive.steering_angle = 0.0
            self.get_logger().error(
                f'[{self._ts()}] !!! EMERGENCY STOP ({direction}) !!! | '
                f'speed was: {self._effective_speed():.2f} m/s | iTTC={min_ittc:.3f}s')
        else:
            closeness = 1.0 - np.clip(min_ittc / threshold, 0.0, 1.0)
            brake_speed = max(self._effective_speed() * (1.0 - closeness * 0.5), 0.0)
            brake_msg.drive.speed = brake_speed
            self.get_logger().warn(
                f'[{self._ts()}] {direction} SIDE BRAKE | '
                f'{self._effective_speed():.2f} → {brake_speed:.2f} m/s | '
                f'iTTC={min_ittc:.3f}s')

        self.drive_pub.publish(brake_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
