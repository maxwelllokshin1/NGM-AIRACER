#!/usr/bin/env python3
"""
Car dashboard — serves a live HTML telemetry dashboard fed by /car_info_debug.

Usage (inside the container, after sourcing ROS):
    ros2 run follow_the_gap car_dashboard

Same /car_info_debug data source as car_info_monitor.py (Float32MultiArray:
speed, odom_speed, steering, gap_start, gap_end, best), plus /lap_info
(Float32MultiArray: lap_count, current_lap_elapsed, last_lap_time,
best_lap_time) published by reactive_node.py's lap tracker — pushed to a
browser dashboard over Server-Sent Events, opened automatically.

The HTTP server binds 0.0.0.0 and docker-compose.yml uses network_mode:
host, so http://localhost:<PORT> is reachable from the Windows host
browser too — open it there directly if no in-container browser is
available.
"""

import json
import os
import queue
import shutil
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None

PORT = 8080


def find_dashboard_html():
    """Look in the installed share dir first, then fall back to the source tree."""
    if get_package_share_directory is not None:
        try:
            share = get_package_share_directory('follow_the_gap')
            candidate = os.path.join(share, 'dashboard', 'car_dashboard.html')
            if os.path.exists(candidate):
                return candidate
        except Exception:
            pass
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, '..', 'dashboard', 'car_dashboard.html')


HTML_PATH = find_dashboard_html()


class DashboardState:
    """Shared between the ROS callback thread and HTTP server threads."""

    def __init__(self):
        self.lock = threading.Lock()
        self.clients = []
        self.latest = {
            'speed': 0.0, 'odom_speed': 0.0, 'steering': 0.0,
            'gap_start': 0.0, 'gap_end': 0.0, 'best': 0.0, 't': 0.0,
            'lap_count': 0.0, 'lap_current': 0.0, 'lap_last': 0.0, 'lap_best': 0.0,
            'lidar_angle_min': 0.0, 'lidar_angle_inc': 0.0, 'pred_steering': 0.0,
            'pred_speed': 0.0, 'wheelbase': 0.25, 'max_lidar_range': 3.0, 'lidar_ranges': [],
        }

    def update(self, partial):
        # merge rather than replace — /car_info_debug and /lap_info are
        # two separate topics/callbacks, each only knows its own fields
        with self.lock:
            self.latest = {**self.latest, **partial}
            snapshot = dict(self.latest)
            for q in self.clients:
                q.put(snapshot)

    def register(self):
        q = queue.Queue()
        with self.lock:
            self.clients.append(q)
        return q

    def unregister(self, q):
        with self.lock:
            if q in self.clients:
                self.clients.remove(q)


state = DashboardState()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # keep terminal quiet, GET /stream logs on every heartbeat otherwise

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            self._serve_html()
        elif self.path == '/stream':
            self._serve_stream()
        else:
            self.send_response(404)
            self.end_headers()

    def _serve_html(self):
        try:
            with open(HTML_PATH, 'rb') as f:
                body = f.read()
        except FileNotFoundError:
            self.send_response(500)
            self.end_headers()
            self.wfile.write(b'car_dashboard.html not found at ' + HTML_PATH.encode())
            return
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_stream(self):
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Connection', 'keep-alive')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

        q = state.register()
        try:
            self.wfile.write(f'data: {json.dumps(state.latest)}\n\n'.encode())
            self.wfile.flush()
            while True:
                try:
                    data = q.get(timeout=15)
                    self.wfile.write(f'data: {json.dumps(data)}\n\n'.encode())
                    self.wfile.flush()
                except queue.Empty:
                    self.wfile.write(b': keep-alive\n\n')
                    self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            state.unregister(q)


class CarDashboard(Node):
    def __init__(self):
        super().__init__('car_dashboard')
        self.sub = self.create_subscription(Float32MultiArray, '/car_info_debug', self.debug_callback, 10)
        self.lap_sub = self.create_subscription(Float32MultiArray, '/lap_info', self.lap_callback, 10)
        self.lidar_view_sub = self.create_subscription(Float32MultiArray, '/lidar_view', self.lidar_view_callback, 10)
        self.get_logger().info('DASHBOARD INITIALIZED...')

    def debug_callback(self, msg):
        speed, odom_speed, steering, gap_start, gap_end, best = msg.data
        state.update({
            'speed': speed,
            'odom_speed': odom_speed,
            'steering': steering,
            'gap_start': gap_start,
            'gap_end': gap_end,
            'best': best,
            't': time.time(),
        })

    def lap_callback(self, msg):
        lap_count, lap_current, lap_last, lap_best = msg.data
        state.update({
            'lap_count': lap_count,
            'lap_current': lap_current,
            'lap_last': lap_last,
            'lap_best': lap_best,
        })

    def lidar_view_callback(self, msg):
        # [angle_min, angle_increment, steering, speed, wheelbase, max_lidar_range, *ranges]
        header = msg.data[:6]
        ranges = list(msg.data[6:])
        angle_min, angle_inc, steering, speed, wheelbase, max_range = header
        state.update({
            'lidar_angle_min': angle_min,
            'lidar_angle_inc': angle_inc,
            'pred_steering': steering,
            'pred_speed': speed,
            'wheelbase': wheelbase,
            'max_lidar_range': max_range,
            'lidar_ranges': ranges,
        })


def start_http_server():
    server = ThreadingHTTPServer(('0.0.0.0', PORT), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


def open_dashboard(url):
    """
    Launch a browser pointed at url without blocking. The stdlib webbrowser
    module falls back to GenericBrowser for unregistered command names like
    'epiphany-browser', which calls Popen(...).wait() and blocks until the
    browser window is closed — which would stall this whole script (the ROS
    node never gets constructed) until the user closes it. Spawn detached
    via Popen directly instead.
    """
    os.environ.setdefault('DISPLAY', ':99')
    for browser in ('epiphany-browser', 'epiphany', 'chromium', 'firefox', 'xdg-open'):
        path = shutil.which(browser)
        if not path:
            continue
        try:
            subprocess.Popen(
                [path, url],
                start_new_session=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            return True
        except OSError:
            continue
    return False


def main(args=None):
    rclpy.init(args=args)

    start_http_server()
    url = f'http://localhost:{PORT}'

    print('\n' + '=' * 60)
    print('  CAR DASHBOARD LIVE')
    print(f'  {url}')
    print('  (network_mode: host — also reachable from the Windows')
    print('   host browser at the same URL)')
    print('=' * 60 + '\n')

    if not open_dashboard(url):
        print('No in-container browser found — open the URL above manually.\n')

    node = CarDashboard()
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
