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

Also serves a live parameter tuner (Tuner tab): GET /params reads
reactive_node's current parameter values over its standard ROS2
parameter service, POST /set_param writes one back live (no restart
needed), and POST /restart repositions the car at its configured spawn
pose (via /initialpose, same topic RViz's "2D Pose Estimate" uses) and
clears reactive_node's own stuck/lap/recovery state.

The HTTP server binds 0.0.0.0 and docker-compose.yml uses network_mode:
host, so http://localhost:<PORT> is reachable from the Windows host
browser too — open it there directly if no in-container browser is
available.
"""

import json
import math
import os
import queue
import shutil
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Empty as EmptyMsg
from geometry_msgs.msg import PoseWithCovarianceStamped
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters, GetParameters
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None

PORT = 8080

# Single source of truth for the Tuner tab — GET /params reads current
# values from reactive_node itself (not from this dict), so this only
# needs to say each param's ROS type and a human label. Keep names in
# sync with reactive_node.py's declare_parameter() calls / gap_params.yaml.
TUNABLE_PARAMS = {
    'bubble_radius': {
        'type': 'double', 'label': 'Bubble Radius (m)', 'default': 0.3,
        'desc': "Safety zone around the closest obstacle the lidar sees — the car won't aim through it. Higher = wider berth, more cautious. Lower = cuts closer, tighter lines.",
    },
    'preprocess_conv_size': {
        'type': 'int', 'label': 'Preprocess Conv Size', 'default': 3,
        'desc': 'How many lidar readings get averaged together to smooth out sensor noise. Higher = smoother but blurs out small gaps.',
    },
    'max_lidar_range': {
        'type': 'double', 'label': 'Max Lidar Range (m)', 'default': 3.5,
        'desc': 'How far the car "pretends" it can see — readings beyond this get capped. Higher = reacts sooner. Lower = only reacts to close stuff, more panicky.',
    },
    'speed_max': {
        'type': 'double', 'label': 'Max Speed (m/s)', 'default': 15.0,
        'desc': "Hard ceiling on speed — not a cruising speed, just the max it's ever allowed to hit.",
    },
    'steering_gain': {
        'type': 'double', 'label': 'Steering Gain', 'default': 0.38,
        'desc': 'How hard the car turns toward its target point. Higher = sharper/snappier (can overshoot). Lower = lazier, might not turn enough.',
    },
    'max_steering': {
        'type': 'double', 'label': 'Max Steering (deg)', 'default': 30.0,
        'desc': 'The steepest angle the car is ever allowed to steer, in either direction.',
    },
    'alpha_straight': {
        'type': 'double', 'label': 'Alpha — Straight', 'default': 0.6,
        'desc': 'Steering smoothing used when aiming nearly straight. Higher (toward 1) = very stable, ignores tiny wiggles. Lower = twitchier even on straights.',
    },
    'alpha_curve': {
        'type': 'double', 'label': 'Alpha — Curve', 'default': 0.6,
        'desc': 'Steering smoothing used when aiming sharply (a real curve). Higher = smoother but laggier reaction to the turn. Lower (toward 0) = reacts almost instantly, but jitterier.',
    },
    'fov_deg': {
        'type': 'double', 'label': 'Field of View (deg/side)', 'default': 70.0,
        'desc': "How far to EACH side the car looks. Higher = can see further around bends (may start turning early). Lower = can't see around bends as early, less premature turning.",
    },
    'gap_aim_aggressiveness': {
        'type': 'double', 'label': 'Gap Aim Aggressiveness', 'default': 0.5,
        'desc': 'How much the car aims at the farthest visible point vs. the immediate gap center. 0 = always centered/cautious. 1 = aim far — can cut corners early.',
    },
    'straight_min_forward_frac': {
        'type': 'double', 'label': 'Straight Min Forward Frac', 'default': 0.75,
        'desc': '% of Max Lidar Range that must be clear straight ahead to call this a straightaway and speed up. Higher = stricter, needs a wide-open track.',
    },
    'straight_max_angle_deg': {
        'type': 'double', 'label': 'Straight Max Angle (deg)', 'default': 6.0,
        'desc': "Steering has to stay under this angle to count as \"driving straight.\" Higher = more forgiving of wobble, speeds up more easily.",
    },
    'straight_ema_alpha': {
        'type': 'double', 'label': 'Straight EMA Alpha', 'default': 0.2,
        'desc': 'How fast the straight-vs-curve detector updates its mind. Higher = notices changes faster. Lower = slower/steadier assessment.',
    },
    'lap_leave_dist': {
        'type': 'double', 'label': 'Lap Leave Dist (m)', 'default': 3.0,
        'desc': "How far the car must get from the start point before a lap can even start counting — stops it counting a lap the instant it moves an inch.",
    },
    'lap_return_dist': {
        'type': 'double', 'label': 'Lap Return Dist (m)', 'default': 1.5,
        'desc': 'How close the car has to get back to the start point to count as "lap complete." Higher = looser finish line.',
    },
    'lap_min_time': {
        'type': 'double', 'label': 'Lap Min Time (s)', 'default': 5.0,
        'desc': 'Minimum seconds between lap counts, purely to stop double-counting the same crossing from position jitter.',
    },
    'recovery_straight_time': {
        'type': 'double', 'label': 'Recovery Straight Time (s)', 'default': 0.8,
        'desc': 'When stuck, how long the car reverses in a straight line before it starts steering away from the wall.',
    },
    'recovery_clear_dist': {
        'type': 'double', 'label': 'Recovery Clear Dist (m)', 'default': 1.0,
        'desc': "Space needed on BOTH sides before the car decides it's free and resumes driving. Higher = more patient/cautious before continuing.",
    },
    'recovery_max_time': {
        'type': 'double', 'label': 'Recovery Max Time (s)', 'default': 6.0,
        'desc': 'Safety cutoff — after this many seconds of reversing, the car gives up looking for "fully clear" and just resumes anyway, so it can never back up forever.',
    },
    'wheelbase': {
        'type': 'double', 'label': 'Wheelbase (m)', 'default': 0.25,
        'desc': "The car's real front-to-rear axle distance. A hardware spec, not really a tuning knob — only used to draw the dashboard's predicted-path curve.",
    },
    'lidar_view_every_n': {
        'type': 'int', 'label': 'Lidar View Every N Scans', 'default': 10,
        'desc': "How often the dashboard's lidar-view picture refreshes. Doesn't affect driving at all — display only.",
    },
}

# Set once CarDashboard() is constructed in main() — Handler is instantiated
# per-request by ThreadingHTTPServer with no other way to reach the node.
dashboard_node = None


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
        elif self.path == '/params':
            self._serve_params()
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        if self.path == '/set_param':
            self._handle_set_param()
        elif self.path == '/restart':
            self._handle_restart()
        else:
            self.send_response(404)
            self.end_headers()

    def _json_response(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(body)

    def _read_json_body(self):
        length = int(self.headers.get('Content-Length', 0) or 0)
        raw = self.rfile.read(length) if length > 0 else b'{}'
        return json.loads(raw or b'{}')

    def _serve_params(self):
        names = list(TUNABLE_PARAMS.keys())
        values = dashboard_node.get_reactive_params(names) if dashboard_node else None
        out = {}
        for name, meta in TUNABLE_PARAMS.items():
            live_value = (values or {}).get(name)
            out[name] = {
                'label': meta['label'],
                'desc': meta.get('desc', ''),
                'type': meta['type'],
                'default': meta.get('default'),
                # fall back to the known default if reactive_node isn't
                # reachable yet, so the box still starts with something
                # sensible instead of empty
                'value': live_value if live_value is not None else meta.get('default'),
            }
        self._json_response(200, out)

    def _handle_set_param(self):
        try:
            payload = self._read_json_body()
            name = payload['name']
            value = payload['value']
        except Exception:
            self._json_response(400, {'ok': False, 'reason': 'bad request body'})
            return
        meta = TUNABLE_PARAMS.get(name)
        if meta is None:
            self._json_response(400, {'ok': False, 'reason': f'unknown param: {name}'})
            return
        if dashboard_node is None:
            self._json_response(503, {'ok': False, 'reason': 'dashboard node not ready'})
            return
        ok, reason = dashboard_node.set_reactive_param(name, value, meta['type'])
        self._json_response(200 if ok else 500, {'ok': ok, 'reason': reason})

    def _handle_restart(self):
        if dashboard_node is None:
            self._json_response(503, {'ok': False, 'reason': 'dashboard node not ready'})
            return
        ok, reason = dashboard_node.do_restart()
        self._json_response(200 if ok else 500, {'ok': ok, 'reason': reason})

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

        # live parameter tuning — talks to reactive_node's own standard
        # ROS2 parameter services, no custom IPC needed
        self.get_params_client = self.create_client(GetParameters, '/reactive_node/get_parameters')
        self.set_params_client = self.create_client(SetParameters, '/reactive_node/set_parameters')
        self.bridge_get_params_client = self.create_client(GetParameters, '/bridge/get_parameters')

        # restart button — reposition via /initialpose (gym_bridge listens
        # for this, it's the same topic RViz's "2D Pose Estimate" uses),
        # then tell reactive_node to clear its own stuck/lap/recovery state
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.reactive_reset_pub = self.create_publisher(EmptyMsg, '/reactive_reset', 10)

        self.get_logger().info('DASHBOARD INITIALIZED...')

    def _call_service_sync(self, client, request, timeout=2.0):
        """Bridge a ROS2 service call to a blocking HTTP handler thread.

        call_async() is safe to invoke from any thread — it just hands the
        request to the middleware and returns a Future. The response is
        delivered on whichever thread is spinning this node (main thread,
        via rclpy.spin()), which fires add_done_callback() and sets the
        Event this thread is waiting on.
        """
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(request)
        done = threading.Event()
        holder = {}

        def _on_done(f):
            holder['result'] = f.result()
            done.set()

        future.add_done_callback(_on_done)
        if not done.wait(timeout):
            return None
        return holder.get('result')

    def get_reactive_params(self, names):
        req = GetParameters.Request()
        req.names = names
        resp = self._call_service_sync(self.get_params_client, req)
        if resp is None:
            return None
        out = {}
        for name, pv in zip(names, resp.values):
            if pv.type == ParameterType.PARAMETER_DOUBLE:
                out[name] = pv.double_value
            elif pv.type == ParameterType.PARAMETER_INTEGER:
                out[name] = pv.integer_value
            elif pv.type == ParameterType.PARAMETER_BOOL:
                out[name] = pv.bool_value
        return out

    def set_reactive_param(self, name, value, ptype):
        p = Parameter()
        p.name = name
        pv = ParameterValue()
        if ptype == 'int':
            pv.type = ParameterType.PARAMETER_INTEGER
            pv.integer_value = int(value)
        else:
            pv.type = ParameterType.PARAMETER_DOUBLE
            pv.double_value = float(value)
        p.value = pv

        req = SetParameters.Request()
        req.parameters = [p]
        resp = self._call_service_sync(self.set_params_client, req)
        if resp is None:
            return False, 'reactive_node did not respond (is it running?)'
        if not resp.results:
            return False, 'no result returned'
        result = resp.results[0]
        return result.successful, result.reason

    def do_restart(self):
        req = GetParameters.Request()
        req.names = ['sx', 'sy', 'stheta']
        resp = self._call_service_sync(self.bridge_get_params_client, req)
        if resp is None:
            return False, 'could not read start pose from the gym bridge (/bridge node)'
        sx, sy, stheta = (v.double_value for v in resp.values)

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.pose.position.x = sx
        pose_msg.pose.pose.position.y = sy
        pose_msg.pose.pose.orientation.z = math.sin(stheta / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(stheta / 2.0)
        self.initialpose_pub.publish(pose_msg)

        self.reactive_reset_pub.publish(EmptyMsg())
        return True, f'restarted at ({sx:.2f}, {sy:.2f})'

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
    global dashboard_node
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
    dashboard_node = node
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
