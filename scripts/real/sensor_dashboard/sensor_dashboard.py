#!/usr/bin/env python3
"""
Sensor dashboard - live lidar + camera viewer, built to run on a Raspberry Pi 3.

    python3 sensor_dashboard.py                  # then open http://<pi-ip>:8090
    python3 sensor_dashboard.py --lidar rplidar:/dev/ttyUSB0 --lidar sick:192.168.0.1

Standalone: no ROS needed. The only hard requirement is Python 3.7+ (stdlib).
Optional extras unlock more sources:

    sudo apt install python3-opencv     USB / V4L2 cameras (and the demo camera)
    sudo apt install python3-serial     every USB / serial lidar
    rpicam-apps (preinstalled on Raspberry Pi OS)   Pi CSI ribbon-cable cameras
    ROS 2 sourced in the shell          adds a "ROS 2 /scan" lidar option

The lidar dropdown lists (drivers live in lidar_drivers.py):
    * --lidar TYPE:ADDRESS entries you name. TYPE is hokuyo, rplidar, ld06 (LD06/LD19/LD14)
      or sick; ADDRESS is a serial port or host[:port]. Add ,baud=N to force a baud rate.
      With no --lidar, a Hokuyo at 192.168.0.10:10940 (as in autonomy.launch.py) is listed.
    * every USB serial port found, once per lidar type that could be on it. Nothing is
      sent to a port until you pick one of its entries (use --serial-exclude for a port
      that belongs to something else, e.g. a motor-controller MCU).
    * ROS 2 topic (only if rclpy imports) - works for any lidar that has a ROS driver,
      and while another program (e.g. urg_node2) already holds the lidar
    * Demo (simulated) - for checking the dashboard with no hardware
    Mounting fix-ups for all of them: --lidar-yaw DEG rotates the plot, --lidar-mirror flips it.

The camera dropdown lists V4L2 devices, Pi CSI cameras and a demo test pattern.

A sensor is only opened while at least one browser is watching it, and is shared
between browsers, so extra viewers cost the Pi nothing extra. Stop watching and
it is closed (and the lidar laser switched off, if this script switched it on).

There is no authentication: anyone on the network can watch the camera. Use
--host 127.0.0.1 if that matters.
"""

from __future__ import annotations

import argparse
import base64
import glob
import json
import math
import os
import queue
import re
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None

from lidar_drivers import build_lidars, shutdown_ros

HERE = os.path.dirname(os.path.abspath(__file__))
HTML_PATH = os.path.join(HERE, 'sensor_dashboard.html')


# ---------------------------------------------------------------------------
# Feeds: run one sensor only while somebody is watching, fan frames out
# ---------------------------------------------------------------------------

class Source:
    """One selectable sensor. run() blocks, calling publish(frame) per frame."""

    id = ''
    label = ''

    def run(self, publish, stop, note):
        raise NotImplementedError


class Feed:
    """Runs a Source in its own thread while it has subscribers."""

    def __init__(self, source):
        self.source = source
        self.lock = threading.Lock()
        self.subs = []
        self.thread = None
        self.stop = None
        self.state = 'idle'   # idle | connecting | streaming | error
        self.msg = ''
        self.fps = 0.0
        self._n = 0
        self._t0 = time.time()

    def subscribe(self):
        q = queue.Queue(maxsize=2)
        with self.lock:
            self.subs.append(q)
            if len(self.subs) == 1:
                self._start()
        return q

    def unsubscribe(self, q):
        with self.lock:
            if q in self.subs:
                self.subs.remove(q)
            if not self.subs and self.stop is not None:
                self.stop.set()
                self.state, self.msg, self.fps = 'idle', '', 0.0

    def shutdown(self):
        with self.lock:
            if self.stop is not None:
                self.stop.set()
            thread = self.thread
        if thread is not None:
            thread.join(timeout=3.0)

    def _start(self):
        old = self.thread
        if old is not None and old.is_alive():
            old.join(timeout=3.0)   # let the previous run release the device
        stop = threading.Event()
        self.stop = stop
        self.state, self.msg, self.fps = 'connecting', '', 0.0
        self._n, self._t0 = 0, time.time()
        self.thread = threading.Thread(target=self._run, args=(stop,), daemon=True)
        self.thread.start()

    def _run(self, stop):
        def publish(item):
            self._publish(item, stop)

        def note(text):
            if not stop.is_set() and self.state == 'connecting':
                self.msg = text

        while not stop.is_set():
            try:
                self.source.run(publish, stop, note)
            except Exception as e:  # keep retrying: cable pulled, device busy, ...
                if stop.is_set():
                    break
                self.state, self.msg, self.fps = 'error', f'{type(e).__name__}: {e}', 0.0
            stop.wait(2.0)

    def _publish(self, item, stop):
        if stop.is_set():
            return
        if isinstance(item, dict):
            item = json.dumps(item, separators=(',', ':')).encode()
        now = time.time()
        self._n += 1
        if now - self._t0 >= 1.0:
            self.fps = self._n / (now - self._t0)
            self._n, self._t0 = 0, now
        if self.state != 'streaming':
            self.state, self.msg = 'streaming', ''
        for q in list(self.subs):
            try:
                q.put_nowait(item)
            except queue.Full:
                try:
                    q.get_nowait()   # drop the stale frame, keep the newest
                except queue.Empty:
                    pass
                try:
                    q.put_nowait(item)
                except queue.Full:
                    pass


class Registry:
    """The sources currently offered in one dropdown, plus a Feed per source."""

    def __init__(self, build):
        self.build = build   # () -> (list[Source], list[str] notes)
        self.lock = threading.Lock()
        self.sources = {}
        self.feeds = {}
        self.notes = []
        self.refresh()

    def refresh(self):
        found, notes = self.build()
        with self.lock:
            merged = {}
            for s in found:
                merged[s.id] = self.sources.get(s.id, s)   # keep the object a Feed already holds
            for sid, feed in self.feeds.items():
                if feed.subs and sid not in merged:        # don't drop something being watched
                    merged[sid] = feed.source
            self.sources = merged
            self.notes = notes

    def feed(self, sid):
        with self.lock:
            src = self.sources.get(sid)
            if src is None:
                return None
            if sid not in self.feeds:
                self.feeds[sid] = Feed(src)
            return self.feeds[sid]

    def listing(self):
        with self.lock:
            return [{'id': s.id, 'label': s.label} for s in self.sources.values()]

    def status(self):
        with self.lock:
            return {sid: {'state': f.state, 'msg': f.msg, 'fps': round(f.fps, 1)}
                    for sid, f in self.feeds.items() if f.state != 'idle'}

    def shutdown(self):
        with self.lock:
            feeds = list(self.feeds.values())
        for f in feeds:
            f.shutdown()


# ---------------------------------------------------------------------------
# Camera: V4L2 via OpenCV, Pi CSI via rpicam-vid, demo pattern
# ---------------------------------------------------------------------------

# Pi-internal V4L2 nodes (video codecs, ISP, CSI receiver) that are not cameras you can open
_NOT_A_CAMERA = ('bcm2835-codec', 'bcm2835-isp', 'rpivid', 'pispbe', 'rp1-', 'unicam', 'rpi-hevc')


class CvCamera(Source):
    def __init__(self, sid, label, device, args):
        self.id, self.label, self.device = sid, label, device
        self.args = args

    def run(self, publish, stop, note):
        a = self.args
        note(f'opening {self.device}')
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        try:
            if not cap.isOpened():
                raise RuntimeError(f'could not open {self.device} (is another program using it?)')
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))   # cheapest for the Pi's USB + CPU
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.cam_width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.cam_height)
            cap.set(cv2.CAP_PROP_FPS, a.cam_fps)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            interval, last, fails = 1.0 / a.cam_fps, 0.0, 0
            while not stop.is_set():
                if not cap.grab():
                    fails += 1
                    if fails > 20:
                        raise RuntimeError('camera stopped delivering frames (unplugged?)')
                    time.sleep(0.05)
                    continue
                fails = 0
                now = time.time()
                if now - last < interval:   # camera runs faster than we need: skip the decode + encode
                    continue
                ok, frame = cap.retrieve()
                if not ok:
                    continue
                last = now
                ok, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), a.cam_quality])
                if ok:
                    publish(buf.tobytes())
        finally:
            cap.release()


class RpiCamera(Source):
    def __init__(self, index, model, args):
        self.id = f'rpicam:{index}'
        self.label = f'Pi camera {index} - {model}'
        self.index, self.args = index, args

    def run(self, publish, stop, note):
        a = self.args
        exe = shutil.which('rpicam-vid') or shutil.which('libcamera-vid')
        if exe is None:
            raise RuntimeError('rpicam-vid / libcamera-vid not found')
        note(f'starting {os.path.basename(exe)}')
        cmd = [exe, '-t', '0', '-n', '--codec', 'mjpeg',
               '--width', str(a.cam_width), '--height', str(a.cam_height),
               '--framerate', str(a.cam_fps), '--quality', str(a.cam_quality),
               '--camera', str(self.index), '-o', '-']
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
        tail = deque(maxlen=4)
        done = threading.Event()

        def drain_stderr():
            for line in proc.stderr:
                tail.append(line.decode(errors='replace').strip())

        def watchdog():   # a blocked read() can't see `stop`, so kill the process to unblock it
            while not done.is_set():
                if stop.wait(0.2):
                    proc.kill()
                    return

        threading.Thread(target=drain_stderr, daemon=True).start()
        threading.Thread(target=watchdog, daemon=True).start()
        buf = bytearray()
        try:
            while not stop.is_set():
                chunk = proc.stdout.read(65536)
                if not chunk:
                    if stop.is_set():
                        return
                    raise RuntimeError('camera process exited: ' + ' | '.join(t for t in tail if t))
                buf += chunk
                while True:   # cut the byte stream into JPEGs at SOI (FFD8) ... EOI (FFD9)
                    start = buf.find(b'\xff\xd8')
                    if start < 0:
                        buf.clear()
                        break
                    end = buf.find(b'\xff\xd9', start + 2)
                    if end < 0:
                        del buf[:start]
                        break
                    publish(bytes(buf[start:end + 2]))
                    del buf[:end + 2]
        finally:
            done.set()
            proc.kill()
            proc.wait()


class DemoCamera(Source):
    id = 'demo'
    label = 'Demo (test pattern)'

    def __init__(self, args):
        self.args = args

    def run(self, publish, stop, note):
        a = self.args
        w, h = a.cam_width, a.cam_height
        t0 = time.time()
        while not stop.is_set():
            t = time.time() - t0
            img = np.zeros((h, w, 3), np.uint8)
            img[:] = np.linspace(30, 90, h, dtype=np.uint8)[:, None, None]
            cv2.circle(img, (int(w / 2 + w / 3 * math.sin(t)), int(h / 2 + h / 4 * math.sin(t * 1.7))),
                       h // 10, (80, 180, 255), -1)
            cv2.putText(img, 'DEMO CAMERA', (16, 36), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            cv2.putText(img, time.strftime('%H:%M:%S'), (16, h - 16), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (255, 255, 255), 2)
            ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), a.cam_quality])
            if ok:
                publish(buf.tobytes())
            stop.wait(1.0 / a.cam_fps)


def list_v4l2():
    out = []
    for path in sorted(glob.glob('/sys/class/video4linux/video*'),
                       key=lambda p: int(re.sub(r'\D', '', os.path.basename(p)) or 0)):
        def read(name, default=''):
            try:
                with open(os.path.join(path, name)) as f:
                    return f.read().strip()
            except OSError:
                return default
        name = read('name')
        if read('index', '0') != '0':   # a webcam's 2nd node is metadata, not video
            continue
        if name.startswith(_NOT_A_CAMERA):
            continue
        out.append(('/dev/' + os.path.basename(path), name or 'camera'))
    return out


def list_rpicam():
    exe = shutil.which('rpicam-hello') or shutil.which('libcamera-hello')
    if exe is None:
        return []
    try:
        out = subprocess.run([exe, '--list-cameras'], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                             universal_newlines=True, timeout=8).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    return [(int(m.group(1)), m.group(2)) for m in re.finditer(r'^\s*(\d+)\s*:\s*(\S+)', out, re.M)]


def build_cameras(args):
    sources, notes = [], []
    v4l2 = list_v4l2() if sys.platform.startswith('linux') else []
    if cv2 is not None:
        for dev, name in v4l2:
            sources.append(CvCamera(f'v4l2:{dev}', f'{name} ({dev})', dev, args))
    elif v4l2:
        notes.append('OpenCV is not installed, so USB cameras are not listed '
                     '(sudo apt install python3-opencv).')
    for index, model in list_rpicam():
        sources.append(RpiCamera(index, model, args))
    if cv2 is not None and not args.no_demo:
        sources.append(DemoCamera(args))
    return sources, notes


# ---------------------------------------------------------------------------
# HTTP
# ---------------------------------------------------------------------------

LIDARS = None
CAMERAS = None


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def do_GET(self):
        url = urlparse(self.path)
        qs = parse_qs(url.query)
        sid = qs.get('id', [''])[0]
        if url.path in ('/', '/index.html'):
            self._html()
        elif url.path == '/api/sources':
            if 'refresh' in qs:
                LIDARS.refresh()
                CAMERAS.refresh()
            self._json(200, {'lidars': LIDARS.listing(), 'cameras': CAMERAS.listing(),
                             'notes': LIDARS.notes + CAMERAS.notes})
        elif url.path == '/api/status':
            self._json(200, {'lidar': LIDARS.status(), 'camera': CAMERAS.status()})
        elif url.path == '/api/lidar/stream':
            self._stream(LIDARS, sid, lambda item: item)
        elif url.path == '/api/camera/stream':
            self._stream(CAMERAS, sid, base64.b64encode)   # JPEG bytes -> text an SSE line can carry
        else:
            self._json(404, {'error': 'not found'})

    def _json(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def _html(self):
        try:
            with open(HTML_PATH, 'rb') as f:   # re-read each time so edits show up on refresh
                body = f.read()
        except FileNotFoundError:
            self._json(500, {'error': 'sensor_dashboard.html not found next to the script'})
            return
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def _stream(self, registry, sid, encode):
        """Server-sent events: one `data:` line per frame, for as long as the browser stays connected."""
        feed = registry.feed(sid)
        if feed is None:
            self._json(404, {'error': f'unknown source {sid!r}'})
            return
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-cache, no-store')
        self.end_headers()
        q = feed.subscribe()
        try:
            while True:
                try:
                    item = q.get(timeout=2.0)
                except queue.Empty:
                    self.wfile.write(b': keep-alive\n\n')   # also how we notice a browser that went away
                    self.wfile.flush()
                    continue
                self.wfile.write(b'data: ' + encode(item) + b'\n\n')
                self.wfile.flush()
        except OSError:   # browser went away
            pass
        finally:
            feed.unsubscribe(q)


def lan_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('10.255.255.255', 1))   # UDP connect sends nothing, it just picks the outgoing interface
        return s.getsockname()[0]
    except OSError:
        return '127.0.0.1'
    finally:
        s.close()


def parse_args(argv=None):
    p = argparse.ArgumentParser(description='Live lidar + camera dashboard for the Raspberry Pi.')
    p.add_argument('--host', default='0.0.0.0', help='interface to bind (default: all)')
    p.add_argument('--port', type=int, default=8090)
    p.add_argument('--lidar', action='append', metavar='TYPE:ADDRESS[,baud=N]',
                   help='a lidar to list; repeatable. TYPE: hokuyo | rplidar | ld06 | sick. ADDRESS: serial '
                        'port or host[:port]. E.g. rplidar:/dev/ttyUSB0  sick:192.168.0.1  '
                        'hokuyo:192.168.0.10:10940. Default: hokuyo:192.168.0.10:10940')
    p.add_argument('--lidar-hz', type=float, default=10.0, help='max lidar frames/sec sent to the browser')
    p.add_argument('--lidar-yaw', type=float, default=0.0, metavar='DEG',
                   help='rotate every lidar plot by this many degrees counter-clockwise (mounting offset)')
    p.add_argument('--lidar-mirror', action='store_true',
                   help='mirror every lidar plot left/right (if a lidar spins the other way to what is assumed)')
    p.add_argument('--serial-exclude', action='append', default=[], metavar='DEV',
                   help='serial port NOT to offer as a lidar; repeatable (e.g. a motor-controller MCU)')
    p.add_argument('--ros-topic', default='/scan')
    p.add_argument('--cam-width', type=int, default=640)
    p.add_argument('--cam-height', type=int, default=480)
    p.add_argument('--cam-fps', type=int, default=15)
    p.add_argument('--cam-quality', type=int, default=70, help='JPEG quality 1-100')
    p.add_argument('--no-demo', action='store_true', help='hide the simulated lidar/camera')
    return p.parse_args(argv)


def main(argv=None):
    global LIDARS, CAMERAS
    args = parse_args(argv)
    LIDARS = Registry(lambda: build_lidars(args))
    CAMERAS = Registry(lambda: build_cameras(args))

    def on_sigterm(signum, frame):
        raise KeyboardInterrupt   # so `systemctl stop` / `kill` still run the cleanup below

    signal.signal(signal.SIGTERM, on_sigterm)
    server = ThreadingHTTPServer((args.host, args.port), Handler)

    print('\n' + '=' * 60)
    print('  SENSOR DASHBOARD LIVE')
    print(f'  http://{lan_ip()}:{args.port}   (this machine: http://localhost:{args.port})')
    print(f'  lidars : {", ".join(s["label"] for s in LIDARS.listing()) or "none found"}')
    print(f'  cameras: {", ".join(s["label"] for s in CAMERAS.listing()) or "none found"}')
    for note in LIDARS.notes + CAMERAS.notes:
        print(f'  note   : {note}')
    print('=' * 60 + '\n')

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        LIDARS.shutdown()   # lets a lidar we switched on switch its laser back off
        CAMERAS.shutdown()
        shutdown_ros()


if __name__ == '__main__':
    main()
