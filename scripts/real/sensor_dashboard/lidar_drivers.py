"""
Lidar drivers for sensor_dashboard.py.

Every driver is a class with an `id`, a `label` and a blocking `run(publish, stop, note)`
that calls `publish(frame)` for each scan. A frame is a dict:

    a0    angle of r[0] in radians. 0 = straight ahead, positive = counter-clockwise (to the left)
    da    radians between consecutive samples
    r     ranges in millimetres, 0 = no return
    dmin  smallest trustworthy range in mm
    dmax  largest range in mm

Built in:

    hokuyo   Hokuyo URG / UST / UTM (SCIP 2.0) - Ethernet or USB serial
    rplidar  Slamtec RPLIDAR A1 / A2 / A3 / S1 (standard scan) - serial, baud auto-probed
    ld06     LDROBOT LD06 / LD19 / LD14 - serial, 230400 baud, streams by itself
    sick     SICK TiM / LMS, CoLa-A over Ethernet (port 2111)
    ros      any lidar that already publishes a ROS 2 sensor_msgs/LaserScan

Anything else (YDLIDAR, ...): run its ROS driver and pick the ROS entry, or add a class here -
copy LdRobotLidar (streaming) or SickLidar (poll/response), then register it in DRIVERS.

Written from the vendors' published protocol documents; only the Hokuyo/RPLIDAR/LDROBOT/SICK
byte formats are exercised by tests (against simulated devices), none against real hardware.
"""

from __future__ import annotations

import glob
import math
import os
import random
import re
import socket
import struct
import sys
import time

try:
    import serial
except ImportError:
    serial = None

try:
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
except ImportError:
    rclpy = None


# ---------------------------------------------------------------------------
# Transports: a serial port or a TCP socket, same interface
# ---------------------------------------------------------------------------

class TcpTransport:
    def __init__(self, host, port):
        self.sock = socket.create_connection((host, port), timeout=3.0)
        self.sock.settimeout(1.0)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def write(self, data):
        self.sock.sendall(data)

    def read(self):
        """Whatever arrived (up to 1 s of waiting); b'' on timeout."""
        try:
            data = self.sock.recv(4096)
        except socket.timeout:
            return b''
        if not data:
            raise ConnectionError('lidar closed the connection')
        return data

    def reset_input(self):
        """Throw away what's buffered. Time-boxed: a device that streams constantly never goes quiet."""
        self.sock.settimeout(0.05)
        end = time.time() + 0.2
        try:
            while time.time() < end and self.sock.recv(4096):
                pass
        except (socket.timeout, BlockingIOError):
            pass
        finally:
            self.sock.settimeout(1.0)

    def set_dtr(self, on):
        pass

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


class SerialTransport:
    def __init__(self, path, baud):
        if serial is None:
            raise RuntimeError('pyserial is not installed (sudo apt install python3-serial)')
        self.ser = serial.Serial(path, baud, timeout=1.0)

    def write(self, data):
        self.ser.write(data)

    def read(self):
        return self.ser.read(max(1, self.ser.in_waiting))

    def reset_input(self):
        self.ser.reset_input_buffer()

    def set_dtr(self, on):
        self.ser.dtr = on

    def close(self):
        try:
            self.ser.close()
        except OSError:
            pass


def is_serial_address(address):
    return address.startswith('/dev/') or re.match(r'^(COM\d+|tty\w+)$', address, re.I) is not None


def open_transport(address, baud, default_port=None):
    """`/dev/ttyUSB0` opens a serial port; `host[:port]` opens TCP (also handy with ser2net)."""
    if is_serial_address(address):
        return SerialTransport(address, baud)
    host, _, port = address.partition(':')
    if not port and default_port is None:
        raise ValueError(f'{address!r} needs a port (host:port)')
    return TcpTransport(host, int(port or default_port))


# ---------------------------------------------------------------------------
# Common pieces
# ---------------------------------------------------------------------------

class LidarSource:
    """Base class: label/id, the shared --lidar-* options, and frame construction."""

    id = ''
    label = ''

    def __init__(self, args):
        self.hz = args.lidar_hz
        self.yaw = math.radians(args.lidar_yaw)
        self.mirror = args.lidar_mirror

    def frame(self, a0, da, ranges, dmin, dmax):
        """Apply --lidar-mirror / --lidar-yaw so every driver gets the same mounting fix-ups."""
        if self.mirror:
            a0 = -(a0 + (len(ranges) - 1) * da)
            ranges = ranges[::-1]
        return {'a0': a0 + self.yaw, 'da': da, 'r': ranges, 'dmin': dmin, 'dmax': dmax}

    def run(self, publish, stop, note):
        raise NotImplementedError


class ScanAssembler:
    """For lidars that stream individual (angle, distance) points: keep the newest reading per
    angular bin and hand out a uniform 360-degree frame on request. Bins not refreshed within
    max_age seconds (an object moved, the lidar lost a return) read as 'no return'."""

    def __init__(self, bins, clockwise, max_age=0.5):
        self.n = bins
        self.step = 360.0 / bins
        self.cw = clockwise
        self.max_age = max_age
        self.r = [0] * bins
        self.t = [0.0] * bins

    def add(self, angle_deg, dist_mm, now):
        theta = -angle_deg if self.cw else angle_deg          # everything downstream is counter-clockwise
        i = int(((theta + 180.0) % 360.0) / self.step) % self.n
        self.r[i] = dist_mm
        self.t[i] = now

    def snapshot(self, now):
        age = self.max_age
        r = [d if now - t <= age else 0 for d, t in zip(self.r, self.t)]
        return -math.pi + math.radians(self.step) / 2, math.radians(self.step), r


# ---------------------------------------------------------------------------
# Hokuyo SCIP 2.0 (URG / UST / UTM)
# ---------------------------------------------------------------------------

class ScipError(Exception):
    """Reply was well-formed but not what we asked for. Reconnecting is the fix."""


class ScipDataError(ScipError):
    """One scan arrived corrupted. The link is still in sync - skip the frame."""


def scip_sum(data):
    """SCIP 2.0 checksum char: low 6 bits of the byte sum, offset into printable range."""
    return (sum(data) & 0x3F) + 0x30


def scip_decode3(raw):
    """Every 3 chars carry one 18-bit distance in mm, 6 bits per char."""
    return [((raw[i] - 0x30) << 12) | ((raw[i + 1] - 0x30) << 6) | (raw[i + 2] - 0x30)
            for i in range(0, len(raw) - 2, 3)]


class ScipLink:
    """Request/response SCIP 2.0 over a transport."""

    def __init__(self, transport):
        self.t = transport
        self.buf = b''

    def command(self, cmd, timeout=2.0):
        """Send a command; return (status_code, remaining_reply_lines)."""
        self.t.write(cmd.encode() + b'\n')
        deadline = time.time() + timeout
        self.buf = b''
        while not self.buf.endswith(b'\n\n'):   # every SCIP reply ends with a blank line
            if time.time() > deadline:
                raise TimeoutError(f'no reply to {cmd[:2]}')
            self.buf += self.t.read().replace(b'\r', b'')
        lines = self.buf.rstrip(b'\n').split(b'\n')
        if len(lines) < 2 or lines[0][:2] != cmd[:2].encode():
            raise ScipError(f'unexpected reply to {cmd[:2]}: {lines[:2]!r}')
        status = lines[1]
        if len(status) != 3 or scip_sum(status[:2]) != status[2]:
            raise ScipError(f'bad status line {status!r}')
        return status[:2].decode(), lines[2:]

    def sensor_params(self):
        code, rest = self.command('PP')
        if code != '00':
            raise ScipError(f'PP status {code}')
        raw = {}
        for line in rest:
            m = re.match(rb'^(\w+):(.*);.$', line)   # KEY:VALUE;checksum
            if m:
                raw[m.group(1).decode()] = m.group(2).decode()
        need = ('DMIN', 'DMAX', 'ARES', 'AMIN', 'AMAX', 'AFRT')
        missing = [k for k in need if k not in raw]
        if missing:
            raise ScipError(f'sensor did not report {", ".join(missing)}')
        return {k: int(raw[k]) for k in need}

    def read_scan(self, first, last):
        code, rest = self.command('GD%04d%04d01' % (first, last))
        if code != '00':
            raise ScipError(f'GD status {code}')
        if len(rest) < 2:
            raise ScipDataError('short scan reply')
        raw = bytearray()
        for line in rest[1:]:   # rest[0] is the timestamp; data lines end in a checksum char
            if not line or scip_sum(line[:-1]) != line[-1]:
                raise ScipDataError('data checksum mismatch')
            raw += line[:-1]
        return scip_decode3(raw)


class HokuyoLidar(LidarSource):
    def __init__(self, address, baud, args):
        super().__init__(args)
        self.address, self.baud = address, baud
        self.id = f'hokuyo:{address}'
        self.label = f'Hokuyo - {address}'

    def run(self, publish, stop, note):
        note(f'connecting to {self.address}')
        transport = open_transport(self.address, self.baud or 115200, default_port=10940)
        link = ScipLink(transport)
        laser_on_by_us = False
        try:
            if isinstance(transport, SerialTransport):
                try:
                    link.command('SCIP2.0')   # older URGs boot in SCIP 1.1
                except ScipError:
                    pass
            p = link.sensor_params()
            code, _ = link.command('BM')      # 00 = laser now on, 02 = someone else already had it on
            if code not in ('00', '02'):
                raise ScipError(f'BM status {code}')
            laser_on_by_us = code == '00'

            step = 2 * math.pi / p['ARES']
            a0 = (p['AMIN'] - p['AFRT']) * step
            interval = 1.0 / self.hz
            bad = 0
            while not stop.is_set():
                t0 = time.time()
                try:
                    ranges = link.read_scan(p['AMIN'], p['AMAX'])
                    bad = 0
                except ScipDataError:
                    bad += 1
                    if bad > 10:
                        raise
                    continue
                dmin = p['DMIN']
                ranges = [d if d >= dmin else 0 for d in ranges]   # below DMIN are error codes, not distances
                publish(self.frame(a0, step, ranges, dmin, p['DMAX']))
                stop.wait(max(0.0, interval - (time.time() - t0)))
        finally:
            if laser_on_by_us:   # only switch it off if we're the one who switched it on
                try:
                    link.command('QT', timeout=1.0)
                except Exception:
                    pass
            transport.close()


# ---------------------------------------------------------------------------
# Slamtec RPLIDAR (A1 / A2 / A3 / S1), standard scan mode
# ---------------------------------------------------------------------------

RPLIDAR_BAUDS = (115200, 256000, 1000000, 460800)   # A1 | A2M12/A3/S1 | S2 | C1 - tried in this order


def rp_cmd(cmd, payload=b''):
    """A5 <cmd> [len payload xor-checksum]"""
    if not payload:
        return bytes([0xA5, cmd])
    msg = bytes([0xA5, cmd, len(payload)]) + payload
    cs = 0
    for b in msg:
        cs ^= b
    return msg + bytes([cs])


def rp_descriptor(buf):
    """Find a 7-byte response descriptor (A5 5A, 30-bit length + 2-bit mode, type).
    Returns (end_offset, length, dtype) or None."""
    i = buf.find(b'\xa5\x5a')
    if i < 0 or len(buf) < i + 7:
        return None
    length = int.from_bytes(buf[i + 2:i + 6], 'little') & 0x3FFFFFFF
    return i + 7, length, buf[i + 6]


def rp_parse_node(b):
    """5-byte standard-scan node -> (angle_deg clockwise, dist_mm), or None if it isn't a valid node."""
    start, inv_start = b[0] & 1, (b[0] >> 1) & 1
    if start == inv_start or not (b[1] & 1):    # framing bits: S must differ from !S, C must be 1
        return None
    angle = ((b[1] >> 1) | (b[2] << 7)) / 64.0
    dist = (b[3] | (b[4] << 8)) / 4.0
    return angle, int(dist)


class RplidarLidar(LidarSource):
    def __init__(self, address, baud, args):
        super().__init__(args)
        self.address, self.baud = address, baud
        self.id = f'rplidar:{address}'
        self.label = f'RPLIDAR - {address}'

    def run(self, publish, stop, note):
        if is_serial_address(self.address):
            bauds = [self.baud] if self.baud else list(RPLIDAR_BAUDS)
        else:
            bauds = [None]   # TCP (e.g. RPLIDAR S2E): no baud rate to find
        for baud in bauds:
            note(f'looking for an RPLIDAR on {self.address}' + (f' at {baud} baud' if baud else ''))
            t = open_transport(self.address, baud or 115200, default_port=20108)
            try:
                if self._healthy(t):
                    self._scan(t, publish, stop, note)
                    return
            finally:
                self._quiesce(t)
                t.close()
        tried = ', '.join(str(b) for b in bauds if b)
        raise RuntimeError('no RPLIDAR answered' + (f' (tried {tried} baud)' if tried else '')
                           + ' - wrong port, or not an RPLIDAR')

    def _wait_descriptor(self, t, seconds):
        buf, deadline = bytearray(), time.time() + seconds
        while time.time() < deadline:
            buf += t.read()
            d = rp_descriptor(buf)
            if d:
                end, length, dtype = d
                return buf[end:], length, dtype
        return None

    def _healthy(self, t):
        t.write(rp_cmd(0x25))          # STOP whatever it was doing
        time.sleep(0.02)
        t.reset_input()
        t.write(rp_cmd(0x52))          # GET_HEALTH
        got = self._wait_descriptor(t, 1.5)
        if not got or got[2] != 0x06 or got[1] != 3:
            return False
        rest = bytearray(got[0])
        deadline = time.time() + 1.0
        while len(rest) < 3 and time.time() < deadline:
            rest += t.read()
        if len(rest) < 3:
            return False
        if rest[0] == 2:
            raise RuntimeError('RPLIDAR reports a hardware error (code 0x%04x)' % (rest[1] | rest[2] << 8))
        return True

    def _scan(self, t, publish, stop, note):
        note('starting motor')
        t.set_dtr(False)                                    # A1: adapter DTR low = motor on
        t.write(rp_cmd(0xF0, struct.pack('<H', 660)))       # A2/A3: motor PWM (ignored by the others)
        t.reset_input()
        t.write(rp_cmd(0x20))                               # SCAN
        got = self._wait_descriptor(t, 5.0)                 # motor spin-up can take a couple of seconds
        if not got or got[2] != 0x81 or got[1] != 5:
            raise RuntimeError('RPLIDAR did not start a standard scan (unsupported model or mode)')
        buf = bytearray(got[0])
        asm = ScanAssembler(720, clockwise=True)
        last_emit, interval = 0.0, 1.0 / self.hz
        while not stop.is_set():
            data = t.read()
            if not data:
                raise TimeoutError('no data from the RPLIDAR for 1 s')
            buf += data
            now = time.time()
            n = 0
            while len(buf) - n >= 5:
                node = rp_parse_node(buf[n:n + 5])
                if node is None:
                    n += 1                                  # out of sync: slide one byte
                    continue
                asm.add(node[0], node[1], now)
                n += 5
            del buf[:n]
            if now - last_emit >= interval:
                last_emit = now
                a0, da, r = asm.snapshot(now)
                publish(self.frame(a0, da, r, 100, 40000))

    def _quiesce(self, t):
        try:
            t.write(rp_cmd(0x25))                           # STOP
            t.write(rp_cmd(0xF0, struct.pack('<H', 0)))     # motor PWM 0
            t.set_dtr(True)                                 # A1 motor off
        except Exception:
            pass


# ---------------------------------------------------------------------------
# LDROBOT LD06 / LD19 / LD14 - streams 47-byte packets by itself
# ---------------------------------------------------------------------------

def _crc8_table():
    table = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ 0x4D) & 0xFF if c & 0x80 else (c << 1) & 0xFF   # poly 0x4D, MSB first
        table.append(c)
    return table


_LD_CRC = _crc8_table()


def ld_crc8(data):
    c = 0
    for b in data:
        c = _LD_CRC[c ^ b]
    return c


def ld_parse_packet(p):
    """47-byte packet -> [(angle_deg clockwise, dist_mm), ...] for its 12 points."""
    start = (p[4] | p[5] << 8) / 100.0
    end = (p[42] | p[43] << 8) / 100.0
    span = (end - start) % 360.0
    out = []
    for k in range(12):
        dist = p[6 + 3 * k] | p[7 + 3 * k] << 8
        out.append(((start + span * k / 11.0) % 360.0, dist))
    return out


class LdRobotLidar(LidarSource):
    def __init__(self, address, baud, args):
        super().__init__(args)
        self.address, self.baud = address, baud
        self.id = f'ld06:{address}'
        self.label = f'LDROBOT LD06/LD19 - {address}'

    def run(self, publish, stop, note):
        note(f'listening on {self.address}')
        t = open_transport(self.address, self.baud or 230400, default_port=None)
        try:
            asm = ScanAssembler(360, clockwise=True)
            buf = bytearray()
            last_ok, last_emit, interval = time.time(), 0.0, 1.0 / self.hz
            while not stop.is_set():
                buf += t.read()
                now = time.time()
                if now - last_ok > 3.0:
                    raise RuntimeError('no valid LD06/LD19 packets for 3 s '
                                       '(wrong port or baud, or not an LDROBOT lidar)')
                while True:
                    i = buf.find(b'\x54\x2c')                 # header + "12 points per packet"
                    if i < 0:
                        del buf[:-1]
                        break
                    del buf[:i]
                    if len(buf) < 47:
                        break
                    if ld_crc8(buf[:46]) != buf[46]:
                        del buf[:1]
                        continue
                    for angle, dist in ld_parse_packet(buf[:47]):
                        asm.add(angle, dist, now)
                    del buf[:47]
                    last_ok = now
                if now - last_emit >= interval and now - last_ok < 1.0:
                    last_emit = now
                    a0, da, r = asm.snapshot(now)
                    publish(self.frame(a0, da, r, 20, 12000))
        finally:
            t.close()


# ---------------------------------------------------------------------------
# SICK TiM / LMS, CoLa-A over TCP (poll one scan at a time)
# ---------------------------------------------------------------------------

def sick_parse_telegram(text):
    """'sRA LMDscandata ... DIST1 <scale> <offset> <start> <step> <n> <n values> ...' ->
    (a0_rad, da_rad, ranges_mm). Only the DIST1 block is read, so header fields can vary."""
    tok = text.strip('\x02\x03 ').split(' ')
    if tok[0] == 'sFA':
        raise RuntimeError('SICK returned an error telegram: ' + text.strip('\x02\x03'))
    if 'DIST1' not in tok:
        raise RuntimeError('SICK reply has no DIST1 block: ' + text[:80])
    k = tok.index('DIST1')
    scale = struct.unpack('>f', bytes.fromhex(tok[k + 1].zfill(8)))[0]
    start = int(tok[k + 3], 16)
    if start >= 1 << 31:
        start -= 1 << 32                                  # signed 32-bit, 1/10000 degree
    step = int(tok[k + 4], 16)                            # 1/10000 degree
    count = int(tok[k + 5], 16)
    ranges = [int(int(v, 16) * scale) for v in tok[k + 6:k + 6 + count]]
    return math.radians(start / 10000.0), math.radians(step / 10000.0), ranges


class SickLidar(LidarSource):
    def __init__(self, address, baud, args):
        super().__init__(args)
        self.address = address
        self.id = f'sick:{address}'
        self.label = f'SICK TiM/LMS - {address}'

    def run(self, publish, stop, note):
        note(f'connecting to {self.address}')
        t = open_transport(self.address, 0, default_port=2111)
        buf, interval = b'', 1.0 / self.hz
        try:
            while not stop.is_set():
                t0 = time.time()
                t.write(b'\x02sRN LMDscandata\x03')
                deadline = t0 + 3.0
                while b'\x03' not in buf:
                    if time.time() > deadline:
                        raise TimeoutError('no scan telegram from the SICK for 3 s')
                    buf += t.read()
                telegram, _, buf = buf.partition(b'\x03')
                a0, da, ranges = sick_parse_telegram(telegram.decode('ascii', 'replace'))
                publish(self.frame(a0, da, ranges, 20, 30000))
                stop.wait(max(0.0, interval - (time.time() - t0)))
        finally:
            t.close()


# ---------------------------------------------------------------------------
# ROS 2 LaserScan (any lidar that has a ROS driver) and the simulated lidar
# ---------------------------------------------------------------------------

class RosScanLidar(LidarSource):
    def __init__(self, topic, args):
        super().__init__(args)
        self.id = f'ros:{topic}'
        self.label = f'ROS 2 topic - {topic}'
        self.topic = topic

    def run(self, publish, stop, note):
        note(f'waiting for {self.topic}')
        if not rclpy.ok():
            rclpy.init()
        node = rclpy.create_node(f'sensor_dashboard_{os.getpid()}')
        last = [0.0]
        interval = 1.0 / self.hz

        def on_scan(msg):
            now = time.time()
            if now - last[0] < interval:
                return
            last[0] = now
            ranges = [int(x * 1000) if math.isfinite(x) and x >= msg.range_min else 0
                      for x in msg.ranges]
            publish(self.frame(msg.angle_min, msg.angle_increment, ranges,
                               int(msg.range_min * 1000), int(msg.range_max * 1000)))

        node.create_subscription(LaserScan, self.topic, on_scan, qos_profile_sensor_data)
        try:
            while not stop.is_set():
                rclpy.spin_once(node, timeout_sec=0.2)
        finally:
            node.destroy_node()


def shutdown_ros():
    if rclpy is not None and rclpy.ok():
        rclpy.shutdown()


class DemoLidar(LidarSource):
    id = 'demo'
    label = 'Demo (simulated lidar)'

    def run(self, publish, stop, note):
        # 270 deg / 0.25 deg like a UST-10LX, in a room with one orbiting post
        step = 2 * math.pi / 1440
        a0 = -540 * step
        t0 = time.time()
        while not stop.is_set():
            t = time.time() - t0
            px, py = 0.4 * math.sin(t * 0.5), 0.6 * math.sin(t * 0.3)
            cx, cy = 1.6 * math.cos(t * 0.7) - px, 2.4 * math.sin(t * 0.7) - py
            c2 = cx * cx + cy * cy - 0.25 * 0.25
            ranges = []
            for i in range(1081):
                th = a0 + i * step
                dx, dy = -math.sin(th), math.cos(th)
                tx = (2.5 - px) / dx if dx > 1e-9 else ((-2.5 - px) / dx if dx < -1e-9 else 1e9)
                ty = (4.0 - py) / dy if dy > 1e-9 else ((-3.0 - py) / dy if dy < -1e-9 else 1e9)
                d = min(tx, ty)
                b = dx * cx + dy * cy
                disc = b * b - c2
                if disc >= 0:
                    hit = b - math.sqrt(disc)
                    if 0 < hit < d:
                        d = hit
                ranges.append(int((d + random.gauss(0, 0.004)) * 1000) if d < 10 else 0)
            publish(self.frame(a0, step, ranges, 60, 10000))
            stop.wait(1.0 / self.hz)


# ---------------------------------------------------------------------------
# What appears in the dropdown
# ---------------------------------------------------------------------------

# name -> (class, aliases, label used for auto-listed serial ports)
DRIVERS = {
    'hokuyo': (HokuyoLidar, ('urg', 'ust', 'utm'), 'Hokuyo (USB)'),
    'rplidar': (RplidarLidar, ('slamtec',), 'RPLIDAR A/S series'),
    'ld06': (LdRobotLidar, ('ld19', 'ld14', 'ldrobot', 'ldlidar'), 'LDROBOT LD06 / LD19'),
    'sick': (SickLidar, ('tim', 'lms'), 'SICK TiM / LMS'),
}
_ALIASES = {alias: name for name, (_, aliases, _) in DRIVERS.items() for alias in aliases + (name,)}
HOKUYO_VID = '15d1'


def parse_lidar_spec(spec):
    """'rplidar:/dev/ttyUSB0,baud=256000' -> ('rplidar', '/dev/ttyUSB0', 256000)"""
    kind, sep, rest = spec.partition(':')
    name = _ALIASES.get(kind.lower())
    if not sep or name is None or not rest:
        raise ValueError(f'bad --lidar {spec!r}: expected TYPE:ADDRESS with TYPE one of '
                         f'{", ".join(sorted(_ALIASES))}, e.g. rplidar:/dev/ttyUSB0 or sick:192.168.0.1')
    address, *options = rest.split(',')
    baud = None
    for opt in options:
        key, _, value = opt.partition('=')
        if key.strip() != 'baud' or not value.strip().isdigit():
            raise ValueError(f'bad option {opt!r} in --lidar {spec!r} (only baud=N is supported)')
        baud = int(value)
    return name, address, baud


def usb_ids(dev):
    """(vendor_id, product_id, product_name) of the USB device behind a tty, or None."""
    def read(path):
        try:
            with open(path) as f:
                return f.read().strip()
        except OSError:
            return None

    d = os.path.realpath('/sys/class/tty/' + os.path.basename(dev) + '/device')
    for _ in range(5):
        vid = read(os.path.join(d, 'idVendor'))
        if vid:
            return vid.lower(), (read(os.path.join(d, 'idProduct')) or '').lower(), read(os.path.join(d, 'product'))
        d = os.path.dirname(d)
    return None


def serial_entries(ports):
    """One dropdown entry per (port, lidar type that could plausibly be on it).
    ports: [(dev, usb_ids_or_None)]. Nothing is sent to a port until one of its entries is chosen."""
    out = []
    for dev, ids in ports:
        vid, product = (ids[0], ids[2]) if ids else (None, None)
        for name, (_, _, label) in DRIVERS.items():
            if name == 'sick':
                continue                                     # Ethernet only
            if name == 'hokuyo' and vid not in (None, HOKUYO_VID):
                continue                                     # a Hokuyo shows up with Hokuyo's USB id
            if name != 'hokuyo' and vid == HOKUYO_VID:
                continue
            out.append((name, dev, f'{label} - {dev}' + (f' [{product}]' if product else '')))
    return out


def list_serial_ports(exclude):
    if not sys.platform.startswith('linux'):
        return []
    skip = {os.path.realpath(p) for p in exclude}
    out, seen = [], set()
    for dev in sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')):
        real = os.path.realpath(dev)
        if real not in skip and real not in seen:
            seen.add(real)
            out.append((dev, usb_ids(dev)))
    return out


def make_lidar(name, address, baud, args):
    return DRIVERS[name][0](address, baud, args)


def build_lidars(args):
    sources, notes = [], []
    specs = [parse_lidar_spec(s) for s in (args.lidar or ['hokuyo:192.168.0.10:10940'])]
    for name, address, baud in specs:
        sources.append(make_lidar(name, address, baud, args))
    taken = {addr for _, addr, _ in specs}

    if serial is None:
        if sys.platform.startswith('linux'):
            notes.append('python3-serial is not installed, so USB/serial lidars are not listed '
                         '(sudo apt install python3-serial).')
    else:
        for name, dev, label in serial_entries(list_serial_ports(args.serial_exclude)):
            if dev not in taken:
                s = make_lidar(name, dev, None, args)
                s.label = label
                sources.append(s)
    if rclpy is not None:
        sources.append(RosScanLidar(args.ros_topic, args))
    if not args.no_demo:
        sources.append(DemoLidar(args))
    return sources, notes
