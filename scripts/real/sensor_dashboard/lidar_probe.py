#!/usr/bin/env python3
"""
Work out what is on a serial port when the dashboard shows nothing.

    python lidar_probe.py            # lists the serial ports it can see
    python lidar_probe.py COM3       # Windows
    python lidar_probe.py /dev/ttyUSB0

Close the dashboard tab (or stop the dashboard) first: only one program can hold a port.

Phase 1 is passive - it only listens, at each common baud rate, and reports how many bytes
arrived and whether they form LD06/LD19 packets. Phase 2 (skip with --passive-only) sends
the standard RPLIDAR "get health" request at each baud rate; that request is harmless to
other devices but is still a write, so it only runs on the port you named.
"""

from __future__ import annotations

import argparse
import sys
import time

from lidar_drivers import hexdump, ld_crc8, rp_cmd, rp_descriptor

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit('pyserial is not installed: pip install pyserial')

PASSIVE_BAUDS = (230400, 115200, 256000, 460800, 1000000, 128000, 153600, 921600)
RPLIDAR_BAUDS = (115200, 256000, 1000000, 460800)


def show_ports():
    ports = list(list_ports.comports())
    if not ports:
        print('No serial ports found. On Windows that usually means the USB-serial driver is not installed '
              '(check Device Manager for a warning icon).')
    for p in ports:
        ids = f'{p.vid:04x}:{p.pid:04x}' if p.vid else '----:----'
        print(f'  {p.device:<12} {ids}  {p.description}')
    print('\nRun again with the port name, e.g.  python lidar_probe.py', ports[0].device if ports else 'COM3')


def open_port(port, baud):
    try:
        return serial.Serial(port, baud, timeout=0.2)
    except serial.SerialException as e:
        sys.exit(f'Could not open {port}: {e}\n(Is the dashboard, or another program, still holding it?)')


def listen(port, baud, seconds):
    ser = open_port(port, baud)
    try:
        ser.reset_input_buffer()
        buf, end = bytearray(), time.time() + seconds
        while time.time() < end:
            buf += ser.read(4096)
        return bytes(buf)
    finally:
        ser.close()


def count_ld_packets(data):
    good = headers = 0
    i = 0
    while True:
        i = data.find(b'\x54\x2c', i)
        if i < 0 or i + 47 > len(data):
            break
        headers += 1
        if ld_crc8(data[i:i + 46]) == data[i + 46]:
            good += 1
            i += 47
        else:
            i += 1
    return headers, good


def printable_ratio(data):
    return sum(32 <= b < 127 or b in (9, 10, 13) for b in data) / len(data) if data else 0.0


def rplidar_health(port, baud):
    """Returns (found, detail)."""
    ser = open_port(port, baud)
    try:
        ser.write(rp_cmd(0x25))                 # STOP
        time.sleep(0.05)
        ser.reset_input_buffer()
        ser.write(rp_cmd(0x52))                 # GET_HEALTH
        buf, end = bytearray(), time.time() + 1.2
        while time.time() < end:
            buf += ser.read(256)
            d = rp_descriptor(buf)
            if d and len(buf) >= d[0] + 3 and d[2] == 0x06:
                return True, 'health status: ' + {0: 'good', 1: 'warning', 2: 'ERROR'}.get(buf[d[0]], str(buf[d[0]]))
        return False, f'{len(buf)} bytes back' + (f': {hexdump(buf, 16)}' if buf else '')
    finally:
        ser.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('port', nargs='?')
    ap.add_argument('--seconds', type=float, default=1.5, help='listen time per baud rate (default 1.5)')
    ap.add_argument('--passive-only', action='store_true', help='never write anything to the port')
    args = ap.parse_args()
    if not args.port:
        return show_ports()

    print(f'Phase 1: listening on {args.port} (nothing is sent)\n')
    print(f'  {"baud":>8}  {"bytes":>6}  {"LD06/LD19 packets":>17}  first bytes')
    verdict = []
    for baud in PASSIVE_BAUDS:
        data = listen(args.port, baud, args.seconds)
        headers, good = count_ld_packets(data)
        note = ''
        if data and printable_ratio(data) > 0.9:
            note = '  <- mostly readable text: ' + repr(data[:40])
        print(f'  {baud:>8}  {len(data):>6}  {good:>9} ({headers} hdr)  {hexdump(data, 12)}{note}')
        if good >= 3:
            verdict.append(('ld06', baud, f'{good} valid LD06/LD19 packets'))
    print()

    if not args.passive_only:
        print('Phase 2: RPLIDAR "get health" at each baud rate\n')
        for baud in RPLIDAR_BAUDS:
            found, detail = rplidar_health(args.port, baud)
            print(f'  {baud:>8}  {"RPLIDAR answered - " if found else "no answer - "}{detail}')
            if found:
                verdict.append(('rplidar', baud, detail))
        print()

    print('Result')
    if verdict:
        kind, baud, why = verdict[0]
        print(f'  Looks like: {kind} at {baud} baud ({why}).')
        print(f'  Run:  python sensor_dashboard.py --lidar {kind}:{args.port},baud={baud}')
    else:
        print('  Nothing recognisable answered. Check, in this order:')
        print('   1. Bytes in Phase 1 at any baud? If NONE at all: the lidar is not sending - check its power')
        print('      (a spinning motor does not prove the data side is up) and that the TX/RX lines are wired to the')
        print('      adapter. If some bytes but no packets: it is a different lidar (e.g. YDLIDAR) or a different protocol.')
        print('   2. Send those first bytes to me along with the lidar model and I can add support.')


if __name__ == '__main__':
    main()
