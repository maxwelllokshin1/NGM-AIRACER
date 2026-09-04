#!/usr/bin/env python3
"""
select_map.py — interactive map picker for the f1tenth_gym sim bridge.

Arrow-key + Enter menu over the maps in
ros2_ws/src/f1tenth_gym_ros/maps/, plus a "no map" option that leaves
sim.yaml untouched. Doesn't modify gym_bridge_launch.py or gym_bridge.py
at all — gym_bridge_launch.py reads `map_path` straight out of sim.yaml
at launch time (no launch argument for it), so this script just edits
that one value in place, rebuilds only the f1tenth_gym_ros package so the
installed config picks it up, then hands off to the normal launch.

Usage:
    python3 ~/scripts/sim/select_map.py
(scripts/sim/run_gym_bridge.sh calls this automatically.)
"""

import curses
import os
import re
import subprocess
import sys

WORKSPACE = os.path.expanduser('~/ros2_workspaces')
MAPS_DIR = os.path.join(WORKSPACE, 'src', 'f1tenth_gym_ros', 'maps')
SIM_YAML = os.path.join(WORKSPACE, 'src', 'f1tenth_gym_ros', 'config', 'sim.yaml')

NO_MAP_LABEL = 'No map (keep whatever sim.yaml already has)'


def discover_maps():
    if not os.path.isdir(MAPS_DIR):
        return []
    names = set()
    for f in os.listdir(MAPS_DIR):
        base, ext = os.path.splitext(f)
        if ext == '.yaml':
            names.add(base)
    return sorted(names)


def pick_with_curses(options):
    def _run(stdscr):
        curses.curs_set(0)
        idx = 0
        while True:
            stdscr.clear()
            h, w = stdscr.getmaxyx()
            stdscr.addstr(0, 0, 'Select a map for the sim'[:w - 1])
            stdscr.addstr(1, 0, '↑/↓ move  ⏎ select  q cancel'[:w - 1])
            stdscr.addstr(2, 0, '-' * min(40, w - 1))
            for i, opt in enumerate(options):
                row = 4 + i
                if row >= h:
                    break
                marker = '> ' if i == idx else '  '
                style = curses.A_REVERSE if i == idx else curses.A_NORMAL
                stdscr.addstr(row, 0, (marker + opt)[:w - 1], style)
            stdscr.refresh()
            key = stdscr.getch()
            if key in (curses.KEY_UP, ord('k')):
                idx = (idx - 1) % len(options)
            elif key in (curses.KEY_DOWN, ord('j')):
                idx = (idx + 1) % len(options)
            elif key in (curses.KEY_ENTER, 10, 13):
                return idx
            elif key in (ord('q'), 27):
                return None
    return curses.wrapper(_run)


def set_map_path(map_name):
    with open(SIM_YAML, 'r') as f:
        text = f.read()
    new_path = os.path.join(MAPS_DIR, map_name)
    new_text, n = re.subn(
        r"(map_path:\s*)'[^']*'",
        lambda m: "{}'{}'".format(m.group(1), new_path),
        text,
        count=1,
    )
    if n == 0:
        print('Could not find "map_path:" in {} — leaving it untouched.'.format(SIM_YAML))
        return False
    with open(SIM_YAML, 'w') as f:
        f.write(new_text)
    return True


def rebuild_f1tenth_gym_ros():
    print('Rebuilding f1tenth_gym_ros so the map change takes effect...')
    subprocess.run(
        ['bash', '-c',
         'source /opt/ros/humble/setup.bash && cd "{}" && '
         'colcon build --packages-select f1tenth_gym_ros'.format(WORKSPACE)],
        check=True,
    )


def launch_gym_bridge():
    print('\nLaunching gym_bridge_launch.py ...\n')
    os.execvp('bash', ['bash', '-c',
        'source /opt/ros/humble/setup.bash && '
        'source "{}/install/setup.bash" && '
        'ros2 launch f1tenth_gym_ros gym_bridge_launch.py'.format(WORKSPACE)])


def main():
    if not sys.stdin.isatty():
        print('select_map.py needs an interactive terminal — run it directly, not piped.')
        sys.exit(1)

    maps = discover_maps()
    if not maps:
        print('No maps found under {} — launching with sim.yaml as-is.'.format(MAPS_DIR))
        launch_gym_bridge()
        return

    options = [NO_MAP_LABEL] + maps
    choice_idx = pick_with_curses(options)
    if choice_idx is None:
        print('Cancelled — nothing launched.')
        sys.exit(1)

    choice = options[choice_idx]
    if choice == NO_MAP_LABEL:
        print('Keeping the map already set in sim.yaml.')
    else:
        print('Selected map: {}'.format(choice))
        if set_map_path(choice):
            rebuild_f1tenth_gym_ros()

    launch_gym_bridge()


if __name__ == '__main__':
    main()
