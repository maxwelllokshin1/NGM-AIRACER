# Real hardware run info

Everything here talks to actual hardware: the Hokuyo lidar over Ethernet,
the VESC over `/dev/ttyACM0`, and (for `start_motor.sh`) an Xbox controller.
None of it touches the simulator.

## One-time setup (after a fresh `docker-compose up -d`)

```bash
bash ~/scripts/real/setup_motor.sh
```

Clones `vesc`, `ackermann_mux`, `teleop_tools` if missing, installs the
VESC-related apt packages, builds the stack in dependency order, and patches
`vesc.yaml`'s serial port to `/dev/ttyACM0`. Only needs to be re-run if those
build artifact volumes (`ros2_build`/`ros2_install`) get wiped.

## Full autonomy stack

```bash
bash ~/scripts/real/run_autonomy.sh
```

`chmod`s `/dev/ttyACM0`, builds `AEB_System`, `movement_node`, `autonomy`,
and launches [`autonomy.launch.py`](../../ros2_ws/src/racer_ros/autonomy/launch/autonomy.launch.py):
lidar (`urg_node2`, lifecycle-managed), VESC driver stack, `ackermann_mux`,
the AEB `safety_node`, and the pure-pursuit `movement_node`/`trajectory`
node. The VESC driver is deliberately delayed 8s so `trajectory` has a
valid drive command queued before the serial port opens (a
`0.0` steering default was previously enough to kill the VESC on boot).

**Recommended:** run the dead-man's switch alongside it, in a second
terminal, so you have an instant kill switch:
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_workspaces/install/setup.bash
sudo -E env PYTHONPATH=$PYTHONPATH LD_LIBRARY_PATH=$LD_LIBRARY_PATH \
    python3 ~/scripts/real/deadmans_switch.py
```
Press SPACE to zero the drive command (publishes to `/teleop`, which
`ackermann_mux` prioritizes over `/drive`), press SPACE again to resume.

## Manual controller driving (no autonomy)

```bash
bash ~/scripts/real/start_motor.sh
```

Launches just the VESC driver stack + mux
([`motor_test_launch.py`](motor_test_launch.py)) for driving with an Xbox
controller via `controller_bridge.py` (run separately, per the script's
printed instructions — not part of this repo's `scripts/`).

## Isolated motor/servo bench tests

Bypass the mux entirely, publish straight to `/ackermann_cmd`:

```bash
python3 ~/scripts/real/motor_test.py   # drive speed only, steering fixed at 0
python3 ~/scripts/real/servo_test.py   # steering angle only, speed fixed at 0
```

Use these to sanity-check the VESC/servo wiring and calibration
(`speed_to_erpm_gain`, `steering_angle_to_servo_gain`, etc. in
`f1tenth_stack/config/vesc.yaml`) before trusting the full autonomy stack.

## Sensor dashboard (lidar + camera viewer, runs on the Raspberry Pi)

```bash
python3 ~/scripts/real/sensor_dashboard/sensor_dashboard.py
# then open  http://<pi-ip>:8090  from any browser on the network
```

A standalone page with a **lidar dropdown** and a **camera dropdown**. Pick a
sensor and its display fills in live: a top-down lidar plot (with nearest /
front / left / right distances, hover for exact range + bearing, and a Values
tab with a per-10° table) and the camera picture. No ROS required — pure
Python 3.7+ stdlib. Optional extras unlock more dropdown entries:

```bash
sudo apt install python3-opencv   # USB / V4L2 cameras
sudo apt install python3-serial   # every USB / serial lidar
```

What shows up in the dropdowns (press **Refresh sources** after plugging
something in):

| Dropdown | Entry | Notes |
|---|---|---|
| Lidar | Whatever you name with `--lidar TYPE:ADDRESS` | See "Supported lidars" below. With no `--lidar`, a Hokuyo at `192.168.0.10:10940` (as in `autonomy.launch.py`) is listed. |
| Lidar | Every USB serial port found | One entry per lidar type that could be on it (a CP210x adapter offers RPLIDAR + LD06, a Hokuyo USB id offers Hokuyo). **Nothing is written to a port until you pick one of its entries.** |
| Lidar | ROS 2 topic `/scan` | Only listed if `rclpy` imports (source ROS first). Works for any lidar with a ROS driver, and while another program (e.g. `urg_node2`) already holds the lidar. |
| Camera | USB webcams | Via OpenCV, MJPG 640×480 @ 15 fps (`--cam-width/--cam-height/--cam-fps/--cam-quality`). |
| Camera | Pi CSI camera | Via `rpicam-vid` / `libcamera-vid` (preinstalled on Raspberry Pi OS). |
| both | Demo | Simulated lidar / test-pattern camera, to check the page with no hardware (`--no-demo` hides them). |

### Supported lidars

| `--lidar` type | Lidars | Address |
|---|---|---|
| `hokuyo` | URG / UST / UTM (SCIP 2.0), e.g. UST-10LX | `192.168.0.10[:10940]` or `/dev/ttyACM1` |
| `rplidar` | Slamtec RPLIDAR A1 / A2 / A3 / S1 (baud is auto-probed) | `/dev/ttyUSB0` (or `host:port`, e.g. an S2E) |
| `ld06` | LDROBOT LD06 / LD19 / LD14 (`ld19`, `ld14` also accepted) | `/dev/ttyUSB0` |
| `sick` | SICK TiM / LMS over Ethernet (CoLa-A) | `192.168.0.1[:2111]` |
| *(ROS entry)* | anything else with a ROS driver, e.g. YDLIDAR | — |

```bash
python3 sensor_dashboard.py --lidar rplidar:/dev/ttyUSB0 --lidar sick:192.168.0.1
python3 sensor_dashboard.py --lidar ld06:/dev/ttyUSB0,baud=230400
```

Mounting fix-ups apply to every lidar: `--lidar-yaw 90` rotates the plot, and
`--lidar-mirror` flips left/right if a lidar spins the opposite way to what's
assumed. Rotating lidars (RPLIDAR, LD06) are assumed to spin clockwise with
0° at their front marking; if the plot is mirrored or turned, use those two flags.
To add another lidar, copy `LdRobotLidar` (streaming) or `SickLidar` (poll and
reply) in [lidar_drivers.py](sensor_dashboard/lidar_drivers.py) and register it
in `DRIVERS`.

**Only the Hokuyo/RPLIDAR/LD06/SICK byte formats are tested, and only against
simulated devices** written from the vendors' protocol documents — none of it
has run on a real lidar yet. If a lidar shows nothing, the error appears in the
lidar panel (wrong port / baud / model is called out).

**Controls above the lidar plot**
- **See from … to … [unit]**: type the closest and farthest distance to show, in
  m / cm / mm / ft / in — it's converted to metres (shown next to the boxes).
  Anything outside is hidden from the plot, the readings and the Values table.
  The farthest number is also the plot's radius. Saved per browser.
- **Refresh (Hz)**: scans per second sent to the dashboard (0.5–50), applied
  live to the running lidar for every open page. The lidar itself may spin
  slower than this (an RPLIDAR A1 does roughly 5–10 revolutions/s, an LD06 10);
  the green pill shows the rate you actually get.
- **Close objects:** a dotted circle marks the lidar's own blind zone (an
  RPLIDAR A1/A2 can't measure closer than ~0.15 m and reports nothing there —
  that's the sensor, not the software). Outside it, a dropped "no return" no
  longer erases a real reading and a nearer reading isn't overwritten by the
  background behind it, so close and thin objects stay put instead of blinking.

**Nothing shows / it keeps retrying?**
- The terminal prints a timestamped line for every open, failure (with the reason)
  and stream start. Add `--debug` to also see the raw bytes, per-second packet
  counts and (RPLIDAR/Hokuyo) each command and reply.
- `python lidar_probe.py COM3` (or `/dev/ttyUSB0`) listens on the port at every
  common baud rate, tries the RPLIDAR health request, and prints which lidar it is
  and the exact `--lidar ...` command to use. Stop the dashboard first — only one
  program can hold a serial port. Run it with no argument to list the ports.

Notes:
- If a serial port belongs to something else (say a motor-controller MCU), keep
  it out of the list with `--serial-exclude /dev/ttyACM0`.
- A sensor is only opened while a browser is watching it, and is shared between
  browsers. Close the tab (or pick the blank entry) and it's released; if the
  script turned the lidar's laser on it turns it back off.
- The page has no login, so anyone on the network can watch the camera. Use
  `--host 127.0.0.1` to keep it local.
- On a Pi 3, drop `--cam-fps` / `--cam-width` first if the CPU gets warm; the
  lidar side is cheap.

## Known gotchas

- **`/dev/ttyACM0` permissions**: every launch path here `chmod`s it, but if
  you still get a serial permission error, the device may have enumerated
  under a different name — check `ls /dev/ttyACM*` and update `vesc.yaml`.
- **`WARNING: Package name "AEB_System" does not follow the naming
  conventions"`**: cosmetic colcon warning from the mixed-case package name,
  safe to ignore.
- **`follow_the_gap` is sim-only as written**: `reactive_node` subscribes to
  `/ego_racecar/odom` unconditionally
  ([reactive_node.py:50](../../ros2_ws/src/racer_ros/follow_the_gap/follow_the_gap/reactive_node.py#L50)),
  which only exists when `gym_bridge` is running. On real hardware odom
  comes from `vesc_to_odom_node` unnamespaced — remap that subscription (or
  add a launch-time topic remap) before trying to run `follow_the_gap` on
  the car.
