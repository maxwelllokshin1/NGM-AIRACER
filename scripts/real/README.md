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
