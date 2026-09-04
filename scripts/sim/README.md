# Sim run info

Everything here talks to the `f1tenth_gym_ros` physics bridge instead of real
hardware. No lidar, no VESC, no `/dev/ttyACM0` — just the simulator.

## Order of operations

1. **VNC first** (only if you need to see RViz — skip if you're headless):
   ```bash
   bash ~/scripts/start_vnc.sh
   ```
   Connect a VNC client to `localhost:5900`.

2. **Terminal 1 — the sim bridge** (leave running):
   ```bash
   bash ~/scripts/sim/run_gym_bridge.sh
   ```
   First prompts you with an arrow-key/Enter menu ([select_map.py](select_map.py))
   to pick a map from `ros2_ws/src/f1tenth_gym_ros/maps/`, or "No map" to
   leave `sim.yaml` alone — then starts `gym_bridge`, `map_server`,
   `robot_state_publisher`, and RViz. The picker only edits `map_path` in
   `sim.yaml` and rebuilds `f1tenth_gym_ros` so that takes effect; it
   never touches `gym_bridge_launch.py` or `gym_bridge.py`.

3. **Terminal 2 — a driving stack** (pick one):
   ```bash
   bash ~/scripts/sim/run_follow_the_gap.sh    # reactive follow-the-gap
   # or
   bash ~/scripts/sim/run_autonomy_sim.sh      # pure-pursuit + AEB
   ```

4. **Terminal 3 — optional telemetry** (only if using follow_the_gap, pick one):
   ```bash
   ros2 run follow_the_gap car_info_monitor   # terminal readout
   # or
   bash ~/scripts/sim/run_car_dashboard.sh    # live HTML dashboard, auto-opens
   ```
   The dashboard ([car_dashboard.py](../../ros2_ws/src/racer_ros/follow_the_gap/follow_the_gap/car_dashboard.py))
   subscribes to the same `/car_info_debug` topic as `car_info_monitor.py`
   and serves a JARVIS-styled speedometer/steering/gap dashboard at
   `http://localhost:8080` over Server-Sent Events. It tries to auto-open a
   browser inside the container (needs `epiphany-browser`, added to the
   `dev` Dockerfile stage — rebuild the image to get it). Until then, or if
   none is found, just open `http://localhost:8080` in your Windows host
   browser directly — `docker-compose.yml` uses `network_mode: host`, so
   the port is already reachable there, the same way `localhost:5900` (VNC)
   is.

## Topics (how the two terminals talk to each other)

The bridge publishes/subscribes unnamespaced except odom:

| Topic                | Type                  | Direction (bridge) |
|-----------------------|-----------------------|---------------------|
| `/scan`               | `LaserScan`           | publishes           |
| `/ego_racecar/odom`   | `Odometry`            | publishes           |
| `/drive`              | `AckermannDriveStamped` | subscribes         |

`follow_the_gap`'s `reactive_node` is hardcoded to exactly these topics
([reactive_node.py](../../ros2_ws/src/racer_ros/follow_the_gap/follow_the_gap/reactive_node.py#L44-L54)),
which is why it only works against the sim bridge, not real hardware
(real odom comes from `vesc_to_odom_node`, unnamespaced).

## Known issues (already fixed, but read this if they come back)

**`ModuleNotFoundError: No module named 'transforms3d'` when `gym_bridge` starts**
- Cause: `f1tenth_gym_ros` is bind-mounted into the container at *runtime*
  (`docker-compose.yml`), not baked into the image at build time, so its
  `transforms3d` dependency was never installed for the image.
- Permanent fix: `pip3 install transforms3d` is now in the root
  [`Dockerfile`](../../Dockerfile) (`dev` target, next to the f1tenth_gym
  install). Takes effect on the *next image rebuild*.
- If you're on a container built before that fix: `run_gym_bridge.sh`
  auto-installs it as a fallback, or run `pip3 install transforms3d`
  manually. It won't survive a container recreation unless the image is
  rebuilt — `~/.local` isn't a persisted volume.

**RViz `Map` display stays empty, only the raw `LaserScan` outline is visible**
- Check the terminal for `lifecycle_manager-3: error while loading shared
  libraries: libdiagnostic_updater.so: cannot open shared object file`.
  `lifecycle_manager` is what calls `configure`/`activate` on `map_server`
  — if it crashes on startup, `map_server` never activates and RViz's
  `Map` display has nothing to draw, even though the bridge and LaserScan
  are working fine.
- Live fix (no rebuild): `sudo apt-get update && sudo apt-get install -y
  ros-humble-diagnostic-updater`, then re-run `run_gym_bridge.sh`.
- Root cause: `epiphany-browser` was originally installed in the same
  `apt-get` transaction as the nav2 stack in the Dockerfile; its
  webkit/gstreamer dependency chain made apt's solver swap out
  `diagnostic_updater`'s shared library. Fixed by moving `epiphany-browser`
  to its own separate `apt-get` layer *after* the nav2 stack, and
  explicitly pinning `ros-humble-diagnostic-updater` in that install —
  see [`Dockerfile`](../../Dockerfile). Takes effect on the next rebuild.

**RViz `RobotModel` shows "No transform from [ego_racecar/...]" errors**
- This is downstream of the above — `gym_bridge` is what publishes those
  transforms. If it crashed, RViz has nothing to work with. Fix `gym_bridge`
  first, the TF errors resolve on their own.

**TigerVNC: "connection actively refused" on `localhost:5900`**
- `x11vnc` bails out early with "Wayland display server detected" if it
  sees `WAYLAND_DISPLAY`/`XDG_SESSION_TYPE=wayland` in its environment,
  *regardless* of the `-display :99` flag pointing it at the virtual Xvfb
  display. Fixed by unsetting `WAYLAND_DISPLAY` and forcing
  `XDG_SESSION_TYPE=x11` before launching `x11vnc` — already in both
  [`start_container.sh`](../start_container.sh) (entrypoint) and
  [`start_vnc.sh`](../start_vnc.sh).

**Xvfb fails silently: `_XSERVTransMakeAllCOTSServerListeners: failed to create listener`**
- Cause: `/tmp/.X11-unix` (bind-mounted from the host) doesn't always come
  through with mode `1777`, so Xvfb can't create its socket — and then
  x11vnc falls back to the host's real (Wayland) session instead of `:99`.
  Both entrypoint scripts above now `chmod 1777` it on every start, so this
  should self-heal. If it still happens, run:
  ```bash
  sudo mkdir -p /tmp/.X11-unix && sudo chmod 1777 /tmp/.X11-unix
  ```
