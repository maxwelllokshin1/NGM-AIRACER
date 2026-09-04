#!/bin/bash
# runs the container's entry point for devcontainer service
# starts vnc headless display, parks the container

set -e

# /tmp/.X11-unix is bind-mounted from the host (docker-compose.yml) and its
# permissions don't always come through as 1777, which makes Xvfb fail to
# bind its socket and silently leaves x11vnc with nothing to serve.
sudo mkdir -p /tmp/.X11-unix
sudo chmod 1777 /tmp/.X11-unix

# x11vnc refuses to start if it sees Wayland session env vars, even when
# pointed at an Xvfb display via -display. Force X11 before launching it.
unset WAYLAND_DISPLAY
export XDG_SESSION_TYPE=x11

# start virutal display + vnc (what start_vnc.sh does already)
Xvfb :99 -screen 0 1920x1080x24 &
sleep 1
x11vnc -display :99 -forever -nopw -shared -rfbprot 5900 &

export DISPLAY=:99

echo "[entrypoint] VNC started on 99 and display port also on 99"

# hands off to where ever cmd was passed
exec "$@"