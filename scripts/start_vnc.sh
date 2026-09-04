#!/bin/bash
pkill Xvfb 2>/dev/null
pkill x11vnc 2>/dev/null
pkill fluxbox 2>/dev/null

# /tmp/.X11-unix is bind-mounted from the host and its permissions don't
# always come through as 1777, which makes Xvfb fail to bind its socket.
sudo mkdir -p /tmp/.X11-unix
sudo chmod 1777 /tmp/.X11-unix

# x11vnc refuses to start if it sees Wayland session env vars, even when
# pointed at an Xvfb display via -display. Force X11 before launching it.
unset WAYLAND_DISPLAY
export XDG_SESSION_TYPE=x11

Xvfb :99 -screen 0 1920x1080x24 &
sleep 2
export DISPLAY=:99
fluxbox &
sleep 1
x11vnc -display :99 -nopw -listen 0.0.0.0 -forever &
sleep 1
echo "✓ VNC ready at localhost:5900"
echo "  Connect with VNC Viewer → localhost:5900"