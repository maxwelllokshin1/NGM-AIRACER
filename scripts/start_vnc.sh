#!/bin/bash
pkill Xvfb 2>/dev/null
pkill x11vnc 2>/dev/null
pkill fluxbox 2>/dev/null

Xvfb :99 -screen 0 1920x1080x24 &
sleep 2
export DISPLAY=:99
fluxbox &
sleep 1
x11vnc -display :99 -nopw -listen 0.0.0.0 -forever &
sleep 1
echo "✓ VNC ready at localhost:5900"
echo "  Connect with VNC Viewer → localhost:5900"