#!/usr/bin/env bash
# Opens two terminals: the LED status node and the arm hardware launch.
# Usage: ./start_robot.sh

WS="${WS:-/home/pi/MSD-SPOT-robot-Rev-4-main}"

if [ ! -f "$WS/install/setup.bash" ]; then
  echo "Cannot find $WS/install/setup.bash - build the workspace first (or set WS=/path)."
  exit 1
fi

echo "Binding 8BitDo controller (2dc8:3106) to xpad..."
sudo modprobe xpad
echo "2dc8 3106" | sudo tee /sys/bus/usb/drivers/xpad/new_id > /dev/null || true

TMP="$(mktemp -d)"

cat > "$TMP/leds.sh" <<EOF
#!/usr/bin/env bash
sudo bash -c 'source $WS/install/setup.bash && ros2 run robot_status_leds led_state_node --ros-args -p led_backend:=rpi_neopixel -p led_count:=60 -p led_pin:=18 -p brightness:=0.4 -p moving_color:=FF0000 -p static_color:=00FF00 -p blinking_color:=FF0000'
exec bash
EOF

cat > "$TMP/hardware.sh" <<EOF
#!/usr/bin/env bash
sudo bash -c 'source $WS/install/setup.bash && ros2 launch robot_moveit_config hardware.launch.py'
exec bash
EOF

chmod +x "$TMP/leds.sh" "$TMP/hardware.sh"

open_terminal() {
  local title="$1" script="$2"

  if command -v gnome-terminal >/dev/null 2>&1; then
    gnome-terminal --title="$title" -- bash "$script" &
  elif command -v lxterminal >/dev/null 2>&1; then
    lxterminal --title="$title" -e "bash $script" &
  elif command -v qterminal >/dev/null 2>&1; then
    qterminal -e "bash $script" &
  elif command -v xfce4-terminal >/dev/null 2>&1; then
    xfce4-terminal --title="$title" -e "bash $script" &
  elif command -v konsole >/dev/null 2>&1; then
    konsole -e bash "$script" &
  elif command -v x-terminal-emulator >/dev/null 2>&1; then
    x-terminal-emulator -e bash "$script" &
  else
    return 1
  fi
}

if [ -n "$DISPLAY" ] || [ -n "$WAYLAND_DISPLAY" ]; then
  if open_terminal "LED node" "$TMP/leds.sh" && open_terminal "Arm hardware" "$TMP/hardware.sh"; then
    exit 0
  fi
fi

# No desktop terminal available (e.g. connected over SSH): use tmux instead.
if ! command -v tmux >/dev/null 2>&1; then
  echo "No graphical terminal found and tmux is not installed (sudo apt install tmux)."
  exit 1
fi

if tmux has-session -t robot 2>/dev/null; then
  echo "A tmux session named 'robot' already exists. Attach with: tmux attach -t robot"
  exit 1
fi

tmux new-session -d -s robot -n leds "bash $TMP/leds.sh"
tmux new-window -t robot -n hardware "bash $TMP/hardware.sh"
echo "Started in tmux. Attach with: tmux attach -t robot   (Ctrl+b then n switches windows)"
