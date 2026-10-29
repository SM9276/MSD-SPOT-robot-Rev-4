#!/usr/bin/env bash
# Starts the LED status node in the background, then opens two terminals: the arm
# hardware launch and the target gamepad.
# Usage: ./start_robot.sh

WS="${WS:-/home/pi/MSD-SPOT-robot-Rev-4-main}"

if [ ! -f "$WS/install/setup.bash" ]; then
  echo "Cannot find $WS/install/setup.bash - build the workspace first (or set WS=/path)."
  exit 1
fi

echo "Binding 8BitDo controller (2dc8:3106) to xpad..."
sudo modprobe xpad
echo "2dc8 3106" | sudo tee /sys/bus/usb/drivers/xpad/new_id > /dev/null || true

# FTDI USB-serial adapters default to a 16 ms latency timer, which makes every
# Modbus reply take ~16 ms. Drop it to 1 ms. Does nothing if the adapter has no such setting.
LAT=/sys/bus/usb-serial/devices/ttyUSB0/latency_timer
if [ -e "$LAT" ]; then
  echo "ttyUSB0 latency_timer was $(cat "$LAT") ms, setting to 1 ms"
  echo 1 | sudo tee "$LAT" > /dev/null || true
fi

TMP="$(mktemp -d)"

# LED status node runs in the background (no terminal). Output: /tmp/led_node.log
# Stop it with: sudo pkill -f '[l]ed_state_node'
sudo pkill -f '[l]ed_state_node' 2>/dev/null || true
sudo -v
setsid nohup sudo -n bash -c "source $WS/install/setup.bash && ros2 run robot_status_leds led_state_node --ros-args -p led_backend:=rpi_neopixel -p led_count:=60 -p led_pin:=18 -p brightness:=0.4 -p moving_color:=FF0000 -p static_color:=00FF00 -p blinking_color:=FF0000" > /tmp/led_node.log 2>&1 &
echo "LED node started in the background (log: /tmp/led_node.log)"

cat > "$TMP/hardware.sh" <<EOF
#!/usr/bin/env bash
sudo bash -c 'source $WS/install/setup.bash && ros2 launch robot_moveit_config hardware.launch.py'
exec bash
EOF

cat > "$TMP/target.sh" <<EOF
#!/usr/bin/env bash
sudo bash -c 'source $WS/install/setup.bash && ros2 launch spotarm_target_gamepad target_gamepad.launch.py'
exec bash
EOF

chmod +x "$TMP/hardware.sh" "$TMP/target.sh"

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
  if open_terminal "Arm hardware" "$TMP/hardware.sh" && open_terminal "Target gamepad" "$TMP/target.sh"; then
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

tmux new-session -d -s robot -n hardware "bash $TMP/hardware.sh"
tmux new-window -t robot -n target "bash $TMP/target.sh"
echo "Started in tmux. Attach with: tmux attach -t robot   (Ctrl+b then n switches windows)"
