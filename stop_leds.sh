#!/usr/bin/env bash
# Stops the LED status node that start_robot.sh / start_robot_servo.sh run in the background.
# Usage: ./stop_led.sh

if pgrep -f '[l]ed_state_node' > /dev/null; then
  sudo pkill -f '[l]ed_state_node'
  sleep 1
  if pgrep -f '[l]ed_state_node' > /dev/null; then
    echo "LED node did not stop. Force it with: sudo pkill -9 -f '[l]ed_state_node'"
    exit 1
  fi
  echo "LED node stopped."
else
  echo "LED node is not running."
fi
