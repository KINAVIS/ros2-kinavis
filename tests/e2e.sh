#!/bin/bash
# The node on a recorded receiver, in a sourced ROS 2 workspace: every fix it
# publishes must be the reference driver's for the same epoch.
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
data="$here/../kinavis_navsat_core/tests/data"
name="${1:-ublox-neo-m9n-nmea}"
out="$(mktemp)"

ros2 topic echo --csv /fix sensor_msgs/msg/NavSatFix > "$out" &
echo_pid=$!
trap 'kill $echo_pid 2>/dev/null || true' EXIT
# Discovery before the first fix.
sleep 3
ros2 run kinavis_navsat nmea_navsat --ros-args \
  -p input:="$data/$name.nmea" -p use_receiver_time:=true -p replay_rate:=2.0
sleep 2
kill $echo_pid
wait $echo_pid 2>/dev/null || true
python3 -I "$here/check_fixes.py" "$out" "$data/$name.reference.csv"
