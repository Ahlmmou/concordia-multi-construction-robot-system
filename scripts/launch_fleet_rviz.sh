#!/usr/bin/env bash
# Run from a terminal with ROS2 and the simulation workspace sourced.
set -euo pipefail
fleet_project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
fleet_workspace_setup="$fleet_project_dir/../../install/setup.bash"
if [[ -f "$fleet_workspace_setup" ]]; then
  set +u
  source "$fleet_workspace_setup"
  set -u
fi
python3 "$fleet_project_dir/scripts/fleet_rviz_relay.py" --ros-args -p use_sim_time:=true &
fleet_relay_pid=$!
python3 "$fleet_project_dir/scripts/fleet_live_status.py" &
fleet_status_pid=$!
trap 'kill "$fleet_relay_pid" "$fleet_status_pid" 2>/dev/null || true' EXIT INT TERM
rviz2 -d "$fleet_project_dir/rviz/fleet_simulation.rviz" --ros-args -p use_sim_time:=true -r /tf:=/fleet_visualization/tf -r /tf_static:=/fleet_visualization/tf_static
