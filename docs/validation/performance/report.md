# Performance follow-up, 2026-10-06

Live Step 2 session: five Gazebo robots, one tb1 Nav2 stack, one RViz viewer with all five robot models and both tb1 costmaps. Existing uncommitted test launch/configuration is still required to reproduce this exact session.

Before: Gazebo elapsed simulation/wall-time ratio 0.4718 (median reported RTF 0.47). After lighter visualization relay and restored hardware-rendered Gazebo viewer: elapsed ratio 0.5126 (median 0.51). Improvement is approximately 9%, not a large speedup. Viewer window dimensions changed during the trial, so this is indicative rather than a controlled benchmark. RViz displayed 31 FPS; restored Gazebo displayed 61 FPS with models loaded. Render FPS is distinct from simulation RTF. At RTF 0.51, a commanded 0.15 m/s corresponds to approximately 0.077 metres per wall-clock second.

Changes: visualization relay copies TF metadata rather than recursively copying numeric transform data; large costmap arrays are reused while copying headers. The focused relay supports selecting tb1 datasets while retaining TF for all five robots. Static auxiliary map/odom links are optional and rely on the Step 1 measured planar frame identity.

A camera-rate plugin trial did not work reliably and was discarded. Runtime always_send_full_costmap=false was accepted but did not enable incremental publications (global full maps=3, updates=0; local full maps=6, updates=0 over 12 seconds); restored true. No robot velocity limits, physics timestep, collision geometry, sensor rates, spawn poses or map assets changed during this follow-up. Gazebo viewer restart preserved simulation state.

Remaining: simulation server is still below real time; investigate its collision/ray-sensor and publication costs before claiming a substantial speedup. Step 2 navigation is still blocked by the previously measured localization/physical arrival mismatch. Rendering improvements do not resolve it.

To reopen only the hardware-rendered Gazebo viewer after closing its previous instance:

```bash
source /home/umroot/crs_ws/install/setup.bash
unset LIBGL_ALWAYS_SOFTWARE
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GAZEBO_MODEL_PATH=/home/umroot/crs_ws/install/crs_multibot/share/crs_multibot/models:$GAZEBO_MODEL_PATH
export GAZEBO_MODEL_DATABASE_URI=
gzclient --gui-client-plugin=libgazebo_ros_eol_gui.so
```
