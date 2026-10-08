# Five-robot mission controller

The fleet is fixed at `tb1` (Hilti/Jaibot leader) and `tb2`–`tb5` (auxiliaries).
Controller code: `scripts/fleet_mission_controller.py`; geometry:
`scripts/formation_geometry.py`; configuration: `config/fleet_mission.yaml`.
The original allocator V3's preferred rectangle and candidate-ring search are
retained, extended with actual footprints, per-robot maps, action ownership and
repeated missions. Legacy prototype files remain available but must not run
alongside this controller.

## Build and environment

The tested build uses `/tmp` so the existing workspace installation is preserved.
Run from this repository:

```bash
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/fleet_build_logs build \
  --base-paths "$PWD" --packages-select crs_multibot \
  --build-base /tmp/fleet_build --install-base /tmp/fleet_install --symlink-install
source /tmp/fleet_install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Run the last three environment commands in **every** ROS terminal. Domain 42
isolates this simulation from old launch processes and MCP servers configured
for domain 0. ROS2 MCP initially inspected the domain-0 simulation; operations
in domain 42 use ROS2/rclpy directly. To inspect domain 42 through MCP, configure
its server process with the same two environment variables.

## Launch Gazebo, RViz and controller

```bash
ros2 launch crs_multibot fleet_simulation.launch.py
```

Exactly one launch is permitted. Legacy goal forwarding and automatic drive
nodes are disabled. Nav2 recovery commands also pass through the velocity
smoother and coordinator. Do not run teleoperation or other goal controllers.
AMCL initializes from each known simulation spawn pose at activation.

To start Gazebo/Nav2 first and the controller separately:

```bash
ros2 launch crs_multibot fleet_simulation.launch.py controller:=false
# In another sourced terminal:
ros2 launch crs_multibot fleet_controller.launch.py
```

The robots will stay still until the controller starts and verifies preflight.
To test without rendering:

```bash
ros2 launch crs_multibot fleet_simulation.launch.py \
  rviz:=false gazebo_gui:=false manual_navigation:=true
```

To add RViz to the headless test simulation without launching a second relay:

```bash
ros2 run crs_multibot fleet_rviz_relay.py --ros-args -p use_sim_time:=true
# Another terminal:
rviz2 -d "$(ros2 pkg prefix crs_multibot)/share/crs_multibot/rviz/fleet_simulation.rviz" \
  --ros-args -p use_sim_time:=true \
  -r /tf:=/fleet_visualization/tf \
  -r /tf_static:=/fleet_visualization/tf_static
```

## Simulation clock and positioning

The EV9 world uses a 4 ms ODE timestep at a target 250 updates per second.
The live trial increased simulation real-time factor from approximately 0.16
to 0.36–0.39. Render FPS and simulation clock speed are separate measurements.

`fleet_simulation.launch.py` defaults to `simulation_localization:=true`:
Gazebo world odometry supplies robot positions through an identity map-to-odom
transform for each namespace; AMCL remains active but its TF broadcast is off.
All five odometry positions were checked against Gazebo to within 0.03 mm.
This is simulation ground-truth positioning, not a test of AMCL convergence.
The physical/localization disagreement guard remains enabled.
Use `simulation_localization:=false` to test sensor-based AMCL localization.
Other worlds require independent verification of the map/world alignment.

## Maintain a boundary while driving in RViz

Static-frame subscriptions retain both the simulation map/odom edge and the
robot model edges, and the supervisor restores them after TF clock resets.
New missions preserve the current corner axes rather than rotating the whole
boundary to follow small incidental changes in the leader heading.

The defaults `maintain_boundary: true` and `flexible_boundary: true` let tb1
follow its own Nav2 route while tb2–tb5 seek positions around its actual pose
(half-length 4.2 m, half-width 3.4 m). Follower identities are interchangeable:
a minimum-distance assignment with switching hysteresis chooses their positions.
Nearby feasible alternatives are searched when a preferred position is occupied.
An unavailable follower position does not reject tb1's route. The blue outline
is a visualization, with its vertices ordered by actual position; tb1 may cross
it, and enclosure is not guaranteed during movement. Physical pair clearance
and local obstacle checks remain active. All five actual poses must reach their
allocated targets before the mission reports READY. Navigation failures can
still pause the fleet.

Set `flexible_boundary: false` to restore rigid corner identities, staged 1 m
moves, whole-rectangle feasibility checks, and the follower catch-up hold.

Launch with `gazebo_gui:=false rviz:=true` to reduce rendering load. Wait for
READY, select **2D Goal Pose** in RViz, and click/drag a destination. The tool
publishes `/formation/primary_goal`, which commands the whole fleet. The blue
outline joins the four actual follower poses. Do not send independent tb1
Nav2 goals for this coordinated operation.

To restore the older line-following transit and destination rectangle,
set `maintain_boundary: false` in the controller configuration.

## Live progress updates

The RViz fleet launch opens a read-only **Fleet — live updates** window.
It shows the current phase, stop reason, follower adjusting its corner,
tb1's distance to the goal, actual odometry speeds, simulation clock speed,
and a recent event history. It stays visible over RViz and can be moved or
closed without changing the mission. A stopped goal can be replaced using
2D Goal Pose without pressing Resume.

To reopen the panel in the same ROS domain:

```bash
ros2 run crs_multibot fleet_live_status.py
```

## Inspect before moving

```bash
ros2 run crs_multibot audit_fleet.py --seconds 45 \
  --output /tmp/fleet_audit.json \
  --map-yaml "$(ros2 pkg prefix crs_multibot)/share/crs_multibot/config/EVfloor9_outer_only.yaml"
ros2 topic echo /formation/state std_msgs/msg/String
ros2 lifecycle get /map_server
ros2 param get /map_server yaml_filename
ros2 topic info /map -v
ros2 topic info /tb1/scan -v
ros2 topic info /tb1/cmd_vel -v
ros2 action list -t
```

Wait for `READY`. Readiness requires one map publisher, one simulation clock,
one robot-description publisher per robot, one gated smoother per robot,
no competing motor publishers, scans, fresh per-robot map-to-base transforms,
both costmaps, active AMCL/Nav2/costmap lifecycle nodes, and both navigation and
planning action servers. The supervisor publishes zero velocity whenever these
checks fail. `/map_server` being absent in the old graph did not imply `/map`
was empty; live topic messages and file equality are the decisive evidence.
The clean launch has a discoverable, active `/map_server`.

Simulation readiness also compares map-frame localization with Gazebo model
positions and refuses movement when their error exceeds 0.35 m. This assumes
the tested world's map and Gazebo coordinates align. The coordinated test
exposed excessive tb2 drift; do not disable this check to force that test through.
For a real robot deployment without Gazebo, set `verify_gazebo_localization:
false` and provide an independently validated localization system.

Real frames are **unprefixed** `map`, `odom`, `base_footprint`, `base_link`:

- `/{robot}/tf`, `/{robot}/tf_static`: independent TF trees.
- `/{robot}/odom`, `/{robot}/scan`, `/{robot}/robot_description`.
- Hilti scan frame: `Front_sensor_link`; auxiliary scan frame: `base_scan`.
- Shared occupancy map: `/map`.
- `/{robot}/global_costmap/costmap`, `/{robot}/local_costmap/costmap`.
- `/{robot}/navigate_to_pose`, `/{robot}/compute_path_to_pose` actions.
- Nav2 commands: `cmd_vel_nav → cmd_vel_uncoordinated → cmd_vel`.

The visualization relay prefixes only its copied frames under
`/fleet_visualization`; the mission controller consumes original per-robot TF.

## Work rectangle, primary goals, repeat and stop

Request the initial work rectangle in a verified open area:

```bash
ros2 service call /formation/form_rectangle std_srvs/srv/Trigger '{}'
```

Send a primary goal using the dedicated controller topic, **not**
`/tb1/goal_pose`. This example is near the initial open work area; every incoming
goal is still checked against the live map/costmap and full Hilti footprint:

```bash
ros2 topic pub --once /formation/primary_goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.1, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
```

For another mission publish another message on the same topic. An accepted new
mission cancels the owned previous actions and waits for their results before
starting replacement goals. For a yaw angle θ, set z=sin(θ/2), w=cos(θ/2).

```bash
ros2 service call /formation/stop std_srvs/srv/Trigger '{}'
ros2 service call /formation/resume std_srvs/srv/Trigger '{}'
```

`stop` cancels owned actions and continuously publishes zero commands. `resume`
rechecks preflight and clears the bounded retry counters. Ctrl-C in the simulation
launch shuts down all nodes. Ctrl-C in the standalone controller sends zero
commands; the velocity smoother also times out stale navigation commands. Never
remove the gate and remap Nav2 directly to motor topics while actions are active.

## State and formation behavior

`PREFLIGHT → READY → FORMING_LINE → TRAVELLING → FORMING_RECTANGLE → READY`.
`WAITING` holds on impossible layouts or exhausted action retries. `STOPPED`
is an operator stop. Navigation recovery remains Nav2's responsibility; rejected,
unreachable or aborted goals use bounded retries with a delay.

Auxiliaries assemble sequentially, limiting crossing approaches. Transit targets
sample arclength along the leader's **actual travelled trail**, rather than a
straight vector across bends. Each auxiliary computes its own path and uses its
own NavigateToPose server. Targets can move to nearby feasible candidates when
obstacles block ideal slots. A changed target replaces an action only after the
minimum update interval and displacement/yaw thresholds; unchanged targets do
not cause repeated cancellation. The leader/upstream robots stop advancing if
the next robot falls too far behind.

At leader arrival, the four rectangle slots are checked in the leader's current
orientation. The controller uses nearby feasible alternative positions or waits
if it cannot allocate all four safely. Completion requires actual map-frame
poses within configured position and yaw tolerances; sending goals or receiving
one Nav2 result alone is insufficient.

The velocity gate checks predicted swept **circumscribed footprint discs** for
all pairs and stops conflicting commands, including motion into a stationary
peer. It also checks projected oriented footprints against the lidar-populated
local costmaps. This protects separation even when another robot is beyond lidar
range. It is conservative and can deadlock in a narrow passage; it is not a joint
multi-robot path planner or a guarantee against an arbitrarily fast unseen object.
Stopped robots may require Nav2 recovery, a new goal, or operator intervention.

## Configuration and model corrections

Edit `config/fleet_mission.yaml`, then restart the controller. A custom file:

```bash
ros2 launch crs_multibot fleet_controller.launch.py config:=/absolute/path/fleet_mission.yaml
```

Defaults: rectangle half-length 4.2 m, half-width 3.4 m; transit spacing 3.0 m;
minimum inter-footprint separation 0.4 m; leader speed 0.60 m/s; auxiliary speed
0.80 m/s; angular speed 0.50 rad/s; target update threshold 0.8 m; minimum goal
interval 4 simulation seconds. Data freshness uses simulation timestamps;
command/clock watchdogs use wall time. Goal-update thresholds, retry delays,
search radius and arrival tolerances are configurable. For higher speeds, also
change the Nav2 DWB and velocity-smoother limits in both `params/nav2_params*.yaml`.
The gate clamps commands to the configured controller limits.

Hilti collision-mesh measurements exceed the old footprint. The new conservative
polygon is x∈[-1.20,1.40], y∈[-0.85,0.85] m. Its local costmap is now 8×8 m to
cover the footprint plus lookahead. Auxiliary footprint is
x∈[-0.30,0.475], y∈[-0.30,0.30] m, covering its body and four wheels.
Circumscribed-disc checks deliberately exceed polygon separation in tight spaces.
Drive-plugin calibration now uses auxiliary wheel separation 0.46 m and diameter
0.20 m, and Hilti diameter 0.703125 m, matching existing collision geometry.
Meshes, PGM/YAML maps, XCF and Blender sources were preserved. Pre-edit copies
are in `backups/formation_20261005_162058/`.

## Incremental validation

Pure geometry/action tests:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q tests
```

Independent navigation (only on the headless launch with manual mode enabled):

```bash
python3 scripts/validate_fleet_navigation.py --timeout 240 \
  --output /tmp/fleet_navigation_validation.json
# Optional single robot:
python3 scripts/validate_fleet_navigation.py --robot tb3 --step 0.8
```

This checks targets against the live map and each global costmap and records
actual start/end positions, displacement, distance to goal and action status.
It stops all motion at the first unsuccessful check. A primary goal automatically
exits manual mode. Use the formation primary-goal topic for coordinated tests.

Validation evidence and remaining limitations are recorded in
`docs/validation/REPORT.md`. Do not treat unexecuted coordinated/detour tests as
passed. For static detours choose goals on opposite sides of an occupied wall
from the audited map; for moving-obstacle tests use a temporary Gazebo model,
confirm its lidar returns and local-costmap occupancy, and remove it afterward.
Do not edit the user's permanent meshes/maps to create test obstacles.

Coordinated, bounded free-space mission test (only after independent tests pass):

```bash
python3 scripts/validate_fleet_mission.py --distance 1.5 --timeout 600 \
  --output /tmp/fleet_mission_validation.json
```

The script logs actual Gazebo poses, observed controller phases and final state.
It always stops the controller when the test finishes. A timeout during line
assembly is **not** proof of successful coordinated transit or rectangle arrival.

For a temporary moving-obstacle fixture after free-space transit has passed,
start with the robots stopped, select verified free coordinates ahead of the
planned path, and use the following terminal sequence. Replace `X`, `Y` with
those audited coordinates (they are intentionally not guessed here):

```bash
ros2 service list -t | rg 'spawn_entity|delete_entity|set_entity_state'
ros2 interface show gazebo_msgs/srv/SpawnEntity
ros2 interface show gazebo_msgs/srv/SetEntityState
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity \
  "{name: fleet_test_obstacle, xml: '<sdf version=\"1.6\"><model name=\"fleet_test_obstacle\"><static>true</static><link name=\"body\"><collision name=\"collision\"><geometry><box><size>0.5 0.5 1.0</size></box></geometry></collision><visual name=\"visual\"><geometry><box><size>0.5 0.5 1.0</size></box></geometry></visual></link></model></sdf>', initial_pose: {position: {x: X, y: Y, z: 0.5}, orientation: {w: 1.0}}, reference_frame: world}"
ros2 topic echo /tb1/scan sensor_msgs/msg/LaserScan --once --field ranges
# Move the temporary fixture only after confirming lidar/costmap detection:
ros2 service call /gazebo/set_entity_state gazebo_msgs/srv/SetEntityState \
  '{state: {name: fleet_test_obstacle, pose: {position: {x: X, y: Y, z: 0.5}, orientation: {w: 1.0}}, reference_frame: world}}'
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity \
  '{name: fleet_test_obstacle}'
```

A single SetEntityState call is a repositioning, not a continuous moving-obstacle
test. To test motion, update the fixture along a known clear trajectory at a
fixed rate, record each robot's scan/costmap and actual motion, and stop if its
approach speed exceeds the tested stopping envelope. This fixture procedure has
not been counted as tested unless the validation report records its execution.
