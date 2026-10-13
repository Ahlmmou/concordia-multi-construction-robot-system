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
allocated targets before the mission reports READY. Primary navigation failures and data safety checks can
still pause the fleet.

The RViz goal arrow specifies tb1's final heading, not its heading throughout
travel. Hilti's Nav2 goal checker and supervisor `primary_yaw_tolerance` both use
0.10 rad (about 5.7 degrees); the auxiliary supervisor tolerance remains 0.45 rad.
A goal is not complete merely because tb1 reaches the requested position: the
primary heading must also meet its tighter tolerance. Final alignment remains
subject to the same obstacle, collision and command-ownership gates.

Moving-slot updates in LEGACY flexible travel check a candidate path while the
auxiliary retains its current owned navigation action. A valid current path
result starts cancellation; only the old action's final result releases ownership
and permits immediate dispatch of the checked update. Target, mission, readiness
and footprint feasibility are rechecked at handoff, and old cached velocity
commands are cleared. Failed or stale background plans retain current navigation.
Operator stops and recovery discard queued updates and cancel the owned action,
even while a path check is pending. Normal planning throttles remain unchanged.
Green slot arrows, labels and footprint outlines still show current formation
targets. Other formation modes retain the original update workflow.

During flexible travel, a stalled auxiliary, exhausted auxiliary retry budget,
or auxiliary action-response timeout starts recovery for that robot only.
Healthy robots continue subject to the existing obstacle and peer velocity gates.
The recovering robot retains cancellation ownership until its action result;
then checked escape/detour motion runs and its formation target resumes after
clearance release. Stale data, invalid localization, primary failures and operator
stops still retain their fleet safety holds. Mission completion still requires
all five actual poses.

Moving flexible corners reserve tb1's current footprint rather than its final
destination. A primary path request checks actual peers without reserving the
followers' moving future corners; those corners may temporarily coincide with
the final goal. Map checks, actual peer clearance and swept collision gates
remain active for every robot. Rigid and optional mode reservations are unchanged.

Set `flexible_boundary: false` to restore rigid corner identities, staged 1 m
moves, whole-rectangle feasibility checks, and the follower catch-up hold.

Normal fleet navigation disables DWB detailed trajectory failure logging,
trajectory evaluation publication, debug trajectory markers and cost-grid point
cloud publication in both Nav2 configurations. Normal paths, scans, costmaps and
footprints remain available for RViz. These options are read when DWB is configured;
changing parameter values on an already configured controller alone is insufficient.

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

## Manual formation modes

`/formation/mode_request` accepts a `std_msgs/msg/String` containing exactly one
of `OPEN_RECTANGLE`, `COMPACT_RECTANGLE`, `SINGLE_FILE`, or `WALL_WORKSPACE`.
`tb1` is the Hilti/Jaibot primary and `tb2`–`tb5` are auxiliary robots.

```bash
ros2 topic pub --once /formation/mode_request std_msgs/msg/String "{data: OPEN_RECTANGLE}"
ros2 topic pub --once /formation/mode_request std_msgs/msg/String "{data: COMPACT_RECTANGLE}"
ros2 topic pub --once /formation/mode_request std_msgs/msg/String "{data: SINGLE_FILE}"
ros2 topic pub --once /formation/mode_request std_msgs/msg/String "{data: WALL_WORKSPACE}"
```

Open rectangle uses the configured 4.2 m longitudinal and 3.4 m lateral half
spacings. Compact uses 2.8 m and 2.2 m. These are centres, not clearances.
Targets must pass map/global-costmap footprint checks and the unchanged 0.4 m
auxiliary / 0.6 m primary pair clearance. Manual assembly rejects an exact layout
that cannot fit; it does not silently relax slots or clearance. Rectangles move
using the existing staged boundary transit after assembly. Work-boundary
containment and footprint collision clearance remain separate concepts.

Requests stop travel, invalidate previous planning generations, wait for owned
navigation action results (a cancel acknowledgement is insufficient), then
assemble auxiliaries sequentially with tb1 stationary. Active mode changes only
when all measured poses satisfy position/yaw tolerances and actions are idle.
Timeout, invalid footprints, map obstacles, or obstructing peers lead to WAITING
with zero permitted motor commands and a reason. A new request retries from
current poses. `/formation/stop` is the immediate supervisor stop interface.

Single-file assembly defaults to a horizontal-first approach: each auxiliary moves along map X until clear of stationary peers, then turns toward its assigned line slot. Rear robots clear a row before front robots use it. The only fallback is one configured outward clearance step followed by the same horizontal approach; if neither simple route fits, the fleet holds. Single-file assembly does not use adaptive clearance circling. `single_file_assembly_axis_yaw` (default 0 radians) and `single_file_clearance_step` configure this behavior. Each waypoint is confirmed from actual poses and action results before turning toward the slot.

Single file bootstraps a verified straight line at 3 m spacing. That initial line
is explicitly assembly geometry; subsequent breadcrumbs are tb1's actual map-TF
travelled positions. Each follower has a monotonic path cursor and receives short
waypoints capped at bends, rather than heading directly toward a point behind
tb1. History persists across successive single-file goals. Leader arrival does
not complete transit until followers settle on their travelled-path slots.
Single file is a transit arrangement: it does **not** enclose tb1. No follower
polygon is drawn in single-file mode. New mode requests interrupt travel; travel
requests during formation assembly are refused. Wall workspace is stationary.

Wall settings specify an occupied map point (`wall_x`, `wall_y`), an accessible
normal (`wall_normal_yaw`, pointing from wall into free space), and a maximum
nearby-wall distance. A free-footprint probe verifies the accessible side. tb1
must be on that side within the configured distance. Auxiliaries form two rows
behind tb1, outside the wall-facing workspace. Row depth, separation, and lateral
spacing are configurable. The orange outline represents the intended wall work
area separately from transit geometry. Default wall coordinates intentionally
require site configuration; an unverified wall request blocks safely.

`/formation/state` includes requested/active modes, transition state, progress,
wall-clock transition duration, formation errors, action statuses, blocked
reasons, and minimum conservative signed footprint clearance. `poses` are map
TF; `ground_truth_poses` are Gazebo ModelStates and are recorded separately with
localization discrepancies. The default simulation uses identity map-to-world
localization, **not AMCL-estimated accuracy**. RViz retains the fleet TF relay,
robot models/names, desired footprint outlines, individual Nav2 paths, and the
requested/active/transition text marker.

### Reproducible EV9 test scenario

The generator reads the existing EV9 world/map and adds static test boxes to new
files only. One box specification drives both SDF collision geometry and map
rasterization. It adds a 6 m passage with a bend and a separate wall at y=12 m.

```bash
python3 scripts/generate_formation_scenario.py
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/formation_build_logs build --base-paths "$PWD" \
  --packages-select crs_multibot --build-base /tmp/formation_build \
  --install-base /tmp/formation_install --symlink-install
source /tmp/formation_install/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOG_DIR=/tmp/formation_ros_logs
# Inspect processes first; run exactly one fleet simulation/controller.
ros2 launch crs_multibot fleet_simulation.launch.py gazebo_gui:=false rviz:=true
# To use the generated scenario instead, stop only the existing fleet launch first:
ros2 launch crs_multibot fleet_simulation.launch.py gazebo_gui:=false rviz:=true \
  world:="$PWD/worlds/ev9_formation_test.world" \
  map_yaml:="$PWD/config/ev9_formation_test.yaml" \
  mission_config:="$PWD/config/formation_scenario_mission.yaml"
```

Bounded validation commands (matching environment in every terminal):

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests -q
python3 scripts/audit_fleet.py --seconds 40 --output /tmp/fleet_audit.json \
  --map-yaml config/ev9_formation_test.yaml
python3 scripts/validate_formation_modes.py --mode OPEN_RECTANGLE --timeout 620 \
  --output validation/formations/open_rectangle.json
python3 scripts/validate_formation_modes.py --mode COMPACT_RECTANGLE --timeout 620 \
  --output validation/formations/compact_rectangle.json
python3 scripts/validate_formation_modes.py --mode SINGLE_FILE --timeout 620 \
  --output validation/formations/single_file.json
# Goals must first be checked against the live map and current fleet positions.
python3 scripts/validate_formation_modes.py --goal X Y YAW --timeout 900 \
  --output validation/formations/travel.json
python3 scripts/validate_formation_modes.py --mode WALL_WORKSPACE --timeout 620 \
  --output validation/formations/wall_workspace.json
```

The recorder always requests supervisor stop on completion, timeout, failure, or
interruption. It resumes an explicitly tested stationary IDLE fleet before a
travel test. Results include physical displacements, sampled formation errors,
minimum footprint clearance, action/transition events, and stop acknowledgement.
See `validation/formations/RESULTS.md` for observed results and limitations.

### Original behavior and optional modes

The formation experiments are retained for future manual testing. Keep the
default `LEGACY` mode and `flexible_boundary: true` for normal RViz goal testing.
Request `OPEN_RECTANGLE`, `COMPACT_RECTANGLE`, `SINGLE_FILE`, or `WALL_WORKSPACE`
explicitly on `/formation/mode_request` when resuming those experiments. Their
saved results and unfinished live scenarios are listed in
`validation/formations/RESULTS.md`; preserving them does not imply that every
scenario passed.

The controller starts in `LEGACY`: the original configured flexible boundary,
line transit, and destination work rectangle. The additional modes are opt-in;
no mode request is required for existing primary-goal workflows.
`/formation/rectangle` retains its original work-rectangle behavior and restores
the original dimensions and flexible-boundary configuration.

To return from an optional mode, stop the current mission and request:

```bash
ros2 topic pub --once /formation/mode_request std_msgs/msg/String "{data: LEGACY}"
```

This cancels owned actions and waits for their results before restoring the
original controller configuration. It does not move the robots by itself; the
next primary goal uses the original formation workflow.

### Concurrent single-file assembly and independent recovery

Single-file assembly dispatches all auxiliaries whose verified routes are clear
in the same tick. Each robot advances its own horizontal-clearance and slot
waypoints after its own cancellation result. Bounded detours on both sides of
a blocked approach are checked with actual footprints through turns and travel;
clearance limits are preserved. The primary holds during assembly.

During SINGLE_FILE travel, a blocked auxiliary cancels only its owned goal,
tries the existing checked clearance-waypoint search, and rejoins its retained
travelled-trail slot after the detour. Healthy robots may continue during this
recovery; collision checks still stop conflicting commands. A follower in
recovery does not cause the coordination lag gate to hold the robot ahead.
Final arrival still requires all actual follower trail positions. Status reports
`recovering_robots` and `recovery_waypoints`. An auxiliary with no feasible
detour holds individually and retries; this is not reported as goal completion.

`config/single_file_navigation.xml` is used only for explicit SINGLE_FILE mode.
It allows 2000 ms for action/service responses and replans every five seconds
instead of each second. The original LEGACY Nav2 behavior tree is unchanged.

The added L scenario can be traversed via these map-checked goals (each request
requires READY; all followers settle before the next leg):

```bash
python3 scripts/validate_formation_modes.py --goal 17 1.6 0 --timeout 900 --output validation/formations/concurrent_corridor_entry.json
python3 scripts/validate_formation_modes.py --goal 32 1.6 0 --timeout 900 --output validation/formations/concurrent_corridor_elbow.json
python3 scripts/validate_formation_modes.py --goal 32 19.5 1.5707963267948966 --timeout 900 --output validation/formations/concurrent_corridor_top.json
```

The intermediate entry and elbow goals prevent Nav2 choosing a shorter route
around the outside of the test walls. Use the matching ROS environment described
above. These commands record observed passes or failures, and stop safely after
each leg.

For flexible-formation testing, RViz shows blue robot labels (`tb1` marked
primary), green assigned slots, the primary goal, navigation paths, and the
follower polygon. The large floating status block is removed; use Fleet Live
Updates for goal progress, current actions, and blocked reasons.
