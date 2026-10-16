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

Hilti's Gazebo differential-drive plugin drives two axle pairs (all four wheels).
The imported CAD joint names are opposite physical robot left/right, so each
plugin left joint uses `right_wheel_joint_*` and each plugin right joint uses
`left_wheel_joint_*`. Both pairs use the existing 1.17188 m separation and
0.703125 m diameter. Torque, friction, collision geometry, speed limits and the
supervisor gate are unchanged. SDF drive changes require reloading the Gazebo
model; changing a ROS navigation parameter does not activate a second axle.

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

When reconfiguring tb1 costmaps, stop the supervisor and reset/start the owning
navigation stack through its lifecycle manager. Do not independently clean up
and configure `/tb1/global_costmap/global_costmap` or the local costmap: the
planner/controller can retain references to the previous costmap storage. In a
live test this caused every plan to fail with “start position is off the global
costmap” despite valid localization, exhausting the supervisor retry limit.

```bash
ros2 service call /formation/stop std_srvs/srv/Trigger '{}'
ros2 service call /tb1/lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes '{command: 3}'
ros2 service call /tb1/lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes '{command: 0}'
```

Commands 3 and 0 mean RESET and STARTUP. Require `success: true` at each step.
This preserves Gazebo poses and leaves the supervisor stopped. Verify planning
with `/tb1/compute_path_to_pose` before sending a fresh RViz primary goal;
path computation alone does not command movement. A lifecycle reset reloads the
existing node parameters; changes to YAML files require relaunching the nodes.

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

### Final goal alignment

Mission completion requires position error at most 0.15 m and heading error at
most 0.05 rad for tb1 or 0.2618 rad (15 degrees) for auxiliaries. Nav2 checks
0.10 m, with heading tolerances of 0.04 rad for Hilti and 15 degrees for
auxiliaries, and rechecks XY position during final orientation alignment. The
flexible formation freezes final follower slots around the requested primary
pose and uses its heading, rather than freezing each follower's travel heading.
Feasible alternative slots remain subject to the existing footprint and peer
clearance checks; tolerances apply to each selected target. These are acceptance
limits, not a guarantee of achieved accuracy in every obstacle configuration.

### Hilti priority and independent obstacle routes

Hilti has right of way over every auxiliary for forward and reverse crossings.
The gate still stops both robots if stopping the auxiliary would leave Hilti's
sweep obstructed. After the clearance delay, the auxiliary receives a checked
temporary waypoint out of the way; Hilti waits until that move is safe.

During travel, auxiliary targets and commands reserve the next 2 m (or the
configured speed times collision horizon, whichever is greater) of Hilti's
current map-frame Nav2 path, including its turns and footprint clearance. A
stale/missing path falls back to a preview toward the active goal. An auxiliary
already in this corridor may move outward without reducing its existing gap;
it may not move deeper into the corridor. This is a near-term reservation, not
a full multi-robot space-time planner. Current peer collision checks remain active.

Near-stationary Hilti turns compare safe forward and reverse arcs and prefer
the endpoint closer to the goal; this does not force an arc near final alignment.
Each blocked auxiliary independently checks both lateral detours and straight
escapes, preferring a shorter checked route to its own formation target. Equal
left/right ties are broken independently; an unsafe side is never forced to
make the group split. Once clear, each auxiliary resumes its formation target.

### Stairwell exclusion

The default shared map is `config/EVfloor9_formation_aligned.yaml`. It uses the
GIMP map aligned to the fixed Gazebo reference, with the existing stairwell
exclusion retained and six traced elevator shafts in front of the stair entrance.
The elevator wall layer is `config/EVfloor9_elevator_walls.png`; its unequal shaft
shapes and shared partitions are included by the aligned-map generator, while
the passage between the two banks remains clear.
The standalone RViz launcher displays the running map server's
`/map`; starting RViz alone does not start or reload the map server. Both formation
and shared navigation launches default to this map unless `map_yaml` is overridden.

The original protected map, `config/EVfloor9_stairs_keepout.yaml`, fills the
enclosed stairwell containing tb4's former goal at (41.9143, -0.6827), including
the doorway between (37.8, -0.67) and (37.8, -1.8). The enclosed interior is
about 25.28 m². The original `EVfloor9_outer_only` map remains unchanged.

The occupied region belongs to the static map, so dynamic obstacle clearing
does not remove it. Every robot's global planner and target checks use that
map. The supervisor additionally checks every forward, reverse, arc, and escape
command's swept footprint against `/map`; local lidar cannot authorize entry
into the exclusion. Existing footprint padding and peer clearance remain active.

Regenerate the protected image from the documented enclosure reference with:

```bash
python3 scripts/generate_stair_keepout.py --spec config/stair_keepout.yaml
python3 scripts/generate_aligned_formation_map.py
```

The generator seals the approximate door endpoints into the existing walls and
fills only the connected interior. It refuses an enclosure that leaks outside
the local search boundary or exceeds the expected maximum area. No Gazebo
physical walls or robot spawn positions are changed.

### Give Hilti extra maneuvering room

During flexible fleet travel, `primary_space_enabled` temporarily suspends
formation chasing when Hilti's padded map footprint has little surrounding
space, its motion is blocked, or its progress stalls while auxiliaries are close.
Auxiliaries seek outward parking positions rather than approaching their normal
formation slots. The preferred buffer reserves Hilti's full rotation radius,
the auxiliary radius, normal primary separation, and an additional 1.2 m.

Parking candidates prefer the greatest available separation within a 3 m local
search, stay out of Hilti's next 6 m of planned route, and check swept footprints
against all three maps, stationary peers, and already selected parking positions.
A checked partial retreat is allowed when it gains space but the full buffer
cannot fit. A robot already clear continues toward its final formation slot through
checked waypoints outside Hilti's reserved space. One without a complete
outward route tries its own checked reverse or reverse-arc recovery rather
than cancelling that recovery on every parking update. When no checked
motion is available, it holds rather than squeezing toward Hilti. Targets remain stable while the
robot moves. All available parking goals are selected in the same supervisor
update and sent through independent asynchronous Nav2 actions; no auxiliary
waits for another to arrive. Conflicting routes still yield through the gate.

The velocity gate rejects inward auxiliary motion inside this buffer, including
local recovery commands. Auxiliaries can turn in place to depart when the normal
physical collision and map checks permit it. Future-route reservations allow an
already intruding robot to depart without decreasing its centre distance to the
route; all original physical footprint and separation limits still apply.

Formation chasing resumes only after Hilti has remained clear for 5 simulation
seconds. If its final pose remains tight, the auxiliaries continue holding clear
rather than forcing a crowded final formation. Status includes
`primary_space_active` and the temporary `primary_space_targets`.

Hilti starts a local recovery after a persistent velocity-gate or peer-conflict
block for `primary_maneuver_delay` (0.5 simulation seconds). A Nav2 action that
stalls without translation also triggers recovery. The supervisor first cancels
its owned action and waits for cancellation before commanding motion. It prefers
checked reverse and reverse arcs, then considers forward arcs. Hilti does not fall back to a stationary turn
in this recovery.
After backing at least 0.15 m, it checks the complete forward heading
correction (up to 10 simulation seconds of swept motion). It keeps backing
until that arc is clear, then drives forward toward the goal. Once selected, the forward command stays committed and is rechecked against
current maps and peers each update. Goal-bearing changes cannot switch it
back into reverse. Nav2 resumes after 0.25 m of forward progress, or after
0.15 m when aligned within 0.12 rad or within 0.4 m of the goal. A newly
blocked forward segment stops and replans after one simulation second,
without changing gear. Near-goal heading alignment is not treated as a
translation stall; angular progress toward the final heading counts as progress.
Each maneuver is bounded by 1.8 m displacement, 1.2 rad rotation, or 35
simulation seconds; reaching a bound also replans the original goal. All swept-map and peer-clearance checks remain
active; starting inside costmap inflation uses the existing inflation escape
policy. An unverified escape produces zero velocity. Three attempts without
0.3 m of actual progress toward the goal enter an explicit waiting state;
backing away does not reset that budget.

Auxiliary escape checks include both stationary and commanded peer motion.
Reverse and reverse arcs are considered even when the current footprint is
already outside costmap inflation. Action cancellation must finish first;
all commands retain the supervisor velocity gate and physical clearances.
Partial outward waypoints may progressively leave the reserved route rather
than requiring a single waypoint to clear the entire corridor.

If an auxiliary starts with a mapped overlap while lidar verifies its physical
body is clear, a checked reverse may leave that overlap. The full configured
padding plus the normal static guard allowance is retained: occupied static-map
cells under the footprint must form a shrinking subset at every sample, with
no new occupied or unknown cells. Global lethal cells obey the same rule;
inflation uses the existing non-deepening escape policy. Lidar checks the body
throughout the swept motion. This escape may temporarily cross the extra
maneuvering buffer, but all physical peer clearances and the reserved-route
gate still apply. It cannot authorize entry into the stair exclusion.

### Nearby alternatives for obstructed auxiliary slots

Auxiliary formation slots can move within `alternative_radius` (1.5 m by
default), searched from the nominal slot outward in `candidate_step` increments
(0.3 m). Each candidate keeps the slot heading and passes full map, footprint,
peer, and Hilti-reservation checks. Nav2 must produce a path before it receives
a navigation goal. A failed path or stalled auxiliary goal rejects that local
candidate and tries a nearby position instead of repeatedly retrying it.
Rejected positions are remembered for the current mission; a new primary goal
resets them. Final slots are revalidated against current maps and peers.
A selected alternative stays stable while it remains feasible. The nominal
anchor stays fixed, so repeated failures cannot drift the slot arbitrarily far
away. If the neighborhood has no safe candidate, the auxiliary holds rather
than entering an obstacle. Clear auxiliaries also use these adjusted slots
while Hilti is maneuvering. Status reports `auxiliary_slot_adjustments`, with
the nominal position, selected position, and offset in metres.

### Short steering sequences in tight spaces

`local_maneuver_search` incrementally compares up to three two-second steering
segments. Both forward and reverse left/right arcs, straight motion, and
auxiliary stationary turns are candidates. Different first steering directions
remain in the search, allowing a forward-left arc followed by a right arc or a
change of gear. A sequence must improve progress or obstacle-buffer clearance;
loops without improvement are rejected. Search work is bounded per supervisor
update, and the robot holds while its search is pending.

Every segment checks swept oriented footprints against maps, stationary peers,
and predicted commanded peer motion. Each executing segment is checked again
against current data; a new obstruction stops it and requests another search.
Action ownership, the final velocity gate, stair exclusion, footprint padding,
and 0.4 m auxiliary / 0.6 m Hilti physical clearance settings remain active.
The extra maneuvering buffer can be used temporarily during a checked sequence,
whose endpoint must restore that preferred spacing (or improve the original
crowded spacing). This avoids rejecting a useful first turn solely because its
centre briefly moves closer while its actual footprint remains clear.
Existing mapped-overlap escape checks also apply to forward arcs that shrink
the overlap without adding any new occupied static-map cells.

### Keep settled auxiliaries in place

After Hilti reaches its goal, each auxiliary independently locks its accepted
slot as soon as it meets the position and 15-degree heading tolerances. Slot
assignment and nearby-slot searches preserve that position while the remaining
robots finish. Clearance recovery prefers moving the unfinished neighbor rather
than moving a settled auxiliary based on robot number. A new primary goal
releases these locks. Otherwise only a newly detected obstacle at the held
position releases that robot: an increase in local obstacle overlap or an
actual peer entering its physical separation envelope. Missing sensor data,
position-estimate drift, slot swaps, planned routes, and parking preferences
do not release a settled robot. Its commanded linear and angular velocities
remain zero through the final gate while held.

Publishing an auxiliary name on `/formation/hold_auxiliary` explicitly accepts
its current feasible position as its slot for the current mission. Cancellation
retains the supervisor's action ownership until Nav2 returns its result. Status
reports these positions in `held_auxiliary_slots`.

Parking-style recovery first checks broad reverse arcs of at least
`parking_reverse_distance` (0.65 m), followed by forward steering out of the
space. At 0.12 m/s and the normal two-second checked segments, this produces
about 0.72 m of continuous backing. `parking_turn_rate` defaults to 0.15 rad/s
(about 0.8 m turning radius). Consecutive identical controls execute without
stopping between samples. Short reverse runs cannot immediately change gear
or reverse steering direction; gear and steering changes also carry a search
penalty. The complete maneuver is checked before execution, and current safety
checks may still interrupt it. Nearby obstacle-buffer clearance and room for
turning contribute to the score, so backing away from the goal can be useful.

Moving auxiliary slot selection prefers the nearest feasible candidate whose
entire padded footprint is at or below `preferred_slot_cost` (0, clear by
default). It checks global costmap inflation and the local costmap where the
candidate is covered. If no clear candidate exists within `alternative_radius`,
it chooses the lowest-cost feasible nearby candidate. Held final slots remain
locked. Supervisor reloads can restore the prior mission and accepted holds
atomically through `/formation/restore_mission`, without publishing a new goal
that briefly releases those robots.
