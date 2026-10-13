# Concordia multi-construction robot system

Five-robot Gazebo simulation with ROS 2 Humble and Nav2. `tb1` is the Hilti/Jaibot primary robot; `tb2`–`tb5` are auxiliaries. The fleet supervisor owns navigation commands and enforces collision checks and a velocity gate.

**Continue development on `test_mcp`.** Read [AGENTS.md](AGENTS.md) before changing this repository. This README replaces the old prototype/master-branch setup instructions.

## Development handoff — 2026-10-13

The latest simulation implementation at this handoff is commit `4371ff1`. The default formation is the flexible rectangle (`LEGACY` mode). Open rectangle, single-file and other experimental modes also exist; do not change the default while reproducing the flexible formation.

Completed changes:

- Interchangeable auxiliary slots remain enabled. Assignment searches the 24 permutations of four slots, with a switching penalty to avoid needless reassignment. Slot targets update along the primary robot's path; the RViz green slots show the allocated targets. Full geometry and feasibility behavior are documented in [FLEET_MISSIONS.md](docs/FLEET_MISSIONS.md).
- `a8d5648`: corrected future-target reservation deadlocks and allowed flexible auxiliaries to recover independently. A stuck auxiliary should recover while the others move when safety permits. Primary navigation failures and data-safety checks can still pause the fleet.
- `a5981a1`: disabled unused Nav2 DWB trajectory debugging on all five robots. Green formation slots and fleet visualization remain available. No measured overall journey-time improvement was established for this change.
- `9f9da68`: prepares updated flexible-slot paths before cancelling the old follower navigation action, reducing the handoff gap while retaining action ownership and collision checks. No matched whole-mission speed benchmark was established.
- `49ce174`: both Hilti's Nav2 goal checker and supervisor require final heading error within **0.10 rad (about 5.7 degrees)** of the RViz goal arrow. Auxiliary supervisor yaw tolerance remains 0.45 rad.
- `4371ff1`: Hilti's differential-drive plugin now drives **both axle pairs/all four wheels**, with explicit dimensions for each pair. Previously only the first pair was driven. Torque, acceleration, friction, geometry and speed limits were preserved.

Configured supervisor limits are 0.60 m/s for Hilti, 0.80 m/s for auxiliaries and 0.50 rad/s angular speed. These are limits, not guaranteed physical speeds. Wall-clock duration also depends on Gazebo's real-time factor; previous observations were approximately 0.22 while moving and 0.4 while idle on the original machine.

### What was tested and what remains unresolved

- **165 automated tests passed** after the four-wheel drivetrain change.
- Controlled isolated Gazebo comparison: at a ±0.30 rad/s command, the new model's average turn response improved approximately 47 times for positive turns and 23 times for negative turns. Straight/reverse motion was also checked. These are isolated drivetrain measurements, not full-formation journey speedups.
- In the latest live +0.25 rad heading test, Hilti's navigation action succeeded at approximately **65 wall seconds / 13.6 simulation seconds**. Final physical heading error after stopping was 0.09925 rad, within tolerance.
- **The full formation test failed its 180-second timeout because auxiliaries did not all finish.** Do not report this as a passed formation mission. The earlier heading test took about 147 wall seconds, but starting poses and auxiliary workload differed, so it is not a controlled live timing comparison.
- Minimum observed footprint clearance in the latest live test was 1.5075 m. The supervisor acknowledged stop and all final motor commands were zero. No clearance threshold was reduced.

Evidence and limitations: [four-wheel validation report](validation/performance/hilti_fourwheel_2026-10-13.md), with raw JSON captures beside it.

**Next task:** reproduce the default flexible formation, then investigate auxiliary final-position/heading convergence and Nav2 retries. Preserve interchangeable slots, independent recovery, supervisor ownership and all collision clearances. Measure simulation time separately from wall time before claiming performance gains.

The original live fleet was left STOPPED, with Gazebo headless and RViz/relay active. This describes the end of testing on that machine; it does not describe a fresh clone's runtime state.

## Set up another machine

Use **Ubuntu 22.04, ROS 2 Humble and Gazebo Classic** to reproduce the tested stack. On Windows, use an Ubuntu 22.04 WSL2 environment with GUI support for RViz. Install ROS Humble and configure its apt repository first; see the [ROS installation instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

```bash
sudo apt update
sudo apt install git ros-humble-desktop ros-humble-gazebo-ros-pkgs \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-turtlebot3-navigation2 ros-humble-rmw-cyclonedds-cpp \
  python3-colcon-common-extensions python3-rosdep python3-pytest

mkdir -p ~/crs_ws/src
cd ~/crs_ws/src
git clone --branch test_mcp \
  https://github.com/Ahlmmou/concordia-multi-construction-robot-system.git

source /opt/ros/humble/setup.bash
cd ~/crs_ws
# Run once if rosdep has not already been initialized:
# sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

Rebuild locally rather than copying `build/`, `install/` or Python virtual environments from the old machine. These setup instructions were assembled from the existing environment; a clean-machine installation has not yet been validated.

### Launch the fleet

In each ROS terminal, source the environment and use matching domain/middleware settings:

```bash
source /opt/ros/humble/setup.bash
source ~/crs_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch crs_multibot fleet_simulation.launch.py \
  gazebo_gui:=false rviz:=true
```

Run exactly one fleet simulation launch. The launch preserves fleet TF and RViz relay configuration. Do not run prototype goal senders or teleoperation alongside the supervisor.

For mission controls, RViz testing, stop/resume and validation commands, use [FLEET_MISSIONS.md](docs/FLEET_MISSIONS.md). That guide also documents an alternative isolated build under `/tmp`; source the installation you actually built, consistently across ROS and MCP processes.

To run automated checks from the repository:

```bash
source /opt/ros/humble/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests -q
```

### Codex and MCP migration

Install Codex on the new machine and sign in with your account; see the [official Codex setup](https://developers.openai.com/codex/cli). Start Codex from this repository so it reads `AGENTS.md`.

The simulation repository does **not** contain the original machine's Codex/MCP installations. Transfer the configuration privately and recreate the tool environments:

| Original machine location | Purpose |
| --- | --- |
| `/home/umroot/.codex/config.toml` | Codex settings and MCP server launch configuration |
| `/home/umroot/mcp/ros2_mcp/` | ROS 2 MCP source, upstream `wise-vision/ros2_mcp` |
| `/home/umroot/mcp/gazebo-mcp/` | Gazebo MCP source, upstream `kvgork/gazebo-mcp` |
| `/home/umroot/.local/bin/rviz-mcp` | RViz MCP entry point; reinstall its package/environment rather than copying this executable alone |

Update absolute paths for the new username and workspace. Keep credentials outside Git. The original ROS MCP runs through `uv run mcp_ros_2_server`; the Gazebo MCP runs through its Python environment with `python -m gazebo_mcp.fastmcp_server`. Both source ROS Humble and the workspace first. Consult each server's own installation instructions and preserve the Classic/Humble backend rather than automatically selecting a newer Gazebo/ROS stack.

Configure ROS/Gazebo MCP processes with the **same `ROS_DOMAIN_ID=42` and `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` as the fleet**. Codex normally reads `~/.codex/config.toml`; use `codex mcp list` and `/mcp` inside Codex to check connections. See [official MCP configuration documentation](https://developers.openai.com/codex/mcp).

**Verify live connectivity.** The RViz MCP previously returned mock results, and Gazebo MCP tools can fall back to mock data. A successful tool response alone does not prove it inspected this simulation. Confirm actual ROS topics, five robot models, live simulation clock and physical motion using ROS CLI/rclpy when necessary.

### Context to give the next Codex session

> Continue on `test_mcp`. Read `AGENTS.md`, the development handoff in `README.md`, and `docs/FLEET_MISSIONS.md`. The simulation baseline is `4371ff1`: interchangeable flexible slots, independent auxiliary recovery, prepared slot-path handoffs, reduced Nav2 debugging, 0.10-rad Hilti heading tolerance and all-four-wheel Hilti drive. Run Gazebo headlessly with RViz open. Preserve supervisor ownership, collision checks and velocity gating. The latest Hilti heading action succeeded, but the full formation timed out waiting for auxiliaries. Reproduce and investigate auxiliary convergence next. Distinguish live MCP results from mocks and wall time from simulation time.

Git preserves the code, configuration and committed validation evidence. It does not preserve running processes, current robot poses, temporary scripts under `/tmp`, local backups, installed packages, authentication or the full previous conversation. A fresh launch uses the repository's configured spawn poses.
