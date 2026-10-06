STEP 1: spawn correction verified; tb1 navigation BLOCKED.

The original running launch PID 660 loaded `worlds/ev_9.world`. The saved live SDF confirms `ev_9` at (-19.76, -7.98), floor link at (30, -2), yaw 1.5708, and the expected `EVfloor9_outer_only.dae` for both collision and visual geometry. The composed mesh transform is (10.24, -9.98), yaw 1.5708. DAE scene-node transforms and metre units are applied before that transform.

The simulation runs on ROS domain 0, with Cyclone DDS and software GL. `/map_server` is active and is the only `/map` publisher, loading the expected YAML/PGM. Earlier “Node not found” is not the current state; a read-only probe on domain 42 found no simulation nodes. The ROS MCP topic-list result was also incomplete; direct domain-0 graph inspection was used.

Before moving any coordinates, Gazebo world poses were compared with odometry and map poses. Their planar coordinates agree to millimetres; namespaced TF uses unprefixed `map -> odom -> base_footprint`. These are world/map metres, not DAE-local coordinates. Auxiliary map-to-odom transforms have small pitch/height effects; no exact 3D identity is claimed.

The closest robot to the slanted wall is tb4, actual pre-restart position (5.14781, -2.30001), yaw approximately -0.00075. Its nearest mesh wall is `Basic Wall Generic - 8" [125099]`. Tb5, at approximately (-3.25255, 4.49999), has ample clearance; the screen description alone would misidentify the robot.

The whole group was translated (-0.20, +0.50) m, retaining the 8.4 by 6.8 m corner rectangle and centred tb1. `config/fleet_spawn.yaml` now supplies all Gazebo spawn and AMCL initial x/y/yaw values, including quaternion yaw in the legacy initialpose publication. Launch syntax and the loaded runtime AMCL parameters agree. No formation controller was implemented and no navigation goals were sent.

The configurable margin is 0.30 m, departure allowance 0.30 m, and footprint padding 0.05 m. Clearance is measured outside a complete 360-degree turning envelope; requiring 0.65 m outside it also accommodates a 0.30 m initial departure in any direction. Nav2 envelope radii are 1.63783 m for tb1 and 0.56181 m for auxiliaries. Physical SDF collision projections fit within the configured footprints: physical rotation radii are 1.41865 and 0.51347 m. Conservative triangle projections cover wall geometry up to 3.3 m height. Map occupied and unknown cells are treated as obstacles, with a half-cell-diagonal allowance. All ten robot pairs pass, including simultaneous turn/departure allowances.

Measured after restart (x/y in metres, yaw in radians; clearances outside the unpadded turn envelope):

| Robot | x | y | yaw | Mesh clearance | Map clearance |
|---|---:|---:|---:|---:|---:|
| tb1 | 0.49999 | 1.60006 | -0.000000 | 4.008 | 3.819 |
| tb2 | 4.70824 | 5.00000 | -0.000035 | 7.849 | 7.621 |
| tb3 | -3.69227 | -1.80000 | -0.000038 | 1.684 | 1.470 |
| tb4 | 4.70728 | -1.80000 | -0.000030 | 1.300 | 1.029 |
| tb5 | -3.69317 | 5.00000 | -0.000029 | 8.484 | 8.269 |

The navigation map is more conservative than the mesh near the group: nearest-obstacle distances differ by about 0.19–0.27 m, up to 0.27 m for tb4. The first candidate passed mesh/map checks but its departure-margin envelope entered tb4's inscribed-cost inflation. The final group translation was increased by 0.20 m north and checked against actual costmaps. Both sources were checked independently; they were not treated as identical. All five complete padded turn/departure/margin disks also contain zero lethal, inscribed, or unknown cells in their own global costmaps (`costmap_clearance.json`); residual soft inflation is reported, not mistaken for walls. See `configured_clearance.json` and `live_clearance.json` for complete wall and pair checks.

After restart `/clock` advanced from 5.5 to 7.3 seconds during the audit. Tb1 AMCL, controller, planner, smoother, behavior server, BT navigator, waypoint follower, velocity smoother, and both costmaps are active. `/tb1/navigate_to_pose` is available. The published map matches the edited PGM exactly and all 81,157 wall cells are present as obstacles in tb1's global costmap. The global map and local odometry frames and expected TF edges are present. Command publishers are only Nav2 controller/behavior servers and velocity smoothers; no competing teleoperation or formation publisher is present.

BLOCKED: tb1's scan delivered 159 messages with zero finite returns in every message. Its 3.3 m maximum range is shorter than the nearby walls in the corrected open area, and its FOV is only about 60 degrees. AMCL has the requested initial pose but that is not evidence of sensor-based localization convergence. Tb1 velocity smoother also has `use_sim_time=false` while odometry and Nav2 use simulation time. These existing navigation issues were inspected and left for the next step.

Smallest next task: set tb1 velocity smoother to simulation time and verify its clock. Then inspect the leader lidar range/FOV and obtain finite wall returns before testing localization or navigation.

The original launch file is backed up in `backups/step1_20261006_123355/`. The complete live world, model poses/velocities, runtime parameters, and localization were preserved before the clean restart in this directory. The MCP named snapshot reported only two models, so the complete seven-model SDF and explicit five-robot state capture are the preservation evidence. Existing user changes were retained.

RViz verification: one live five-robot map view is now running. RViz uses `map`, with visualization-only `tbN/odom` and `tbN/base_footprint` prefixes. Each displayed pose matches its original namespaced map pose exactly; the maximum measured difference from Gazebo world x/y is 6.01 mm. All 44 URDF link frames resolve, and all five displayed Nav2 polygons match their source polygons. Top-left is tb5, top-right tb2, bottom-left tb3, bottom-right tb4, with tb1 at the centre (unrotated map view). Auxiliary physics drift since spawning is also present in Gazebo and tracked by RViz; it is not a display coordinate offset.

`rviz/fleet_spawn.rviz` persistently enables all ten global/local costmaps. The native RViz screenshot `rviz_spawn.png` shows the costmaps, models, and footprints. Scan QoS is Best Effort to match the relay. The main simulation launch now uses the single RViz-only launch when `enable_rviz=true`; the existing session was updated without restarting Gazebo or sending goals.
