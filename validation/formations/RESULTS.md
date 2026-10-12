# Formation validation record

Unit suite: 124 passed (`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests -q`, ROS Humble sourced).

Startup preserves the original LEGACY controller, flexible boundary and configured dimensions. Optional modes require an explicit request. Legacy runtime readiness and idle actions are recorded in `legacy_default_runtime.json`. This is a readiness observation, not a completed legacy navigation test.

| Recording | Observed result |
| --- | --- |
| compact_rectangle.json | True |
| compact_rectangle_timeout.json | False |
| compact_to_open.json | True |
| corridor_top_goal.json | False |
| horizontal_first_single_file.json | True |
| open_before_travel.json | True |
| open_rectangle.json | True |
| open_sustained_movement.json | True |
| scenario_open.json | True |
| single_file_approach_blocked.json | False |
| single_file_before_corridor.json | True |
| single_file_cancelled_for_horizontal_first.json | False |
| single_file_stationary_peer_blocked.json | False |

Open rectangle assembly, compact/open transitions, sustained open-rectangle movement, and horizontal-first single-file assembly passed their recorded checks. Earlier failed or interrupted trials are retained. The corridor-top trip failed early when a follower trail waypoint was rejected; arrival through the bend is not validated. Wall-workspace live assembly and single-file-to-rectangle return remain unvalidated.

The ROS MCP could not inspect fleet domain 42; RViz MCP was in mock mode and Gazebo MCP did not find fleet models. These are not live validation passes. Actual fleet observations use ROS CLI/rclpy and the real GUI.

Ground-truth Gazebo poses are recorded separately from map TF. The verified static world/map alignment does not demonstrate AMCL localization accuracy. Clearance checks, command ownership and velocity gate remain enabled.

Reproducible launch and request commands are in `docs/FLEET_MISSIONS.md`. Existing EV9 assets are preserved; the added scenario uses its own aligned world/map. Performance tuning and successful corridor navigation remain outstanding.

Concurrent rerouted assembly passed its live check (71.53 seconds wall time).
The final retry moved primarily tb2; the other three had already moved
concurrently in the preceding interrupted recording. Do not interpret this
retry alone as proof of four simultaneous departures.

Single-file travel to the corridor entry passed: all five moved approximately
14.1 metres, minimum observed physical footprint clearance 1.121 metres, and
all five zero commands were observed after stop. The elbow and earlier direct
top trials were interrupted to follow operator routing/goal instructions.
The replacement top goal from inside the corridor is currently being recorded
in `corridor_top_from_inside.json`; arrival is not yet a validated pass.

Independent per-auxiliary recovery and return to the retained trail have unit
coverage. That latest recovery revision has not yet been loaded into the moving
supervisor or live fault-tested. The running trip uses the preceding tested
concurrent-assembly/trail-retry revision. Load the latest controller only while
stationary. Further speed tuning and live recovery validation remain open.

## Operator handoff: original flexible formation restored

Corridor experiments were stopped at the operator's request. The original EV9
world/map and configured `fleet_spawn.yaml` positions were restored, with
`LEGACY` active, `flexible_boundary: true`, and no mission queued.
`flexible_restored_spawn.json` passed: READY, all five zero motor commands,
spawn position errors below 0.025 metres, and no overhead status marker.
The real RViz screenshot is `flexible_restored_rviz.png`. Gazebo server and
RViz are running; no Gazebo GUI is running. Goal testing is handed back to the
operator using RViz's 2D Goal Pose publishing `/formation/primary_goal`.

The overhead black status block is removed; blue robot names, assigned green
slots during movement, navigation paths, primary goal and follower polygon
remain. Detailed status stays in Fleet Live Updates.

The earlier `corridor_top_from_inside.json` failed with a follower motion stall.
`corridor_top_turn_fix.json` was interrupted for this operator handoff and does
not validate corridor arrival. The latest turn/recovery fixes have unit
coverage (124 passing tests); successful live traversal of the bend remains
unvalidated. No further corridor goals are running.
