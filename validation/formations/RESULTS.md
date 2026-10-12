# Formation validation record

Unit suite: 122 passed (`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests -q`, ROS Humble sourced).

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
