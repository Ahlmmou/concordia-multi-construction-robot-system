# Live validation — 2026-10-05

The controller is implemented, but coordinated missions are **not yet validated**.
The assembly test was stopped when the separation gate held tb2 near the leader
and its localization disagreed with Gazebo by approximately 0.7 m. All motors
were stopped through `/formation/stop`. Do not treat this as a completed mission.

## Verified preflight

ROS2 MCP inspected the original domain-0 graph. Duplicate simulation launches
and legacy goal-forwarding controllers were stopped. The clean test ran with
ROS_DOMAIN_ID=42 and rmw_cyclonedds_cpp; terminal/rclpy tools inspected this
isolated domain because the MCP server remained configured for domain 0.

Exactly five models: Hilti tb1 and auxiliaries tb2–tb5. Actual raw TF frames are
unprefixed map/odom/base_footprint/base_link in separate namespaces. Clock,
scan topics, localization, both costmaps, NavigateToPose and ComputePathToPose
servers were observed. The controller reached READY after checking all required
lifecycle nodes active and exclusive command ownership.

`/map_server` was discoverable and active. Published `/map` exactly matched the
original EVfloor9_outer_only PGM raster. All **81,157 occupied wall cells** were
occupied in every robot's global costmap. Static/inflation layers and lidar
subscriptions were present. The preserved map and floor mesh match HEAD hashes.
See [preflight.json](preflight.json); one early tb2 lifecycle query timed out,
then the controller's subsequent readiness checks completed.

## Individual navigation

Each row requires Nav2 status SUCCEEDED plus actual TF and Gazebo displacement.
These are short, sequential free-space tests through the command gate, not
obstacle-detour tests.

| Robot | Gazebo displacement | TF displacement | Result |
|---|---:|---:|---|
| tb1 | 0.515 m | 0.501 m | Passed |
| tb2 | 0.527 m | 0.507 m | Passed |
| tb3 | 0.524 m | 0.494 m | Passed |
| tb4 | 0.817 m | 0.668 m | Passed, including turn |
| tb5 | 0.526 m | 0.509 m | Passed |

The first Hilti test timed out after movement; DWB and goal-checker tolerances
were inconsistent. Both were corrected and fresh stacks passed the retest.
tb4's first candidate was rejected by footprint/map checks before sending an
action; a verified-free alternative passed. Evidence is retained in
navigation_first_three.json, tb4_navigation.json, tb5_navigation.json and
initial_navigation_failure.json.

## Coordinated test and remaining work

A verified-free leader target was accepted. Sequential FORMING_LINE assembly
moved tb2 approximately 4.7 m while tb1 and the other auxiliaries held position.
The pairwise gate then blocked tb2's approach to the stationary leader. This
shows conflict detection holding motion; it does **not** demonstrate successful
rerouting. Localized tb2 position then differed from Gazebo by approximately
0.7 m. The test was interrupted and stopped; evidence is in
[coordinated_assembly.json](coordinated_assembly.json).

The final controller adds a configurable Gazebo/localization discrepancy check
(default 0.35 m) so this condition prevents further movement. Short individual
tests do not establish long-distance odometry/localization accuracy. Resolve
auxiliary slip/calibration/localization and add or verify planning around the
coordinator's reserved peer clearance before repeating coordinated assembly.
The conservative swept-disc gate can otherwise deadlock where Nav2 sees a
geometrically narrower passable route.

On 2026-10-06 the final controller was restarted without a motion request.
It remained in PREFLIGHT, correctly reporting tb1's then-current 0.65 m
Gazebo/localization disagreement and holding zero motor commands. The added
localization check therefore also received a live refusal-path check.

TRAVELLING, static-obstacle detours, unexpected moving-obstacle avoidance,
destination rectangle, relaxed-position rejoining, and repeated complete
missions were **not reached/tested live**. Their implementation is provided,
but acceptance remains pending these incremental tests. Fixture commands in
the guide are instructions, not evidence of executed tests.

Package build passed. Twelve geometry/action-ownership unit tests passed.
Those tests verify footprint checks, curved trail placement, conflict gating,
and suppression of redundant goal cancellation; they do not replace live
mission acceptance. Full commands and recovery instructions are in
[FLEET_MISSIONS.md](../FLEET_MISSIONS.md).
