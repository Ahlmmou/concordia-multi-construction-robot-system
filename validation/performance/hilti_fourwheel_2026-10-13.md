# Hilti four-wheel drive validation — 2026-10-13

The previous Gazebo plugin drove only the first axle pair. The updated SDF drives both pairs with the existing wheel dimensions. Torque, acceleration, friction, speed limits, collision geometry and supervisor command ownership are unchanged.

## Controlled isolated comparison

Two models, original and updated, ran side by side in a separate headless Gazebo server, ROS domain 84. Commands never went directly to the live fleet. Each motion command lasted four simulation seconds, followed by two simulation seconds of zero commands.

| Command | Original observed motion | Four-wheel observed motion |
| --- | --- | --- |
| +0.30 rad/s turn | +0.00199 rad/s average | +0.09388 rad/s average |
| −0.30 rad/s turn | −0.00314 rad/s average | −0.07080 rad/s average |
| +0.25 m/s straight | 0.9931 m distance | 0.9935 m distance |
| −0.25 m/s reverse | 0.9520 m distance | 0.9789 m distance |

Turn response improved about 47 times for positive turns and 23 times for negative turns in this isolated test. Actual turn rates remain below commanded rates. Stop phases were recorded; maximum updated-model stop-phase displacement was about 0.021 m after reversing. These are model response measurements, not whole-formation journey speedups. Raw poses, joint rates and commands are in `hilti_fourwheel_isolated_2026-10-13.json`.

## Live fleet validation

Stopped the supervisor and verified zero commands before replacing only tb1 at its saved physical pose. All five position changes after settling were below 1.2 mm. Gazebo stayed headless; RViz and its relay stayed active. See `hilti_fourwheel_apply_2026-10-13.json`.

Requested a +0.25 rad heading change through the formation supervisor, with both Hilti heading tolerances at 0.10 rad. The first recorded successful tb1 action was at 65.31 wall seconds / 13.6 simulation seconds, with heading error 0.08185 rad. After stopping, the physical heading error was 0.09925 rad. The earlier heading test completed in 146.55 wall seconds / 39.6 simulation seconds. Starting poses and auxiliary workload differed; this is not a controlled live before/after benchmark.

The overall formation test FAILED its 180-second timeout because auxiliaries did not all finish. The saved validator and wrapper retain `passed: false`; Hilti's successful action and acceptable physical heading do not convert the full mission into a pass. Minimum observed footprint clearance was 1.5075 m; safe stop was acknowledged and every final motor command was zero. No clearance threshold was reduced.

Raw live validation and physical-heading summary are the corresponding `hilti_fourwheel_heading_*_2026-10-13.json` files. The isolated probe was shut down after testing. The live fleet was left STOPPED.

## Automated checks

165 tests passed, including a geometric drivetrain regression verifying every physical wheel is driven exactly once, left/right pairing, axle alignment and per-pair separation. Gazebo SDF validation and `git diff --check` passed.
