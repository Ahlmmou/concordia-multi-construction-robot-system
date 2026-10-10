# Validation

All 58 regression checks pass with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q tests`. This includes crossings with each auxiliary, auxiliary priority, head-on stationary-peer protection, third-robot protection, rotating footprints, automatic release, safe clearance candidate selection, map rejection, frozen recovery targets, yielding action failures and stalled in-place-turn detection. Shell syntax and Git whitespace checks pass.

The original stalled goal was replayed for 150 seconds. tb1 yielded to tb4 repeatedly and automatically resumed; tb1 moved 7.22 m and tb4 7.81 m. All auxiliaries moved. The minimum observed conservative physical footprint clearance was 0.458 m, above the configured 0.4 m separation. The controller stayed out of WAITING. A short localization verification pause recovered automatically. See live_replay.json.

After loading the final stalled-turn recovery refinement, a 75-second continuation reached the leader's arrival tolerance and left the followers assembling their final positions. All robots advanced, with minimum measured physical footprint clearance 0.834 m. Several localization verification pauses recovered automatically. No new yielding encounter or temporary clearance maneuver occurred during that continuation; waypoint recovery is covered by regression checks, not a live maneuver in these runs. See final_continuation.json.

## Obstacle escape and reverse-until-clear

All 85 regression checks pass. New checks cover backing out of inflation, following a buffer edge in reverse, rejecting lethal/unknown cells and rear obstacles, static/peer verifier rejection, small overlap thresholds, automatic release, and safe slowdown before a stop.

In the initial recovery replay tb2 advanced over 11 m, and tb5 performed a short reverse escape. The final 100-second reverse-until-clear replay confirmed tb5 executed -0.12 m/s reverse commands, cleared the buffer, requested fresh navigation and advanced over 6 m toward its formation position. The final snapshot had no blocked robots or active obstacle recovery, and tb5 retained an active navigation action. Measured minimum physical footprint clearance was 2.305 m. See reverse_recovery.json. The fleet remained in BOUNDARY_TRAVELLING; this does not establish completion of the primary goal.

## Wider turns and target clearance

The final suite passes all 88 tests. Added coverage checks oriented footprint clearance for reserved formation targets and wider arcs for slow turns. Tb1 now uses 0.22 m/s rolling arcs at 0.15 rad/s when the full swept path is safe, giving a nominal radius of about 1.47 m.

In the subsequent 100-second live replay tb1 regained an active navigation action and moved 0.934 m; observed minimum physical footprint clearance was 2.166 m. It did not reach the destination. Physical rotation remained much smaller than the commanded rotation, so these changes do not establish that the simulated drivetrain turning problem is resolved.
