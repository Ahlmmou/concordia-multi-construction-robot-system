# Physics and simulation localization follow-up (2026-10-08)

Baseline live real-time factor was 0.16–0.20 with a 1 ms default ODE timestep. A 4 ms timestep and target rate 250 increased measured factors to 0.363 stationary, 0.390 during partial motion, and 0.409 during resumed boundary transit. This is about 2.5× the previous 0.16 rate, still below real time. The world persists the new physics settings.

AMCL drift independently paused the current mission at >0.35 m disagreement with Gazebo. One pose reset did not prevent later drift. All five Gazebo odometry positions were independently compared against physical ModelStates and agreed within 0.03 mm. The fleet simulation now defaults to identity map-to-odom transforms and disables AMCL TF broadcast; sensor-based localization remains available with simulation_localization:=false. This is simulation ground-truth positioning, not AMCL convergence evidence. The physical position disagreement, collision and boundary gates remain enabled.

After enabling world positioning, the existing RViz primary goal resumed. A 20-second observation recorded enclosure in all 397 ModelStates samples; peak tb1 odometry speed was 0.456 m/s in that window. Some followers had already reached their current segment, so this observation does not measure every robot's maximum speed. Five-robot world-odometry alignment and all five runtime AMCL broadcast flags were verified.

Sixteen formation/action tests pass; both true and false localization launch rewrites were checked, and the world physics XML target rate multiplies to one simulation second per wall second. No physics collision or sensing geometry was modified.

Final moving check: real-time factor 0.386; tb1 enclosed in all 379 samples. All five robots physically moved in the 20-second window: {'tb1': 1.138510293001521, 'tb2': 1.326415876911971, 'tb3': 1.3722473123315473, 'tb4': 1.1376482337462859, 'tb5': 1.3006426535148268}. The current operator goal remains active in BOUNDARY_TRAVELLING, with no blocked robots.
