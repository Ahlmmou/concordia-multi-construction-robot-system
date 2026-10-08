# Boundary turn correction

The original follower-distance gate zeroed both linear and angular commands whenever any corner was more than 0.8 m from the instantaneous leader-aligned rectangle. This blocked tb1 while followers were already ahead at their next stage, and coupled steering yaw to enclosure axes. The corrected gate uses the fixed enclosure axes, allows rotation in place, and holds only translation that worsens an excessive corner error. Clearance/physical collision checks remain active.

Stage driving orientations now follow the travel direction independently of rectangle axes. Final enclosure rotation remains bounded to 0.1 rad per stage, and completion verifies both leader arrival and final enclosure orientation. The live status includes separate follower-hold information and identifies turning.

The operator goal, existing formation stage and positions were preserved during the runtime controller reload. tb1 yaw changed from -58.8 to -73.1 degrees. All 442 sampled physical positions kept tb1 enclosed; measured real-time factor was 0.452. This validates resumed turning, not completed navigation to the distant operator goal. Formation/action/frame regressions (35) and updated status feedback checks (12) passed.
