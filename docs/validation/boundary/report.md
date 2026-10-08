# Moving boundary test

A primary goal at (2.4998, 1.6003, 0) was issued from tb1 near (0.4998, 1.6003). All five robots moved approximately 1.65–1.71 m and completed within the configured 0.4 m localization arrival tolerance. The mission completed in 163.65 wall seconds. A 45-second ModelStates observation recorded tb1 inside the four actual follower corners in all 316 samples. Peak odometry speeds were tb1 0.313, tb2 0.401, tb3 0.410, tb4 0.406, tb5 0.407 m/s. A final stationary observation measured real-time factor 0.162 and enclosure in all 190 samples.

Gazebo viewer was closed; RViz displayed all five models, the corner outline, and the primary-goal tool. This verifies a straight open-space translation. Arbitrary detours and large final rotations have not been validated in simulation. Whole-boundary footprint checks pause obstructed segments. Small final rotation steps have unit coverage.
