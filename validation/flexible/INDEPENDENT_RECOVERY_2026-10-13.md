# Independent flexible follower recovery

The original live observation showed tb3 escaping an obstacle buffer with a -0.12 m/s reverse command; a subsequent observation showed recovery cleared and tb3 navigating again. The fleet did not report a follower-lag hold. Tb1's final goal lay near tb4's moving target, and both path-dispatch checks reserved incompatible future positions. The other actions were cancelled/failed while tb3 alone was active.

Flexible travel now checks followers against the primary's current footprint instead of reserving its final destination. Primary requests retain actual peer checks without treating moving corner targets as simultaneous final reservations. Both planner dispatch and planner-result acceptance use this policy. Map, oriented footprint, clearance, ownership and swept motor checks remain enabled; all speed and clearance parameters are unchanged.

Auxiliary retry exhaustion, action response timeout and actual-motion watchdog stalls now enter the existing checked per-robot obstacle escape/detour recovery in flexible travel. Only that robot's action is cancelled, and ownership remains until its result. Primary failures, rigid/other formation workflows, stale data, localization discrepancies and operator stops retain their holds. READY still requires all robots to arrive.

All 136 regression tests passed. New tests cover the future-goal reservation deadlock, actual peer overlap rejection, independent auxiliary retry/stall/timeout recovery, cancellation ownership, healthy motion during another robot's recovery and stopping for a real peer clearance conflict. Git whitespace checks passed.

The supervisor was safely reloaded after verifying zero commands and released actions, preserving Gazebo server PID 674, RViz PID 720 and the relay. The retained primary goal was replayed for 240 wall seconds / 52.5 simulation seconds. All five moved: tb1 5.126 m, tb2 7.822 m, tb3 2.659 m, tb4 7.056 m and tb5 7.353 m. Minimum sampled physical footprint clearance was 1.156 m. Primary position error was 0.297 m, but yaw error remained 0.935 rad. Final formation completion timed out; this is not a successful complete mission.

The replay showed normal independent navigation, not a triggered auxiliary recovery: the separate observer recorded zero recovery samples. Newly extended watchdog/retry recovery is regression-tested but not demonstrated by a live induced fault in this replay. Several brief localization-verification pauses cleared automatically. The recorder acknowledged stop and observed all five zero motor commands. The existing primary turning/final orientation limitation remains unresolved.

Live parameter and motor observation confirmed primary/auxiliary caps 0.60/0.80 m/s, angular cap 0.50 rad/s, clearances 0.6/0.4 m, and collision horizon 2 s. RViz MCP reported mock mode; live evidence used ROS CLI/rclpy. See the replay and observation JSON files in this directory.

After the bounded recorder stopped, supervisor resume was acknowledged. A later live status read reported READY, reason "Primary goal and feasible follower positions reached", LEGACY active, no follower waits and no obstacle recoveries. Thus the retained mission subsequently completed according to the supervisor; the bounded replay JSON remains a timeout record rather than being relabeled as a pass.
