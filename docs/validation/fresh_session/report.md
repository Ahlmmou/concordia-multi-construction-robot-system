# Clean restart validation

All previous domain-42 fleet processes and temporary recovery launches were stopped before rebuilding crs_multibot. The fresh fleet launch writes output to /tmp/fleet_fresh_session.log. All five robots spawned at the saved spawn positions, all ten Nav2 navigation/planning action servers were present, and the supervisor reached READY with mission 0 and no blocked robots. There was exactly one clock publisher and one publisher/subscriber pair on /formation/primary_goal. RViz was confirmed open.

Saved settings: leader 0.60 m/s, auxiliaries 0.80 m/s, angular limit 0.50 rad/s; 4 ms ODE timestep and target 250 updates/s; simulation world localization; enclosing rectangle and follower pause gates. The observed fresh-session real-time factor was 0.439. Eighteen formation/action/frame-recovery regression checks pass.

Frame fixes retain multiple static publishers at startup and restore fixed transforms after TF clock cache clears. New missions derive boundary axes from the corner robots and keep the leader stationary during initial assembly. The fresh session is left waiting for a new RViz goal; no old mission was replayed.
