# EV9 map alignment correction

The Gazebo world, composed model/link transforms, DAE metre units and scene-node matrices were applied when projecting wall triangles onto the XY plane. Mesh-wall projected occupancy was rasterized at the original map resolution. Boundaries were sampled every third cell and registered to the PGM occupied boundaries using nearest-neighbour similarity fitting with a soft-L1 loss (0.05 m) and 90% closest correspondences. Other trim fractions (70–95%) gave consistent scale estimates of 1.00737–1.00754.

The old navigation raster was approximately 0.74% larger than Gazebo. Resolution changed from 0.0257 to 0.02551200315342095 m/cell; origin changed from (-37.17785,-33.8028,0) to (-36.748513534724125,-33.690327890042596,0). The PGM bytes, Gazebo mesh/collision/world geometry and five-robot spawn settings are unchanged. Original YAML is backed up under backups/map_alignment_20261006/EVfloor9_outer_only.yaml.

A free-rotation fit suggested about 0.144 degrees of residual rotation. The applied correction deliberately keeps origin yaw zero: Nav2 Humble's static layer uses map resolution and origin XY directly when resizing its axis-aligned costmap, without applying origin orientation (https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/plugins/static_layer.cpp). A nonzero map-origin yaw would therefore make RViz and the costmap disagree. This limited correction improves alignment but does not claim identical geometry.

Median mesh-to-map boundary error improved from 0.1310 to 0.0403 m; whole-floor 90th percentile from 0.3864 to 0.1516 m. Near the formation (|x|<12, -10<y<10), 95th percentile improved from 0.1799 to 0.0841 m. East-side shape differences remain, with 95th-percentile error 0.3707 m. These are sampled boundary distances, not paired corner errors, and fitting/measurement use the same boundary dataset; they are not independent validation.

All five full-footprint turn/departure/clearance checks pass against the corrected map and unchanged Gazebo geometry. The live /map and /tb1/global_costmap/costmap have matching corrected resolution and XY origin. All 81,157 original occupied map cells remain present. The live map was reloaded through /map_server/load_map; simulation state was preserved in runtime_before.json. Tb1 localization was reseeded at its actual stationary Gazebo position before repeating a goal through RViz's native Nav2 Goal tool. Its starting position differs from the original spawn because this continues the existing test session.

Persistent correction loads through the existing s_gazebo_multi_nav2_world.launch.py map-server configuration. For the current Step 2 configuration, use the existing local test launch:

```bash
source /home/umroot/crs_ws/install/setup.bash
unset LIBGL_ALWAYS_SOFTWARE
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch crs_multibot tb1_navigation_test.launch.py
```

The Step 2 test launch/profile and sensor changes remain separate uncommitted work; this alignment commit alone does not capture the entire tested navigation configuration.

Repeat test: RViz goal (3.51734,1.63593,3.13595 rad), Gazebo start (7.09913,1.83624,0.14438), measured end before cancellation (3.58999,1.40772,0.13416). Physical movement 3.5352 m; physical position error 0.2395 m meets 0.25 m tolerance, but heading error 3.0018 rad fails 0.4 rad tolerance. Navigation did not finish within 240 wall seconds and was canceled. Auxiliaries did not move; no qualifying collision contacts were recorded.

The final_map value in navigation.json is the last available transform, NOT a current final pose: lookup later failed with past-extrapolation errors because map/odom and odom/base transforms no longer had overlapping times in the observer's ten-second TF cache. Its reported map error must not be interpreted as a valid final-arrival measurement. Remaining work is to inspect live AMCL TF freshness, Nav2 transform errors and controller oscillation/rotation before repeating navigation. The alignment correction alone has not produced a navigation PASS.

Stop verification: action result status 5 (canceled), last observer velocity command zero. An eight-second follow-up received no new velocity commands. Gazebo planar speed was 0.00154 m/s and yaw rate 0.000573 rad/s, consistent with the existing small passive physics drift rather than commanded navigation. See stopped.json.
