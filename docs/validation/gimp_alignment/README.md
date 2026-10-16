# GIMP wall alignment

`config/EVfloor9_gazebo_aligned.xcf` keeps the Gazebo reference layer at its original coordinates, opacity and pixels, with content and position locked. The corrected map sits beneath it; the original map remains as a hidden third layer. The source `EVfloor9_edited_with_gazebo.xcf` and original PNG are untouched.

The corrected black wall outlines are rasterized from the reference layer's alpha, rather than using a global translation or scale: the original mismatch varied by location. The map retains a filled central obstacle, relocated to the reference bounds. The PNG contains the corrected map alone, without the cyan reference, at the original 4215 × 2980 dimensions, with binary black/white pixels.

All 43,770 reference pixels with alpha greater than 128 coincide with occupied pixels in the corrected PNG (100%; original map: 34.30%). This verifies agreement with the saved GIMP reference, not independent agreement with a running Gazebo world. Antialiased reference fringes are converted to binary wall pixels. The reference already meets the bottom canvas boundary; the correction does not reconstruct geometry beyond that canvas.

The preserved filled obstacle remains a blocked area, including its original closed entrance treatment. This is an image-editing result; live ROS maps, map metadata, Gazebo geometry, fleet settings and navigation clearance limits were not changed.

## Formation launch default

Formation and shared navigation launches now default to `config/EVfloor9_formation_aligned.yaml`. Its PNG uses the aligned black/white image and preserves every cell added by the established stair-exclusion generator, adding 226 occupied doorway cells. Resolution, origin and occupancy thresholds are unchanged. Regenerate with `python3 scripts/generate_aligned_formation_map.py`.

Validation: 33 targeted stair exclusion and formation geometry tests pass, including stair goal rejection and forward/reverse velocity gating for all five robots on both the original and aligned protected maps. The installed package was refreshed, and `ros2 launch crs_multibot fleet_simulation.launch.py --show-args` resolves the new default map. RViz remains enabled and Gazebo GUI disabled by default; fleet TF relay and controller settings are unchanged. No live navigation run was performed: RViz MCP reported disconnected/HTTP 503, ROS topic discovery showed no `/map`, and process inspection found no running RViz, map server or Gazebo server. Gazebo MCP's reported running state with zero time/iterations was therefore not accepted as evidence of a live simulation.
