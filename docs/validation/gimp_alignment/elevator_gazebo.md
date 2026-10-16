# Restored Gazebo elevator walls

`models/floor_plan/meshes/EVfloor9_with_elevators.blend` preserves the 49 original scene objects from `EVfloor9_outer_only.blend` and adds an editable collection of 18 elevator-bank wall segments. The six unequal shaft shapes, stepped corners and shared partitions come from `EVfloor9_L.dae` at GitHub dev commit `a8a587547c581289ee31312957f66c8b1bdf311e`. That branch has Blender-exported DAE meshes but no `.blend` project; the existing repository Blender project is therefore the editable base. The local source DAE exactly matches the downloaded GitHub file, SHA-256 `43fd9b2503fbe6ef447d996998496b8bf02ac0f4bf01e9c48f47917555ab40be`.

Only wall segments belonging to the two banks were restored; adjoining corridor walls were clipped at the bank bounds and the cuts capped. All 18 segments are closed meshes, extending from Z=0 to 3.67 m. The passage between the banks remains open. `EVfloor9_elevators.dae` exports only the restored collection, in metres. `floor_plan.sdf` loads it through a separate link with the same placement as the existing floor, for both visual and collision geometry. The original outer-wall DAE, model placement, map metadata, robot configuration and clearance limits are unchanged.

Rebuild using Blender 3.6:

```bash
blender --background --factory-startup --python scripts/restore_elevator_mesh.py
```

The script checks the source hash, preserves the original Blender file and exports a new project and mesh. It also writes `elevator_mesh_audit.json`. Tests independently read the exported triangles, verify watertight topology and the model's visual/collision links, and project the geometry into map coordinates. For the physical projection, median distance to the traced map wall is zero, 95th percentile is 0.0510 m and maximum is 0.1804 m. These residual differences reflect the detailed source versus the raster tracing; the geometry is not claimed to match every map pixel exactly.

Validation: 37 targeted geometry, stair-exclusion and formation-geometry tests pass. The installed package was refreshed. The running world received the same added mesh as a static `ev9_elevators` entity at the existing floor-model placement, preserving the robots and the loaded corrected map. Future launches include the added link directly in `ev_9`; the temporary separate live entity is unnecessary after restarting.

A temporary Gazebo ray sensor at (31, -1.2, 1) measured the facing north and south physical walls at 1.8131 m and 1.6545 m. A real Gazebo camera above the banks rendered all six shafts (`elevator_gazebo_camera.png`). Sensor messages were observed directly through ROS; the MCP sensor snapshot capability reported unsupported for the Classic backend. The validation probe was removed after capture. Blender and the Gazebo GUI were opened for user inspection; the GUI was enabled at the user's explicit request. No formation motion was commanded; the existing supervisor readiness hold remains in effect.

The previous unaligned map with its cyan Gazebo overlay was opened in GIMP for viewing only. It was not loaded into RViz or the map server.
