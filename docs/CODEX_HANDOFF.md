# Codex handoff — 2026-10-16

Read this file and `AGENTS.md` at the start of the next session. The user is
switching from this Ubuntu environment to Windows and requested that all relevant
work, unsaved editor state and session context be preserved. This document is a
repository-backed handoff, not an automatic restoration of chat memory or running
applications.

## Repository and working rules

- Repository: https://github.com/Ahlmmou/concordia-multi-construction-robot-system
- Work only on branch **`test_mcp`**. Check the branch before every commit.
- Local author: `ahlam <ahlmmou@gmail.com>`.
- Linux checkout: `/home/ahlam/crs_ws/src/concordia-multi-construction-robot-system`.
- ROS workspace: `/home/ahlam/crs_ws`; ROS 2 Humble under `/opt/ros/humble`.
- Preserve user edits; never reset, discard, overwrite or commit unrelated work.
- Commit completed, tested work. Push only when the user authorizes it. The user
  explicitly requested a push during this session, and then a final save/handoff.
- Gazebo normally runs with its server active and GUI disabled; RViz uses its GUI.
  The user explicitly requested Blender and Gazebo GUIs for this inspection, so
  both were opened. That request does not change the normal launch defaults.
- Keep fleet TF relays, supervisor command ownership, collision checks, velocity
  gate and clearance limits intact. tb1 is the Hilti/Jaibot primary; tb2–tb5 are
  auxiliaries. Never weaken safety checks to make a test pass.

## What the user requested and what was completed

1. Inspected GIMP's old two-layer map: black/white edited map below a cyan Gazebo
   wall reference at 80% opacity. Identified locally varying misalignment.
2. At the user's direction, kept the Gazebo layer fixed and reconstructed the map
   wall outlines from that reference. Preserved the filled stair obstacle and the
   original map as a hidden layer. The corrected PNG is opaque binary black/white.
3. Made the aligned, stair-protected map the default for formation launches and
   the shared navigation launch. Installed the package files into the ROS
   workspace so the launches can find the new assets.
4. Downloaded the `dev` branch of
   `Pplll-pppl/concordia-multi-construction-robot-system` for the detailed plan.
   Prepared a GIMP tracing project with a locked reference at 45% opacity in
   Multiply mode, placed at pixel offset **(0, 500)** without scaling.
5. The user pointed to **two facing banks of three unequal elevator shafts**, in
   front of the stair entrance. Traced all six surrounding wall outlines and
   shared partitions, retaining their stepped corners and leaving the passage
   between the banks open. Added them to the formation-map generation pipeline.
6. Verified the new map in live RViz and ROS costmaps before committing, as the
   user requested. Then restored corresponding physical walls into the Blender
   model and Gazebo visual/collision geometry. Opened Blender and Gazebo GUIs.
7. Finally the user requested the **old unaligned map with the cyan Gazebo overlay
   for viewing only**. Opened that comparison in GIMP. **Do not put the old map
   back into RViz or the map server.** The corrected formation map stays active.

## Important files — use the right version

| File | Purpose |
| --- | --- |
| `config/EVfloor9_formation_aligned.yaml` | **Current default navigation/formation map metadata.** |
| `config/EVfloor9_formation_aligned.png` | Current binary map: aligned outer walls, stair exclusion and elevator walls. |
| `scripts/generate_aligned_formation_map.py` | Rebuilds the current PNG/YAML from aligned base, old stair exclusion and transparent elevator wall layer. |
| `config/EVfloor9_gazebo_aligned.png` | Aligned base before elevator additions. Do not confuse with the final formation map. |
| `config/EVfloor9_gazebo_aligned.xcf` | Layered aligned base; fixed Gazebo reference and preserved hidden original. |
| `config/EVfloor9_elevator_tracing.xcf` | Saved layered project with traced elevator walls; GitHub plan is available as a hidden reference. |
| `config/EVfloor9_elevator_walls.png` | Transparent wall-only layer used by the formation map generator. |
| `config/EVfloor9_edited_with_gazebo.xcf` | **Old unaligned comparison**, original black/white map plus cyan Gazebo overlay. Viewing only. |
| `config/EVfloor9_edited.png`, `.yaml` | Original edited map associated with that comparison; archived, not launch defaults. |
| `models/floor_plan/meshes/EVfloor9_with_elevators.blend` | Editable Blender project, original scene plus restored elevator collection. |
| `models/floor_plan/meshes/EVfloor9_elevators.dae` | Export of only the restored elevator walls. |
| `models/floor_plan/floor_plan.sdf` | Existing outer-floor link plus a separate elevator visual/collision link. |
| `scripts/restore_elevator_mesh.py` | Blender 3.6 script to regenerate the new Blender project and elevator export from the source DAE. |
| `docs/session/2026-10-16/` | Dated snapshots of unsaved editor state taken during the final handoff. These are review copies, not production defaults. |
| `docs/validation/gimp_alignment/` | Reports, live map/costmap checks, screenshots, mesh audit and Gazebo sensor evidence. |

The final map retains resolution **0.02551200315342095 m/cell**, origin
**[-36.748513534724125, -33.690327890042596, 0]**, `negate: 0`, occupied threshold
**0.65**, and free threshold **0.196**. Its dimensions are **4215 × 2980**.
Pure black is occupied; pure white is free. ROS float32 resolution differs in its
last digits, as expected.

The old stair generator/spec remain unchanged: `scripts/generate_stair_keepout.py`
and `config/stair_keepout.yaml`. The aligned map retains every cell that generator
added, including 226 doorway cells missing from the initial aligned drawing.

## Geometry provenance and limits

- GitHub reference commit: `a8a587547c581289ee31312957f66c8b1bdf311e`.
- The branch's `config/EVfloor-Floor-Plan-Level-1.yaml` selects
  `EVfloor-Floor-Plan-Level-7-test.pgm`. That image is byte-identical to this
  repository's existing copy; SHA-256:
  `eff5a59c73e723a3ff9da6e788d2bc900ff82286932048ab3ff119be0d481449`.
- The GitHub branch has **no `.blend` file**. Its Blender-exported
  `models/floor_plan/meshes/EVfloor9_L.dae` is also byte-identical to the local
  source; SHA-256:
  `43fd9b2503fbe6ef447d996998496b8bf02ac0f4bf01e9c48f47917555ab40be`.
- Used local `EVfloor9_outer_only.blend` as the editable base. Preserved its 49
  original scene objects, adding 18 clipped, capped, closed wall segments in a
  separate collection. Wall height is 3.67 m. Exported only that collection.
- Existing `EVfloor9_outer_only.dae`, floor-model placement and outer geometry
  were not changed. New link uses the same pose: `30 -2 0 0 0 1.5708`, within the
  world floor placement `-19.76 -7.98 0 0 0 0`.
- The detailed plan and original Gazebo geometry have small local differences.
  Median physical-projection distance to the traced map walls is zero, 95th
  percentile 0.0510 m, maximum 0.1804 m. Do not claim perfect pixel-for-pixel
  agreement of the physical mesh. No clearance limits were reduced.
- The detailed reference does not cover the aligned canvas's top 500 rows.
  Existing bottom-edge clipping was not reconstructed beyond the canvas.

## Verification already performed

Latest targeted suite: **37 passed**:

```bash
source /opt/ros/humble/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  tests/test_elevator_geometry.py \
  tests/test_stair_keepout.py \
  tests/test_formation_geometry.py
```

`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` avoids an unrelated installed anyio/old-pytest
plugin incompatibility (`ModuleNotFoundError: _pytest.scope`). ROS setup is
required for `rclpy`. The tests check all six enclosures, the open passage,
preserved stair protection, fleet safety behavior, watertight exported meshes,
shared visual/collision geometry and physical projection.

Live observations:

- `/map` exactly matched the final GIMP binary export.
- All **16,652 elevator-wall cells** were occupied in the received tb1, tb2, tb3
  and tb5 global costmaps. No tb4 global costmap arrived during that observation.
- RViz displayed both banks with inflated obstacle boundaries. Its configured
  map display uses `/map`; TF remaps remain `/fleet_visualization/tf` and
  `/fleet_visualization/tf_static`.
- Gazebo `/clock` advanced from 182.4 to 212.3 seconds during map verification.
- A temporary Gazebo lidar at (31, -1.2, 1) measured facing elevator walls at
  1.8131 m north and 1.6545 m south. A real camera rendered the six shafts.
  The validation sensor model was deleted successfully after capture.
- No movement goal was sent. Map/geometry validation is **not** a successful
  end-to-end formation navigation test.

## Outstanding runtime issue — investigate before moving robots

The last observed supervisor preflight held all robots because **tb4's gated
Nav2 velocity smoother was not ready**. Earlier startup messages also mentioned
other robots while they were starting, but tb4 remained the reported blocker.
Do not bypass the gate or assume the fleet is ready after reboot. Check node
ownership, lifecycle states and current costmaps on a clean launch.

An older `docs/validation/fresh_session/report.md` records a successful READY
state from an earlier session; it does not override this session's observation.
The live Gazebo MCP status continued reporting zero simulation time/iterations
despite a real advancing ROS clock. Use direct ROS observations as evidence.
MCP Classic sensor snapshots were unsupported. RViz MCP connected live and
listed displays, but native screenshots were unavailable through that tool, so
desktop captures were used. Never label mock/disconnected outputs as live proof.

## Restart on Ubuntu

Runtime processes and `/tmp` files will not survive reboot or an environment
switch. Do not restore stale ROS process IDs. Check for an existing simulation
before starting another one.

```bash
cd /home/ahlam/crs_ws/src/concordia-multi-construction-robot-system
git branch --show-current  # must be test_mcp
source /opt/ros/humble/setup.bash
source /home/ahlam/crs_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=0
ros2 launch crs_multibot fleet_simulation.launch.py gazebo_gui:=false rviz:=true
```

The current installed package was refreshed using
`cmake --install /home/ahlam/crs_ws/build/crs_multibot`; it uses symlinks to this
checkout. On a new checkout/machine, build and source the workspace normally.
Do not start a second ROS domain to mask a graph conflict. A Windows checkout
can preserve/review the source and assets; the ROS/Gazebo commands above target
the Ubuntu environment, not native Windows.

The running session received the elevator mesh as an additional static model
named `ev9_elevators`, without restarting or moving robots. On a clean restart,
`floor_plan.sdf` includes the same geometry directly inside `ev_9`. **Do not spawn
the separate addon again after restarting**, or the walls will be duplicated.

Rebuild the final map if a verified drawing changes:

```bash
python3 scripts/generate_aligned_formation_map.py
```

Blender was not installed system-wide. Portable **Blender 3.6.23** was downloaded
from the official Blender site into `/tmp/blender-3.6.23-linux-x64`. That executable
is temporary. Use installed/portable Blender 3.6 with Collada export support to
open the committed `.blend` file or run:

```bash
blender --background --factory-startup --python scripts/restore_elevator_mesh.py
```

This regeneration starts from the original base and original source DAE. Review
any dated Blender snapshot first if the user made further manual edits; do not
overwrite those edits by blindly regenerating.

## Commits from this session

All are on `test_mcp` and had been pushed before the final handoff request:

- `85f5c9f` — Align editable EV9 map outlines with fixed Gazebo reference.
- `f049585` — Use Gazebo-aligned map by default for formation testing.
- `786ab21` — Restore six elevator shaft walls in aligned formation map.
- `cf79f15` — Restore Blender-sourced elevator visuals and collisions in Gazebo.

The final save commit adds this handoff, the previously untracked original
comparison map files, and dated unsaved editor snapshots. Python bytecode caches
are ignored, not deleted or committed. Check Git status and the remote-tracking
branch to identify that final save commit.

## Suggested next-chat prompt

> Read `AGENTS.md` and `docs/CODEX_HANDOFF.md` on branch `test_mcp`. Continue from
> the saved map/Blender/Gazebo elevator restoration. Preserve the corrected map
> as the formation default and use the old cyan-overlay map for viewing only.
> Inspect dated editor snapshots before regenerating assets. Check tb4's gated
> velocity-smoother readiness before attempting formation movement.
