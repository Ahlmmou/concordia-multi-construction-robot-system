# Editor snapshots before switching to Windows

These files preserve editor documents that were marked modified during the final
save audit. They are separate from the tested production assets.

- `aligned_map_unsaved_state.xcf`: open GIMP aligned-base document (image 3).
- `elevator_tracing_unsaved_state.xcf`: open GIMP elevator drawing (image 4).
- `blender_unsaved_state.blend`: the modified open Blender project, saved through
  Blender's own save command. Its main editor may open as a Python console;
  Shift+F5 over that editor returns to the 3D viewport.

Unsaved flags can reflect layer visibility, selection, view or other editor
changes as well as geometry/pixels. Review these snapshots before replacing any
tested map or mesh. The verified navigation map remains
`config/EVfloor9_formation_aligned.yaml`; the old map is a comparison only.

See `docs/CODEX_HANDOFF.md` for the detailed session summary, tests, source
provenance, restart commands and outstanding tb4 readiness issue.
