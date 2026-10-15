# Repository working rules

- All project changes and commits must use the exact branch `test_mcp`.
- Never commit on another branch. Check the branch before every commit.
- Commit completed, tested changes. Do not push unless explicitly requested.
- Use the existing `Ahlmmou/concordia-multi-construction-robot-system` GitHub repository and repository-local author `ahlam <ahlmmou@gmail.com>` for commits.
- Preserve existing uncommitted work: never reset, discard, overwrite, or include unrelated changes in commits.
- If switching branches endangers edits, use an isolated worktree on `test_mcp` when possible; explain genuine blockers without discarding work.
- Run Gazebo headlessly with its simulation server active and GUI disabled.
- Open RViz with its GUI enabled; preserve fleet TF and RViz relay configuration.
- Use available MCP tools to inspect and test the running simulation. Distinguish mock/disconnected results from observed live results; use ROS CLI or scripts for missing capabilities.
- Preserve the fleet supervisor's command ownership, collision checks, and velocity gate. Never reduce clearance limits to make validation pass.
- tb1 is the Hilti/Jaibot primary robot; tb2–tb5 are auxiliaries.
