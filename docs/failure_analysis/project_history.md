# Project History

This file tracks the current `jetauto_orin_amr` project from the fresh
spec-driven repository reset. The revised project spec is the source of truth
for structure and development flow.

## Timeline

### 2026-05-07

What changed:
- Reframed the repository around the revised JetAuto Orin AMR project spec.
- Reset the git repository to a fresh `main` history.
- Moved ROS packages into the domain layout under `src/platform`,
  `src/perception`, `src/localization`, `src/navigation`, `src/safety`, and
  `src/tools`.
- Removed simulator packages and old git-history backup artifacts from the
  active project.

Why it mattered:
- The workspace now matches the development flow in the spec and can proceed
  package by package without carrying the old documentation structure forward.

Evidence:
- `colcon list --base-paths src`
- `jetauto_orin_amr_project_spec.md`
