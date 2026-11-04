# ADR 0001: The site world model is a Python module of the manager

Status: accepted, November 2026.

## Decision

Multi-camera time alignment, association, fusion and global road-user IDs live in one module,
`manager/world_model.py`, run per site inside the manager process (`manager/site_feed.py`) and
served as the site state. Cameras (`jp_server`, C++) only publish their own observations. The
browser draws the site state and fuses nothing.

## Reason

- The workload is small: up to 4 cameras × ~50 tracks at ~10 Hz is far below a millisecond per
  step in plain Python (the TUMTraf evaluation replays 2 × 1033 frames four times in 3.4 s).
- The manager already subscribes to every camera, owns the site clock and the configuration: no
  new process, port, supervisor or deployment step.
- Pure functions without I/O: deterministic, unit-testable without a browser or GPU, and the same
  code runs live and offline (`tools/eval_fusion.py`).

## Alternatives considered

- **C++ `jp_site` process** (proposal of November 2026, git history
  `docs/world-model-architecture.md`): reuses `BevKalman` and `jp_eval`, but adds a process,
  ports, supervision and a second build artefact for little performance gain.
- **Fusion in `jp_server`**: each server sees one camera; it would need the others' streams.
- **Keep fusion in the browser**: results differed between views and browsers, were not available
  headless, and averaged positions of different instants (measured: 91 % duplicates with one
  camera 300 ms late).

## Consequences

- One implementation of association and IDs; frontend tests forbid fusion code in `web/src`.
- Camera tracks are fused as independent estimates (inverse-covariance weighting). Cameras do
  not share a filter: measurement-level fusion would need detections, not tracks, from C++.
- If the manager restarts, global IDs start over (new run).
