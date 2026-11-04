# AGENTS.md — rules for coding agents (and humans)

These rules are mandatory. Read them before changing anything; read
[docs/architecture.md](docs/architecture.md) before changing data flow, schemas or ownership.

## 1. Purpose and scope

JunctionPerception builds a metric bird's-eye-view (BEV) digital twin of an intersection from the
video of one or more fixed cameras:

```text
video → detection → tracking → ground-plane localization → heading → multi-camera fusion
      → one site world state → BEV visualization
```

In scope: recorded and live camera sources, single- and multi-camera sites, synchronized
playback, camera calibration (homography, control points, map placement), zones as site
geometry, traffic-light state, the site world model, the BEV console, benchmark evaluation
against TUMTraf labels.

Out of scope (removed on purpose, recoverable from git tag `checkpoint/pre-bev-refactor`): the 3D
twin, conflict and safety events, warnings, risk assessment, lane/queue/waiting-time analytics,
traffic statistics and reports. Do not reintroduce them without an architecture decision record
(ADR, §8).

## 2. Architecture and ownership

| Layer | Where | Owns | Must not |
|---|---|---|---|
| Camera perception (C++) | `src/`, `include/jp/` → `jp_server`, `jp_run` | frames and their timestamps + time source, detection, tracking, camera-local track IDs, ground localization, motion, heading and its source, position/speed uncertainty, validation flag, traffic-light colour; publishes **camera observations** (`/ws/world`, schema 2) | associate tracks across cameras; assign site IDs |
| Site world model (Python) | `manager/world_model.py` | time alignment of cameras, association, fusion, **global road-user IDs**, lifecycle (tracked/coasting/dropped), unfused/degraded mode; produces the **site state** (schema `site_state` v1) | re-run perception; invent headings or sizes |
| Manager / API (Python) | `manager/jp_manager.py`, `manager/site_feed.py` | sites, cameras, configuration and calibration files, process orchestration, site playback clock, serving the site state (`/sites/<id>/ws/site`), proxying camera servers, logs, health | contain association or fusion logic outside `world_model.py` |
| Frontend (TypeScript) | `web/src/` | drawing the site state in the BEV, camera video, configuration and calibration UIs, playback controls, zoom/pan/selection | fuse, deduplicate, re-identify, estimate headings/speeds/sizes, compute traffic measures, read `/ws/world` |

Dependency direction: C++ → (observations) → world model → (site state) → frontend. The
frontend reads physical road-user state **only** from `web/src/net/siteState.ts`
(`subscribeSite`). Rendering transforms (`web/src/bev/siteFrame.ts`) may move points into screen
space; they never change a road user's state.

## 3. Where new work goes

| You want to… | Put it in |
|---|---|
| change detection, tracking, localization, heading or their uncertainty | C++ module (`detection`, `tracking`, `localization`), exposed through `core::WorldObject` |
| change how cameras are aligned, matched, fused or how IDs live | `manager/world_model.py` (+ tests in `manager/tests/test_world_model.py`) |
| add a site/camera setting or API endpoint | `manager/jp_manager.py`; frontend client in `web/src/net/manager.ts` |
| add something drawn in the BEV | `web/src/bev/bevView.ts`, from fields the site state already carries |
| need a new physical quantity in the BEV | add it to the producer (C++ or world model) and to the schema first (§5), then draw it |
| evaluate accuracy | `jp_eval` (single camera vs labels), `tools/eval_fusion.py` (site model vs labels) |

## 4. Single source of truth

- One implementation per behaviour: one association/fusion (`world_model.py`), one heading
  estimate per camera (`TrackLocalizer`), one site-state schema, one observation schema.
- Thresholds live next to their owner: fusion in `FusionSettings` (`configs/manager.yaml`,
  `world_model:`), perception in `configs/pipeline.yaml`. Never copy a threshold into another
  layer.
- Unknown stays unknown: `heading_rad: null` with `heading_source: null`, `length_m: null` or
  `size_source: "class"`; never `0`, never a direction from OSM or a lane, never more precision
  than measured.

## 5. Schemas and APIs

- Camera observation: `core::WorldState` ↔ `src/twin/world_state_json.cpp`, fixture
  `tests/fixtures/camera_observation_v2.json` (C++ writes it byte-for-byte; the Python model reads
  it). Site state: `manager/world_model.py` ↔ `web/src/net/siteState.ts`, fixture
  `tests/fixtures/site_state_v1.json` (both sides test against it).
- Any field change: update producer, consumer, fixture and docs/architecture.md in the same
  commit. Additive optional fields keep the version; renaming, removing or changing the meaning of
  a field bumps the version and keeps a reader for the previous one where recordings exist.
- New endpoints are documented in the module docstring of `jp_manager.py` (or the
  `TwinServer` comment) and in docs/architecture.md.

## 6. Code conventions

- C++23, clang-format-18 and clang-tidy-18 clean, `-Werror`. Python 3.12, standard library +
  aiohttp/yaml/numpy only, type hints, no I/O in `world_model.py`. TypeScript strict, ESLint +
  Prettier, no new runtime dependencies without an ADR.
- Match the surrounding style and comment density. Comments say why, not what.
- No speculative abstractions, wrappers or "for later" hooks. No dead code: delete it; git keeps
  it.

## 7. Verification (run before every commit)

There is no Node or Python on the developer PC; run everything in the development image
(`scripts/in_docker.sh`, e.g. on the GPU server in a scratch copy, never in the production
working copy):

```bash
scripts/in_docker.sh scripts/build_and_test.sh all release   # C++ (format, build, ctest, tidy), Python, web
scripts/in_docker.sh scripts/build_and_test.sh python        # world model only
```

Architecture guards that fail the build: `web/tests/architecture.test.ts` (no 3D engine, no
camera streams in the frontend, no fusion/heading derivation, one road-user type),
`manager/tests/test_world_model.py` (fusion, time alignment, IDs, schema contract),
`tests/twin/world_state_json_test.cpp` (observation contract).

Accuracy claims need numbers from `jp_eval` or `tools/eval_fusion.py` on TUMTraf, reported with
the data used and their limits.

## 8. Removing code and recording decisions

- Remove obsolete code completely (files, imports, styles, config keys, docs, tests); never leave
  disconnected modules "in case". Never delete user data: `scenes/` site folders, `models/`,
  `map_cache/`, `.cache/manager/`, datasets, untracked folders of the user.
- Significant architectural changes (new process, new owner of a calculation, schema version,
  removed capability, new dependency) get a short ADR in `docs/adr/NNNN-title.md`: decision,
  reason, alternatives, consequences. Update docs/architecture.md to what exists, not to plans.

## 9. Definition of Done

A task is done when all of these hold:

1. The behaviour is implemented once, in the owning layer (§2), with its thresholds there.
2. Schemas, fixtures, both sides of each contract and docs/architecture.md agree.
3. `scripts/build_and_test.sh all` passes in the development image; new behaviour has tests
   (synthetic data at least; TUMTraf numbers when accuracy changes).
4. No dead code, unused config keys or stale docs remain from the change.
5. Limitations and unverified parts are stated in the commit message or the docs, not hidden.
6. Work is on a feature branch in small commits; nothing is deployed or merged to `main`
   without the owner's go-ahead. Production (the running console on the GPU server) is never
   touched by development work.
