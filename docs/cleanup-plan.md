# Short-term architecture cleanup

Proposal for review, November 2026: a focused cleanup of about 4 developer-days. No new process or
service, no change of the perception algorithms, every feature kept. The site-level world model
(`docs/world-model-architecture.md`, branch `feature/world-model-plan`) stays the long-term
roadmap and is not a prerequisite for anything here.

## 1. Findings (repository evidence)

### 1.1 Duplicated or inconsistent logic

| # | Finding | Evidence | Harm |
|---|---|---|---|
| D1 | **Two cross-camera merge rules.** The 3D view merges with `CameraFusion` (gates 3 m / 1.5 m, velocity check, hysteresis ×1.6, capture-time gate 200 ms); the BEV finds duplicates with its own code (same gates, no hysteresis, time gate 250 ms) and drops one of each pair | `web/src/twin/fusion.ts`; `web/src/bev/bevModel.ts:32,330–363` (`duplicateCandidates`); `web/src/bev/bevView.ts:17,` `deduplicate` | The same moment yields different road users, counts and lane numbers in the two views; two places to fix |
| D2 | **"Road users" counter differs between views.** The side panel counts `TwinView.objects()` (merged) in 3D and `BevView.objects()` (per camera, duplicates included) in BEV | `web/src/pages/twin.ts` `renderPanel` | Visible inconsistency in the main counter |
| D3 | **Mirrored wiring of the two views.** Every map, camera, signal, transform, focus and world-state call is written twice, once per view, in the managed page and again in single-server mode | `web/src/pages/twin.ts`: 31 mirrored calls; `startSingle` repeats the managed wiring | Each new feature must be wired four times; easy to forget one |
| D4 | **Different definitions of "moving", "stopped" and "stale"** | `twinView.ts:47,62–67` (stale 2.5 s; moving 1.2 / 0.5 m/s with its own σ rule); `bevModel.ts:18–23` (stopped < 1.4 m/s, σ ≤ 1); `bevView.ts:13` (stale 2.5 s, separate constant); `laneAnalytics.ts:22`; `net/client.ts:129` (stale 5 s) | The views disagree on what is standing; tuning one leaves the other behind |
| D5 | **Same road-axis transform twice** | `twinView.ts:1102` `road()` and `bevView.ts:560` `road()` | Duplicate sign conventions for sensor-frame sites |
| D6 | **Browser-storage helpers copied** (try/catch read/write) | 8 files: `bevMode.ts`, `bevPanel.ts`, `laneAnalytics.ts`, `pages/twin.ts`, `flowField.ts`, `ui/resizer.ts`, `ui/shell.ts`, `ui/theme.ts` | Noise; inconsistent defaults |
| D7 | Two homography functions with different horizon handling | `net/calibrationApi.ts:122` (`applyHomography`: null only at w ≈ 0) vs `bev/geometry.ts:175` (`homography`: null behind the horizon) | Small; both used correctly today — left as is, documented |

### 1.2 Unclear responsibilities

- `web/src/twin/` mixes rendering (`twinView`, `meshes`, `orbitRig`) with data decisions (`fusion`,
  `flowField`, `interpolation`); `web/src/bev/` mixes rendering (`bevView`) with analytics
  (`bevModel`, `laneAnalytics`). Nothing marks which frontend code *decides* things about the
  world (documented in the last review: fusion, invented headings, lane analytics).
- `web/src/pages/twin.ts` (1 177 lines) holds the page controller, the toolbar and Layers menu,
  map alignment, the traffic-light bar and the single-server mode.
- The data ownership between C++, Python and the browser is nowhere written down.

### 1.3 Deployment

- **`scripts/deploy.sh` cannot work against the current server.** The production working copy
  deletes three tracked repository scenes (`munich_5538`, `tumtraf_s110_south1`,
  `tumtraf_s110_south2`) so that they do not appear as sites, and keeps its real site
  (`tumtraf_s110_south2_s03`) as an untracked folder in `scenes/`. `git push` into the dirty
  working copy (`receive.denyCurrentBranch=updateInstead`) is refused, and on the server three C++
  tests fail because the scenes are missing. The last three deploys were done by hand (backup
  patch, forced checkout, re-delete). The site folder also lives where `git checkout -f` or
  `git clean` can touch it.

### 1.4 Verified unused code

Checked with a whole-repository search (`web/src`, `web/tests`, `tools`): no caller.

| Symbol | File |
|---|---|
| `segmentIntersection` | `web/src/bev/geometry.ts` |
| `BevView.siteAt`, `BevView.metresPerPixel` | `web/src/bev/bevView.ts` |
| `BevView.onStatus` (called, never assigned) | `web/src/bev/bevView.ts` |
| `BevWorld.drop` | `web/src/bev/bevModel.ts` |
| `TwinView.setLostHold`, `TwinView.fusionMaxSkew` | `web/src/twin/twinView.ts` (settable by URL parameter in the constructor; the setter and getter have no caller) |

Not removed although they look unused: exported types used only in their own file (export is
harmless), the single-server mode (`startSingle`, `scripts/run_server.sh` is documented and used
for one camera without the manager), `tools/convert_koper.py` and `manualInferance/` (untracked
work of the user).

### 1.5 Missing tests

- No Python tests at all; `manager/traffic_stats.py` (crossings, statistics, SQLite) is untested.
- No test fixes the cross-camera merge rule shared by both views (there is none to share yet).
- The fidelity harness is a manual tool, not a regression test.

### 1.6 Branches

| Branch | State | Action |
|---|---|---|
| `feature/bev-experiment`, `feature/3d-fidelity`, `feature/twin-layout` (local and origin) | fully contained in `main` (0 commits ahead) | delete after approval |
| `feature/world-model-plan` (local) | 1 commit ahead: `docs/world-model-architecture.md` | keep as the roadmap (merge the doc or leave the branch) |
| `chore/cleanup-plan` (local) | this document | — |
| `archive/pre-v2` (local, origin) | contained in `main`, history of v1 | keep |
| `server/main` (remote-tracking ref of the server, 103 behind) | stale reference | prune the remote ref |
| Untracked in the local checkout: `manualInferance/` (with a Python environment), `scenes/carla_*`, `scenes/koper_*`, `scenes/tumtraf_s110_south2_s03`, `tools/convert_koper.py` | not in git | decide: commit, move or ignore (§3, Q9) |

No branch conflicts: everything but the two documentation branches is merged.

## 2. Ownership of data and calculations (target, without new processes)

| Owner | Owns | Does not own |
|---|---|---|
| **C++ `jp_server`** (per camera) | detection, tracking, localization, heading and its source, uncertainties, zones and VRU events, signal colour, calibration fit, world state v1 | anything spanning cameras |
| **Python manager** | sites and cameras (files), orchestration, site playback clock, proxy, statistics (crossings, events, reports, SQLite), health | perception, geometry beyond line projection |
| **Frontend `web/src/world/`** (new folder, moved code) | the *only* frontend code that decides about the world: cross-camera matching (one rule), motion classification, lane status; marked experimental, to move to the backend with the world model | rendering |
| **Frontend `twin/`, `bev/`, `pages/`, `ui/`** | drawing, interaction, layout; no own thresholds or merge rules | data decisions |

## 3. Quick cleanup (low risk, ~4 days)

Ordered by benefit per effort. Each task leaves every feature working; each ends with
`scripts/in_docker.sh scripts/build_and_test.sh all release` green.

### Q1. Make deploy work again: production sites outside the repository's scenes — 0.5 day

- **Files:** `manager/jp_manager.py` (read `JP_SCENES_DIR`, falling back to `scenes_dir` of the
  config), `scripts/run_manager.sh` (pass `JP_SCENES_DIR`), `scripts/deploy.sh` (pass it from
  `deploy.env`), `deploy.env.example`, `.gitignore` (`/sites/`), `docs/architecture.md`.
- **On the server (with approval, at a quiet moment):** move `scenes/tumtraf_s110_south2_s03` to
  `sites/` (gitignored) inside the working copy, set `JP_SCENES_DIR=/workspace/sites`, restore the
  three tracked scenes with `git checkout -- scenes`; the map cache is keyed by site id and stays.
- **Benefit:** `deploy.sh` works, the server's working copy is clean, all C++ tests pass on the
  server, the production site is safe from checkouts.
- **Risk:** medium (production configuration). Mitigation: backup tar of the site folder; the old
  layout works again by unsetting `JP_SCENES_DIR`.
- **Verify:** `git status` clean on the server; console lists the same single site with its
  cameras, zones, regions and statistics; `deploy.sh` run end to end; `ctest` 114/114 on the
  server.

### Q2. One cross-camera merge rule for both views — 0.5 day

- **Files:** `web/src/twin/fusion.ts` (→ `web/src/world/fusion.ts`), `web/src/bev/bevModel.ts`
  (`duplicateCandidates` replaced by grouping with `CameraFusion` on raw positions and capture
  times), `web/src/bev/bevView.ts` (`deduplicate`, `MAX_MERGE_SKEW_S` removed; "Duplicates hidden"
  keeps the group's most confident track), `web/src/bev/bevPanel.ts` (skew text),
  `web/tests/bev.test.ts`, `web/tests/fusion.test.ts`.
- **Benefit:** one definition of "the same road user" (gates, velocity, 200 ms, hysteresis); BEV
  duplicate links, lane counts and the 3D merge agree. Fixes D1 and, with Q3, D2.
- **Risk:** low–medium: BEV duplicate behaviour changes slightly (hysteresis, 200 instead of
  250 ms).
- **Verify:** new test "BEV groups = CameraFusion groups for the same tracks"; fidelity harness
  sync / async runs: BEV duplicate count equals the 3D merge count per frame.

### Q3. One feed for both views — 0.75 day

- **Files:** `web/src/pages/twin.ts`, `web/src/twin/twinView.ts`, `web/src/bev/bevView.ts`, new
  `web/src/world/views.ts` (interface `WorldView` with `addSite`, `addCamera`, `setMap`,
  `setMapTransform`, `setSignals`, `push`, `focus`, `overview`; a `ViewGroup` forwards to every
  view).
- **Benefit:** the page wires sites, cameras, maps, signals and world states once (31 mirrored
  calls and the duplicated `startSingle` wiring go); the side panel counts from one place
  (`objects()` of the active view, after the shared merge rule) — fixes D2 and D3.
- **Risk:** low (type-checked, behaviour unchanged).
- **Verify:** build and tests; in the browser, managed and single-server mode: both views show the
  same sites, maps, signals and road users; the "Road users" counter is equal in 3D and BEV.

### Q4. Shared thresholds and the `world/` folder — 0.5 day

- **Files:** new `web/src/world/thresholds.ts` (stale time, standing speed and σ, "moving" for
  heading trust, merge gates); moved: `twin/fusion.ts`, `bev/bevModel.ts`,
  `bev/laneAnalytics.ts`, `bev/geometry.ts` → `world/`; imports in `twinView.ts`, `bevView.ts`,
  `bevPanel.ts`, `bevMode.ts`, `pages/twin.ts`, tests, `tools/fidelity/harness.js` (if it
  imports paths; it uses runtime objects only); one shared `roadAxisAt(site map, transform)` in
  `world/` replacing both `road()` (D5).
- **Benefit:** one place for every number that decides about the world; the folder name says what
  is decision logic (D4, D5, §1.2).
- **Risk:** low for the move; the 3D "moving" rule is kept as its own named constant (no visual
  change). `net/client.ts`'s 5 s reconnect timeout is a different concept and stays.
- **Verify:** build and tests unchanged; `git diff -M` shows moves.

### Q5. Remove verified dead code — 0.1 day

- **Files:** as listed in §1.4.
- **Benefit:** less surface. **Risk:** very low.
- **Verify:** `tsc`, lint, tests; repository-wide search for the names returns nothing.

### Q6. Essential regression tests — 0.5 day

- **New `manager/tests/test_traffic_stats.py`** (standard `unittest`, no new dependency): crossing
  direction; one crossing per track and line within `RECROSS_S`; a new run after a seek or loop;
  report totals for a small fixture database. Run from `scripts/build_and_test.sh`.
- **Web:** the shared merge rule (Q2); `ViewGroup` forwards every call (Q3); thresholds used by
  both views (Q4).
- **Benefit:** the statistics, which hold the operator's data, get a safety net before any later
  change. **Risk:** none for production.
- **Verify:** `build_and_test.sh all` runs the Python tests and fails if one fails.

### Q7. Split `pages/twin.ts` — 0.5 day

- **Files:** `web/src/pages/twin.ts` → `pages/twin.ts` (page controller), new
  `ui/twinToolbar.ts` (`setupControls`, `layersMenu`, `analysisToggle`, `alignButton`), new
  `ui/signalBar.ts` (traffic-light placement bar).
- **Benefit:** the page file shrinks to about 600 lines with one job. **Risk:** low (moves).
- **Verify:** build; toolbar, Layers, Align map and Traffic lights work in the browser.

### Q8. One storage helper — 0.25 day (optional)

- **Files:** new `web/src/ui/storage.ts` (`readFlag`, `writeFlag`, `readJson`, `writeJson`, all
  failure-tolerant); the 8 files of D6.
- **Benefit:** less boilerplate, one default rule. **Risk:** very low (same keys).
- **Verify:** settings survive a reload (theme, panel width, BEV mode, card state, camera grid).

### Q9. Repository hygiene and documentation — 0.25 day

- Delete the three merged feature branches (local and origin) and the stale `server/main` ref;
  keep `archive/pre-v2` (needs approval: deleting remote branches).
- Decide the untracked folders: `manualInferance/` → its own repository or `.gitignore` (it holds
  a Python environment); CARLA and Koper scenes → commit if they belong to the project, else
  ignore.
- `docs/architecture.md`: the ownership table of §2, the BEV, the camera grid and the Layers menu,
  the `world/` folder.

### Summary

| Task | Days | Risk | Main benefit |
|---|---|---|---|
| Q1 deploy and sites folder | 0.5 | medium | deploy works, server clean, tests pass there |
| Q2 one merge rule | 0.5 | low–medium | consistent road users and counts across views |
| Q3 one feed for both views | 0.75 | low | wiring once; consistent counter |
| Q4 thresholds and `world/` | 0.5 | low | decisions in one visible place |
| Q5 dead code | 0.1 | very low | less surface |
| Q6 regression tests | 0.5 | none | statistics and shared rules protected |
| Q7 split `twin.ts` | 0.5 | low | readable page code |
| Q8 storage helper (optional) | 0.25 | very low | less boilerplate |
| Q9 hygiene and docs | 0.25 | low | clear repository and ownership |
| **Total** | **≈ 3.85** (3.6 without Q8) | | |

Smallest set with the largest effect: **Q1, Q2, Q3, Q6** (≈ 2.25 days).

## 4. Correctness fixes (need more implementation time)

Not part of the cleanup; each changes results and needs its own measurement.

| # | Issue | Evidence | Fix outline | Effort |
|---|---|---|---|---|
| C1 | Phantom headings: a 1–2 m jump along the viewing ray of a standing vehicle passes the motion test and is held | live cam1 track 2: 1.5 m in 0.6 s at 137°, held on a 260° one-way road; gate 1.64 ≥ 2 × 0.78 | `TrackLocalizer::apply_motion`: require a net displacement (e.g. ≥ 1.5 m over ≥ 0.8 s) and test the speed against its σ along the motion; evaluate standing-vehicle heading error on TUMTraf | 1–1.5 d |
| C2 | Statistics count a road user once per camera in overlaps, and time crossings by reception, not capture | `traffic_stats.py` `CameraCounter.process` (per camera, `time.time()`) | site-level de-duplication of crossings in the manager (same line and direction, close in time and place, other camera); time from the site clock mapping for recordings | 1–2 d |
| C3 | Missing heading becomes 0° in the 3D interpolator; 3D shows invented headings (flow field, road side) | `twin/interpolation.ts` (`?? 0`), `twinView.ts` `entryHeading` | keep "unknown" (disc footprint already shows it); flow field only as a marked hint | 0.5–1 d |
| C4 | Lane direction learned per browser (localStorage): different queues in different browsers | `bev/laneAnalytics.ts` `save/load` | a direction property on lane zones, set in the calibration (drawn order or a flip button); learned direction only as a suggestion | 1 d |
| C5 | GPU fallback to the CPU is silent (only a log line) | 2026-11-04: all servers on CPU at 0.5 fps | jp_server reports its execution provider in `/api/status`; console marks the camera degraded | 0.5 d |
| C6 | Conflicts only within one camera | `vru::EventDetector` per jp_server | needs site-level state → long term | — |

## 5. Long term (unchanged roadmap)

`docs/world-model-architecture.md`: capture timestamps, timestamp-aligned and uncertainty-aware
fusion, global IDs, heading provenance, lanes with stop lines and signal groups, backend
analytics, persistent site statistics, one site world state, the `jp_site` process. The cleanup
prepares it: Q2–Q4 put every frontend decision into `web/src/world/`, which is exactly the code
that moves to the backend later; Q6 gives the statistics tests that the migration must keep green.

## 6. What stays untouched

- C++ perception: detection, tracking, localization, heading logic, Kalman filter, calibration
  math, events, signal reading, evaluation and benchmark code.
- World state schema v1 and its fixture; the JSONL format; the manager's REST API and site clock.
- Statistics schema and report format (Q6 only tests them).
- BEV and 3D appearance and behaviour (except the BEV duplicate rule aligned to the 3D one, Q2).
- Calibration page, map tools, playback, camera grid, Layers menu, footprints, analysis mode.
- The fidelity harness (adapted only if a moved path breaks it).

## 7. Order and verification

1. Q5 and Q6 first (tests before moves). 2. Q4 (moves). 3. Q2 and Q3. 4. Q7, Q8. 5. Q9 docs.
6. Q1 last, on a separate branch with its own server step, at a quiet moment.

After each step: `build_and_test.sh all release` (C++, web, Python), the fidelity harness for Q2–Q4
(footprints = backend 0.000 m, BEV duplicates = 3D merges), and a browser check of BEV, 3D,
calibration, playback, statistics and the camera grid. Deploy only after approval.
