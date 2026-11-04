# ADR 0003: BEV is the only twin; analytics and events are out of scope

Status: accepted, November 2026.

## Decision

The product is the video-to-BEV digital twin. Removed from the active code: the 3D twin
(Three.js), VRU and conflict events, warnings, lane/queue/waiting-time analytics, traffic
statistics (line counts, reports, SQLite) and the 3D fidelity harness. Zones remain as site
geometry, traffic-light state remains as observed world state.

## Reason

The removed parts duplicated logic (two merge rules, two motion definitions), made physical
decisions in the browser (headings from flow fields and OSM, lane directions learned per
browser) and computed per camera what needs the fused site state. Keeping them would block a
single source of truth.

## Alternatives considered

Hiding the 3D tab and keeping the modules: rejected, disconnected code keeps costing and misleads.

## Consequences

- Everything removed is recoverable from git (tag `checkpoint/pre-bev-refactor`, branches
  `feature/3d-fidelity`, `feature/bev-experiment`). Existing statistics databases
  (`.cache/manager/stats.sqlite`) are left untouched on disk.
- Analytics, if they return, are computed in the backend from the site state (one owner), with an
  ADR.
