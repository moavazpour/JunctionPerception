# ADR 0004: Lane queue and passing counts return, in the backend

Status: accepted, November 2026.

## Decision

The site owner asked for each lane's queue in the twin and a live statistics view: how many
vehicles passed each lane, and the intersection's state now. They are computed in the manager
(`manager/traffic.py`) from the site state, on the world model's global road-user IDs, and
sent with it as `traffic` (site state v1, additive field). Lanes are drawn in the camera images
(`regions.json` lines with three or more points). The user sets each lane's direction of travel
there with two clicks; nothing derives it. The removed "intersection" zone type had no
consumer; zones are crosswalks and lanes.

## Reason

ADR 0003 removed analytics because they ran per camera and in the browser, with directions
guessed from drawing order, OSM or learned flow. On the fused state with global IDs a vehicle
seen by several cameras is counted once. With a direction the user sets, no direction is
invented.

## Alternatives considered

- Lanes as site zones drawn on the map: rejected by the owner, who draws them on the camera
  image, where lane markings are visible.
- Counts in the browser: rejected (AGENTS.md §2); every viewer would count differently.
- Stored history (intervals on disk): not requested; counts are live and start over with each
  run.

## Consequences

- Waiting times, conflicts, warnings and reports stay out of scope (ADR 0003).
- A lane two cameras both outline is two lanes; both count the same vehicle once each.
- Counts depend on the world model's ID stability: an ID switch inside a lane can drop a pass
  (the new ID entered mid-lane and travelled less than `min_travel_m`) or, rarely, count one
  twice. Not yet measured against labels.
