# ADR 0002: Every observation says what its time is; the backend aligns, or refuses to fuse

Status: accepted, November 2026.

## Decision

Camera observations carry `timestamp_ns` and `time_source`: `media` (recording time), `capture`
(absolute capture time) or `arrival` (receive time of live streams, stamped by `jp_server` on
arrival instead of the decoder clock). The world model compares cameras only on one clock kind:
media times under the site playback clock (plus a per-camera `time_offset_ms`), or absolute
times (plus the same offset). It predicts every camera track to one site instant before
association; otherwise the site runs explicitly unfused (`fusion.mode = "unfused"` with the
reason).

## Reason

Elapsed video time is not capture time; streams have no capture time without RTP/RTCP. The old
browser fusion merged tracks of different instants; with one camera 300 ms late, 91 % of the
overlap was duplicated. Hiding that limitation would produce confident but wrong road users.

## Alternatives considered

- RTCP sender reports (NTP) for live streams: correct, but needs a different decoder path than
  OpenCV's FFmpeg capture. Left for when live multi-camera sites exist.
- Estimating offsets automatically from co-observed road users: possible later (residual along
  the velocity); today offsets are configured.

## Consequences

- Recorded multi-camera sites are fused only while they play by the site clock.
- Measured on TUMTraf R02 s03: a 300 ms clock error degrades the result; configuring the
  matching offset restores it exactly (docs/evaluation.md).
- Live-stream timing includes network and decoder latency, uncorrected unless configured.
