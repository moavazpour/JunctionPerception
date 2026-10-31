import type { WorldEvent, WorldState } from '../net/schema';

const HISTORY_LENGTH = 12;

export const EVENT_LABELS: Record<WorldEvent['type'], string> = {
  VRU_ON_CROSSWALK: 'VRU on crosswalk',
  VRU_VEHICLE_CONFLICT: 'VRU-vehicle conflict',
};

/** One line of detail for an event, e.g. "#17, #42 · 1.4 m in 1.2 s · crosswalk_n". */
export function describeEvent(event: WorldEvent): string {
  const parts = [event.object_ids.map((id) => `#${id}`).join(', ')];
  if (event.min_distance_m !== null && event.time_to_closest_s !== null) {
    parts.push(
      `${event.min_distance_m.toFixed(1)} m gap in ${event.time_to_closest_s.toFixed(1)} s`,
    );
  }
  if (event.vehicle_state !== null && event.vehicle_state !== 'moving') {
    parts.push(`vehicle ${event.vehicle_state}`);
  }
  if (event.zone !== null) {
    parts.push(event.zone);
  }
  return parts.join(' · ');
}

/** Active events of the latest state and the most recently ended ones. */
export class EventLog {
  private active: WorldEvent[] = [];
  private ended: WorldEvent[] = [];

  update(state: WorldState): void {
    this.active = state.events.filter((e) => e.state === 'active');
    for (const event of state.events.filter((e) => e.state === 'ended')) {
      this.ended = [event, ...this.ended.filter((e) => e.id !== event.id)].slice(0, HISTORY_LENGTH);
    }
  }

  /** Forgets everything, e.g. after a seek. */
  clear(): void {
    this.active = [];
    this.ended = [];
  }

  entries(): { event: WorldEvent; active: boolean }[] {
    return [
      ...this.active.map((event) => ({ event, active: true })),
      ...this.ended.map((event) => ({ event, active: false })),
    ];
  }
}

// Timestamps above this are absolute (Unix time), below it relative to the start of a recording.
const ABSOLUTE_TIMESTAMP_NS = 1e15;

/** "12.3 s" for recording-relative timestamps, UTC clock time for absolute ones. */
export function formatTimestamp(timestampNs: number): string {
  if (timestampNs < ABSOLUTE_TIMESTAMP_NS) {
    return `${(timestampNs / 1e9).toFixed(1)} s`;
  }
  return `${new Date(timestampNs / 1e6).toISOString().slice(11, 21)} UTC`;
}
