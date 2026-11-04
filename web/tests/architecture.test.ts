/**
 * Architecture rules of the frontend (AGENTS.md): it draws the backend's site state and edits
 * configuration; it never fuses, re-identifies or estimates road users itself.
 */
import { readdirSync, readFileSync, statSync } from 'node:fs';
import { join, relative } from 'node:path';
import { fileURLToPath } from 'node:url';

import { describe, expect, it } from 'vitest';

const web = fileURLToPath(new URL('..', import.meta.url));
const src = join(web, 'src');

function files(dir: string): string[] {
  return readdirSync(dir).flatMap((name) => {
    const path = join(dir, name);
    return statSync(path).isDirectory() ? files(path) : path.endsWith('.ts') ? [path] : [];
  });
}

const sources = files(src).map((path) => ({
  path: relative(web, path).replaceAll('\\', '/'),
  text: readFileSync(path, 'utf8'),
}));

function offenders(pattern: RegExp, except: string[] = []): string[] {
  return sources.filter((f) => pattern.test(f.text) && !except.includes(f.path)).map((f) => f.path);
}

describe('frontend architecture', () => {
  it('has no 3D engine', () => {
    const pkg = JSON.parse(readFileSync(join(web, 'package.json'), 'utf8')) as {
      dependencies?: Record<string, string>;
      devDependencies?: Record<string, string>;
    };
    expect(Object.keys({ ...pkg.dependencies, ...pkg.devDependencies })).not.toContain('three');
    expect(offenders(/from ['"]three/)).toEqual([]);
  });

  it('reads road users only from the site state, never from camera streams', () => {
    // /ws/world is the per-camera observation stream: the manager's world model consumes it.
    // Addresses are template literals; doc comments may name the streams.
    expect(offenders(/`[^`]*\/ws\/world/)).toEqual([]);
    expect(offenders(/`[^`]*\/ws\/site/)).toEqual(['src/net/client.ts']);
  });

  it('defines the road-user state in one module', () => {
    expect(offenders(/interface RoadUser\b|interface WorldObject\b/)).toEqual([
      'src/net/siteState.ts',
    ]);
  });

  it('does not fuse, associate or re-identify road users', () => {
    expect(offenders(/\b(fus(e|ion)|associat|dedup|duplicate|merge[A-Z])\w*\s*\(/i)).toEqual([]);
    expect(sources.map((f) => f.path).filter((p) => /fusion|tracker|associat/i.test(p))).toEqual(
      [],
    );
  });

  it('does not derive headings from motion or map data', () => {
    // A heading from a velocity (atan2 of vy, vx) or a road axis is the backend's decision.
    expect(offenders(/atan2\([^)]*\bv[xy]\b/)).toEqual([]);
    expect(offenders(/heading\w*\s*=\s*[^;]*\b(roadAxis|osm|lane)\w*/i)).toEqual([]);
  });

  it('has no traffic analytics, events or conflicts', () => {
    expect(offenders(/\b(conflict|queue(Length|M)\b|waitingTime|laneAnalytics|EventLog)/)).toEqual(
      [],
    );
  });
});
