import { fileURLToPath } from 'node:url';

import { defineConfig } from 'vitest/config';

const page = (file: string): string => fileURLToPath(new URL(`./${file}`, import.meta.url));

export default defineConfig({
  build: {
    // Three.js alone is about 500 kB (127 kB gzipped).
    chunkSizeWarningLimit: 600,
    rollupOptions: {
      // One page per console view; they share the shell and the theme.
      input: {
        overview: page('index.html'),
        twin: page('twin.html'),
        calibrate: page('calibrate.html'),
        site: page('site.html'),
        newSite: page('new-site.html'),
        system: page('system.html'),
        logs: page('logs.html'),
      },
      output: {
        // Three.js in its own chunk: it changes rarely and stays cached.
        manualChunks: { three: ['three'] },
      },
    },
  },
  server: {
    // Development against a running manager: JP_CONSOLE=http://<host>:8080 npm run dev
    proxy: process.env['JP_CONSOLE']
      ? {
          '/manager': process.env['JP_CONSOLE'],
          '/sites': { target: process.env['JP_CONSOLE'], ws: true },
        }
      : undefined,
  },
  test: {
    include: ['tests/**/*.test.ts'],
  },
});
