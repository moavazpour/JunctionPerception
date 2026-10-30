import { fileURLToPath } from 'node:url';

import { defineConfig } from 'vitest/config';

export default defineConfig({
  build: {
    // Three.js alone is about 500 kB (127 kB gzipped).
    chunkSizeWarningLimit: 600,
    rollupOptions: {
      input: {
        twin: fileURLToPath(new URL('./index.html', import.meta.url)),
        calibrate: fileURLToPath(new URL('./calibrate.html', import.meta.url)),
      },
      output: {
        // Three.js in its own chunk: it changes rarely and stays cached.
        manualChunks: { three: ['three'] },
      },
    },
  },
  test: {
    include: ['tests/**/*.test.ts'],
  },
});
