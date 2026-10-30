import { defineConfig } from 'vitest/config';

export default defineConfig({
  build: {
    // Three.js alone is about 500 kB (127 kB gzipped).
    chunkSizeWarningLimit: 600,
    rollupOptions: {
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
