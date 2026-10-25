import { defineConfig } from "vite";

export default defineConfig({
  server: {
    host: true,
    proxy: { "/mqtt": { target: "ws://localhost:9001", ws: true } },
  },
  build: { chunkSizeWarningLimit: 1500 },
});
