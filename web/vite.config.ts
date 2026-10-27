import react from "@vitejs/plugin-react";
import { defineConfig } from "vite";

export default defineConfig({
  plugins: [react()],
  server: {
    host: true,
    proxy: { "/mqtt": { target: "ws://localhost:9001", ws: true } },
  },
  build: { chunkSizeWarningLimit: 2000 },
});
