import react from "@vitejs/plugin-react";
import { execSync } from "node:child_process";
import { existsSync, readFileSync } from "node:fs";
import { defineConfig } from "vite";

// Version of the whole system (repo VERSION file; /VERSION inside the Docker build) and the git commit.
const versionFile = ["../VERSION", "/VERSION"].find((p) => existsSync(p));
const version = versionFile ? readFileSync(versionFile, "utf8").trim() : "dev";
let commit = process.env.JP_GIT_SHA ?? "";
if (!commit)
  try {
    commit = execSync("git rev-parse --short HEAD", { stdio: ["ignore", "pipe", "ignore"] }).toString().trim();
  } catch {
    commit = "";
  }

export default defineConfig({
  plugins: [react()],
  define: { __APP_VERSION__: JSON.stringify(version), __GIT_SHA__: JSON.stringify(commit) },
  server: {
    host: true,
    proxy: { "/mqtt": { target: "ws://localhost:9001", ws: true } },
  },
  build: { chunkSizeWarningLimit: 2000 },
});
