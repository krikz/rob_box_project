import { defineConfig } from "vite";

// Phase 2+ WebXR client. base='/quest/' — Caddy reverse proxy (см. ADR-0027 §4.1).
// Артефакт собирается в dist/, грузится через https://10.1.1.11/quest/ .
export default defineConfig({
  base: "/quest/",
  root: ".",
  build: {
    outDir: "dist",
    emptyOutDir: true,
    sourcemap: false,
    target: "es2022",
    rollupOptions: {
      output: {
        manualChunks: {
          three: ["three"]
        }
      }
    }
  },
  server: {
    port: 5173,
    proxy: {
      "/quest": {
        target: "ws://127.0.0.1:8443",
        ws: true,
        rewriteWsOrigin: true
      }
    }
  },
  test: {
    environment: "jsdom",
    globals: true,
    include: ["tests/**/*.test.ts"]
  }
});