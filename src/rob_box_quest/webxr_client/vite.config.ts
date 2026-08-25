import { defineConfig } from "vite";

// Phase 1.5: клиент собирается vite'ом, артефакт уезжает в dist/.
// base='/quest/' нужен для Caddy reverse-proxy (см. дизайн §7).
// Никаких CDN: все зависимости через npm + bundle.
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
          three: ["three"],
          gui: ["lil-gui"]
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
    globals: true,
    environment: "jsdom",
    include: ["tests/**/*.test.ts"]
  }
});