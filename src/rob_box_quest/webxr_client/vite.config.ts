import { defineConfig } from "vite";

// Phase 1.5: клиент собирается vite'ом, артефакт уезжает в dist/.
// base='/' — ассеты Three.js идут по /assets/*, обслуживаются Caddy file_server
// из /srv/quest_static (см. docker/vision/quest/Caddyfile).
// WSS-канал /quest остаётся в aiohttp через reverse_proxy.
// Никаких CDN: все зависимости через npm + bundle.
export default defineConfig({
  base: "/",
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