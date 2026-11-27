import { defineConfig } from "vite";
import vue from "@vitejs/plugin-vue";

const upstream = process.env.WORKBENCH_HTTP_ORIGIN || "http://127.0.0.1:8081";
export default defineConfig({
  base: "/workbench/",
  plugins: [vue()],
  build: {
    outDir: "../web/workbench",
    emptyOutDir: true,
    rolldownOptions: {
      external: [/^three(?:\/|$)/, /^\/vendor\//],
      output: {
        entryFileNames: "assets/workbench-[hash].js",
        chunkFileNames: "assets/[name]-[hash].js",
        assetFileNames: "assets/[name]-[hash][extname]",
      },
    },
  },
  server: {
    proxy: Object.fromEntries(
      [
        "/api",
        "/robot_sim",
        "/vendor",
        "/roslib.min.js",
        "/panel_templates.html",
      ].map((path) => [path, { target: upstream }]),
    ),
  },
});
