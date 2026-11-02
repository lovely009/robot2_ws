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
      output: {
        entryFileNames: "assets/workbench.js",
        chunkFileNames: "assets/[name].js",
        assetFileNames: "assets/[name][extname]",
      },
    },
  },
  server: {
    proxy: Object.fromEntries(
      [
        "/api",
        "/robot_sim",
        "/js",
        "/css",
        "/vendor",
        "/roslib.min.js",
        "/panel_templates.html",
      ].map((path) => [path, { target: upstream }]),
    ),
  },
});
