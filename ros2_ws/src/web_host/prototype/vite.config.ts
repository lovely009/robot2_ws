import { defineConfig } from "vite";
import vue from "@vitejs/plugin-vue";
import { execFileSync } from "node:child_process";
import { fileURLToPath } from "node:url";

const robot_sim = fileURLToPath(new URL("../../robot_sim/", import.meta.url));
const metadata = execFileSync(
  "python3",
  [fileURLToPath(new URL("read_models.py", import.meta.url))],
  { encoding: "utf8" },
);

export default defineConfig({
  plugins: [
    vue(),
    {
      name: "prototype-read-only-models",
      configureServer(server) {
        server.middlewares.use("/api/prototype-models", (_req, res) => {
          res.setHeader("Content-Type", "application/json; charset=utf-8");
          res.end(metadata);
        });
      },
    },
  ],
  server: {
    fs: { allow: [fileURLToPath(new URL(".", import.meta.url)), robot_sim] },
  },
  define: { __ROBOT_SIM_ROOT__: JSON.stringify(`/@fs${robot_sim}`) },
});
