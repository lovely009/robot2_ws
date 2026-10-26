import { defineConfig } from "@playwright/test";

export default defineConfig({
  testDir: "./test/browser",
  workers: 1,
  use: {
    baseURL: "http://127.0.0.1:8092",
    viewport: { width: 1600, height: 1000 },
    launchOptions: {
      executablePath:
        process.env.PLAYWRIGHT_CHROMIUM_EXECUTABLE_PATH || undefined,
      args: ["--use-angle=swiftshader", "--enable-unsafe-swiftshader"],
    },
  },
  webServer: {
    command: "python3 test/serve_static.py",
    url: "http://127.0.0.1:8092/workbench/",
    reuseExistingServer: false,
  },
});
