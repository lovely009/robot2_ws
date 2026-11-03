import { defineConfig } from '@playwright/test';

// 所有页面和 ROS 请求由测试拦截，不启动 HTTP/ROS 服务。
export default defineConfig({
  testDir: './browser',
  testMatch: 'ros2_control.spec.ts',
  workers: 1,
  outputDir: '/tmp/robot2-control-migration-20261029/browser-results',
  use: {baseURL:'http://control.test',
    launchOptions:{executablePath:process.env.PLAYWRIGHT_CHROMIUM_EXECUTABLE_PATH || undefined}},
});
