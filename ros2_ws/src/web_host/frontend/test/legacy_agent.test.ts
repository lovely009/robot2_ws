import { afterEach, beforeEach, expect, it, vi } from "vitest";

const { callService } = vi.hoisted(() => ({ callService: vi.fn() }));
vi.mock("../../web/js/core/ros.js", () => ({ callService }));

const stm32 = {
  path: "/dev/serial/by-id/usb-STMicroelectronics_STM32H723-if00",
  label: "STM32H723 (/dev/ttyACM0)",
  accessible: true,
};
const other = { path: "/dev/ttyUSB0", label: "USB UART", accessible: true };
let saved: string;
let select: {
  value: string;
  disabled: boolean;
  options: any[];
  replaceChildren(): void;
  append(option: any): void;
};

beforeEach(() => {
  vi.resetModules();
  saved = "";
  vi.stubGlobal("location", { search: "" });
  select = {
    value: "", disabled: false, options: [],
    replaceChildren() { this.options = []; this.value = ""; },
    append(option) { this.options.push(option); },
  };
  vi.stubGlobal("localStorage", {
    getItem: () => saved || null,
    setItem: (_key: string, value: string) => { saved = value; },
  });
  vi.stubGlobal("document", {
    getElementById: () => select,
    createElement: () => ({ value: "", textContent: "", disabled: false }),
  });
});
afterEach(() => vi.unstubAllGlobals());

async function refresh(ports: object[], status = { state: "stopped", device: "" }) {
  callService.mockImplementation(async (name: string) => name === "/bench/options"
    ? { serial_ports_json: JSON.stringify(ports) }
    : { status_json: JSON.stringify(status) });
  const { refreshAgentControls } = await import("../../web/js/core/agent.js");
  await refreshAgentControls();
}

it("无历史选择时自动选中唯一可访问的 STM32，并保存选择", async () => {
  await refresh([other, stm32]);
  expect(select.value).toBe(stm32.path);
  expect(saved).toBe(stm32.path);
});

it("历史路径失效时按串口标签识别 STM32", async () => {
  saved = "/dev/old";
  await refresh([{ ...stm32, path: "/dev/ttyACM0" }]);
  expect(select.value).toBe("/dev/ttyACM0");
});

it("不覆盖有效的历史选择", async () => {
  saved = other.path;
  await refresh([other, stm32]);
  expect(select.value).toBe(other.path);
});

it("运行中的 Agent 优先于历史选择和 STM32", async () => {
  saved = stm32.path;
  await refresh([stm32], { state: "running", device: "/dev/active" });
  expect(select.value).toBe("/dev/active");
});

it.each([
  [other],
  [{ ...stm32, accessible: false }],
  [stm32, { ...stm32, path: "/dev/second-stm32" }],
  [],
])("没有唯一可访问的 STM32 时保持未选择：%j", async (...ports) => {
  await refresh(ports);
  expect(select.value).toBe("");
});
