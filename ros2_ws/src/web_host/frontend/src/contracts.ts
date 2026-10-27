export type Screen = "run" | "teach" | "sim";
export type PanelId =
  | "jointList"
  | "arm"
  | "readout"
  | "policy"
  | "command"
  | "feedforward"
  | "deviceParams"
  | "actionLibrary"
  | "sim"
  | "curve"
  | "events"
  | "imu";
export interface WorkbenchSnapshot {
  firmwareConnectionKey: string;
  connected: boolean;
  connectionText: string;
  agent: string;
  agentDevice: string;
  agentError: string;
  canStopAgent: boolean;
  model: string;
  selectedId: number;
  selectedName: string;
  selectedState: string;
  total: number;
  online: number;
  active: number;
  fault: number;
  motionAt: number | null;
  outputTopic: string;
  session: string;
  mappingError: string;
}
export interface MountedPanel {
  render(dirty: Set<string>): void;
  unmount?(): void;
  intervalMs?: number;
}
