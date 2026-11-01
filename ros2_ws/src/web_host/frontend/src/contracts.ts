export type Screen = "run" | "teach" | "sim" | "motion";
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
  managed: {pending: boolean; enabled: boolean; ownsControl: boolean; armed: boolean; mcuAuthorized: boolean; ready: boolean;
    message: string; blockers: string[]};
  firmwareConnectionKey: string;
  connected: boolean;
  connectionText: string;
  agent: string;
  agentDevice: string;
  agentError: string;
  canStopAgent: boolean;
  model: string;
  selectedId: number;
  motors: { id: number; model: string }[];
  selectedName: string;
  selectedState: string;
  selectedCanCommand: boolean;
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
