export type Screen = "run" | "teach" | "sim" | "motion";
export type PanelId =
  | "jointList"
  | "arm"
  | "readout"
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
  motors: { id: number; model: string }[];
  total: number;
  online: number;
  active: number;
  fault: number;
  motionAt: number | null;
  communication: {link: string; clock: string; authority: string; detail: string};
  gravity: {enabled: boolean | null; active: boolean; blocked: string; pending: boolean};
  session: string;
  mappingError: string;
}
export interface MountedPanel {
  render(dirty: Set<string>): void;
  unmount?(): void;
  intervalMs?: number;
}
