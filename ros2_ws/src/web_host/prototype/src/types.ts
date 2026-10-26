export interface Joint {
  name: string;
  parent: string;
  child: string;
  type: string;
  lower: number;
  upper: number;
  motor: {
    id: number;
    type: string;
    bus: string;
    name: string;
    command?: { kp: number; kd: number };
  } | null;
}
export interface RobotModel {
  name: string;
  joints: Joint[];
  gazebo: boolean;
  mujoco: boolean;
}
export type Screen = "run" | "teach" | "sim";
export type Variant = "A" | "B" | "C";
export interface DemoEvent {
  at: string;
  level: "info" | "ok" | "warn";
  text: string;
}

declare global {
  const __ROBOT_SIM_ROOT__: string;
}
