export interface SlaveState {
  status: State;
  sensors: Record<string, boolean>;
}

export type State = "IDLE";

export type SlaveSettings = Record<SettingsKeys, any>;

export type SettingsKeys = "sensorThreshold";

export type CommandType = string;

export interface CommandParams {
  direction?: string;
  state?: "on" | "off";
  speed?: number;
  acceleration?: number;
  rows?: number;
  cols?: number;
  startX?: number;
  startY?: number;
  gridWidth?: number;
  gridLength?: number;
  pickupX?: number;
  pickupY?: number;
  x?: number;
  y?: number;
  enabled?: boolean;
  accel?: number;
}

export interface CommandMessage {
  type: CommandType;
  params?: CommandParams;
}

export type Command = string | CommandMessage;
