export interface SlaveState {
  status: State;
  sensors: Record<string, boolean>;
}

export type State = "IDLE";

export type SlaveSettings = Record<SettingsKeys, any>;

export type SettingsKeys = "sensorThreshold";

export type CommandType =
  | "start"
  | "stop"
  | "home"
  | "goto"
  | "extend"
  | "retract"
  | "suction"
  | "setSpeed"
  | "setAccel";

export interface CommandMessage {
  type: CommandType;
  params?: any;
}

export type Command = string | CommandMessage;
