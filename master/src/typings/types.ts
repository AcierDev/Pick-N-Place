export interface SlaveState {
  status: State;
  sensors: Record<string, boolean>;
}

export type State = "IDLE";

export type SlaveSettings = Record<SettingsKeys, any>;

export type SettingsKeys = "sensorThreshold";

export type Command = string;
