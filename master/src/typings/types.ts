export interface SlaveState {
  status?: State;
  position?: {
    x: number;
    y: number;
  };
  sensors?: {
    xEndstop: boolean;
    yEndstop: boolean;
    armExtended: boolean;
    suctionEnabled: boolean;
  };
  motion?: {
    speed: number;
    acceleration: number;
  };
  error?: string;
  isHomed?: boolean;
}

export type State =
  | "IDLE"
  | "HOME_REQUESTED"
  | "HOMING_X"
  | "HOMING_Y"
  | "AWAITING_START"
  | "MOVING_TO_PICK"
  | "MOVING_TO_TARGET"
  | "PICKING"
  | "PLACING"
  | "RETRACTING"
  | "WAITING_TO_RETRIEVE"
  | "EXECUTING_PATTERN"
  | "MANUAL_MOVING"
  | "MOVING"
  | "ERROR";

export type SlaveSettings = Record<SettingsKeys, any>;

export type SettingsKeys =
  | "sensorThreshold"
  | "speed"
  | "acceleration"
  | "boxWidth"
  | "boxLength"
  | "boxX"
  | "boxY"
  | "pickupX"
  | "pickupY"
  | "rows"
  | "columns";

export type CommandType = string;

export interface CommandParams {
  direction?: string;
  state?: "on" | "off" | "START" | "STOP";
  speed?: number;
  acceleration?: number;
  value?: number;
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

export interface MachinePosition {
  x: number;
  y: number;
  isHomed: boolean;
}

export interface MachineStatus {
  state: State;
  position: MachinePosition;
  sensors: {
    xEndstop: boolean;
    yEndstop: boolean;
    armExtended: boolean;
    suctionEnabled: boolean;
  };
  motion?: {
    speed?: number;
    acceleration?: number;
  };
  pattern?: {
    current: number;
    total: number;
    boxX?: number;
    boxY?: number;
    boxWidth?: number;
    boxLength?: number;
    pickupX?: number;
    pickupY?: number;
  };
  error?: string;
}

export type SuctionCommand = {
  type: "suction";
  params: {
    state: boolean | "on" | "off";
  };
};
