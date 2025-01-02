import { SerialPort } from "serialport";
import { ReadlineParser } from "@serialport/parser-readline";
import { SlaveState, SlaveSettings, Command, State } from "./typings/types";
import { detectMicrocontrollerPort } from "./util/portDetection";
import chalk from "chalk";

// Define log styling constants
const LOG_STYLES = {
  OUTGOING: {
    COMMAND: chalk.yellow("⟸"),
    SERIAL: chalk.cyan,
    PREFIX: chalk.yellow("CMD"),
  },
  INCOMING: {
    RAW: chalk.gray("●"),
    STATE: chalk.green("⟹"),
    DEBUG: chalk.magenta("◆"),
    ERROR: chalk.red("✖"),
    INFO: chalk.blue("ℹ"),
    POSITION: chalk.yellow("◎"),
  },
  STATUS: {
    SUCCESS: chalk.green("✓"),
    ERROR: chalk.red("✗"),
    CONNECTING: chalk.blue("⟲"),
  },
  formatPosition: (x: number, y: number) =>
    chalk.cyan(`(${x.toFixed(1)}, ${y.toFixed(1)})`),
  formatHex: (data: Buffer) =>
    chalk.gray(data.toString("hex").match(/../g)?.join(" ")),
  formatTime: () => chalk.gray(`[${new Date().toLocaleTimeString()}]`),
};

export class SerialCommunication {
  private port: SerialPort | null = null;
  private parser: ReadlineParser | null = null;
  private stateUpdateCallback?: (state: SlaveState) => void;
  private debugMessageCallback?: (message: string) => void;
  private messageCallback?: (message: string) => void;
  private errorCallback?: (message: string, code: number) => void;
  private responseCallback?: (response: {
    success: boolean;
    message: string;
  }) => void;

  // Message type constants matching slave
  private static readonly STATE_MSG = "S";
  private static readonly DEBUG_MSG = "D";
  private static readonly INFO_MSG = "I";
  private static readonly ERROR_MSG = "E";
  private static readonly RESPONSE_MSG = "R";

  constructor() {
    this.port = null;
    this.parser = null;
  }

  async connect(): Promise<boolean> {
    const portPath = await detectMicrocontrollerPort();
    if (!portPath) {
      console.log(
        LOG_STYLES.STATUS.ERROR,
        chalk.red("Microcontroller not found")
      );
      return false;
    }

    try {
      console.log(
        LOG_STYLES.STATUS.CONNECTING,
        `Attempting to connect to ${chalk.cyan(portPath)} at ${chalk.yellow(
          "115200"
        )} baud`
      );

      this.port = new SerialPort({
        path: portPath,
        baudRate: 115200,
        autoOpen: false,
      });

      // Add error listener before opening
      this.port.on("error", (err) => {
        console.error("Serial port error:", err.message);
      });

      await new Promise<void>((resolve, reject) => {
        this.port!.open((err) => {
          if (err) {
            reject(err);
          } else {
            resolve();
          }
        });
      });

      // Create parser for text messages
      this.parser = new ReadlineParser({ delimiter: "\n" });

      // Handle raw data first
      this.port.on("data", (data: Buffer) => {
        // Check if it's a position update (binary message)
        if (data[0] === 0x50 && data.length >= 9) {
          this.handlePositionUpdate(data);
        } else {
          // Pass other messages to text parser
          this.parser?.write(data);
        }
      });

      // Handle parsed text messages
      this.parser.on("data", (line: string) => {
        const data = Buffer.from(line);
        if (data[0] !== 0x50) {
          // Skip position updates
          this.handleIncomingData(data);
        }
      });

      console.log(
        LOG_STYLES.STATUS.SUCCESS,
        chalk.green(`Connected to microcontroller on ${chalk.cyan(portPath)}`)
      );
      return true;
    } catch (error) {
      console.log(
        LOG_STYLES.STATUS.ERROR,
        chalk.red("Connection failed:"),
        chalk.gray(error)
      );
      return false;
    }
  }

  private checkConnection() {
    if (!this.port || !this.parser) {
      throw new Error("Serial port not connected");
    }
  }

  sendCommand(command: Command): void {
    this.checkConnection();
    let jsonCommand: any;

    if (typeof command === "string") {
      jsonCommand = {
        type: command,
        params: {},
      };
    } else {
      jsonCommand = command;
    }

    const serializedCommand = JSON.stringify(jsonCommand);
    // Remove the extra "CMD" prefix
    const fullCommand = `${serializedCommand}\n`;

    console.log(
      LOG_STYLES.formatTime(),
      LOG_STYLES.OUTGOING.COMMAND,
      LOG_STYLES.OUTGOING.PREFIX,
      LOG_STYLES.OUTGOING.SERIAL(fullCommand.trim())
    );

    this.port!.write(fullCommand);
  }

  sendSettings(settings: SlaveSettings): void {
    this.checkConnection();
    console.log(
      chalk.yellow("⟸ Sending settings:"),
      chalk.cyan(JSON.stringify(settings))
    );
    this.port!.write(`SETTINGS ${JSON.stringify(settings)}\n`);
  }

  onStateUpdate(callback: (state: SlaveState) => void): void {
    this.stateUpdateCallback = callback;
    this.checkConnection();
  }

  onMessage(callback: (message: string) => void): void {
    this.messageCallback = callback;
    this.checkConnection();
  }

  onDebugMessage(callback: (message: string) => void): void {
    this.debugMessageCallback = callback;
    this.checkConnection();
  }

  getPort(): SerialPort | null {
    return this.port;
  }

  private parseStateMessage(data: Buffer): Partial<SlaveState> {
    const state: Partial<SlaveState> = {};
    const message = data.toString().slice(1); // Remove the 'S' prefix

    // Parse key-value pairs
    const pairs = message.trim().split(" ");
    pairs.forEach((pair) => {
      const [key, value] = pair.split("=");
      if (!key || !value) return;

      switch (key) {
        case "status":
          state.status = value as State;
          break;

        case "pos":
          const [x, y] = value.split(",").map(Number);
          state.position = { x, y };
          break;

        case "sensors":
          const [xEndstop, yEndstop, armExtended, suctionEnabled] = value
            .split(",")
            .map((v) => v === "1");
          state.sensors = {
            xEndstop,
            yEndstop,
            armExtended,
            suctionEnabled,
          };
          break;
      }
    });

    return state;
  }

  private handleIncomingData(data: Buffer) {
    const msgType = data[0];

    try {
      switch (msgType) {
        case 0x50: // Position update ('P')
          if (data.length >= 9) {
            const x = data.readFloatLE(1);
            const y = data.readFloatLE(5);

            if (Number.isFinite(x) && Number.isFinite(y)) {
              console.log(
                LOG_STYLES.formatTime(),
                LOG_STYLES.INCOMING.POSITION,
                LOG_STYLES.formatPosition(x, y)
              );
              this.emitStateUpdate({ position: { x, y } });
            }
          }
          break;

        case 0x53: // State update ('S')
          const stateMsg = data.slice(1).toString().trim();
          console.log(
            LOG_STYLES.formatTime(),
            LOG_STYLES.INCOMING.STATE,
            chalk.green(stateMsg)
          );
          const state = this.parseStateMessage(data);
          this.emitStateUpdate(state);
          break;

        case 0x44: // Debug message ('D')
          const debugMsg = data.slice(1).toString().trim();
          console.log(
            LOG_STYLES.formatTime(),
            LOG_STYLES.INCOMING.DEBUG,
            chalk.magenta(debugMsg)
          );
          this.emitDebugMessage(debugMsg);
          break;

        case 0x49: // Info message ('I')
          const infoMsg = data.slice(1).toString().trim();
          console.log(
            LOG_STYLES.formatTime(),
            LOG_STYLES.INCOMING.INFO,
            chalk.blue(infoMsg)
          );
          this.emitMessage(infoMsg);
          break;

        case 0x45: // Error message ('E')
          const errorMsg = data.slice(1).toString().trim();
          console.log(
            LOG_STYLES.formatTime(),
            LOG_STYLES.INCOMING.ERROR,
            chalk.red(errorMsg)
          );
          break;

        default:
          if (process.env.DEBUG) {
            console.log(
              LOG_STYLES.formatTime(),
              LOG_STYLES.INCOMING.RAW,
              LOG_STYLES.formatHex(data)
            );
          }
      }
    } catch (error) {
      if (process.env.DEBUG) {
        console.log(
          LOG_STYLES.formatTime(),
          LOG_STYLES.INCOMING.ERROR,
          chalk.red("Failed to parse message:"),
          LOG_STYLES.formatHex(data)
        );
      }
    }
  }

  private emitStateUpdate(state: Partial<SlaveState>) {
    this.stateUpdateCallback?.(state as SlaveState);
  }

  private emitDebugMessage(message: string) {
    this.debugMessageCallback?.(message);
  }

  private emitMessage(message: string) {
    this.messageCallback?.(message);
  }

  private emitError(message: string, code: number) {
    this.errorCallback?.(message, code);
  }

  private emitResponse(response: { success: boolean; message: string }) {
    this.responseCallback?.(response);
  }

  sendMove(x: number, y: number): void {
    const buffer = Buffer.alloc(9); // 1 byte command + 2 floats
    buffer[0] = 0x4d; // 'M'
    buffer.writeFloatLE(x, 1);
    buffer.writeFloatLE(y, 5);
    this.port?.write(buffer);
  }

  private handlePositionUpdate(data: Buffer) {
    try {
      const x = data.readFloatLE(1);
      const y = data.readFloatLE(5);

      if (Number.isFinite(x) && Number.isFinite(y)) {
        console.log(
          LOG_STYLES.formatTime(),
          LOG_STYLES.INCOMING.POSITION,
          LOG_STYLES.formatPosition(x, y)
        );
        this.emitStateUpdate({ position: { x, y } });
      }
    } catch (error) {
      if (process.env.DEBUG) {
        console.log(
          LOG_STYLES.formatTime(),
          LOG_STYLES.INCOMING.ERROR,
          chalk.red("Failed to parse position:"),
          LOG_STYLES.formatHex(data)
        );
      }
    }
  }
}
