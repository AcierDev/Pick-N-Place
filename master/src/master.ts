export { Master };
import { SerialCommunication } from "./serialCommunication";
import { SettingsManager } from "./settings/settings";
import { Command, SlaveSettings, SlaveState } from "./typings/types";
import { WebSocketServer } from "./websocket/websocketServer";
import { PlatformIOManager } from "./util/platformioManager";
import path from "path";
import { CLIHandler } from "./cli/cliHandler";
import { fileURLToPath } from "url";
import { dirname } from "path";
import chalk from "chalk";

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);

class Master {
  private serial: SerialCommunication;
  private wss: WebSocketServer | null;
  private settingsManager: SettingsManager;
  private platformIO: PlatformIOManager;
  private currentState: SlaveState;

  constructor() {
    this.serial = new SerialCommunication();
    this.settingsManager = new SettingsManager("./settings.json");
    this.wss = null;
    const slavePath = path.join(__dirname, "../../slave");
    this.platformIO = new PlatformIOManager(slavePath);

    // Initialize with default state
    this.currentState = {
      status: "IDLE",
      sensors: {
        xEndstop: false,
        yEndstop: false,
        armExtended: false,
        suctionEnabled: false,
      },
    };
  }

  async init(): Promise<void> {
    console.log("Initializing master...");
    if (!(await this.platformIO.verifyPlatformIO())) {
      throw new Error("PlatformIO CLI is required but not found");
    }

    // Load settings first
    await this.settingsManager.loadSettings();

    // Create WebSocket server after settings are loaded
    this.wss = new WebSocketServer(8080, this.serial, this);

    const connected = await this.serial.connect();
    if (!connected) {
      throw new Error("Failed to connect to microcontroller");
    }

    // Apply initial settings
    const settings = this.settingsManager.getSettings();
    console.log(
      chalk.blue("📊 Loaded settings:"),
      chalk.cyan(JSON.stringify(settings, null, 2))
    );

    // Update machine status with settings
    console.log(chalk.blue("⚙️ Initializing machine status with settings..."));
    this.wss.updateState({
      motion: {
        speed: settings.speed,
        acceleration: settings.acceleration,
      },
      pattern: {
        current: 0,
        total: settings.rows * settings.columns,
      },
    });

    // Send settings to microcontroller
    console.log(chalk.blue("🔄 Applying settings to microcontroller..."));
    this.serial.sendCommand({
      type: "setSpeed",
      params: { speed: settings.speed },
    });
    this.serial.sendCommand({
      type: "setAccel",
      params: { accel: settings.acceleration },
    });

    this.setupSerialListeners();
    this.setupWebSocketListeners();
    this.sendInitialSettings();
    this.sendInitialState();

    const cli = new CLIHandler(this);
    cli.start();
  }
  sendInitialState() {
    if (!this.wss) return;
    this.wss.broadcastSlaveState(this.currentState);
  }

  private setupSerialListeners(): void {
    this.serial.onStateUpdate((state: SlaveState) => {
      // console.log(
      //   chalk.blue("⟹ Master received state update:"),
      //   chalk.cyan(JSON.stringify(state))
      // );
      this.currentState = state;
      this.wss?.broadcastSlaveState(state);
    });

    const port = this.serial.getPort();
    if (port) {
      port.on("close", async () => {
        console.log("Serial port closed. Attempting to reconnect...");
        await this.attemptReconnection();
      });
    }

    this.serial.onMessage((message: string) => {
      console.log(chalk.green("⟹ Message:"), chalk.cyan(message));
    });

    this.serial.onDebugMessage((message: string) => {
      // console.log(chalk.blue("⟸ Debug message:"), chalk.cyan(message));
    });
  }

  private setupWebSocketListeners(): void {
    if (!this.wss) return;

    this.wss.onCommand((command: Command) => {
      this.serial.sendCommand(command);
    });

    this.wss.onSettingsUpdate((newSettings: Partial<SlaveSettings>) => {
      this.settingsManager.updateSettings(newSettings);
      const updatedSettings = this.settingsManager.getSettings();
      this.serial.sendSettings(updatedSettings);
      this.wss?.broadcastSettings(updatedSettings);
    });
  }

  private sendInitialSettings(): void {
    if (!this.wss) return;
    const settings = this.settingsManager.getSettings();
    this.serial.sendSettings(settings);
    this.wss.broadcastSettings(settings);
  }

  private async attemptReconnection(): Promise<void> {
    console.log("Attempting to reconnect to microcontroller...");
    let connected = false;
    while (!connected) {
      connected = await this.serial.connect();
      if (!connected) {
        console.log("Reconnection failed. Retrying in 5 seconds...");
        await new Promise((resolve) => setTimeout(resolve, 5000));
      }
    }
    console.log("Reconnected to microcontroller");
    this.setupSerialListeners();
    this.sendInitialSettings();
  }

  getCurrentState(): SlaveState {
    return this.currentState;
  }

  getSettings(): SlaveSettings {
    return this.settingsManager.getSettings();
  }

  sendCommand(command: Command): void {
    this.serial.sendCommand(command);

    // Handle speed and acceleration commands
    if (typeof command === "object") {
      if (command.type === "setSpeed" && command.params?.speed !== undefined) {
        this.wss?.updateState({
          motion: {
            ...this.currentState.motion,
            speed: command.params.speed,
          },
        });
      } else if (
        command.type === "setAccel" &&
        command.params?.accel !== undefined
      ) {
        this.wss?.updateState({
          motion: {
            ...this.currentState.motion,
            acceleration: command.params.accel,
          },
        });
      } else {
        this.wss?.inferStateFromCommand(command);
      }
    } else {
      this.wss?.inferStateFromCommand(command);
    }
  }

  updateSettings(newSettings: Partial<SlaveSettings>): void {
    this.settingsManager.updateSettings(newSettings);
    const updatedSettings = this.settingsManager.getSettings();

    // Send settings to microcontroller
    this.serial.sendSettings(updatedSettings);

    // If speed or acceleration changed, update machine status
    if (
      newSettings.speed !== undefined ||
      newSettings.acceleration !== undefined
    ) {
      this.wss?.updateState({
        motion: {
          speed: updatedSettings.speed,
          acceleration: updatedSettings.acceleration,
        },
      });
    } else {
      // For other settings changes, broadcast settings update
      this.wss?.broadcastSettings(updatedSettings);
    }
  }
}

const master = new Master();
master.init().catch((error) => {
  console.error("Error initializing master:", error);
  process.exit(1);
});
