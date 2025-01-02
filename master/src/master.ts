export { Master };
import { SerialCommunication } from "./serialCommunication";
import { SettingsManager } from "./settings/settings";
import {
  Command,
  SlaveSettings,
  SlaveState,
  MachineStatus,
  State,
} from "./typings/types";
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
  private wss: WebSocketServer;
  private settingsManager: SettingsManager;
  private platformIO: PlatformIOManager;
  private currentState: SlaveState;

  constructor() {
    this.serial = new SerialCommunication();
    this.wss = new WebSocketServer(8080, this.serial, this);
    this.settingsManager = new SettingsManager("./settings.json");
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

    await this.settingsManager.loadSettings();
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
    this.wss.broadcastSlaveState(this.currentState);
  }

  private setupSerialListeners(): void {
    this.serial.onStateUpdate((state: SlaveState) => {
      this.currentState = state;
      this.wss.broadcastSlaveState(state);
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
    this.wss.onCommand((command: Command) => {
      this.serial.sendCommand(command);
    });

    this.wss.onSettingsUpdate((newSettings: Partial<SlaveSettings>) => {
      this.settingsManager.updateSettings(newSettings);
      const updatedSettings = this.settingsManager.getSettings();
      this.serial.sendSettings(updatedSettings);
      this.wss.broadcastSettings(updatedSettings);
    });
  }

  private sendInitialSettings(): void {
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
    this.wss.inferStateFromCommand(command);
  }

  updateSettings(newSettings: Partial<SlaveSettings>): void {
    this.settingsManager.updateSettings(newSettings);
    const updatedSettings = this.settingsManager.getSettings();
    this.serial.sendSettings(updatedSettings);
    this.wss.broadcastSettings(updatedSettings);
  }
}

const master = new Master();
master.init().catch((error) => {
  console.error("Error initializing master:", error);
  process.exit(1);
});
