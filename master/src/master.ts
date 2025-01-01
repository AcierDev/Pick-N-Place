export { Master };
import { SerialCommunication } from "./serialCommunication";
import { SettingsManager } from "./settings/settings";
import { Command, SlaveSettings, SlaveState } from "./typings/types";
import { WebSocketServer } from "./websocketServer";
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
    this.wss = new WebSocketServer(8080);
    this.settingsManager = new SettingsManager("./settings.json");
    const slavePath = path.join(__dirname, "../../slave");
    this.platformIO = new PlatformIOManager(slavePath);
    this.currentState = {
      status: "IDLE",
      sensors: {},
    };
  }

  async init(): Promise<void> {
    console.log("Initializing master...");
    if (!(await this.platformIO.verifyPlatformIO())) {
      throw new Error("PlatformIO CLI is required but not found");
    }

    // const uploaded = await this.platformIO.uploadCode();
    // if (!uploaded) {
    //   throw new Error("Failed to upload slave code");
    // }

    await new Promise((resolve) => setTimeout(resolve, 2000));

    await this.settingsManager.loadSettings();
    const connected = await this.serial.connect();
    if (!connected) {
      throw new Error("Failed to connect to microcontroller");
    }
    this.setupSerialListeners();
    this.setupWebSocketListeners();
    this.sendInitialSettings();
    this.sendInitialState();

    const cli = new CLIHandler(this);
    cli.start();
  }
  sendInitialState() {
    this.wss.broadcastState(this.currentState);
  }

  private setupSerialListeners(): void {
    this.serial.onStateUpdate((state: SlaveState) => {
      this.currentState = state;
      this.wss.broadcastState(state);
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
      console.log(chalk.blue("⟸ Debug message:"), chalk.cyan(message));
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

  sendCommand(command: string): void {
    this.serial.sendCommand(command);
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
