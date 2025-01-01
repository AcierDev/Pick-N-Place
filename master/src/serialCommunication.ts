import { SerialPort } from "serialport";
import { ReadlineParser } from "@serialport/parser-readline";
import { SlaveState, SlaveSettings, Command } from "./typings/types";
import { detectMicrocontrollerPort } from "./util/portDetection";
import chalk from "chalk";

export class SerialCommunication {
  private port: SerialPort | null;
  private parser: ReadlineParser | null;

  constructor() {
    this.port = null;
    this.parser = null;
  }

  async connect(): Promise<boolean> {
    const portPath = await detectMicrocontrollerPort();
    if (!portPath) {
      console.error(chalk.red("✗ Microcontroller not found"));
      return false;
    }

    try {
      console.log(
        chalk.blue(`⟸ Attempting to connect to ${portPath} at 9600 baud`)
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

      // Open port
      await new Promise<void>((resolve, reject) => {
        this.port!.open((err) => {
          if (err) {
            console.error("Error opening port:", err.message);
            reject(err);
          } else {
            console.log("Port opened successfully");
            resolve();
          }
        });
      });

      this.parser = this.port.pipe(new ReadlineParser({ delimiter: "\r\n" }));

      console.log(
        chalk.green(
          `✓ Successfully connected to microcontroller on port ${portPath}`
        )
      );
      return true;
    } catch (error) {
      console.error(chalk.red("✗ Error connecting to microcontroller:"), error);
      if (this.port) {
        try {
          await new Promise<void>((resolve) => {
            this.port!.close(() => resolve());
          });
        } catch (closeError) {
          console.error("Error closing port:", closeError);
        }
      }
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
    console.log(chalk.yellow("⟸ Sending command:"), chalk.cyan(command));
    this.port!.write(`${command}\n`);
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
    this.checkConnection();
    this.parser!.on("data", (data: string) => {
      if (data.startsWith("STATE")) {
        const stateData = JSON.parse(data.slice(6));
        callback(stateData);
      }
    });
  }

  onMessage(callback: (message: string) => void): void {
    this.checkConnection();
    this.parser!.on("data", callback);
  }

  onDebugMessage(callback: (message: string) => void): void {
    this.checkConnection();
    this.parser!.on("data", (data: string) => {
      if (data.startsWith("DEBUG")) {
        callback(data.slice(6));
      }
    });
  }

  getPort(): SerialPort | null {
    return this.port;
  }
}
