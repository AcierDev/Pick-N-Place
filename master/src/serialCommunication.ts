import { SerialPort } from "serialport";
import { ReadlineParser } from "@serialport/parser-readline";
import { SlaveState, SlaveSettings, Command } from "./typings/types";
import { detectMicrocontrollerPort } from "./util/portDetection";

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
      console.error("Microcontroller not found");
      return false;
    }

    try {
      console.log(`Attempting to connect to ${portPath} at 9600 baud`);

      this.port = new SerialPort({
        path: portPath,
        baudRate: 9600,
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

      // Debug only parsed messages
      this.parser.on("data", (line) => {
        // console.log("Received:", line);
      });

      console.log(
        `Successfully connected to microcontroller on port ${portPath}`
      );
      return true;
    } catch (error) {
      console.error("Error connecting to microcontroller:", error);
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
    this.port!.write(`${command}\n`);
  }

  sendSettings(settings: SlaveSettings): void {
    this.checkConnection();
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

  getPort(): SerialPort | null {
    return this.port;
  }
}
