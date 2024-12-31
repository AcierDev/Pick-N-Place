import readline from "readline";
import { Master } from "../master";
import { SlaveSettings } from "../typings/types";

export class CLIHandler {
  private rl: readline.Interface;
  private master: Master;

  constructor(master: Master) {
    this.master = master;
    this.rl = readline.createInterface({
      input: process.stdin,
      output: process.stdout,
      prompt: "master> ",
    });
  }

  start(): void {
    console.log('CLI started. Type "help" for available commands.');
    this.rl.prompt();

    this.rl.on("line", (line) => {
      this.handleCommand(line.trim());
      this.rl.prompt();
    });

    this.rl.on("close", () => {
      console.log("CLI terminated");
      process.exit(0);
    });
  }

  private handleCommand(input: string): void {
    const [command, ...args] = input.split(" ");

    switch (command.toLowerCase()) {
      case "help":
        this.showHelp();
        break;

      case "status":
        this.showStatus();
        break;

      case "settings":
        if (args[0] === "show") {
          this.showSettings();
        } else if (args[0] === "set") {
          this.updateSetting(args[1], args[2]);
        } else {
          console.log(
            'Invalid settings command. Use "settings show" or "settings set <key> <value>"'
          );
        }
        break;

      case "send":
        if (args.length > 0) {
          this.master.sendCommand(args.join(" "));
        } else {
          console.log("Usage: send <command>");
        }
        break;

      case "wave":
        this.activateWave();
        break;

      case "exit":
      case "quit":
        this.rl.close();
        break;

      default:
        console.log('Unknown command. Type "help" for available commands.');
    }
  }

  private showHelp(): void {
    console.log(`
Available commands:
  help                    - Show this help message
  status                  - Show current slave state
  settings show          - Show current settings
  settings set <key> <value> - Update a setting
  send <command>         - Send a command to the slave
  wave                   - Activate relay wave effect
  exit/quit              - Exit the program
`);
  }

  private showStatus(): void {
    const state = this.master.getCurrentState();
    console.log("Current state:", JSON.stringify(state, null, 2));
  }

  private showSettings(): void {
    const settings = this.master.getSettings();
    console.log("Current settings:", JSON.stringify(settings, null, 2));
  }

  private updateSetting(key: string, value: string): void {
    if (!key || !value) {
      console.log("Usage: settings set <key> <value>");
      return;
    }

    try {
      // Parse the value to handle numbers and booleans
      let parsedValue: any = value;
      if (value.toLowerCase() === "true") parsedValue = true;
      else if (value.toLowerCase() === "false") parsedValue = false;
      else if (!isNaN(Number(value))) parsedValue = Number(value);

      const newSettings: Partial<SlaveSettings> = {
        [key]: parsedValue,
      };

      this.master.updateSettings(newSettings);
      console.log("Setting updated successfully");
    } catch (error) {
      console.log("Error updating setting:", error);
    }
  }

  private async activateWave(): Promise<void> {
    const delay = (ms: number) =>
      new Promise((resolve) => setTimeout(resolve, ms));
    const relayCount = 4;
    let forward = true;

    try {
      while (true) {
        if (forward) {
          // Forward wave
          for (let i = 1; i <= relayCount; i++) {
            this.master.sendCommand(`turn on ${i}`);
            await delay(200);
            this.master.sendCommand(`turn off ${i}`);
          }
          forward = false;
        } else {
          // Backward wave
          for (let i = relayCount; i >= 1; i--) {
            this.master.sendCommand(`turn on ${i}`);
            await delay(200);
            this.master.sendCommand(`turn off ${i}`);
          }
          forward = true;
        }
      }
    } catch (error) {
      console.log("Wave effect interrupted:", error);
    }
  }
}
