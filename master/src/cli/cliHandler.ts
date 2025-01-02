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

      case "home":
        this.master.sendCommand("home");
        break;

      case "start":
        this.master.sendCommand("start");
        break;

      case "move":
        if (args.length === 2) {
          const x = parseFloat(args[0]);
          const y = parseFloat(args[1]);
          if (!isNaN(x) && !isNaN(y)) {
            this.master.sendCommand(`goto ${x} ${y}`);
          } else {
            console.log("Usage: move <x> <y> (in inches)");
          }
        } else {
          console.log("Usage: move <x> <y> (in inches)");
        }
        break;

      case "speed":
        if (args.length === 1) {
          const speed = parseFloat(args[0]);
          if (!isNaN(speed) && speed > 0 && speed <= 100) {
            this.master.sendCommand({
              type: "setSpeed",
              params: { speed },
            });
          } else {
            console.log("Usage: speed <value> (1-100 inches/sec)");
          }
        } else {
          console.log("Usage: speed <value> (1-100 inches/sec)");
        }
        break;

      case "accel":
        if (args.length === 1) {
          const accel = parseFloat(args[0]);
          if (!isNaN(accel) && accel > 0 && accel <= 100) {
            this.master.sendCommand({
              type: "setAccel",
              params: { accel },
            });
          } else {
            console.log("Usage: accel <value> (1-100 inches/sec²)");
          }
        } else {
          console.log("Usage: accel <value> (1-100 inches/sec²)");
        }
        break;

      case "pattern":
        if (args.length === 6) {
          const rows = parseInt(args[0]);
          const cols = parseInt(args[1]);
          const startX = parseFloat(args[2]);
          const startY = parseFloat(args[3]);
          const pickupX = parseFloat(args[4]);
          const pickupY = parseFloat(args[5]);
          if (
            !isNaN(rows) &&
            !isNaN(cols) &&
            !isNaN(startX) &&
            !isNaN(startY) &&
            !isNaN(pickupX) &&
            !isNaN(pickupY) &&
            rows > 0 &&
            cols > 0
          ) {
            const gridWidth = 20.0;
            const gridLength = 20.0;
            this.master.sendCommand(
              `start ${rows} ${cols} ${startX} ${startY} ${gridWidth} ${gridLength} ${pickupX} ${pickupY}`
            );
          } else {
            console.log(
              "Usage: pattern <rows> <cols> <startX> <startY> <pickupX> <pickupY> " +
                "(rows/cols must be positive integers, positions in inches)"
            );
          }
        } else {
          console.log(
            "Usage: pattern <rows> <cols> <startX> <startY> <pickupX> <pickupY> " +
              "(rows/cols must be positive integers, positions in inches)"
          );
        }
        break;

      case "arm":
        if (args[0] === "extend") {
          this.master.sendCommand("extend");
        } else if (args[0] === "retract") {
          this.master.sendCommand("retract");
        } else {
          console.log("Usage: arm <extend|retract>");
        }
        break;

      case "suction":
        if (args[0] === "on") {
          this.master.sendCommand("suction_on");
        } else if (args[0] === "off") {
          this.master.sendCommand("suction_off");
        } else {
          console.log("Usage: suction <on|off>");
        }
        break;

      case "pick":
        this.master.sendCommand("pick");
        break;

      case "place":
        this.master.sendCommand("place");
        break;

      case "info":
        this.master.sendCommand("?");
        break;

      case "exit":
      case "quit":
        this.rl.close();
        break;

      case "stop":
        this.master.sendCommand("stop");
        console.log("Emergency stop initiated");
        break;

      case "manual":
        if (args.length < 1) {
          console.log("Usage: manual <direction> [speed]");
          console.log("Directions: left, right, forward, backward");
          console.log("Speed: 1-100 (optional, defaults to 50)");
          break;
        }

        const direction = args[0];
        const speed = args.length > 1 ? parseFloat(args[1]) : 50.0;
        const acceleration = speed; // Use same value for acceleration

        if (!["left", "right", "forward", "backward"].includes(direction)) {
          console.log("Invalid direction. Use: left, right, forward, backward");
          break;
        }

        if (isNaN(speed) || speed < 1 || speed > 100) {
          console.log("Speed must be between 1 and 100");
          break;
        }

        // Convert direction to axis and sign
        let axis = "X";
        let sign = "+";
        switch (direction) {
          case "left":
            axis = "X";
            sign = "-";
            break;
          case "right":
            axis = "X";
            sign = "+";
            break;
          case "forward":
            axis = "Y";
            sign = "+";
            break;
          case "backward":
            axis = "Y";
            sign = "-";
            break;
        }

        this.master.sendCommand(
          `MANUAL_MOVE ${axis} ${sign} ${speed} ${acceleration}`
        );
        break;

      case "stop":
        this.master.sendCommand("MANUAL_STOP");
        break;

      default:
        // Forward any unrecognized command directly to the slave
        this.master.sendCommand(
          command + (args.length ? " " + args.join(" ") : "")
        );
    }
  }

  private showHelp(): void {
    console.log(`
Available commands:
  System Commands:
    help                    - Show this help message
    status                  - Show current slave state
    settings show          - Show current settings
    settings set <key> <value> - Update a setting
    exit, quit             - Exit the program
    info                   - Show current machine settings

  Motion Commands:
    home                   - Home both axes
    move <x> <y>          - Move to position in inches
    speed <value>         - Set speed (1-100 inches/sec)
    accel <value>         - Set acceleration (1-100 inches/sec²)

  Pattern Commands:
    pattern <rows> <cols> <startX> <startY> <pickupX> <pickupY>  
           - Generate and execute placement pattern
           - rows/cols: Number of pieces in each direction
           - startX/Y: Grid origin position in inches
           - pickupX/Y: Pickup location in inches
           - Uses default 20x20 inch grid size
    start                  - Start the current cycle

  Manual Control:
    arm <extend|retract>   - Control arm extension
    suction <on|off>       - Control vacuum suction
    pick                   - Execute pick sequence
    place                  - Execute place sequence
    manual <direction> [speed] - Start continuous movement
                              - direction: left, right, forward, backward
                              - speed: 1-100 (optional, default 50)
    stop                      - Stop manual movement
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
}
