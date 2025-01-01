import { WebSocket, WebSocketServer as WSServer } from "ws";
import { Command, SlaveSettings, SlaveState } from "./typings/types";
import chalk from "chalk";

export class WebSocketServer {
  private wss: WSServer;

  constructor(port: number) {
    this.wss = new WSServer({ port });
  }

  broadcastState(state: SlaveState): void {
    this.wss.clients.forEach((client) => {
      if (client.readyState === WebSocket.OPEN) {
        client.send(JSON.stringify({ type: "state", data: state }));
      }
    });
  }

  broadcastSettings(settings: SlaveSettings): void {
    this.wss.clients.forEach((client) => {
      if (client.readyState === WebSocket.OPEN) {
        client.send(JSON.stringify({ type: "settings", data: settings }));
      }
    });
  }

  onCommand(callback: (command: Command) => void): void {
    this.wss.on("connection", (ws) => {
      console.log(chalk.green("✓ New WebSocket client connected"));

      ws.on("message", (message: string) => {
        try {
          const data = JSON.parse(message.toString());
          if (data.type === "command") {
            const cmd = data.data;
            console.log(
              chalk.blue("⟸ WebSocket command:"),
              chalk.cyan(JSON.stringify(cmd))
            );

            // Convert high-level commands to serial protocol
            let serialCommand: string;

            switch (cmd.type) {
              case "start":
                if (
                  cmd.params &&
                  typeof cmd.params.rows === "number" &&
                  typeof cmd.params.cols === "number" &&
                  typeof cmd.params.startX === "number" &&
                  typeof cmd.params.startY === "number" &&
                  typeof cmd.params.pickupX === "number" &&
                  typeof cmd.params.pickupY === "number"
                ) {
                  // Use provided grid dimensions or default to 20x20
                  const gridWidth =
                    typeof cmd.params.gridWidth === "number"
                      ? cmd.params.gridWidth
                      : 20.0;
                  const gridLength =
                    typeof cmd.params.gridLength === "number"
                      ? cmd.params.gridLength
                      : 20.0;

                  serialCommand = `start ${cmd.params.rows} ${cmd.params.cols} ${cmd.params.startX} ${cmd.params.startY} ${gridWidth} ${gridLength} ${cmd.params.pickupX} ${cmd.params.pickupY}`;
                } else {
                  console.error(
                    "Invalid start parameters - requires rows, cols, startX, startY, pickupX, and pickupY (gridWidth and gridLength optional)"
                  );
                  return;
                }
                break;

              case "stop":
                serialCommand = "stop";
                break;

              case "home":
                serialCommand = "home";
                break;

              case "goto":
                if (
                  cmd.params &&
                  typeof cmd.params.x === "number" &&
                  typeof cmd.params.y === "number"
                ) {
                  serialCommand = `goto ${cmd.params.x} ${cmd.params.y}`;
                } else {
                  console.error("Invalid goto parameters");
                  return;
                }
                break;

              case "extend":
                serialCommand = "extend";
                break;

              case "retract":
                serialCommand = "retract";
                break;

              case "suction":
                if (cmd.params && typeof cmd.params.enabled === "boolean") {
                  serialCommand = cmd.params.enabled
                    ? "suction_on"
                    : "suction_off";
                } else {
                  console.error("Invalid suction parameters");
                  return;
                }
                break;

              case "setSpeed":
                if (cmd.params && typeof cmd.params.speed === "number") {
                  serialCommand = `speed ${cmd.params.speed}`;
                } else {
                  console.error("Invalid speed parameter");
                  return;
                }
                break;

              case "setAccel":
                if (cmd.params && typeof cmd.params.accel === "number") {
                  serialCommand = `accel ${cmd.params.accel}`;
                } else {
                  console.error("Invalid acceleration parameter");
                  return;
                }
                break;

              default:
                console.error("Unknown command type:", cmd.type);
                return;
            }

            console.log(
              chalk.yellow("⟹ Serial command:"),
              chalk.cyan(serialCommand)
            );
            callback(serialCommand);
          }
        } catch (error) {
          console.error(chalk.red("✗ Error parsing command message:"), error);
        }
      });

      ws.on("close", () => {
        console.log(chalk.yellow("⚠ WebSocket client disconnected"));
      });
    });
  }

  onSettingsUpdate(callback: (settings: Partial<SlaveSettings>) => void): void {
    this.wss.on("connection", (ws) => {
      ws.on("message", (message: string) => {
        try {
          const data = JSON.parse(message.toString());
          if (data.type === "updateSettings") {
            callback(data.data);
          }
        } catch (error) {
          console.error("Error parsing settings update message:", error);
        }
      });
    });
  }
}
