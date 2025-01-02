import { WebSocket, WebSocketServer as WSServer } from "ws";
import {
  Command,
  SlaveSettings,
  MachineStatus,
  SlaveState,
} from "../typings/types";
import chalk from "chalk";
import { CommandHandler } from "./commandHandler";
import { SerialCommunication } from "../serialCommunication";
import { Master } from "../master";

export class WebSocketServer {
  private wss: WSServer;
  private connectedClients: number = 0;
  private commandHandler: CommandHandler;
  private machineStatus: MachineStatus;
  private master: Master;

  constructor(port: number, serialPort: SerialCommunication, master: Master) {
    this.wss = new WSServer({ port });
    this.commandHandler = new CommandHandler(serialPort);
    this.master = master;

    // Initialize machine status
    this.machineStatus = {
      state: "IDLE",
      position: {
        x: 0,
        y: 0,
        isHomed: false,
      },
      sensors: {
        xEndstop: false,
        yEndstop: false,
        armExtended: false,
        suctionEnabled: false,
      },
    };

    console.log(chalk.cyan(`🌐 WebSocket server started on port ${port}`));

    this.wss.on("connection", (ws, req) => {
      this.handleNewConnection(ws, req);
    });
  }

  private handleNewConnection(ws: WebSocket, req: any): void {
    this.connectedClients++;
    const clientIp = req.socket.remoteAddress;
    console.log(chalk.green(`📱 New client connected from ${clientIp}`));
    console.log(
      chalk.cyan(`👥 Total connected clients: ${this.connectedClients}`)
    );

    // Send immediate status update to the new client
    ws.send(
      JSON.stringify({
        type: "state",
        data: this.machineStatus,
      })
    );

    ws.on("close", () => {
      this.connectedClients--;
      console.log(chalk.yellow(`📴 Client disconnected from ${clientIp}`));
      console.log(
        chalk.cyan(`👥 Total connected clients: ${this.connectedClients}`)
      );
    });
  }

  broadcastState(state: MachineStatus): void {
    console
      .log
      // chalk.blue("⟸ Broadcasting state:"),
      // chalk.cyan(JSON.stringify(state))
      ();
    this.broadcast("state", state);
  }

  broadcastSettings(settings: SlaveSettings): void {
    this.broadcast("settings", settings);
  }

  broadcastError(message: string): void {
    this.broadcast("error", { message });
  }

  private broadcast(type: string, data: any): void {
    this.wss.clients.forEach((client) => {
      if (client.readyState === WebSocket.OPEN) {
        client.send(JSON.stringify({ type, data }));
      }
    });
  }

  private sendErrorToClient(ws: WebSocket, message: string): void {
    ws.send(JSON.stringify({ type: "error", data: { message } }));
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
            let serialCommand: string = "";

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
                  // Store values in settings
                  this.master.updateSettings({
                    rows: cmd.params.rows,
                    columns: cmd.params.cols,
                    boxX: cmd.params.startX,
                    boxY: cmd.params.startY,
                    boxWidth: cmd.params.gridWidth || 20.0,
                    boxLength: cmd.params.gridLength || 20.0,
                  });

                  // Use provided grid dimensions or default to 20x20
                  const gridWidth = cmd.params.gridWidth || 20.0;
                  const gridLength = cmd.params.gridLength || 20.0;

                  serialCommand = `start ${cmd.params.rows} ${cmd.params.cols} ${cmd.params.startX} ${cmd.params.startY} ${gridWidth} ${gridLength} ${cmd.params.pickupX} ${cmd.params.pickupY}`;
                } else {
                  console.error(
                    "Invalid start parameters - requires rows, cols, startX, startY, pickupX, and pickupY (gridWidth and gridLength optional)"
                  );
                  return;
                }
                break;

              case "setSpeed":
                if (cmd.params && typeof cmd.params.speed === "number") {
                  // Store speed in settings
                  this.master.updateSettings({ speed: cmd.params.speed });
                  // Forward command directly without modification
                  callback(cmd);
                } else {
                  console.error("Invalid speed parameter");
                  return;
                }
                break;

              case "setAccel":
                if (cmd.params && typeof cmd.params.accel === "number") {
                  // Store acceleration in settings
                  this.master.updateSettings({
                    acceleration: cmd.params.accel,
                  });
                  // Forward command directly without modification
                  callback(cmd);
                } else {
                  console.error("Invalid acceleration parameter");
                  return;
                }
                break;

              // ... rest of the cases ...
              default:
                serialCommand =
                  cmd.type +
                  (cmd.params ? " " + JSON.stringify(cmd.params) : "");
            }

            if (serialCommand) {
              console.log(
                chalk.yellow("⟹ Serial command:"),
                chalk.cyan(serialCommand)
              );
              callback(serialCommand);
            }
          }
        } catch (error) {
          console.error(chalk.red("✗ Error parsing command message:"), error);
        }
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
          this.broadcastError("Invalid settings format");
        }
      });
    });
  }

  getConnectedClientsCount(): number {
    return this.connectedClients;
  }

  updateState(partialState: Partial<MachineStatus>) {
    this.machineStatus = {
      ...this.machineStatus,
      ...partialState,
      sensors: {
        ...this.machineStatus.sensors,
        ...(partialState.sensors || {}),
      },
      motion: {
        ...this.machineStatus.motion,
        ...(partialState.motion || {}),
      },
    };
    this.broadcastState(this.machineStatus);
  }

  inferStateFromCommand(command: Command) {
    // Handle both string and object commands
    if (typeof command === "string") {
      switch (command) {
        case "home":
          this.updateState({ state: "HOMING_X" });
          break;
        case "extend":
          this.updateState({
            sensors: {
              ...this.machineStatus.sensors,
              armExtended: true,
            },
          });
          break;
        case "retract":
          this.updateState({
            sensors: {
              ...this.machineStatus.sensors,
              armExtended: false,
            },
          });
          break;
        case "suction_on":
          this.updateState({
            sensors: {
              ...this.machineStatus.sensors,
              suctionEnabled: true,
            },
          });
          break;
        case "suction_off":
          this.updateState({
            sensors: {
              ...this.machineStatus.sensors,
              suctionEnabled: false,
            },
          });
          break;
      }
    } else {
      // Handle object commands
      switch (command.type) {
        case "speed":
          // Update machine status with new speed
          this.updateState({
            motion: {
              ...this.machineStatus.motion,
              speed: command.params?.value,
            },
          });
          break;

        case "accel":
          // Update machine status with new acceleration
          this.updateState({
            motion: {
              ...this.machineStatus.motion,
              acceleration: command.params?.value,
            },
          });
          break;

        case "suction":
          if (command.params?.state !== undefined) {
            this.updateState({
              sensors: {
                ...this.machineStatus.sensors,
                suctionEnabled:
                  typeof command.params.state === "boolean"
                    ? command.params.state
                    : command.params.state === "on",
              },
            });
          }
          break;
      }
    }
  }

  broadcastSlaveState(state: SlaveState): void {
    const machineStatus: MachineStatus = {
      ...this.machineStatus,
      state: state.status || "IDLE",
      position: state.position
        ? {
            x: Math.round(state.position.x * 10) / 10,
            y: Math.round(state.position.y * 10) / 10,
            isHomed: state.isHomed ?? this.machineStatus.position.isHomed,
          }
        : this.machineStatus.position,
      sensors: {
        ...this.machineStatus.sensors,
        xEndstop:
          state.sensors?.xEndstop ?? this.machineStatus.sensors.xEndstop,
        yEndstop:
          state.sensors?.yEndstop ?? this.machineStatus.sensors.yEndstop,
        armExtended:
          state.sensors?.armExtended ?? this.machineStatus.sensors.armExtended,
        suctionEnabled:
          state.sensors?.suctionEnabled ??
          this.machineStatus.sensors.suctionEnabled,
      },
    };

    // Update internal state
    this.machineStatus = machineStatus;

    // Broadcast the update
    this.broadcastState(machineStatus);

    // Debug log the state change
    console.log(
      chalk.blue("⟹"),
      chalk.cyan("Machine status updated:"),
      chalk.gray(JSON.stringify(machineStatus.sensors))
    );
  }
}
