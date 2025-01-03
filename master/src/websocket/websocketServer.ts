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

    // Get initial settings from master
    const settings = master.getSettings();

    // Initialize machine status with settings
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
      motion: {
        speed: settings.speed,
        acceleration: settings.acceleration,
      },
      pattern: {
        current: 0,
        total: settings.rows * settings.columns,
        boxX: settings.boxX,
        boxY: settings.boxY,
        boxWidth: settings.boxWidth,
        boxLength: settings.boxLength,
        pickupX: settings.pickupX,
        pickupY: settings.pickupY,
      },
    };

    console.log(chalk.cyan(`🌐 WebSocket server started on port ${port}`));
    console.log(
      chalk.blue("📊 Initial machine status:"),
      chalk.cyan(JSON.stringify(this.machineStatus, null, 2))
    );

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
    const message = JSON.stringify({ type, data });
    // console.log(chalk.blue("⟹ Broadcasting message:"), chalk.cyan(message)); // Add debug log

    let clientCount = 0;
    this.wss.clients.forEach((client) => {
      if (client.readyState === WebSocket.OPEN) {
        client.send(message);
        clientCount++;
      }
    });

    console.log(chalk.blue(`✓ Broadcast to ${clientCount} clients`)); // Add debug log
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
              case "setSpeed":
                if (cmd.params && typeof cmd.params.speed === "number") {
                  this.master.updateSettings({ speed: cmd.params.speed });
                  callback(cmd);
                } else {
                  console.error("Invalid speed parameter");
                }
                break;

              case "setAccel":
                if (cmd.params && typeof cmd.params.accel === "number") {
                  this.master.updateSettings({
                    acceleration: cmd.params.accel,
                  });
                  callback(cmd);
                } else {
                  console.error("Invalid acceleration parameter");
                }
                break;

              case "setPickupLocation":
                if (
                  cmd.params &&
                  typeof cmd.params.x === "number" &&
                  typeof cmd.params.y === "number"
                ) {
                  this.master.updateSettings({
                    pickupX: cmd.params.x,
                    pickupY: cmd.params.y,
                  });
                  // Update machine status
                  this.updateState({
                    pattern: {
                      current: this.machineStatus.pattern?.current ?? 0,
                      total: this.machineStatus.pattern?.total ?? 0,
                      ...this.machineStatus.pattern,
                      pickupX: cmd.params.x,
                      pickupY: cmd.params.y,
                    },
                  });
                  callback(cmd);
                } else {
                  console.error("Invalid pickup location parameters");
                }
                break;

              case "setBoxCorner":
                if (
                  cmd.params &&
                  typeof cmd.params.x === "number" &&
                  typeof cmd.params.y === "number"
                ) {
                  this.master.updateSettings({
                    boxX: cmd.params.x,
                    boxY: cmd.params.y,
                  });
                  // Update machine status
                  this.updateState({
                    pattern: {
                      current: this.machineStatus.pattern?.current ?? 0,
                      total: this.machineStatus.pattern?.total ?? 0,
                      ...this.machineStatus.pattern,
                      boxX: cmd.params.x,
                      boxY: cmd.params.y,
                    },
                  });
                  callback(cmd);
                } else {
                  console.error("Invalid box corner parameters");
                }
                break;

              case "goto":
                if (
                  cmd.params &&
                  typeof cmd.params.x === "number" &&
                  typeof cmd.params.y === "number"
                ) {
                  callback(cmd);
                } else {
                  console.error("Invalid goto parameters");
                }
                break;

              case "home":
              case "stop":
              case "extend":
              case "retract":
                // Simple commands with no parameters
                callback(cmd);
                break;

              case "suction":
                if (
                  (cmd.params && typeof cmd.params.state === "boolean") ||
                  ["on", "off"].includes(cmd.params?.state as string)
                ) {
                  callback(cmd);
                } else {
                  console.error("Invalid suction state parameter");
                }
                break;

              case "start":
                if (
                  cmd.params &&
                  typeof cmd.params.rows === "number" &&
                  typeof cmd.params.cols === "number" &&
                  typeof cmd.params.startX === "number" &&
                  typeof cmd.params.startY === "number"
                ) {
                  callback(cmd);
                } else {
                  console.error("Invalid start parameters");
                }
                break;

              case "manual_move":
                if (
                  cmd.params &&
                  typeof cmd.params.direction === "string" &&
                  typeof cmd.params.state === "string" &&
                  (!cmd.params.speed || typeof cmd.params.speed === "number") &&
                  (!cmd.params.acceleration ||
                    typeof cmd.params.acceleration === "number")
                ) {
                  callback(cmd);
                } else {
                  console.error("Invalid manual move parameters");
                }
                break;

              case "setBoxDimensions":
                if (
                  cmd.params &&
                  typeof cmd.params.length === "number" &&
                  typeof cmd.params.width === "number"
                ) {
                  this.master.updateSettings({
                    boxLength: cmd.params.length,
                    boxWidth: cmd.params.width,
                  });
                  // Update machine status
                  this.updateState({
                    pattern: {
                      current: this.machineStatus.pattern?.current ?? 0,
                      total: this.machineStatus.pattern?.total ?? 0,
                      ...this.machineStatus.pattern,
                      boxLength: cmd.params.length,
                      boxWidth: cmd.params.width,
                    },
                  });
                  callback(cmd);
                } else {
                  console.error("Invalid box dimensions parameters");
                }
                break;

              default:
                console.warn(`Unhandled command type: ${cmd.type}`);
                callback(cmd);
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
      pattern: {
        current: this.machineStatus.pattern?.current ?? 0,
        total: this.machineStatus.pattern?.total ?? 0,
        ...this.machineStatus.pattern,
        ...(partialState.pattern || {}),
      },
    };
    this.broadcastState(this.machineStatus);
  }

  inferStateFromCommand(command: Command) {
    // Handle both string and object commands
    if (typeof command === "string") {
      switch (command) {
        case "home":
          this.updateState({ state: "HOME_REQUESTED" });
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

        case "manual_move":
          if (command.params?.state === "START") {
            this.updateState({ state: "MANUAL_MOVING" });
          } else if (command.params?.state === "STOP") {
            this.updateState({ state: "IDLE" });
          }
          break;
      }
    }
  }

  broadcastSlaveState(state: SlaveState): void {
    // console.log(
    //   chalk.blue("⟹ WebSocket broadcasting state:"),
    //   chalk.cyan(JSON.stringify(state))
    // ); // Add debug log

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

    // console.log(
    //   chalk.blue("⟹ MachineStatus:"),
    //   chalk.cyan(JSON.stringify(machineStatus))
    // );

    // Broadcast the update
    this.broadcast("state", machineStatus);
    console.log(chalk.green("✓ State broadcast complete")); // Add debug log
  }
}
