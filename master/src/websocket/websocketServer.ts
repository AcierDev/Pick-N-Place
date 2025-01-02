import { WebSocket, WebSocketServer as WSServer } from "ws";
import {
  Command,
  SlaveSettings,
  MachineStatus,
  State,
  SlaveState,
} from "../typings/types";
import chalk from "chalk";
import { CommandHandler } from "./commandHandler";
import { SerialCommunication } from "../serialCommunication";

export class WebSocketServer {
  private wss: WSServer;
  private connectedClients: number = 0;
  private commandHandler: CommandHandler;
  private machineStatus: MachineStatus;

  constructor(port: number, serialPort: SerialCommunication) {
    this.wss = new WSServer({ port });
    this.commandHandler = new CommandHandler(serialPort);

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
      ws.on("message", (message: string) => {
        try {
          const data = JSON.parse(message.toString());
          if (data.type === "command") {
            console.log(
              chalk.blue("⟸ WebSocket command:"),
              chalk.cyan(JSON.stringify(data.data))
            );
            this.commandHandler.handleCommand(
              ws,
              data.data,
              this.sendErrorToClient
            );
          }
        } catch (error) {
          console.error(chalk.red("✗ Error parsing command message:"), error);
          this.broadcastError("Invalid command format");
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
            isHomed: this.machineStatus.position.isHomed,
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
