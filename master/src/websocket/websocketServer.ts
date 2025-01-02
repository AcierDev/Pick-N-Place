import { WebSocket, WebSocketServer as WSServer } from "ws";
import { Command, SlaveSettings, SlaveState } from "../typings/types";
import chalk from "chalk";
import { CommandHandler } from "./commandHandler";
import { SerialCommunication } from "../serialCommunication";

export class WebSocketServer {
  private wss: WSServer;
  private connectedClients: number = 0;
  private commandHandler: CommandHandler;

  constructor(port: number, serialPort: SerialCommunication) {
    this.wss = new WSServer({ port });
    this.commandHandler = new CommandHandler(serialPort);

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

  broadcastState(state: SlaveState): void {
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
}
