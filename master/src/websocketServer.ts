import { WebSocket, WebSocketServer as WSServer } from "ws";
import { Command, SlaveSettings, SlaveState } from "./typings/types";

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
      ws.on("message", (message: string) => {
        try {
          const data = JSON.parse(message.toString());
          if (data.type === "command") {
            callback(data.data);
          }
        } catch (error) {
          console.error("Error parsing command message:", error);
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
        }
      });
    });
  }
}
