import { WebSocket } from "ws";
import { Command, CommandMessage } from "../typings/types";
import { SerialCommunication } from "../serialCommunication";
import chalk from "chalk";

export class CommandHandler {
  private serialPort: SerialCommunication;

  constructor(serialPort: SerialCommunication) {
    this.serialPort = serialPort;
  }

  handleCommand(
    ws: WebSocket,
    cmd: Command,
    sendError: (ws: WebSocket, message: string) => void
  ): void {
    try {
      let commandMessage: CommandMessage;

      if (typeof cmd === "string") {
        // Convert string commands to command objects
        commandMessage = {
          type: cmd,
          params: {},
        };
      } else {
        commandMessage = cmd;
      }

      this.serialPort.sendCommand(commandMessage);
    } catch (error) {
      sendError(ws, "Error processing command");
    }
  }

  private convertToSerialCommand(cmd: CommandMessage): CommandMessage {
    // All commands are now JSON objects
    return {
      type: cmd.type,
      params: cmd.params || {},
    };
  }
}
