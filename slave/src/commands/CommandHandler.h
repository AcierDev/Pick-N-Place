#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>

#include <functional>
#include <map>

#include "../communication/Protocol.h"
#include "../types/Types.h"

class CommandHandler {
 private:
  std::map<String, std::function<void(const JsonVariant&)>> commandHandlers;

 public:
  void registerCommand(const String& command,
                       std::function<void(const JsonVariant&)> handler) {
    commandHandlers[command] = handler;
    Protocol::debug("Registered command: " + command);
  }

  void handleCommand(const String& command, const JsonVariant& params) {
    auto it = commandHandlers.find(command);
    if (it != commandHandlers.end()) {
      it->second(params);
    } else {
      Protocol::error("Unknown command: " + command);
      String registeredCommands = "Registered commands:";
      for (const auto& pair : commandHandlers) {
        registeredCommands += " " + pair.first;
      }
      Protocol::debug(registeredCommands);
    }
  }
};