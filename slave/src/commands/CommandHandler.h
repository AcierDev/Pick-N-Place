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
  }

  void handleCommand(const String& command, const JsonVariant& params) {
    if (auto handler = commandHandlers[command]) {
      handler(params);
    } else {
      Protocol::error("Unknown command: " + command);
    }
  }
};