#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>

#include "../types/Types.h"

class Protocol {
 public:
  // Machine state and command response methods
  static void sendState(const MachineState& state);
  static void sendResponse(const Response& response);

  // Logging methods
  static void debug(const String& message);
  static void info(const String& message);
  static void error(const String& message, int code = 0);

 private:
  static void sendJson(const String& type, const JsonDocument& doc);
};