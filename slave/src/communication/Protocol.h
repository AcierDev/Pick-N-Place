#pragma once
#include <Arduino.h>

#include "../types/Types.h"

class Protocol {
 public:
  // Message types (single byte)
  static const uint8_t MSG_STATE = 'S';
  static const uint8_t MSG_POS = 'P';  // New position-specific message
  static const uint8_t MSG_DEBUG = 'D';
  static const uint8_t MSG_INFO = 'I';
  static const uint8_t MSG_ERROR = 'E';
  static const uint8_t MSG_RESPONSE = 'R';

  // Original methods (still needed)
  static void sendState(const MachineState& state);
  static void sendResponse(const Response& response);
  static void debug(const String& message);
  static void info(const String& message);
  static void error(const String& message, int code = 0);

  // Optimized position update method - just declare here
  static void sendPosition(float x, float y);
};