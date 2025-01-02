#pragma once
#include <Arduino.h>

enum class ErrorCode {
  MOTION_ERROR,
  COMMUNICATION_ERROR,
  TIMEOUT_ERROR,
  INVALID_STATE,
  HARDWARE_ERROR
};

class ErrorHandler {
 public:
  static void handleError(ErrorCode code, const String& message) {
    // Log error
    // Take appropriate action
    // Notify master
  }
};