#pragma once

#include <AccelStepper.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Bounce2.h>

#include "PatternGenerator.h"

// Define status enum
enum class Status {
  IDLE,
  BUSY,
  ERROR,
  HOMING_X,
  HOMING_Y,
  MOVING,
  PICKING,
  PLACING,
  RETRACTING
};

// Settings structure
struct Settings {
  double speedInches = 40.0;
  double accelerationInches = 20.0;
  double homingSpeedInches = 7.5;
  double pickDistanceInches = 5.0;
};

class SlaveController {
 private:
  Status currentStatus;
  Settings settings;
  unsigned long stateStartTime;

  // Stepper motors
  AccelStepper* stepperX;
  AccelStepper* stepperY;

  // Endstops
  Bounce xEndstop;
  Bounce yEndstop;

  // Pattern generation
  PatternGenerator patternGenerator;
  std::vector<Point> currentPattern;
  size_t currentPatternIndex;
  bool patternInProgress;

  // Target positions
  long targetX;
  long targetY;

  // Methods
  void processCommand(const String& command);
  void updateSettings(const JsonObject& json);
  void sendState();
  String stateToString(Status state);

  // Motion control
  void homeXAxis();
  void homeYAxis();
  bool moveToXYPosition(const long xPos, const long yPos);

  // Pneumatic control
  void extendArm();
  void retractArm();
  void enableSuction();
  void disableSuction();

  // Sequence control
  bool executePickSequence();
  bool executePlaceSequence();

  // Utility
  bool hasTimeElapsed(const unsigned long duration);
  void handleCommand(const char cmd, const String& params);

 public:
  SlaveController();
  ~SlaveController();
  void setup();
  void loop();
};