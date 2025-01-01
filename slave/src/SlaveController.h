#pragma once

#include <AccelStepper.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Bounce2.h>

#include <vector>

#include "PatternGenerator.h"
#include "config.h"

enum class State {
  IDLE,
  HOME_REQUESTED,
  HOMING_X,
  HOMING_Y,
  AWAITING_START,
  MOVING_TO_PICK,
  MOVING_TO_TARGET,
  PICKING,
  PLACING,
  RETRACTING,
  WAITING_TO_RETRIEVE,
  EXECUTING_PATTERN
};

class SlaveController {
 public:
  SlaveController();
  ~SlaveController();

  void setup();
  void loop();

 private:
  // Configuration
  MotionConfig motionConfig;

  // Hardware objects
  AccelStepper* stepperX;
  AccelStepper* stepperY;
  Bounce xEndstop;
  Bounce yEndstop;

  // State tracking
  State currentState;
  State nextStateAfterMove;
  State nextStateAfterRetract;
  unsigned long stateStartTime;

  // Pattern generation
  PatternGenerator patternGenerator;
  std::vector<Point> currentPattern;
  size_t currentPatternIndex;
  bool patternInProgress;

  // Position tracking
  long targetX;
  long targetY;

  // Setup functions
  void setupEndstops();
  void setupSteppers();
  void setupPneumatics();
  void setupCommunication();

  // Motion control
  void homeXAxis();
  void homeYAxis();
  bool moveToXYPosition(const long xPos, const long yPos);

  // State machine control
  void updateInputs();
  void runStateMachine();
  void startNextState(const State nextState);
  void waitForStartCommand();

  // Sequence control
  bool executePickSequence();
  bool executePlaceSequence();

  // Pneumatic control
  void extendArm();
  void retractArm();
  void enableSuction();
  void disableSuction();

  // Command handling
  void handleCommand(const String& command, const String& params);
  void printCurrentSettings();

  // Utility functions
  bool hasTimeElapsed(const unsigned long duration);

  // Add these new methods
  void sendCurrentState();
  const char* stateToString(State state);

  void checkForCommands();

  void emergencyStop();

  // Debug tracking
  unsigned long lastStateChangeTime;
  unsigned long stateStartCount;
  static constexpr unsigned long STATE_TIMEOUT_MS = 10000;  // 10 second timeout
};