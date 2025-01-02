#pragma once
#include <Arduino.h>
#include <Bounce2.h>

#include "../commands/CommandHandler.h"
#include "../motion/MotionController.h"
#include "../pattern/PatternGenerator.h"
#include "../state/StateMachine.h"

class SlaveController {
 private:
  CommandHandler commandHandler;
  MotionController motionController;
  StateMachine stateMachine;
  PatternGenerator patternGenerator;
  MachineState currentState;

  // Endstop debouncing
  Bounce xEndstop;
  Bounce yEndstop;

  // Pattern execution state
  std::vector<Point> currentPattern;
  size_t currentPatternIndex;
  bool patternInProgress;
  Point currentPickupLocation;

  // Continuous movement state
  bool continuousMovementActive;
  bool continuousMovementIsX;
  bool continuousMovementPositive;
  int32_t originalXSpeed;
  int32_t originalYSpeed;

  // Pneumatic state tracking
  bool suctionEnabled;
  bool armExtended;

  // Private setup methods
  void setupEndstops();
  void setupSteppers();
  void setupPneumatics();
  void setupCommunication();
  void setupStateHandlers();
  void setupCommandHandlers();

  // Private control methods
  void updateInputs();
  void checkForCommands();
  void extendArm();
  void retractArm();
  void enableSuction();
  void disableSuction();
  void handleManualMove(const String& command);
  void startContinuousMovement(bool isXAxis, bool isPositive, float speed,
                               float acceleration);
  void stopManualMovement();
  void updateState();
  void handleJsonCommand();

 public:
  SlaveController();
  ~SlaveController();

  void setup();
  void loop();
  void emergencyStop();
};