#pragma once
#include <Arduino.h>

#include <functional>
#include <map>

#include "../communication/Protocol.h"
#include "../types/Types.h"

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
  EXECUTING_PATTERN,
  MANUAL_MOVING
};

class StateMachine {
 private:
  std::map<State, std::function<void()>> stateHandlers;
  State currentState;
  unsigned long stateStartTime;
  unsigned long stateTimeout;

 public:
  StateMachine()
      : currentState(State::IDLE), stateStartTime(0), stateTimeout(10000) {}

  void registerHandler(State state, std::function<void()> handler) {
    stateHandlers[state] = handler;
  }

  void setState(State newState);
  void update();
  State getCurrentState() const { return currentState; }
  static String stateToString(State state);
};