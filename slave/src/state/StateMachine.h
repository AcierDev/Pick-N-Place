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
  MANUAL_MOVING,
  MOVING
};

class StateMachine {
 private:
  State currentState;
  using StateHandler = void (*)(void*);
  StateHandler handlers[8];  // Fixed array instead of map
  void* handlerContexts[8];

 public:
  StateMachine() : currentState(State::IDLE) {}

  void registerHandler(State state, StateHandler handler, void* context) {
    handlers[static_cast<int>(state)] = handler;
    handlerContexts[static_cast<int>(state)] = context;
  }

  void update() {
    if (auto handler = handlers[static_cast<int>(currentState)]) {
      handler(handlerContexts[static_cast<int>(currentState)]);
    }
  }

  void setState(State newState) {
    if (currentState != newState) {
      Protocol::debug("State transition: " + stateToString(currentState) +
                      " -> " + stateToString(newState));
      currentState = newState;
    }
  }
  State getCurrentState() const { return currentState; }
  static String stateToString(State state);
};