#include "StateMachine.h"

void StateMachine::update() {
  if (auto handler = stateHandlers[currentState]) {
    handler();
  }
}

void StateMachine::setState(State newState) {
  currentState = newState;
  stateStartTime = millis();

  // Optional: Send state change notification through Protocol
  Protocol::info("State changed to: " + stateToString(newState));
}

String StateMachine::stateToString(State state) {
  switch (state) {
    case State::IDLE:
      return "IDLE";
    case State::HOME_REQUESTED:
      return "HOME_REQUESTED";
    case State::HOMING_X:
      return "HOMING_X";
    case State::HOMING_Y:
      return "HOMING_Y";
    case State::AWAITING_START:
      return "AWAITING_START";
    case State::MOVING_TO_PICK:
      return "MOVING_TO_PICK";
    case State::MOVING_TO_TARGET:
      return "MOVING_TO_TARGET";
    case State::PICKING:
      return "PICKING";
    case State::PLACING:
      return "PLACING";
    case State::RETRACTING:
      return "RETRACTING";
    case State::WAITING_TO_RETRIEVE:
      return "WAITING_TO_RETRIEVE";
    case State::EXECUTING_PATTERN:
      return "EXECUTING_PATTERN";
    case State::MANUAL_MOVING:
      return "MANUAL_MOVING";
    default:
      return "UNKNOWN";
  }
}