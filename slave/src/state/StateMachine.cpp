#include "StateMachine.h"

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
    case State::MOVING:
      return "MOVING";
    case State::PICKING:
      return "PICKING";
    case State::PLACING:
      return "PLACING";
    case State::EXECUTING_PATTERN:
      return "EXECUTING_PATTERN";
    case State::MANUAL_MOVING:
      return "MANUAL_MOVING";
    default:
      return "UNKNOWN";
  }
}