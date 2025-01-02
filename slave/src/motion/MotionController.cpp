#include "MotionController.h"

#include "../communication/Protocol.h"
#include "../config/Config.h"
#include "../state/StateMachine.h"

MotionController::MotionController(StateMachine& stateMachine)
    : stateMachine(stateMachine) {
  auto& config = MachineConfig::getInstance();

  xAxis = new AccelStepper(1, config.pins.xPulse, config.pins.xDirection);
  yAxis = new AccelStepper(1, config.pins.yPulse, config.pins.yDirection);

  // Configure default speeds and accelerations
  xAxis->setMaxSpeed(config.motion.getSpeed());
  xAxis->setAcceleration(config.motion.getAcceleration());
  yAxis->setMaxSpeed(config.motion.getSpeed());
  yAxis->setAcceleration(config.motion.getAcceleration());
}

void MotionController::moveTo(Point target) {
  long targetX = ConversionConfig::inchesToSteps(target.x);
  long targetY = ConversionConfig::inchesToSteps(target.y);

  Protocol::debug("Moving to: X=" + String(target.x, 2) +
                  "\" Y=" + String(target.y, 2) + "\"");

  xAxis->moveTo(targetX);
  yAxis->moveTo(targetY);
}

void MotionController::home() {
  // Implement homing sequence
  xAxis->setCurrentPosition(0);
  yAxis->setCurrentPosition(0);
  setHomed(true);
}

void MotionController::stop() {
  Protocol::debug("Emergency stop requested");
  xAxis->stop();
  yAxis->stop();
  Protocol::debug("Motors stopped");
}

bool MotionController::isMoving() const {
  return xAxis->isRunning() || yAxis->isRunning();
}

void MotionController::update() {
  // If we're homing, we need to keep running at constant speed
  if (stateMachine.getCurrentState() == State::HOMING_X) {
    xAxis->runSpeed();
  } else if (stateMachine.getCurrentState() == State::HOMING_Y) {
    yAxis->runSpeed();
  } else {
    // Normal movement updates
    xAxis->run();
    yAxis->run();
  }

  // Log position every ~500ms while moving
  static unsigned long lastDebugTime = 0;
  const unsigned long now = millis();

  if ((isMoving() || stateMachine.getCurrentState() == State::HOMING_X ||
       stateMachine.getCurrentState() == State::HOMING_Y) &&
      (now - lastDebugTime) >= 500) {
    double currentX = ConversionConfig::stepsToInches(xAxis->currentPosition());
    double currentY = ConversionConfig::stepsToInches(yAxis->currentPosition());

    Protocol::debug("Position: X=" + String(currentX, 2) +
                    "\" Y=" + String(currentY, 2) + "\"" + " Speed: X=" +
                    String(xAxis->speed()) + " Y=" + String(yAxis->speed()));

    lastDebugTime = now;
  }
}

void MotionController::setup() {
  xAxis->setMaxSpeed(MachineConfig::getInstance().motion.getSpeed());
  xAxis->setAcceleration(MachineConfig::getInstance().motion.getAcceleration());
  yAxis->setMaxSpeed(MachineConfig::getInstance().motion.getSpeed());
  yAxis->setAcceleration(MachineConfig::getInstance().motion.getAcceleration());
}

MotionController::~MotionController() {
  delete xAxis;
  delete yAxis;
}

void MotionController::startHomingX() {
  auto& config = MachineConfig::getInstance();
  xAxis->setSpeed(
      -config.motion.getHomingSpeed());  // Add back the negative sign
  xAxis->runSpeed();
  Protocol::debug("Starting X homing");
}

void MotionController::startHomingY() {
  auto& config = MachineConfig::getInstance();
  yAxis->setSpeed(
      -config.motion.getHomingSpeed());  // Add back the negative sign
  yAxis->runSpeed();
  Protocol::debug("Starting Y homing");
}

void MotionController::stopX() {
  xAxis->stop();
  xAxis->setSpeed(0);
}

void MotionController::stopY() {
  yAxis->stop();
  yAxis->setSpeed(0);
}

void MotionController::setXHome() {
  xAxis->setCurrentPosition(0);
  Protocol::debug("X axis homed");
}

void MotionController::setYHome() {
  yAxis->setCurrentPosition(0);
  Protocol::debug("Y axis homed");
  setHomed(true);
}