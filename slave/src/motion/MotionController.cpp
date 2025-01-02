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
  // Only send position every ~50ms while moving
  static unsigned long lastPosUpdate = 0;
  const unsigned long now = millis();

  if (isMoving() && (now - lastPosUpdate) >= 50) {
    Protocol::sendPosition(getCurrentX(), getCurrentY());
    lastPosUpdate = now;
  }

  // Normal movement updates
  xAxis->run();
  yAxis->run();
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