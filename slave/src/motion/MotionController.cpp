#include "MotionController.h"

#include "../communication/Protocol.h"
#include "../config/Config.h"
#include "../state/StateMachine.h"

MotionController::MotionController(StateMachine& stateMachine)
    : stateMachine(stateMachine) {
  auto& config = MachineConfig::getInstance();

  xAxis = new AccelStepper(1, config.pins.xPulse, config.pins.xDirection);
  yAxis = new AccelStepper(1, config.pins.yPulse, config.pins.yDirection);

  // Set minimum pulse width to 100 microseconds
  xAxis->setMinPulseWidth(3);
  yAxis->setMinPulseWidth(3);

  // Configure default speeds and accelerations
  xAxis->setMaxSpeed(config.motion.getSpeed());
  xAxis->setAcceleration(config.motion.getAcceleration());
  yAxis->setMaxSpeed(config.motion.getSpeed());
  yAxis->setAcceleration(config.motion.getAcceleration());
}

void MotionController::moveTo(Point target) {
  long targetX = ConversionConfig::inchesToSteps(target.x);
  long targetY = ConversionConfig::inchesToSteps(target.y);
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

  // First set speed to 0
  xAxis->setSpeed(0);
  yAxis->setSpeed(0);

  // Stop the motors
  xAxis->stop();
  yAxis->stop();

  // Reset targets to current position to ensure movement is completely stopped
  xAxis->setCurrentPosition(xAxis->currentPosition());
  yAxis->setCurrentPosition(yAxis->currentPosition());
  xAxis->moveTo(xAxis->currentPosition());
  yAxis->moveTo(yAxis->currentPosition());

  Protocol::debug("Motors stopped");
}

bool MotionController::isMoving() const {
  return xAxis->isRunning() || yAxis->isRunning();
}

void MotionController::update() {
  // If we're in manual movement mode, use runSpeed() instead of run()
  if (stateMachine.getCurrentState() == State::MANUAL_MOVING) {
    // Check position limits before running
    double currentX = getCurrentX();
    double currentY = getCurrentY();

    // Stop movement if we hit limits
    if (currentX <= 0 && xAxis->speed() < 0) {
      xAxis->setSpeed(0);
      xAxis->stop();
    } else if (currentX >= 26.0 && xAxis->speed() > 0) {
      xAxis->setSpeed(0);
      xAxis->stop();
    }

    if (currentY <= 0 && yAxis->speed() < 0) {
      yAxis->setSpeed(0);
      yAxis->stop();
    } else if (currentY >= 35.0 && yAxis->speed() > 0) {
      yAxis->setSpeed(0);
      yAxis->stop();
    }

    xAxis->runSpeed();
    yAxis->runSpeed();
    return;
  }

  // If we're homing X or Y, keep running at constant speed
  if (stateMachine.getCurrentState() == State::HOMING_X) {
    xAxis->runSpeed();
    return;
  } else if (stateMachine.getCurrentState() == State::HOMING_Y) {
    yAxis->runSpeed();
    return;
  }

  // Normal movement updates
  xAxis->run();
  yAxis->run();

  // Remove position update logging during motion
  // static unsigned long lastPosUpdate = 0;
  // const unsigned long now = millis();
  // if ((xAxis->isRunning() || yAxis->isRunning()) &&
  //     (now - lastPosUpdate >= 250)) {
  //   Protocol::sendPosition(getCurrentX(), getCurrentY());
  //   lastPosUpdate = now;
  // }
}

void MotionController::setup() {
  // Enable outputs
  xAxis->enableOutputs();
  yAxis->enableOutputs();

  auto& config = MachineConfig::getInstance();
  int32_t maxSpeed = config.motion.getSpeed();
  int32_t acceleration = config.motion.getAcceleration();

  xAxis->setMaxSpeed(maxSpeed);
  xAxis->setAcceleration(acceleration);
  yAxis->setMaxSpeed(maxSpeed);
  yAxis->setAcceleration(acceleration);
}

MotionController::~MotionController() {
  delete xAxis;
  delete yAxis;
}

void MotionController::startHomingX() {
  auto& config = MachineConfig::getInstance();
  int32_t homingSpeed = -config.motion.getHomingSpeed();
  xAxis->enableOutputs();
  xAxis->setSpeed(homingSpeed);
}

void MotionController::startHomingY() {
  auto& config = MachineConfig::getInstance();
  int32_t homingSpeed = -config.motion.getHomingSpeed();
  yAxis->enableOutputs();
  yAxis->setSpeed(homingSpeed);
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

void MotionController::setManualMoveX(int32_t speed, int32_t acceleration) {
  xAxis->setMaxSpeed(abs(speed));
  xAxis->setAcceleration(acceleration);
  xAxis->setSpeed(speed);
}

void MotionController::setManualMoveY(int32_t speed, int32_t acceleration) {
  yAxis->setMaxSpeed(abs(speed));
  yAxis->setAcceleration(acceleration);
  yAxis->setSpeed(speed);
}

void MotionController::stopManualMove() {
  // First set speed to 0 to prevent further movement
  xAxis->setSpeed(0);
  yAxis->setSpeed(0);

  // Stop the motors and reset their targets to current position
  xAxis->stop();
  yAxis->stop();
  xAxis->setCurrentPosition(xAxis->currentPosition());
  yAxis->setCurrentPosition(yAxis->currentPosition());

  // Clear any remaining movement
  xAxis->moveTo(xAxis->currentPosition());
  yAxis->moveTo(yAxis->currentPosition());
}