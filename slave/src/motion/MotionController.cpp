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
  static unsigned long lastDebugTime = 0;
  const unsigned long now = millis();

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

    // Only debug if motors are actually moving
    if ((abs(xAxis->speed()) > 0 || abs(yAxis->speed()) > 0) &&
        now - lastDebugTime >= 250) {
      Protocol::debug("Manual move - X pos: " + String(getCurrentX(), 2) +
                      " Y pos: " + String(getCurrentY(), 2));
      lastDebugTime = now;
    }
    return;
  }

  // If we're homing X, keep running at constant speed
  if (stateMachine.getCurrentState() == State::HOMING_X) {
    bool stepped = xAxis->runSpeed();  // This needs to be called frequently

    // Debug output every 250ms during homing
    if (now - lastDebugTime >= 250) {  // Changed from 100ms to 250ms
      Protocol::debug(
          "X homing - position: " + String(xAxis->currentPosition()) +
          " speed: " + String(xAxis->speed()) + " stepped: " + String(stepped));
      lastDebugTime = now;
    }
    return;  // Exit early when homing
  }

  // Normal movement updates
  bool xRunning = xAxis->run();
  bool yRunning = yAxis->run();

  // Debug output every 1 second while moving
  if ((xRunning || yRunning) && (now - lastDebugTime >= 1000)) {
    Protocol::debug("X position: " + String(xAxis->currentPosition()) +
                    " speed: " + String(xAxis->speed()));
    Protocol::debug("Y position: " + String(yAxis->currentPosition()) +
                    " speed: " + String(yAxis->speed()));
    lastDebugTime = now;
  }

  // Position updates every 250ms while moving
  static unsigned long lastPosUpdate = 0;
  if ((xRunning || yRunning) &&
      (now - lastPosUpdate >= 250)) {  // Changed from 50ms to 250ms
    Protocol::sendPosition(getCurrentX(), getCurrentY());
    lastPosUpdate = now;
  }
}

void MotionController::setup() {
  Protocol::debug("Setting up motion controller...");

  // Enable outputs
  xAxis->enableOutputs();
  yAxis->enableOutputs();

  auto& config = MachineConfig::getInstance();
  int32_t maxSpeed = config.motion.getSpeed();
  int32_t acceleration = config.motion.getAcceleration();

  Protocol::debug("Max speed: " + String(maxSpeed) + " steps/sec");
  Protocol::debug("Acceleration: " + String(acceleration) + " steps/sec^2");

  xAxis->setMaxSpeed(maxSpeed);
  xAxis->setAcceleration(acceleration);
  yAxis->setMaxSpeed(maxSpeed);
  yAxis->setAcceleration(acceleration);

  Protocol::debug("Motion controller setup complete");
}

MotionController::~MotionController() {
  delete xAxis;
  delete yAxis;
}

void MotionController::startHomingX() {
  auto& config = MachineConfig::getInstance();
  int32_t homingSpeed = -config.motion.getHomingSpeed();

  Protocol::debug("Starting X homing with speed: " + String(homingSpeed) +
                  " steps/sec");

  // Enable the stepper
  xAxis->enableOutputs();

  // Important: Use setSpeed() then runSpeed() in a continuous loop
  xAxis->setSpeed(homingSpeed);

  // Debug initial state
  Protocol::debug("Initial X position: " + String(xAxis->currentPosition()));
}

void MotionController::startHomingY() {
  auto& config = MachineConfig::getInstance();
  int32_t homingSpeed = -config.motion.getHomingSpeed();

  Protocol::debug("Starting Y homing with speed: " + String(homingSpeed) +
                  " steps/sec");

  // Enable the stepper
  yAxis->enableOutputs();

  // Set the speed and start running
  yAxis->setSpeed(homingSpeed);
  bool running = yAxis->runSpeed();

  Protocol::debug("Y motor running: " + String(running ? "YES" : "NO"));
  Protocol::debug("Current Y position: " + String(yAxis->currentPosition()));
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