#pragma once
#include <AccelStepper.h>

#include "../config/ConversionConfig.h"
#include "../state/StateMachine.h"
#include "../types/Types.h"

class MotionController {
 private:
  AccelStepper* xAxis;
  AccelStepper* yAxis;
  bool homed_ = false;
  StateMachine& stateMachine;

 public:
  MotionController(StateMachine& stateMachine);  // Modified constructor
  ~MotionController();

  double getCurrentX() const {
    return ConversionConfig::stepsToInches(xAxis->currentPosition());
  }

  double getCurrentY() const {
    return ConversionConfig::stepsToInches(yAxis->currentPosition());
  }

  void moveTo(Point target);
  void home();
  void stop();
  bool isMoving() const;
  void update();
  void setup();
  bool isHomed() const { return homed_; }
  void setHomed(bool homed) { homed_ = homed; }
  void startHomingX();
  void startHomingY();
  void stopX();
  void stopY();
  void setXHome();
  void setYHome();
  void setManualMoveX(int32_t speed, int32_t acceleration);
  void setManualMoveY(int32_t speed, int32_t acceleration);
  void stopManualMove();
  int32_t getXMaxSpeed() const { return xAxis->maxSpeed(); }
  int32_t getYMaxSpeed() const { return yAxis->maxSpeed(); }
  void setXMaxSpeed(int32_t speed) { xAxis->setMaxSpeed(speed); }
  void setYMaxSpeed(int32_t speed) { yAxis->setMaxSpeed(speed); }
};