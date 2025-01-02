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
};