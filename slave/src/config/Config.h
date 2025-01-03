#pragma once
#include <Arduino.h>

#include "ConversionConfig.h"

struct MotionConfig {
  double speedInches = 40.0;
  double accelerationInches = 40.0;
  const double homingSpeedInches = 5.0;
  const double pickDistanceInches = 5.0;

  int32_t getSpeed() const;
  int32_t getAcceleration() const;
  int32_t getHomingSpeed() const;
  int32_t getPickDistance() const;

  void setSpeed(double speed) { speedInches = speed; }

  void setAcceleration(double accel) { accelerationInches = accel; }
};

struct PinConfig {
  static const uint8_t xPulse = 5;
  static const uint8_t xDirection = 18;
  static const uint8_t xEndstop = 19;
  static const uint8_t yPulse = 16;
  static const uint8_t yDirection = 17;
  static const uint8_t yEndstop = 21;
  static const uint8_t extension = 22;
  static const uint8_t suction = 23;
};

struct TimingConfig {
  static const unsigned long endstopDebounce = 5;
  static const unsigned long pickDelay = 100;
  static const unsigned long pickDuration = 500;
  static const unsigned long placementDelay = 100;
  static const unsigned long placementDuration = 500;
  static const unsigned long retractDelay = 250;
  static const unsigned long waitRetrieveDelay = 1000;
};

struct PatternConfig {
  double boxX = 0.0;
  double boxY = 0.0;
  double pickupX = 0.0;
  double pickupY = 0.0;
  double boxWidth = 0.0;
  double boxLength = 0.0;
};

class MachineConfig {
 public:
  static MachineConfig& getInstance() {
    static MachineConfig instance;
    return instance;
  }

  MotionConfig motion;
  PatternConfig pattern;
  PinConfig pins;
  TimingConfig timing;

 private:
  MachineConfig() = default;
};