#pragma once

#include <Arduino.h>

// Conversion Configuration
struct ConversionConfig {
  static constexpr double STEPS_PER_INCH = 125.0;

  static long inchesToSteps(double inches) {
    return lround(inches * STEPS_PER_INCH);
  }

  static double stepsToInches(long steps) {
    return static_cast<double>(steps) / STEPS_PER_INCH;
  }
};

// Motion Parameters
struct MotionConfig {
  double speedInches = 40.0;
  double accelerationInches = 20.0;
  const double homingSpeedInches = 7.5;
  const double pickDistanceInches = 5.0;

  int32_t getSpeed() const {
    return ConversionConfig::inchesToSteps(speedInches);
  }
  int32_t getAcceleration() const {
    return ConversionConfig::inchesToSteps(accelerationInches);
  }
  int32_t getHomingSpeed() const {
    return ConversionConfig::inchesToSteps(homingSpeedInches);
  }
  int32_t getPickDistance() const {
    return ConversionConfig::inchesToSteps(pickDistanceInches);
  }
};

// Pin Assignments
struct PinConfig {
  // X-Axis
  static constexpr uint8_t xPulse = 6;
  static constexpr uint8_t xDirection = 7;
  static constexpr uint8_t xEndstop = 13;

  // Y-Axis
  static constexpr uint8_t yPulse = 11;
  static constexpr uint8_t yDirection = 12;
  static constexpr uint8_t yEndstop = 10;

  // Pneumatics
  static constexpr uint8_t extension = 3;
  static constexpr uint8_t suction = 4;
};

// Timing Parameters
struct TimingConfig {
  static constexpr uint16_t endstopDebounce = 5;
  static constexpr uint16_t pickDelay = 10;
  static constexpr uint16_t pickDuration = 300;
  static constexpr uint16_t placementDelay = 100;
  static constexpr uint16_t placementDuration = 100;
  static constexpr uint16_t retractDelay = 100;
  static constexpr unsigned long waitRetrieveDelay = 200;
};
