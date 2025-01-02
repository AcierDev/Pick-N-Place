#pragma once

class ConversionConfig {
 public:
  static constexpr double STEPS_PER_INCH = 125.0;

  static long inchesToSteps(double inches) {
    return lround(inches * STEPS_PER_INCH);
  }

  static double stepsToInches(long steps) {
    return static_cast<double>(steps) / STEPS_PER_INCH;
  }
};