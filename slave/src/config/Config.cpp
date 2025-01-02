#include "Config.h"

#include "ConversionConfig.h"

int32_t MotionConfig::getSpeed() const {
  return ConversionConfig::inchesToSteps(speedInches);
}

int32_t MotionConfig::getAcceleration() const {
  return ConversionConfig::inchesToSteps(accelerationInches);
}

int32_t MotionConfig::getHomingSpeed() const {
  return ConversionConfig::inchesToSteps(homingSpeedInches);
}