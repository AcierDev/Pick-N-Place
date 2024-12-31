#ifndef CONFIG_H
#define CONFIG_H

// Pin Definitions
#define STEPPER_X_STEP 6
#define STEPPER_X_DIR 7
#define STEPPER_Y_STEP 11
#define STEPPER_Y_DIR 12
#define ENDSTOP_X_PIN 13
#define ENDSTOP_Y_PIN 10
#define EXTENSION_PIN 3
#define SUCTION_PIN 4

// Constants
#define BAUD_RATE 115200
#define BUTTON_DEBOUNCE_MS 50
#define ENDSTOP_DEBOUNCE_MS 5
#define STEPS_PER_INCH 125.0

// Timing Constants
#define PICK_DELAY_MS 10
#define PICK_DURATION_MS 300
#define PLACE_DELAY_MS 100
#define PLACE_DURATION_MS 100
#define RETRACT_DELAY_MS 100

// Macros
#define TURN_ON(pin) digitalWrite(pin, HIGH)
#define TURN_OFF(pin) digitalWrite(pin, LOW)
#define READ_PIN(pin) digitalRead(pin)

#endif  // CONFIG_H
