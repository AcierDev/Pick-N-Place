#include "SlaveController.h"

// Create a global instance of the controller
SlaveController controller;

// Global setup function required by Arduino
void setup() { controller.setup(); }

// Global loop function required by Arduino
void loop() { controller.loop(); }
