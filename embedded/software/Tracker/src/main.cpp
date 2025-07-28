#include <Arduino.h>

#include "Control.hpp"

Control control;

void setup() {
  delay(1000);
  if (!control.setup()) control.restartDevice();
  delay(100);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(10000));  // Main loop does nothing
}