#pragma once

#include <Arduino.h>

void debugPrint(const String& message) {
  Serial.print("[DEBUG] ");
  Serial.println(message);
}
