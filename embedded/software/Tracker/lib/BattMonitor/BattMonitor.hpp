#pragma once

#include <Arduino.h>

class BattMonitor {
 public:
  BattMonitor(uint8_t sensePin, float scaleFactor = 1.0f);
  void init();
  float getScaledVoltage(uint8_t samples = 1);

 private:
  float getRawVoltage(uint8_t samples = 1);
  float analogReadMilliVolts(uint8_t sensePin);
  uint8_t m_sensePin;
  bool m_initialised;
  float m_scaleFactor;     // Scale factor for voltage conversion
  float m_batteryVoltage;  // Battery voltage in volts

  float m_Vref;          // Reference voltage for ADC (default for ESP32)
  int m_readResolution;  // ADC resolution (12 bits)
};