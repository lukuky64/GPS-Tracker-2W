#include "BattMonitor.hpp"

BattMonitor::BattMonitor(uint8_t sensePin, float scaleFactor) : m_sensePin(sensePin), m_scaleFactor(scaleFactor), m_initialised(false), m_batteryVoltage(0.0f), m_Vref(3.3f), m_readResolution(12) {}

void BattMonitor::init() {
  if (!m_initialised) {
    pinMode(m_sensePin, INPUT);
    // Set the ADC resolution and range
    analogReadResolution(m_readResolution);  // 4096 steps
    m_initialised = true;
  }
}

float BattMonitor::getRawVoltage(uint8_t samples) {
  if (!m_initialised) init();
  float rawV = 0.0f;
  samples = samples == 0 ? 1 : samples;  // Prevent division by zero
  for (uint8_t i = 0; i < samples; i++) {
    uint32_t raw_mV = analogReadMilliVolts(m_sensePin);
    rawV += static_cast<float>(raw_mV) / 1000.0f;  // Convert to volts
  }
  rawV /= samples;
  return rawV;
}

float BattMonitor::getScaledVoltage(uint8_t samples) {
  float rawV = getRawVoltage(samples);
  m_batteryVoltage = rawV * m_scaleFactor;  // Apply scale factor
  return m_batteryVoltage;
}

float BattMonitor::analogReadMilliVolts(uint8_t sensePin) {
  float raw_mV = static_cast<float>(analogRead(sensePin)) * (m_Vref / static_cast<float>((1 << m_readResolution) - 1)) * 1000.0f;
  return raw_mV;
}