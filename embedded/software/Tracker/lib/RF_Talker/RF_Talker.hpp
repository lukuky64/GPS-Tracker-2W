#pragma once

#include <Arduino.h>

#include "LoRa_E22.h"

class RF_Talker {
 public:
  RF_Talker(HardwareSerial& serial, uint8_t ctrl0Pin, uint8_t ctrl1Pin, uint8_t statusPin);

  bool begin(unsigned long baudRate = 9600);

  ResponseStatus sendFixedMessage(uint8_t channel, uint8_t destinationAddL, uint8_t length, const String& message);

  ResponseContainer receiveMessage();

  bool available();

 private:
  LoRa_E22 m_e22Module;
  HardwareSerial& m_RF_Serial;
  uint8_t m_ctrl0Pin;
  uint8_t m_ctrl1Pin;
  uint8_t m_statusPin;
};
