#pragma once

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

#include "LoRa_E22.h"
#include "messages.hpp"

enum E22_900T33S_POWER {
  POWER_33 = 0b00,
  POWER_30 = 0b01,
  POWER_27 = 0b10,
  POWER_24 = 0b11

};

class RF_Talker {
 public:
  RF_Talker(HardwareSerial& serial, uint8_t ctrl0Pin = 255, uint8_t ctrl1Pin = 255, uint8_t statusPin = 255, uint8_t ledPin = 255);

  bool begin();

  // ResponseStatus sendFixedMessage(uint8_t channel, uint8_t destinationAddL, uint8_t length, const String& message);
  bool sendMessage(const void* message);

  bool available();

  bool receiveMessage(ResponseContainer& response);

  int8_t getRFPower() const { return m_rfPower; }
  int8_t setRFPower(int8_t power);
  int mapRFPower(int8_t& power);

  void setupLED(uint8_t ledPin);
  void toggleLED();
  void setLED(bool state);

 private:
  LoRa_E22* m_e22Module;
  HardwareSerial& m_RF_Serial;
  uint8_t m_ctrl0Pin;
  uint8_t m_ctrl1Pin;
  uint8_t m_statusPin;
  uint8_t m_LED_Pin;

  int8_t m_rfPower;  // RF power level (dB)

  struct RF_Config {
    byte m_ADDH;
    byte m_ADDL;
    byte m_CHAN;
  } m_rfConfig;
};
