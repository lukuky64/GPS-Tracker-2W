#pragma once

#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_v3.h>

#include "messages.hpp"

class GPS_Talker {
 public:
  GPS_Talker(HardwareSerial& serial, uint8_t nrstPin, uint8_t timePulsePin = 255, uint8_t ledPin = 255);

  bool begin(unsigned long baudRate = 38400, uint8_t navFreq = 2);

  int32_t getLatitude();

  int32_t getLongitude();

  int32_t getAltitude();

  void hardwareReset();

  static void PVTCallback(UBX_NAV_PVT_data_t* ubxDataStruct);

  static void timePulseISR();  // Interrupt Service Routine for time pulse

  bool checkNewData();

  bool checkTimePulse();  // Check if time pulse event occurred

  GPS_Data* getData();

  void setupLED(uint8_t ledPin);
  void toggleLED(uint8_t ledPin);

 private:
  SFE_UBLOX_GNSS_SERIAL m_GPS_Module;  // SFE_UBLOX_GNSS_SERIAL uses Serial (UART). For I2C or SPI, see Example1 and Example3
  HardwareSerial& m_GPS_Serial;
  unsigned long m_baudRate;  // Default baud rate for u-blox M10 module
  uint8_t m_navFreq;         // Default navigation frequency (2 Hz)

  uint8_t m_nRST_Pin;       // Pin for hardware reset -> GPIO2
  uint8_t m_timePulse_Pin;  // Pin for time pulse -> GPIO3
  uint8_t m_LED_Pin;        // Pin for LED indicator (255 = disabled)

  static GPS_Talker* m_instance;

  volatile bool m_newData;
  GPS_Data m_gpsData;  // Structure to hold GPS data

  bool m_initialised;
};
