#pragma once

#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include "FreeRTOS.h"

class GPS_Talker {
public:
    GPS_Talker(HardwareSerial& serial, uint8_t nrstPin);

    bool begin();

    void requestPVT();

    bool available();

    int32_t getLatitude();

    int32_t getLongitude();

    int32_t getAltitude();

    void hardwareReset();

    void timePulseCallback();

private:
    SFE_UBLOX_GNSS_SERIAL m_GPS_Module; // SFE_UBLOX_GNSS_SERIAL uses Serial (UART). For I2C or SPI, see Example1 and Example3
    HardwareSerial& m_GPS_Serial;
    unsigned long m_baudRate = 38400; // Default baud rate for u-blox M10 module

    uint8_t m_nRST_Pin; // Pin for hardware reset -> GPIO2
    uint8_t m_timePulse_Pin; // Pin for time pulse -> GPIO3

};
