
#pragma once
#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_v3.h>

#include "LoRa_E22.h"

struct MSG_PACKET {
  byte latitude[4];
  byte longitude[4];
  byte altitude[4];
  byte batteryVoltage[1];
  byte rfPower[1];
};

struct GPS_DATA {
  float latitude;   // Latitude in degrees (float)
  float longitude;  // Longitude in degrees (float)
  float altitude;   // Altitude above Mean Sea Level in meters (float)
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint8_t nFixes;
};

struct SYS_DATA {
  float batteryVoltage;
  int8_t rfPower;
};

void packData(MSG_PACKET *packet, const GPS_DATA *gps, const SYS_DATA *sysData);
void unpackResponseData(MSG_PACKET *packet, const ResponseContainer &response);
void unpackGPSData(GPS_DATA *gpsData, const MSG_PACKET *msgPacket);
void unpackSYSData(SYS_DATA *sysData, const MSG_PACKET *msgPacket);
void printGPSData(const GPS_DATA *gpsData);
void printSYSData(const SYS_DATA *sysData);