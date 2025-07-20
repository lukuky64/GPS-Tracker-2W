
#pragma once
#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_v3.h>

#include "LoRa_E22.h"

struct MSG_PACKET {
  byte latitude[4];
  byte longitude[4];
  byte altitude[4];
};

struct GPS_Data {
  int32_t latitude;
  int32_t longitude;
  int32_t altitude;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
};

MSG_PACKET *packData(GPS_Data *gps);
MSG_PACKET *unpackData(ResponseContainer &response);
GPS_Data *unpackData(MSG_PACKET *msgPacket);
void printGPSData(GPS_Data *gpsData);