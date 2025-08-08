#include "messages.hpp"

void printRawPacket(const MSG_PACKET *packet) {
  const uint8_t *bytes = (const uint8_t *)packet;
  for (size_t i = 0; i < sizeof(MSG_PACKET); ++i) {
    UART_USB.printf("%02X ", bytes[i]);
  }
  UART_USB.println();
}

void packData(MSG_PACKET *packet, const GPS_DATA *gps, const SYS_DATA *sysData) {
  FloatBytes lat, lon, alt;
  lat.f = gps->latitude;
  lon.f = gps->longitude;
  alt.f = gps->altitude;
  memcpy(packet->latitude, lat.b, 4);
  memcpy(packet->longitude, lon.b, 4);
  memcpy(packet->altitude, alt.b, 4);
  packet->nFixes[0] = gps->nFixes;
  packet->batteryVoltage[0] = (uint8_t)(sysData->batteryVoltage * 10);
  packet->rfPower[0] = (uint8_t)(sysData->rfPower);
}

void unpackResponseData(MSG_PACKET *packet, const ResponseContainer &response) {
  // memcpy(packet, response.data.c_str(), sizeof(MSG_PACKET));
  // FIX: First 3 bytes are not part of the E22 packet
  memcpy(packet, response.data.c_str() + 3, sizeof(MSG_PACKET));
  //
}

void unpackGPSData(GPS_DATA *gpsData, const MSG_PACKET *msgPacket) {
  FloatBytes lat, lon, alt;
  memcpy(lat.b, msgPacket->latitude, 4);
  memcpy(lon.b, msgPacket->longitude, 4);
  memcpy(alt.b, msgPacket->altitude, 4);
  gpsData->latitude = lat.f;
  gpsData->longitude = lon.f;
  gpsData->altitude = alt.f;
  gpsData->nFixes = msgPacket->nFixes[0];
}

void unpackSYSData(SYS_DATA *sysData, const MSG_PACKET *msgPacket) {
  sysData->batteryVoltage = msgPacket->batteryVoltage[0] / 10.0f;  // scale back
  sysData->rfPower = msgPacket->rfPower[0];
}

void printGPSData(const GPS_DATA *gpsData) {
  char buffer[128];
  // 5dp for latitude and longitude readings gives 1.11m precision
  snprintf(buffer, sizeof(buffer), "Latitude: %.5f \tLongitude: %.5f \tAltitude: %.1fm \tFixes: %d", gpsData->latitude, gpsData->longitude, gpsData->altitude, gpsData->nFixes);
  UART_USB.println(buffer);
}

void printSYSData(const SYS_DATA *sysData) {
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "Battery Voltage: %.2f V\tRF Power: %d", sysData->batteryVoltage, sysData->rfPower);
  UART_USB.println(buffer);
}