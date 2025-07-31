#include "messages.hpp"

void packData(MSG_PACKET *packet, const GPS_DATA *gps, const SYS_DATA *sysData) {
  memcpy(packet->latitude, &gps->latitude, 4);
  memcpy(packet->longitude, &gps->longitude, 4);
  memcpy(packet->altitude, &gps->altitude, 4);
  packet->batteryVoltage[0] = (uint8_t)(sysData->batteryVoltage * 10);  // Scale to 0.1V
  packet->rfPower[0] = (uint8_t)(sysData->rfPower);
}

void unpackResponseData(MSG_PACKET *packet, const ResponseContainer &response) {
  memcpy(packet, response.data.c_str(), sizeof(MSG_PACKET));
  //
}

void unpackGPSData(GPS_DATA *gpsData, const MSG_PACKET *msgPacket) {
  memcpy(&gpsData->latitude, msgPacket->latitude, sizeof(float));
  memcpy(&gpsData->longitude, msgPacket->longitude, sizeof(float));
  memcpy(&gpsData->altitude, msgPacket->altitude, sizeof(float));
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