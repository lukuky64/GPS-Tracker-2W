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
  gpsData->latitude = *(int32_t *)(msgPacket->latitude);
  gpsData->longitude = *(int32_t *)(msgPacket->longitude);
  gpsData->altitude = *(int32_t *)(msgPacket->altitude);
}

void unpackSYSData(SYS_DATA *sysData, const MSG_PACKET *msgPacket) {
  sysData->batteryVoltage = msgPacket->batteryVoltage[0] / 10.0f;  // scale back
  sysData->rfPower = msgPacket->rfPower[0];
}

void printGPSData(const GPS_DATA *gpsData) {
  UART_USB.print(F("Latitude: "));
  UART_USB.print(*(int32_t *)(gpsData->latitude) / 1e7f);
  UART_USB.print(F("\t"));
  UART_USB.print(F("Longitude: "));
  UART_USB.print(*(int32_t *)(gpsData->longitude) / 1e7f);
  UART_USB.print(F("\t"));
  UART_USB.print(F("Altitude: "));
  UART_USB.println(*(int32_t *)(gpsData->altitude) / 1e7f);
}