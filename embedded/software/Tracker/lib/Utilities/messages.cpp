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
  UART_USB.print(F("Latitude: "));
  UART_USB.print((gpsData->latitude) / 1e7f);
  UART_USB.print(F("\t"));
  UART_USB.print(F("Longitude: "));
  UART_USB.print((gpsData->longitude) / 1e7f);
  UART_USB.print(F("\t"));
  UART_USB.print(F("Altitude: "));
  UART_USB.println((gpsData->altitude) / 1e7f);
}

void printSYSData(const SYS_DATA *sysData) {
  UART_USB.print(F("Battery Voltage: "));
  UART_USB.print(sysData->batteryVoltage);
  UART_USB.print(F(" V\t"));
  UART_USB.print(F("RF Power: "));
  UART_USB.println(sysData->rfPower);
}