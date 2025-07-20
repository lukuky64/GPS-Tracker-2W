#include "messages.hpp"

MSG_PACKET *packData(GPS_Data *gps) {
  MSG_PACKET *packet = (MSG_PACKET *)malloc(sizeof(MSG_PACKET));
  // float voltage = batteryMonitor.getScaledVoltage(10);
  memcpy(packet->latitude, &gps->latitude, 4);
  memcpy(packet->longitude, &gps->longitude, 4);
  memcpy(packet->altitude, &gps->altitude, 4);
  return packet;
}

MSG_PACKET *unpackData(ResponseContainer &response) {
  MSG_PACKET *unpackedData = (MSG_PACKET *)malloc(sizeof(MSG_PACKET));

  memcpy(unpackedData, response.data.c_str(), sizeof(MSG_PACKET));

  return unpackedData;
}

GPS_Data *unpackData(MSG_PACKET *msgPacket) {
  GPS_Data *gpsData = (GPS_Data *)malloc(sizeof(GPS_Data));

  gpsData->latitude = *(int32_t *)(msgPacket->latitude);
  gpsData->longitude = *(int32_t *)(msgPacket->longitude);
  gpsData->altitude = *(int32_t *)(msgPacket->altitude);

  return gpsData;
}

void printGPSData(GPS_Data *gpsData) {
  UART_USB.print(F("Latitude: "));
  UART_USB.print(*(int32_t *)(gpsData->latitude) / 1e7f);
  UART_USB.print(F("\t"));
  UART_USB.print(F("Longitude: "));
  UART_USB.print(*(int32_t *)(gpsData->longitude) / 1e7f);
  UART_USB.print(F("\t"));
  UART_USB.print(F("Altitude: "));
  UART_USB.println(*(int32_t *)(gpsData->altitude) / 1e7f);
}