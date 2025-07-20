#include <Arduino.h>

#include "BattMonitor.hpp"
#include "GPS_Talker.hpp"
#include "RF_Talker.hpp"
#include "definitions.hpp"

GPS_Talker gpsTalker(UART_GPS, GPS_NRST, GPS_TIMEPULSE, LED1_PIN);

RF_Talker rfTalker(UART_RF, RF_CTRL0, RF_CTRL1, RF_STATUS);

BattMonitor batteryMonitor(VBAT_SENSE, BATT_SCALE_FACTOR);

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

void setup() {
  bool success = false;
  UART_USB.begin(115200);

  batteryMonitor.init();

  while (!success) {
    if (!gpsTalker.begin(GPS_BAUD_RATE, GPS_NAV_FREQ)) {
      UART_USB.println(F("GPS initialisation failed! Trying again..."));
      gpsTalker.hardwareReset();  // Reset the GPS module
      delay(500);
    } else {
      UART_USB.println(F("GPS initialised successfully!"));
      success = true;
    }
  }

  success = false;
  while (!success) {
    if (!rfTalker.begin()) {
      UART_USB.println(F("RF initialisation failed! Trying again..."));
      gpsTalker.hardwareReset();  // Reset the GPS module
      delay(500);
    } else {
      UART_USB.println(F("RF initialised successfully!"));
      success = true;
    }
  }
}

void loop() {
  if (gpsTalker.checkNewData()) {
    GPS_Data *gpsData = gpsTalker.getData();
    // Print GPS data to USB Serial
    UART_USB.print(F("This GPS Data: "));
    printGPSData(gpsData);

#ifdef BROADCASTER
    MSG_PACKET *msgPacket = packData(gpsData);
    // Send the packed data via RF
    if (rfTalker.sendMessage(msgPacket)) {
      UART_USB.println(F("RF message sent successfully!"));
    } else {
      UART_USB.println(F("Failed to send RF message."));
    }
    free(msgPacket);
#endif
    free(gpsData);
  }

#ifndef BROADCASTER
  if (rfTalker.available()) {
    ResponseContainer response;
    if (rfTalker.receiveMessage(response)) {
      MSG_PACKET *unpackedData = unpackData(response);
      GPS_Data *gpsData = unpackData(unpackedData);

      UART_USB.print(F("Received RF Data: "));
      printGPSData(gpsData);

      free(unpackedData);
      free(gpsData);
    }
  }
#endif
  delay(10);
}