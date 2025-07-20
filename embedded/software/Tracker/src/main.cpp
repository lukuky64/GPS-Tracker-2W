#include <Arduino.h>
#include <FreeRTOS.h>
#include <HardwareSerial.h>
#include <task.h>

#include "BattMonitor.hpp"
#include "GPS_Talker.hpp"
#include "RF_Talker.hpp"
#include "definitions.hpp"
#include "messages.hpp"

// UART UART_RF(UART_RF_TX, UART_RF_RX);
// UART UART_GPS(UART_GPS_TX, UART_GPS_RX);

GPS_Talker gpsTalker(UART_GPS, GPS_NRST, GPS_TIMEPULSE, LED1_PIN);
RF_Talker rfTalker(UART_RF, RF_CTRL0, RF_CTRL1, RF_STATUS);
BattMonitor batteryMonitor(VBAT_SENSE, BATT_SCALE_FACTOR);

void setup() {
  bool success = false;
  UART_USB.begin(115200);

  UART_RF.setRX(UART_RF_RX);
  UART_RF.setTX(UART_RF_TX);

  UART_GPS.setRX(UART_GPS_RX);
  UART_GPS.setTX(UART_GPS_TX);

  batteryMonitor.init();

  while (!success) {
    // TODO: We could collect samples at a higher rate and average them
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