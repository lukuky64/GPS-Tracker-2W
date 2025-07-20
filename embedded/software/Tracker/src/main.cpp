#include <Arduino.h>

#include "GPS_Talker.hpp"
#include "definitions.hpp"

// UART Mapping for RP2040:
// Serial1 -> UART0 -> Pins 12(TX)/13(RX) -> Used for RF Module
// Serial2 -> UART1 -> Pins 20(TX)/21(RX) -> Used for GPS Module

GPS_Talker gpsTalker(UART_GPS, GPS_NRST, GPS_TIMEPULSE, LED1_PIN);

GPS_Data *gpsData = nullptr;

unsigned long previousMillis = 0;
unsigned long intervalMillis = 500;

void setup() {
  bool success = false;
  UART_USB.begin(115200);
  UART_USB.println("Attempting to initialise GPS...");

  while (!success) {
    if (!gpsTalker.begin(GPS_BAUD_RATE, GPS_NAV_FREQ)) {
      UART_USB.println("GPS initialisation failed! Trying again...");
      gpsTalker.hardwareReset();  // Reset the GPS module
      delay(500);
    } else {
      UART_USB.println("GPS initialised successfully!");
      success = true;
    }
  }
}

void loop() {
  if (gpsTalker.checkNewData()) {
    gpsData = gpsTalker.getData();
    // Print GPS data to USB Serial
    UART_USB.print("Latitude: ");
    UART_USB.print(gpsData->latitude);
    UART_USB.print("\t");
    UART_USB.print("Longitude: ");
    UART_USB.print(gpsData->longitude);
    UART_USB.print("\t");
    UART_USB.print("Altitude: ");
    UART_USB.println(gpsData->altitude);
  }
  delay(10);
}
