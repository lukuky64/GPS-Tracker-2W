#include <Arduino.h>

#include "Control.hpp"
#include "hardware/clocks.h"
#include "pico/stdlib.h"

Control control;

void setup() {
  // overlcock to 250MHz
  set_sys_clock_khz(250'000, true);
  delay(2000);
  if (!control.setup()) control.restartDevice();
  delay(100);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(10000));  // Main loop does nothing
}

// #include <SparkFun_u-blox_GNSS_v3.h>
// SFE_UBLOX_GNSS_SERIAL myGNSS;

// #define mySerial Serial2  // Use Serial2 for UART

// void printPVTdata(UBX_NAV_PVT_data_t *ubxDataStruct) {
//   Serial.println();

//   Serial.print(F("Time: "));           // Print the time
//   uint8_t hms = ubxDataStruct->hour;   // Print the hours
//   if (hms < 10) Serial.print(F("0"));  // Print a leading zero if required
//   Serial.print(hms);
//   Serial.print(F(":"));
//   hms = ubxDataStruct->min;            // Print the minutes
//   if (hms < 10) Serial.print(F("0"));  // Print a leading zero if required
//   Serial.print(hms);
//   Serial.print(F(":"));
//   hms = ubxDataStruct->sec;            // Print the seconds
//   if (hms < 10) Serial.print(F("0"));  // Print a leading zero if required
//   Serial.print(hms);
//   Serial.print(F("."));
//   unsigned long millisecs = ubxDataStruct->iTOW % 1000;  // Print the milliseconds
//   if (millisecs < 100) Serial.print(F("0"));             // Print the trailing zeros correctly
//   if (millisecs < 10) Serial.print(F("0"));
//   Serial.print(millisecs);

//   long latitude = ubxDataStruct->lat;  // Print the latitude
//   Serial.print(F(" Lat: "));
//   Serial.print(latitude);

//   long longitude = ubxDataStruct->lon;  // Print the longitude
//   Serial.print(F(" Long: "));
//   Serial.print(longitude);
//   Serial.print(F(" (degrees * 10^-7)"));

//   long altitude = ubxDataStruct->hMSL;  // Print the height above mean sea level
//   Serial.print(F(" Height above MSL: "));
//   Serial.print(altitude);
//   Serial.println(F(" (mm)"));
// }

// void setup() {
//   delay(2000);
//   Serial.begin(115200);
//   Serial.println("SparkFun u-blox GNSS Auto PVT Example (UART)");

//   // pinMode(2, OUTPUT);
//   // digitalWrite(2, HIGH);  // Set the reset pin high to disable reset

//   // Configure Serial2 pins for Raspberry Pi Pico
//   mySerial.setRX(21);
//   mySerial.setTX(20);
//   mySerial.begin(250000);

//   myGNSS.enableDebugging();  // Uncomment to enable debug messages

//   while (myGNSS.begin(mySerial) == false) {
//     Serial.println(F("u-blox GNSS not detected. Please check wiring. Retrying..."));
//     delay(1000);
//   }

//   // delay(100);                    // set to desired baud rate
//   // myGNSS.setSerialRate(115200);  // Set the serial port to the new baud rate

//   // // Restart serial connection at new baud rate
//   // mySerial.end();
//   // mySerial.begin(115200);
//   // delay(100);

//   Serial.println("GNSS connected successfully!");

//   myGNSS.setUART1Output(COM_TYPE_UBX);  // Output UBX only (turn off NMEA noise)
//   // myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);  // Save port settings

//   myGNSS.setNavigationFrequency(20);            // Two solutions per second
//   myGNSS.setAutoPVTcallbackPtr(&printPVTdata);  // Enable automatic NAV PVT messages with callback to printPVTdata
// }

// void loop() {
//   myGNSS.checkUblox();      // Check for the arrival of new data and process it.
//   myGNSS.checkCallbacks();  // Check if any callbacks are waiting to be processed.

//   Serial.print(".");
//   delay(50);
// }
