#include <SparkFun_u-blox_GNSS_v3.h>
SFE_UBLOX_GNSS_SERIAL myGNSS;

#define mySerial Serial2  // Use Serial2 for UART

void setup() {
  delay(2000);
  Serial.begin(115200);
  Serial.println("SparkFun u-blox GNSS Auto PVT Example (UART)");

  mySerial.setRX(21);
  mySerial.setTX(20);
  mySerial.begin(115200);

  // myGNSS.enableDebugging(); // Uncomment to enable debug messages

  while (myGNSS.begin(mySerial) == false) {
    Serial.println(F("u-blox GNSS not detected. Please check wiring. Retrying..."));
    delay(1000);
  }

  myGNSS.setSerialRate(115200);
  mySerial.begin(115200);

  myGNSS.setUART1Output(COM_TYPE_UBX);                // Output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);  // Save port settings

  myGNSS.setNavigationFrequency(20);  // Two solutions per second
  //   myGNSS.setAutoPVT(true);           // Output PVT automatically
  // myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_NAV); // Optional: Save navigation settings
}

void loop() {
  // Request (poll) the position, velocity and time (PVT) information.
  // The module only responds when a new position is available. Default is once per second.
  // getPVT() returns true when new data is received.
  if (myGNSS.getPVT() == true) {
    int32_t latitude = myGNSS.getLatitude();
    Serial.print(F("Lat: "));
    Serial.print(latitude);

    int32_t longitude = myGNSS.getLongitude();
    Serial.print(F(" Long: "));
    Serial.print(longitude);
    Serial.print(F(" (degrees * 10^-7)"));

    int32_t altitude = myGNSS.getAltitudeMSL();  // Altitude above Mean Sea Level
    Serial.print(F(" Alt: "));
    Serial.print(altitude);
    Serial.print(F(" (mm)"));

    // print number of satellites used in the solution
    uint8_t numSV = myGNSS.getSIV();
    Serial.print(F(" NumSV: "));
    Serial.print(numSV);

    Serial.println();
  }
}
