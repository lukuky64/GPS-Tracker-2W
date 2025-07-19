#include "GPS_Talker.hpp"


GPS_Talker::GPS_Talker(HardwareSerial& serial, uint8_t nrstPin) :m_GPS_Serial(serial), m_nRST_Pin(nrstPin) {
    // test
    // test
}

bool GPS_Talker::begin() {

    pinMode(m_nRST_Pin, OUTPUT);
    digitalWrite(m_nRST_Pin, HIGH); // Set the reset pin high to disable reset

    m_GPS_Serial.begin(m_baudRate);

    if (!m_GPS_Module.begin(m_GPS_Serial)) return false;
    return m_GPS_Module.setUART1Output(COM_TYPE_UBX); //Set the UART1 port to output UBX only (turn off NMEA noise)
}

int32_t GPS_Talker::getLatitude() {
    return m_GPS_Module.getLatitude();
}

int32_t GPS_Talker::getLongitude() {
    return m_GPS_Module.getLongitude();
}

int32_t GPS_Talker::getAltitude() {
    return m_GPS_Module.getAltitudeMSL(); // Altitude above Mean Sea Level
}

void GPS_Talker::hardwareReset()
{
    digitalWrite(m_nRST_Pin, LOW); // active LOW reset
    delay(100); // FIXME: This is arbitrary
    digitalWrite(m_nRST_Pin, HIGH);
}

void GPS_Talker::timePulseCallback()
{
    // setup callback function
}