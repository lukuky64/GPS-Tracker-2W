#include "GPS_Talker.hpp"

GPS_Talker* GPS_Talker::m_instance = nullptr;  // Initialize static member

GPS_Talker::GPS_Talker(HardwareSerial& serial, uint8_t nrstPin, uint8_t timePulsePin, uint8_t ledPin) : m_GPS_Serial(serial), m_nRST_Pin(nrstPin), m_timePulse_Pin(timePulsePin), m_LED_Pin(ledPin), m_newData(false), m_timePulseFlag(false), m_initialised(false) { m_instance = this; }

bool GPS_Talker::begin(unsigned long baudRate, uint8_t navFreq) {
  m_baudRate = baudRate;
  m_navFreq = navFreq;

  pinMode(m_nRST_Pin, OUTPUT);
  digitalWrite(m_nRST_Pin, HIGH);  // Set the reset pin high to disable reset

  pinMode(m_timePulse_Pin, INPUT);  // Set the time pulse pin as input

  // setup indication ISR for time pulse
  {
    if (m_LED_Pin != 255) {
      pinMode(m_LED_Pin, OUTPUT);
      digitalWrite(m_LED_Pin, LOW);  // Start with LED off
    }

    if (m_timePulse_Pin != 255) {
      // Attach interrupt for time pulse pin (rising edge)
      attachInterrupt(digitalPinToInterrupt(m_timePulse_Pin), timePulseISR, RISING);
    }
  }

  m_GPS_Serial.begin(m_baudRate);

  if (!m_GPS_Module.begin(m_GPS_Serial)) return false;  // Connect to the u-blox module

  if (!m_GPS_Module.setNavigationFrequency(m_navFreq)) return false;  // Produce #N solutions per second

  if (!m_GPS_Module.setUART1Output(COM_TYPE_UBX)) return false;  // Set the UART1 port to output UBX only (turn off NMEA noise)

  if (!m_GPS_Module.setAutoPVTcallbackPtr(PVTCallback)) return false;  // Set the callback for PVT messages

  m_initialised = true;
  return m_initialised;
}

void GPS_Talker::PVTCallback(UBX_NAV_PVT_data_t* ubxDataStruct) {
  if (ubxDataStruct->flags.bits.gnssFixOK == 1) {
    m_instance->m_gpsData.latitude = ubxDataStruct->lat;   // Latitude in degrees * 1e-7
    m_instance->m_gpsData.longitude = ubxDataStruct->lon;  // Longitude in degrees * 1e-7
    m_instance->m_gpsData.altitude = ubxDataStruct->hMSL;  // Altitude above Mean Sea Level in mm

    // time in UTC
    m_instance->m_gpsData.hour = ubxDataStruct->hour;
    m_instance->m_gpsData.minute = ubxDataStruct->min;
    m_instance->m_gpsData.second = ubxDataStruct->sec;

    m_instance->m_newData = true;
  }
}

void GPS_Talker::timePulseISR() {
  if (m_instance) {
    m_instance->m_timePulseFlag = true;  // Set the time pulse flag

    // Turn on LED if pin is configured
    if (m_instance->m_LED_Pin != 255) {
      digitalWrite(m_instance->m_LED_Pin, !digitalRead(m_instance->m_LED_Pin));  // Turn LED on
    }
  }
}

int32_t GPS_Talker::getLatitude() { return m_GPS_Module.getLatitude(); }

int32_t GPS_Talker::getLongitude() { return m_GPS_Module.getLongitude(); }

int32_t GPS_Talker::getAltitude() {
  return m_GPS_Module.getAltitudeMSL();  // Altitude above Mean Sea Level
}

void GPS_Talker::hardwareReset() {
  digitalWrite(m_nRST_Pin, LOW);  // active LOW reset
  delay(100);                     // FIXME: This is arbitrary
  digitalWrite(m_nRST_Pin, HIGH);
}

// this should be called regularly
bool GPS_Talker::checkNewData() {
  m_GPS_Module.checkUblox();      // Check for the arrival of new data and process it
  m_GPS_Module.checkCallbacks();  // Check if any callbacks are waiting to be processed
  return m_newData;               // Return true if new data is available
}

GPS_Data* GPS_Talker::getData() {
  m_newData = false;
  return &m_gpsData;  // Return last data
}
