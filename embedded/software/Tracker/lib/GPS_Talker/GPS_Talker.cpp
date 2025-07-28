#include "GPS_Talker.hpp"

GPS_Talker* GPS_Talker::m_instance = nullptr;  // Initialize static member

GPS_Talker::GPS_Talker(HardwareSerial& serial, uint8_t nrstPin, uint8_t timePulsePin, uint8_t ledPin) : m_GPS_Serial(serial), m_nRST_Pin(nrstPin), m_timePulse_Pin(timePulsePin), m_LED_Pin(ledPin), m_newData(false), m_initialised(false) { m_instance = this; }

bool GPS_Talker::begin(unsigned long baudRate, uint8_t navFreq) {
  m_baudRate = baudRate;
  m_navFreq = navFreq;

  pinMode(m_nRST_Pin, OUTPUT);
  digitalWrite(m_nRST_Pin, HIGH);  // Set the reset pin high to disable reset

  pinMode(m_timePulse_Pin, INPUT);  // Set the time pulse pin as input

  setupLED(m_LED_Pin);  // Setup LED if configured

  if (m_timePulse_Pin != 255) {
    // Attach interrupt for time pulse pin (rising edge)
    attachInterrupt(digitalPinToInterrupt(m_timePulse_Pin), timePulseISR, RISING);
  }

  m_GPS_Serial.begin(m_baudRate);

  if (!m_GPS_Module.begin(m_GPS_Serial)) return false;  // Connect to the u-blox module

  // m_GPS_Module.setSerialRate(115200);
  // m_GPS_Serial.begin(115200);

  if (!m_GPS_Module.setNavigationFrequency(m_navFreq)) return false;  // Produce #N solutions per second

  if (!m_GPS_Module.setUART1Output(COM_TYPE_UBX)) return false;  // Set the UART1 port to output UBX only (turn off NMEA noise)

  // if (!m_GPS_Module.setAutoPVTcallbackPtr(PVTCallback)) return false;  // Set the callback for PVT messages // NOTE: This doesn't seem to work ATM

  m_initialised = true;
  return m_initialised;
}

// void GPS_Talker::PVTCallback(UBX_NAV_PVT_data_t* ubxDataStruct) {
//   UART_USB.println(F("PVT Callback triggered. Processing data..."));
//   if (ubxDataStruct->flags.bits.gnssFixOK == 1) {
//     m_instance->m_gpsData.latitude = ubxDataStruct->lat * 1e-7f;     // Latitude in degrees (float)
//     m_instance->m_gpsData.longitude = ubxDataStruct->lon * 1e-7f;    // Longitude in degrees (float)
//     m_instance->m_gpsData.altitude = ubxDataStruct->hMSL / 1000.0f;  // Altitude above Mean Sea Level in meters (float)

//     // time in UTC
//     m_instance->m_gpsData.hour = ubxDataStruct->hour;
//     m_instance->m_gpsData.minute = ubxDataStruct->min;
//     m_instance->m_gpsData.second = ubxDataStruct->sec;

//     m_instance->m_newData = true;
//   }
// }

void GPS_Talker::timePulseISR() {
  if (m_instance) {
    m_instance->toggleLED(m_instance->m_LED_Pin);  // Toggle the LED if it is configured
  }
}

int32_t GPS_Talker::getLatitude() { return m_GPS_Module.getLatitude(); }

int32_t GPS_Talker::getLongitude() { return m_GPS_Module.getLongitude(); }

int32_t GPS_Talker::getAltitude() {
  return m_GPS_Module.getAltitudeMSL();  // Altitude above Mean Sea Level
}

void GPS_Talker::hardwareReset() {
  digitalWrite(m_nRST_Pin, LOW);  // active LOW reset
  delay(100);                     // TODO: This is arbitrary
  digitalWrite(m_nRST_Pin, HIGH);
}

// this should be called regularly
bool GPS_Talker::checkNewData() {
  if (m_GPS_Module.getPVT() == true) {
    if (m_GPS_Module.getSIV() > 0) {
      m_gpsData.latitude = m_GPS_Module.getLatitude() * 1e-7f;
      m_gpsData.longitude = m_GPS_Module.getLongitude() * 1e-7f;
      m_gpsData.altitude = m_GPS_Module.getAltitudeMSL() / 1000.0f;
      m_gpsData.hour = m_GPS_Module.getHour();
      m_gpsData.minute = m_GPS_Module.getMinute();
      m_gpsData.second = m_GPS_Module.getSecond();

      // printGPSData(&m_gpsData);
      m_newData = true;
    }
  }
  return m_newData;
}

GPS_DATA GPS_Talker::getData() {
  m_newData = false;
  return m_gpsData;  // Return last data
}

void GPS_Talker::setupLED(uint8_t ledPin) {
  if (ledPin != 255) {
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);  // Start with LED off
  }
}

void GPS_Talker::toggleLED(uint8_t ledPin) {
  if (ledPin != 255) {
    digitalWrite(ledPin, !digitalRead(ledPin));
  }
}

void GPS_Talker::printModuleInfo() {
  UART_USB.print(F("Module Name: "));
  UART_USB.println(m_GPS_Module.getModuleName());
}