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

  // try last baud rate first
  m_GPS_Serial.begin(m_baudRate);
  if (!m_GPS_Module.begin(m_GPS_Serial)) {
    // try default baud rate first
    m_GPS_Serial.begin(9600);
    if (!m_GPS_Module.begin(m_GPS_Serial)) {
      // try different rate if fails
      m_GPS_Serial.begin(115200);
      if (!m_GPS_Module.begin(m_GPS_Serial)) {
        UART_USB.println(F("Failed to connect to GPS module with default baud rates!"));
        return false;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));  // Give the module time to change baud rate
    // set to desired baud rate
    m_GPS_Module.setSerialRate(m_baudRate);  // Set the serial port to the new baud rate

    // Restart serial connection at new baud rate
    m_GPS_Serial.end();
    m_GPS_Serial.begin(m_baudRate);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Verify connection at new baud rate
    if (!m_GPS_Module.isConnected()) {
      UART_USB.println(F("Failed to reconnect at new baud rate!"));
      return false;
    }

    m_GPS_Module.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);  // Save port settings
  }

  if (!m_GPS_Module.setNavigationFrequency(m_navFreq)) return false;  // Produce #N solutions per second
  if (!m_GPS_Module.setUART1Output(COM_TYPE_UBX)) return false;       // Set the UART1 port to output UBX only (turn off NMEA noise)
  // if (!m_GPS_Module.setAutoPVTcallbackPtr(&GPS_Talker::loadPVTData)) return false;

  // if (!m_GPS_Module.setAutoPVTcallbackPtr(PVTCallback)) return false;  // Set the callback for PVT messages // NOTE: This doesn't seem to work ATM

  m_initialised = true;
  return m_initialised;
}

// Print PVT data callback function
void GPS_Talker::loadPVTData(UBX_NAV_PVT_data_t* ubxDataStruct) {
  if (ubxDataStruct->flags.bits.gnssFixOK == 1 && m_instance) {
    m_instance->m_gpsData.latitude = ubxDataStruct->lat * 1e-7f;     // Latitude in degrees (float)
    m_instance->m_gpsData.longitude = ubxDataStruct->lon * 1e-7f;    // Longitude in degrees (float)
    m_instance->m_gpsData.altitude = ubxDataStruct->hMSL / 1000.0f;  // Altitude above Mean Sea Level in meters (float)
    m_instance->m_newData = true;
  }
}

void GPS_Talker::timePulseISR() {
  if (m_instance) {
    m_instance->toggleLED();  // Toggle the LED if it is configured
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
  // BUG: this function is using ~50ms total.
  unsigned long startMicros;  // Start timing
  unsigned long elapsedMicros = 0;

  startMicros = micros();  // Start timing
  if (m_GPS_Module.getPVT()) {
    auto* pvt = m_GPS_Module.packetUBXNAVPVT;  // or similar, check actual member name

    // static uint32_t last_iTOW = 0;
    // uint32_t iTOW = pvt->data.iTOW;
    // UART_USB.printf("iTOW: %lu, dt: %lu ms\n", iTOW, iTOW - last_iTOW);
    // last_iTOW = iTOW;

    uint8_t SIV = pvt->data.numSV;  // Get the number of satellites in view
    if (SIV > 0) {
      m_gpsData.latitude = pvt->data.lat * 1e-7f;
      m_gpsData.longitude = pvt->data.lon * 1e-7f;
      m_gpsData.altitude = pvt->data.hMSL / 1000.0f;
      m_gpsData.nFixes = SIV;

      // printGPSData(&m_gpsData);
      m_newData = true;
    }
  }
  elapsedMicros = micros() - startMicros;  // Calculate elapsed time
  // UART_USB.printf("%lu us\n", elapsedMicros);
  return m_newData;
}

GPS_DATA GPS_Talker::getData() {
  m_newData = false;
  return m_gpsData;  // Return last data
}

void GPS_Talker::setupLED(uint8_t ledPin) {
  m_LED_Pin = ledPin;
  if (m_LED_Pin != 255) {
    pinMode(m_LED_Pin, OUTPUT);
    digitalWrite(m_LED_Pin, LOW);  // Start with LED off
  }
}

void GPS_Talker::toggleLED() {
  if (m_LED_Pin != 255) {
    digitalWrite(m_LED_Pin, !digitalRead(m_LED_Pin));
  }
}

void GPS_Talker::setLED(bool state) {
  if (m_LED_Pin != 255) {
    digitalWrite(m_LED_Pin, state);
  }
}

void GPS_Talker::printModuleInfo() {
  UART_USB.print(F("Module Name: "));
  UART_USB.println(m_GPS_Module.getModuleName());
}