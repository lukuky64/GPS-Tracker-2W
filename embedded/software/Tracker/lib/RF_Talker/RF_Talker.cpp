#include "RF_Talker.hpp"

RF_Talker::RF_Talker(HardwareSerial& serial, uint8_t ctrl0Pin, uint8_t ctrl1Pin, uint8_t statusPin, uint8_t ledPin) : m_RF_Serial(serial), m_ctrl0Pin(ctrl0Pin), m_ctrl1Pin(ctrl1Pin), m_statusPin(statusPin), m_LED_Pin(ledPin) {
  m_e22Module = new LoRa_E22(&m_RF_Serial, m_statusPin, m_ctrl0Pin, m_ctrl1Pin);
  m_rfConfig = {0xFF, 0xFF, 65};  // Broadcast address and channel 65  (850.125 + 65 = 915.125 MHz -> lowest return loss for antenna selected). Looks like Chan = 80 is the upper limit (930.125 MHz)
  m_rfPower = 24;                 // Default RF power level
}

bool RF_Talker::begin() {
  setupLED(m_LED_Pin);

  if (!m_e22Module->begin()) {
    UART_USB.println("E22 module begin() failed");
    return false;
  }

  ResponseStructContainer c = m_e22Module->getConfiguration();
  if (c.status.code != E22_SUCCESS) {
    UART_USB.print("Error getting configuration: ");
    UART_USB.println(c.status.getResponseDescription());
    return false;
  }
  Configuration configuration = *(Configuration*)c.data;

  configuration.OPTION.transmissionPower = mapRFPower(m_rfPower);
  configuration.SPED.airDataRate = AIR_DATA_RATE_010_24;  // AIR_DATA_RATE_111_625;  // lower is better for long range. 2.4kbps
  configuration.CHAN = m_rfConfig.m_CHAN;
  configuration.TRANSMISSION_MODE.enableRSSI = RSSI_ENABLED;

  ResponseStatus rs = m_e22Module->setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);

  if (rs.code == E22_SUCCESS) {
    return true;
  } else {
    UART_USB.print("Error setting configuration: ");
    UART_USB.println(rs.getResponseDescription());
    return false;
  }
}

bool RF_Talker::sendMessage(const void* message) {
  ResponseStatus rs = m_e22Module->sendFixedMessage(m_rfConfig.m_ADDH, m_rfConfig.m_ADDL, m_rfConfig.m_CHAN, message, sizeof(MSG_PACKET));
  if (rs.code != E22_SUCCESS) {
    UART_USB.print("Error sending message: ");
    UART_USB.println(rs.getResponseDescription());
    return false;
  }
  toggleLED();
  return true;
}

bool RF_Talker::available() {
  if (m_e22Module->available() > 1) {
    return true;
  }
  return false;
}

bool RF_Talker::receiveMessage(ResponseContainer& response) {
  response = m_e22Module->receiveMessageRSSI();
  if (response.status.code != E22_SUCCESS) {
    UART_USB.print("Error receiving message: ");
    UART_USB.println(response.status.getResponseDescription());
    return false;
  } else {
    // FIX, values are either -127 (lowest value) or 0 (max value).
    // print RSSI
    UART_USB.print("Received message with RSSI: ");
    UART_USB.print(-(256 - (int)response.rssi));
    UART_USB.println(" dBm");
  }
  return true;
}

void RF_Talker::setupLED(uint8_t ledPin) {
  m_LED_Pin = ledPin;
  if (m_LED_Pin != 255) {
    pinMode(m_LED_Pin, OUTPUT);
    digitalWrite(m_LED_Pin, LOW);  // Start with LED off
  }
}

void RF_Talker::toggleLED() {
  if (m_LED_Pin != 255) {
    digitalWrite(m_LED_Pin, !digitalRead(m_LED_Pin));
  }
}

void RF_Talker::setLED(bool state) {
  if (m_LED_Pin != 255) {
    digitalWrite(m_LED_Pin, state);
  }
}

int8_t RF_Talker::setRFPower(int8_t power) {
  // Update the configuration
  ResponseStructContainer c;
  c = m_e22Module->getConfiguration();
  // It's important get configuration pointer before all other operation
  Configuration configuration = *(Configuration*)c.data;

  configuration.OPTION.transmissionPower = mapRFPower(power);

  ResponseStatus rs = m_e22Module->setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);  // saving if powered down. probably good incase module has to be reset after lockout

  if (rs.code != E22_SUCCESS) {
    UART_USB.print("Error setting RF power: ");
    UART_USB.println(rs.getResponseDescription());
  } else {
    m_rfPower = power;
  }

  return m_rfPower;
}

int RF_Talker::mapRFPower(int8_t& power) {
  int powerMapped;
  if (power <= 24) {
    powerMapped = POWER_24;
    power = 24;
  } else if (power <= 27) {
    powerMapped = POWER_27;
    power = 27;
  } else if (power <= 30) {
    powerMapped = POWER_30;
    power = 30;
  } else if (power <= 33) {
    powerMapped = POWER_33;
    power = 33;
  } else {
    powerMapped = POWER_24;
    return 24;
  }
  return powerMapped;
}

void RF_Talker::restart() {
  return;
  // m_e22Module->restart(); // this is not implemented in the library
}
