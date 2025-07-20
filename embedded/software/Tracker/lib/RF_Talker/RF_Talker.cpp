
#include "RF_Talker.hpp"

RF_Talker::RF_Talker(HardwareSerial& serial, uint8_t ctrl0Pin, uint8_t ctrl1Pin, uint8_t statusPin, uint8_t ledPin) : m_RF_Serial(serial), m_ctrl0Pin(ctrl0Pin), m_ctrl1Pin(ctrl1Pin), m_statusPin(statusPin), m_LED_Pin(ledPin) {
  m_e22Module = new LoRa_E22(&m_RF_Serial, m_statusPin, m_ctrl0Pin, m_ctrl1Pin);
  m_rfConfig = {0xFF, 0xFF, 83};  // Broadcast address and channel 83  (933MHz -> lowest return loss for antenna selected)
}

bool RF_Talker::begin() {
  setupLED(m_LED_Pin);

  if (m_e22Module->begin()) {
    ResponseStructContainer c;
    c = m_e22Module->getConfiguration();
    // It's important get configuration pointer before all other operation
    Configuration configuration = *(Configuration*)c.data;

    // TODO: Play with these settings for improved range
    configuration.OPTION.transmissionPower = POWER_10;      // POWER_22; We are using a 33dB module. POWER_22 will map to 33dB, I think
    configuration.SPED.airDataRate = AIR_DATA_RATE_010_24;  // lower is better for long range
    ResponseStatus rs = m_e22Module->setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);

    if (rs.code != E22_SUCCESS) {
      UART_USB.print("Error setting configuration: ");
      UART_USB.println(rs.getResponseDescription());
      return false;
    }
  }
  return true;
}

bool RF_Talker::sendMessage(const void* message) {
  ResponseStatus rs = m_e22Module->sendFixedMessage(m_rfConfig.m_ADDH, m_rfConfig.m_ADDL, m_rfConfig.m_CHAN, &message, sizeof(MSG_PACKET));
  if (rs.code != E22_SUCCESS) {
    UART_USB.print("Error sending message: ");
    UART_USB.println(rs.getResponseDescription());
    return false;
  }
  toggleLED(m_LED_Pin);
  return true;
}

bool RF_Talker::available() {
  if (m_e22Module->available() > 1) {
    return true;
  }
  return false;
}

bool RF_Talker::receiveMessage(ResponseContainer& response) {
  response = m_e22Module->receiveMessage();
  if (response.status.code != E22_SUCCESS) {
    UART_USB.print("Error receiving message: ");
    UART_USB.println(response.status.getResponseDescription());
    return false;
  }
  return true;
}

void RF_Talker::setupLED(uint8_t ledPin) {
  if (ledPin != 255) {
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);  // Start with LED off
  }
}

void RF_Talker::toggleLED(uint8_t ledPin) {
  if (ledPin != 255) {
    digitalWrite(ledPin, !digitalRead(ledPin));
  }
}