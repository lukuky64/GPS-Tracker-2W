
#include "RF_Talker.hpp"

RF_Talker::RF_Talker(HardwareSerial& serial, uint8_t ctrl0Pin, uint8_t ctrl1Pin, uint8_t statusPin) : m_RF_Serial(serial), m_ctrl0Pin(ctrl0Pin), m_ctrl1Pin(ctrl1Pin), m_statusPin(statusPin) {
  m_e22Module = new LoRa_E22(&m_RF_Serial, m_statusPin, m_ctrl0Pin, m_ctrl1Pin);
  m_rfConfig = {0xFF, 0xFF, 23};  // Broadcast address and channel 23
}

bool RF_Talker::begin() {
  if (m_e22Module->begin()) {
    ResponseStructContainer c;
    c = m_e22Module->getConfiguration();
    // It's important get configuration pointer before all other operation
    Configuration configuration = *(Configuration*)c.data;

    // TODO: Play with these settings for range
    configuration.OPTION.transmissionPower = POWER_10;      // POWER_22;
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