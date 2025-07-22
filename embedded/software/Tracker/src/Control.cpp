#include "Control.hpp"

#define MAX_SETUP_RETRIES 3
#define MAX_LOCKOUT_RST pdMS_TO_TICKS(300'000)  // 5 minutes

Control::Control() : gpsTalker(UART_GPS, GPS_NRST, GPS_TIMEPULSE, LED1_PIN), rfTalker(UART_RF, RF_CTRL0, RF_CTRL1, RF_STATUS, LED2_PIN), batteryMonitor(VBAT_SENSE, BATT_SCALE_FACTOR), GPSFilter(GPS_FILTER_ALPHA, 3), initialised(false) {
  RFBroadcast_MS = 1000 / RF_BROADCAST_FREQ;
  GPSAquisition_MS = 1000 / GPS_UPDATE_FREQ;
  lastGPSUpdateTick = xTaskGetTickCount();
  rfBlast = false;
}

bool Control::setup() {
  UART_USB.begin(115200);

  UART_RF.setRX(UART_RF_RX);
  UART_RF.setTX(UART_RF_TX);

  UART_GPS.setRX(UART_GPS_RX);
  UART_GPS.setTX(UART_GPS_TX);

  batteryMonitor.init();

  initialised = setupRF();
  if (initialised) {
    initialised = setupGPS();
  }
  if (initialised) beginTasks();
  return initialised;
}

bool Control::setupRF() {
  uint8_t setupRetries = 0;
  bool success = false;
  while (!success && setupRetries < MAX_SETUP_RETRIES) {
    if (!rfTalker.begin()) {
      UART_USB.println(F("RF initialisation failed! Trying again..."));
      // rfTalker.hardwareReset();  // Reset the RF module. This is not implemented
      vTaskDelay(pdMS_TO_TICKS(500));
    } else {
      UART_USB.println(F("RF initialised successfully!"));
      success = true;
    }
    setupRetries++;
  }
  return success;
}

bool Control::setupGPS() {
  uint8_t setupRetries = 0;
  bool success = false;
  while (!success && setupRetries < MAX_SETUP_RETRIES) {
    if (!gpsTalker.begin(GPS_BAUD_RATE, GPS_UPDATE_FREQ)) {
      UART_USB.println(F("GPS initialisation failed! Trying again..."));
      gpsTalker.hardwareReset();  // Reset the GPS module
      vTaskDelay(pdMS_TO_TICKS(500));
    } else {
      UART_USB.println(F("GPS initialised successfully!"));
      success = true;
    }
    setupRetries++;
  }
  return success;
}

void Control::beginTasks() {
  xTaskCreate([](void *param) { static_cast<Control *>(param)->GPS_aquisition_task(); }, "GPS_Aquisition", 8192, this, 2, &GPSTaskHandle);
  xTaskCreate([](void *param) { static_cast<Control *>(param)->RF_aquisition_task(); }, "RF_Aquisition", 8192, this, 2, &RFTaskHandle);
  xTaskCreate([](void *param) { static_cast<Control *>(param)->RF_broadcast_task(); }, "RF_Broadcast", 8192, this, 2, &RFBroadcastTaskHandle);
  xTaskCreate([](void *param) { static_cast<Control *>(param)->GPS_lockout_watchdog_task(); }, "GPS_Watchdog", 2048, this, 1, nullptr);
  xTaskCreate([](void *param) { static_cast<Control *>(param)->Rocket_state_task(); }, "Rocket_State", 2048, this, 1, nullptr);
}

void Control::GPS_aquisition_task() {
  while (true) {
    if (gpsTalker.checkNewData()) {
      SemaphoreGuard gpsGuard(thisData.GPSDataMutex);
      if (gpsGuard.acquired()) {
        thisData.GPSData = gpsTalker.getData();  // get new data

        GPSFilter.processSample({thisData.GPSData.altitude, thisData.GPSData.latitude, thisData.GPSData.longitude});  // process new data
        auto filtered = GPSFilter.getFilteredData();

        // asign filtered data
        thisData.GPSData.altitude = filtered[0];
        thisData.GPSData.latitude = filtered[1];
        thisData.GPSData.longitude = filtered[2];

        lastGPSUpdateTick = xTaskGetTickCount();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(GPSAquisition_MS));
  }
}

void Control::RF_broadcast_task() {
  while (true) {
    updateSYSData();

    MSG_PACKET msgPacket;
    SemaphoreGuard gpsGuard(thisData.GPSDataMutex);
    SemaphoreGuard sysGuard(thisData.SYSDataMutex);
    if (gpsGuard.acquired() && sysGuard.acquired()) {
      packData(&msgPacket, &thisData.GPSData, &thisData.SYSData);
    }
    // Send the packed data via RF
    if (rfTalker.sendMessage(&msgPacket)) {
      UART_USB.println(F("RF message sent successfully!"));
      printGPSData(&thisData.GPSData);
      printSYSData(&thisData.SYSData);
    } else {
      UART_USB.println(F("Failed to send RF message."));
    }
    vTaskDelay(pdMS_TO_TICKS(RFBroadcast_MS));
  }
}

void Control::RF_aquisition_task() {
  while (true) {
    if (rfTalker.available()) {
      ResponseContainer response;
      if (rfTalker.receiveMessage(response)) {
        MSG_PACKET unpackedData;
        unpackResponseData(&unpackedData, response);

        SemaphoreGuard gpsGuard(thatData.GPSDataMutex);
        SemaphoreGuard sysGuard(thatData.SYSDataMutex);
        if (gpsGuard.acquired() && sysGuard.acquired()) {
          unpackGPSData(&thatData.GPSData, &unpackedData);
          unpackSYSData(&thatData.SYSData, &unpackedData);

          UART_USB.print(F("Received RF Data: "));
          printGPSData(&thatData.GPSData);
          printSYSData(&thatData.SYSData);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));  // TODO: This is arbitrary, but should be sufficient.
  }
}

void Control::updateSYSData() {
  SemaphoreGuard sysGuard(thisData.SYSDataMutex);
  if (sysGuard.acquired()) {
    thisData.SYSData.batteryVoltage = batteryMonitor.getScaledVoltage();
    thisData.SYSData.rfPower = rfTalker.getRFPower();
  }
}

void Control::GPS_lockout_watchdog_task() {
  while (true) {
    uint32_t now = xTaskGetTickCount();
    if ((now - lastGPSUpdateTick) > MAX_LOCKOUT_RST) {
      UART_USB.println(F("GPS lockout detected! Restarting GPS module..."));
      gpsTalker.hardwareReset();  // Reset the GPS module
    }
    vTaskDelay(pdMS_TO_TICKS(10'000));  // Check every 10 seconds
  }
}

void Control::restartDevice() {
  watchdog_enable(1, 1);  // 1 ms timeout, reset on timeout
  while (true) {
  }  // Wait for watchdog to reset
}

void Control::Rocket_state_task() {
  // We want to read the altitude and check if its above a threshold. this value shouldn't change
  // back after rocket returns to ground and reads below the threshold. We want to start blasting RF
  // when threshold is reached and continue blasting until the rocket is recovered.
  while (true) {
    SemaphoreGuard gpsGuard(thisData.GPSDataMutex);
    if (gpsGuard.acquired()) {
      // TODO: This doesn't account for the starting altitude
      if (thisData.GPSData.altitude > ALTITUDE_THRESHOLD) {
        rfBlast = true;
        rfTalker.setRFPower(22);  // Set RF power to high for blasting
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1000));  // Check every 1 second
  }
}