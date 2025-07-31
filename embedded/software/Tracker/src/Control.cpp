#include "Control.hpp"

#define MAX_SETUP_RETRIES 3
#define MAX_LOCKOUT_RST pdMS_TO_TICKS(300'000)  // 5 minutes

Control::Control() : gpsTalker(UART_GPS, GPS_NRST, GPS_TIMEPULSE, LED2_PIN), rfTalker(UART_RF, RF_CTRL0, RF_CTRL1, RF_STATUS, LED1_PIN), batteryMonitor(VBAT_SENSE, BATT_SCALE_FACTOR), GPSFilter(GPS_FILTER_ALPHA, 3), initialised(false) {
  RFBroadcast_MS = 1000 / RF_BROADCAST_FREQ;
  GPSAquisition_MS = 1000 / GPS_UPDATE_FREQ;
  lastGPSUpdateTick = xTaskGetTickCount();
  rfBlast = false;
  m_startingAltitude = std::numeric_limits<float>::max();  // init to largest value before value is updated so we don't trigger blast threshold

  // Initialize mutexes for thisData and thatData
  thisData.GPSDataMutex = xSemaphoreCreateMutex();

  thisData.SYSDataMutex = xSemaphoreCreateMutex();
  thatData.GPSDataMutex = xSemaphoreCreateMutex();
  thatData.SYSDataMutex = xSemaphoreCreateMutex();
}

bool Control::setup() {
  UART_USB.begin(115200);

  UART_RF.setRX(UART_RF_RX);
  UART_RF.setTX(UART_RF_TX);

  // set rx to pull up to test
  pinMode(UART_RF_RX, INPUT_PULLUP);

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
  // GPS task gets highest priority (3) and runs on core 1 for dedicated processing
  xTaskCreateAffinitySet([](void *param) { static_cast<Control *>(param)->GPS_aquisition_task(); }, "GPS_Aquisition", 4096, this, 3, (1 << 1), &m_taskHandles.GPSTaskHandle);

  // Temporarily disable all other tasks to test GPS performance
  // RF tasks run on core 0 with medium priority (2)
  xTaskCreateAffinitySet([](void *param) { static_cast<Control *>(param)->RF_aquisition_task(); }, "RF_Aquisition", 4096, this, 2, (1 << 0), &m_taskHandles.RFTaskHandle);
  xTaskCreateAffinitySet([](void *param) { static_cast<Control *>(param)->RF_broadcast_task(); }, "RF_Broadcast", 4096, this, 2, (1 << 0), &m_taskHandles.RFBroadcastTaskHandle);

  // // // Low priority tasks run on core 0

  // // FIX: GPS task is slow as shit when other tasks are running
  xTaskCreateAffinitySet([](void *param) { static_cast<Control *>(param)->heartbeat_task(); }, "Heartbeat", 2048, this, 2, (1 << 0), &m_taskHandles.HeartBeatTaskHandle);
  xTaskCreateAffinitySet([](void *param) { static_cast<Control *>(param)->GPS_lockout_watchdog_task(); }, "GPS_Watchdog", 2048, this, 1, (1 << 0), &m_taskHandles.GPSLockoutWatchdogTaskHandle);
  xTaskCreateAffinitySet([](void *param) { static_cast<Control *>(param)->Rocket_state_task(); }, "Rocket_State", 2048, this, 1, (1 << 0), &m_taskHandles.RocketStateTaskHandle);
}

void Control::heartbeat_task() {
  // Initialize watchdog timer for 60 seconds (60000 ms)
  watchdog_enable(60'000, 1);  // 60s timeout, reset on timeout

  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t heartbeatCount = 0;

  while (true) {
    gpsTalker.setLED(false);
    rfTalker.setLED(false);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpsTalker.setLED(true);
    rfTalker.setLED(true);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpsTalker.setLED(false);
    rfTalker.setLED(false);

    heartbeatCount++;
    // Only print heartbeat status every 30 beats (60 seconds)
    if (heartbeatCount % 30 == 0) {
      UART_USB.println(F("Heartbeat OK"));
    }

    // Kick (refresh) the watchdog every loop
    watchdog_update();

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5'000));
    // checkTaskStack();
  }
}

void Control::GPS_aquisition_task() {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t updateCount = 0;
  unsigned long lastPrintTime = 0;

  unsigned long startMicros;  // Start timing
  unsigned long elapsedMicros = 0;

  while (true) {
    startMicros = micros();  // Start timing
    if (gpsTalker.checkNewData()) {
      GPS_DATA localGPSData;  // Local copy for printing
      bool dataProcessed = false;
      {
        // Use timeout to prevent GPS task from blocking indefinitely
        SemaphoreGuard gpsGuard(thisData.GPSDataMutex, pdMS_TO_TICKS(20));
        // loop only takes ~60us
        if (gpsGuard.acquired()) {
          thisData.GPSData = gpsTalker.getData();  // get new data
          lastGPSUpdateTick = xTaskGetTickCount();

          // Re-enable GPS filtering now that we understand the timing
          GPSFilter.processSample({thisData.GPSData.altitude, thisData.GPSData.latitude, thisData.GPSData.longitude});
          auto filtered = GPSFilter.getFilteredData();

          // Assign filtered data
          thisData.GPSData.altitude = filtered[0];
          thisData.GPSData.latitude = filtered[1];
          thisData.GPSData.longitude = filtered[2];

          localGPSData = thisData.GPSData;  // Copy for printing outside mutex
          dataProcessed = true;
          updateCount++;

          // UART_USB.printf("updateCount: %lu\n", updateCount);

          // 3 is enough for 3D fix. but 4 shows better results experimentally
          if (thisData.GPSData.nFixes >= 4) {
            GPS_update_count++;
            updateStartingAltitude(thisData.GPSData.altitude);
          }
        } else {
          // UART_USB.println("Missed GPS sample: mutex not acquired");
        }
      }

      // Print GPS data every X seconds to monitor performance
      unsigned long currentTime = millis();
      if (dataProcessed && (currentTime - lastPrintTime) > 1000) {
        lastPrintTime = currentTime;
        char buffer[128];
        snprintf(buffer, sizeof(buffer), "Local GPS #%lu: Lat=%.5f, Lon=%.5f, Alt=%.1fm, Fixes=%d", updateCount, localGPSData.latitude, localGPSData.longitude, localGPSData.altitude, localGPSData.nFixes);
        UART_USB.println(buffer);
      }
    } else {
      // UART_USB.println("No new GPS data available");
    }

    // elapsedMicros = micros() - startMicros;  // Calculate elapsed time
    // UART_USB.printf("%lu us\n", elapsedMicros);
    // Use proper GPS timing - checkNewData() blocks until new data is available
    // So we can use the original timing since the blocking handles the real rate
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(GPSAquisition_MS));
    // UART_USB.println("Looping GPS");
  }
}

void Control::updateStartingAltitude(float altitude) {
  if (GPS_update_count == GPS_UPDATE_FREQ * 10) {
    m_startingAltitude = altitude;  // Set the starting altitude for the rocket state task
    UART_USB.print(F("Starting altitude set to: "));
    UART_USB.print(m_startingAltitude);
    UART_USB.println(F("m"));
  }
}

void Control::RF_broadcast_task() {
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (true) {
    // HACK: testing print data here while we don't have RF
    updateSYSData();
    MSG_PACKET msgPacket;

    // Try to acquire mutexes with timeout to avoid blocking indefinitely
    bool acquired = false;
    {
      SemaphoreGuard gpsGuard(thisData.GPSDataMutex, pdMS_TO_TICKS(10));
      SemaphoreGuard sysGuard(thisData.SYSDataMutex, pdMS_TO_TICKS(10));

      if (gpsGuard.acquired() && sysGuard.acquired()) {
        packData(&msgPacket, &thisData.GPSData, &thisData.SYSData);
        acquired = true;
      }
    }

    if (acquired) {  // Send the packed data via RF
      if (rfTalker.sendMessage(&msgPacket)) {
#if DEBUG_VERBOSE
        UART_USB.println(F("RF message sent successfully!"));
        // GPS data is already printed by GPS_aquisition_task, only print SYS data here
        printSYSData(&thisData.SYSData);
#endif
      } else {
        UART_USB.println(F("Failed to send RF message."));
      }
    } else {
#if DEBUG_VERBOSE
      UART_USB.println(F("RF broadcast: failed to acquire mutexes"));
#endif
    }

    // Use vTaskDelayUntil for precise timing
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(RFBroadcast_MS));
  }
}

void Control::RF_aquisition_task() {
  while (true) {
    if (rfTalker.available()) {
      ResponseContainer response;
      if (rfTalker.receiveMessage(response)) {
        MSG_PACKET unpackedData;
        unpackResponseData(&unpackedData, response);

        {
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
      UART_USB.println(F("GPS LOCKOUT - RESETTING"));
      gpsTalker.hardwareReset();  // Reset the GPS module
    }
    vTaskDelay(pdMS_TO_TICKS(80'000));
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
    float currentAltitude = 0.0f;
    {
      SemaphoreGuard gpsGuard(thisData.GPSDataMutex, pdMS_TO_TICKS(10));
      if (gpsGuard.acquired()) {
        currentAltitude = thisData.GPSData.altitude;
      }
    }
    // NOTE: If system resets for any reason, this will reset the blasting state
    // TODO: can we get reset reason from device, store rfBlast in flash.
    // TODO: look into geofencing (addGeofence).
    if (currentAltitude - m_startingAltitude > ALTITUDE_THRESHOLD) {
      rfBlast = true;
      rfTalker.setRFPower(22);  // Set RF power to high for blasting
    }

    vTaskDelay(pdMS_TO_TICKS(1'000));  // Check every 1 second
  }
}

void Control::checkTaskStack() {
  // print how much stack is left on each task
  for (const auto &task : m_taskHandleMap) {
    if (task.handle != nullptr && *task.handle != nullptr) {
      UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(*task.handle);
      UART_USB.printf("Task: %s, Stack High Water Mark: %u\n", task.name.c_str(), stackHighWaterMark);
    } else {
      UART_USB.printf("Task: %s, Handle is null or invalid.\n", task.name.c_str());
    }
  }
}