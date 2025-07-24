#pragma once

#include <Arduino.h>
#include <FreeRTOS.h>
#include <HardwareSerial.h>
#include <task.h>

#include "BattMonitor.hpp"
#include "ExpFilter.hpp"
#include "GPS_Talker.hpp"
#include "RF_Talker.hpp"
#include "SemaphoreGuard.hpp"
#include "definitions.hpp"
#include "hardware/watchdog.h"
#include "messages.hpp"

class Control {
 public:
  Control();

  bool setup();

  void restartDevice();

 private:
  void beginTasks();
  void GPS_aquisition_task();
  void RF_aquisition_task();
  void RF_broadcast_task();
  void Rocket_state_task();

  void GPS_lockout_watchdog_task();

  bool setupRF();
  bool setupGPS();

  void updateSYSData();

  void updateStartingAltitude(float altitude);

  TaskHandle_t GPSTaskHandle;
  TaskHandle_t RFTaskHandle;
  TaskHandle_t RFBroadcastTaskHandle;

  uint32_t RFBroadcast_MS;
  uint32_t GPSAquisition_MS;

  GPS_Talker gpsTalker;
  RF_Talker rfTalker;
  BattMonitor batteryMonitor;

  struct theData {
    SemaphoreHandle_t GPSDataMutex;
    SemaphoreHandle_t SYSDataMutex;
    GPS_DATA GPSData;
    SYS_DATA SYSData;
  };

  theData thisData;
  theData thatData;

  ExpFilter GPSFilter;

  uint32_t lastGPSUpdateTick;

  float m_startingAltitude;   // Starting altitude for the rocket state task, in meters
  uint32_t GPS_update_count;  // Count of GPS updates to determine when to set the starting altitude

  bool initialised;

  bool rfBlast;
};
