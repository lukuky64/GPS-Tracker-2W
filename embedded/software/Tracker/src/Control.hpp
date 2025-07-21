#pragma once

#include <Arduino.h>
#include <FreeRTOS.h>
#include <HardwareSerial.h>
#include <task.h>

#include "BattMonitor.hpp"
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

  TaskHandle_t GPSTaskHandle;
  TaskHandle_t RFTaskHandle;
  TaskHandle_t RFBroadcastTaskHandle;

  uint32_t RFBroadcast_MS;
  uint32_t GPSAquisition_MS;

  GPS_Talker gpsTalker;
  RF_Talker rfTalker;
  BattMonitor batteryMonitor;

  SemaphoreHandle_t thisGPSDataMutex;
  SemaphoreHandle_t thatGPSDataMutex;

  SemaphoreHandle_t thisSYSDataMutex;
  SemaphoreHandle_t thatSYSDataMutex;

  SYS_DATA thisSYSData;
  SYS_DATA thatSYSData;

  GPS_DATA thisGPSData;
  GPS_DATA thatGPSData;

  uint32_t lastGPSUpdateTick;

  bool initialised;

  bool rfBlast;
};
