#pragma once

#include <Arduino.h>
#include <FreeRTOS.h>
#include <semphr.h>

#define mutexTimeOut pdMS_TO_TICKS(5000)

class SemaphoreGuard {
 public:
  SemaphoreGuard(SemaphoreHandle_t semaphore, TickType_t timeout = mutexTimeOut) : m_semaphore(semaphore), m_acquired(false) { m_acquired = (xSemaphoreTake(m_semaphore, timeout) == pdTRUE); }

  ~SemaphoreGuard() {
    if (m_acquired) {
      xSemaphoreGive(m_semaphore);
    }
  }

  bool acquired() const { return m_acquired; }

 private:
  SemaphoreHandle_t m_semaphore;
  bool m_acquired;
};