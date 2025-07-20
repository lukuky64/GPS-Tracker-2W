#pragma once

#include <Arduino.h>

// -------------------- PORT MAPPING --------------------

#define UART_USB Serial
#define UART_GPS Serial2  // Changed to Serial2 for UART1 (pins 20/21)
#define UART_RF Serial1   // Changed to Serial1 for UART0 (pins 12/13)

// -------------------- PIN MAPPING --------------------

#define UART_RF_TX 12  // UART0 TX
#define UART_RF_RX 13  // UART0 RX

#define UART_GPS_TX 20  // UART1 TX
#define UART_GPS_RX 21  // UART1 RX

#define GPS_NRST 2
#define GPS_TIMEPULSE 3

#define RF_CTRL0 4
#define RF_CTRL1 5
#define RF_STATUS 6

#define I2C_EXT_SDA 0
#define I2C_EXT_SCL 1

#define VBAT_SENSE 26

#define LED1_PIN 19
#define LED2_PIN 18

// -------------------- CONST DEFINITIONS --------------------
#define GPS_BAUD_RATE 38400  // Default baud rate for u-blox M10 module
#define GPS_NAV_FREQ 2       // Default navigation frequency (2 Hz)
