#pragma once

#include <Arduino.h>

// -------------------- PORT MAPPING --------------------

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
#define GPS_BAUD_RATE 38400    // Default baud rate for u-blox M10 module
#define GPS_UPDATE_FREQ 20     // GPS update frequency
#define GPS_FILTER_ALPHA 0.1f  // Alpha value for exponential filter on GPS data

#define RF_BAUD_RATE 9600       // Default baud rate for RF module
#define RF_BROADCAST_FREQ 0.2f  // Broadcast frequency

#define ALTITUDE_THRESHOLD 200.0f  // Altitude threshold in meters for rocket state detection

// TODO: Calibrate this
#define BATT_SCALE_FACTOR 2.8f  // Voltage divider scaling.
