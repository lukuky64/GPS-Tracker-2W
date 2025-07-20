
#pragma once
#include <Arduino.h>

struct MSG_PACKET {
  byte latitude[4];
  byte longitude[4];
  byte altitude[4];
};
