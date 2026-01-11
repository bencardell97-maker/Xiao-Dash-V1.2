#pragma once

#include <stdint.h>

struct VictronReadings {
  float battV2;
  float battSocPct;
  float battCurrentA;
  float battTimeMin;
  float dcdcOutA;
  float dcdcOutV;
  float dcdcInV;
  float pvWatts;
  float pvAmps;
  float pvYieldKwh;
  unsigned long lastBmvUpdateMs;
  unsigned long lastMpptUpdateMs;
  unsigned long lastDcdcUpdateMs;
};

namespace VictronBle {
extern const char kBmvMac[];
extern const uint8_t kBmvKey[16];
extern const char kMpptMac[];
extern const uint8_t kMpptKey[16];
extern const char kOrionMac[];
extern const uint8_t kOrionKey[16];
}

void victronInit();
VictronReadings victronLoop();
const VictronReadings& victronReadings();
