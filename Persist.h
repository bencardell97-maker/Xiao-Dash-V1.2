#pragma once

#include <Arduino.h>
#include "DashTypes.h"

namespace Persist {
  constexpr uint16_t EEPROM_MAGIC = 0x7ADE;
  constexpr uint16_t SCHEMA_VERSION = 1;
  constexpr size_t EEPROM_BYTES = 1024;
  constexpr int EEPROM_ADDR = 0;
  constexpr uint32_t SAVE_MS = 300000;
}

struct CustomPalette {
  uint16_t card;
  uint16_t frame;
  uint16_t ticks;
  uint16_t text;
  uint16_t accent;
  uint16_t bg;
};

constexpr uint8_t CUSTOM_PALETTE_COUNT = 3;
constexpr uint8_t SCREEN_COUNT = 5;
constexpr size_t WIFI_SSID_LEN = 32;
constexpr size_t WIFI_PASS_LEN = 64;
constexpr size_t VICTRON_MAC_LEN = 18;

struct PersistState {
  uint16_t magic;
  uint16_t version;
  uint8_t pillChannel[SCREEN_COUNT][4];
  uint8_t barChannel[SCREEN_COUNT];
  uint8_t currentScreen;
  uint8_t warnMode[CH__COUNT];
  float   warnT1[CH__COUNT];  // base units
  float   warnT2[CH__COUNT];  // base units
  uint8_t paletteIndex;
  CustomPalette customPalettes[CUSTOM_PALETTE_COUNT];
  // system
  uint8_t brightOn;
  uint8_t brightOff;
  // units
  uint8_t uPressure;
  uint8_t uTemp;
  uint8_t uSpeed;
  uint8_t uLambda;
  float   speedTrimPct;    // e.g., +1.0 = +1% speed; default 0.0
  uint8_t victronEnabled;
  char    wifiSsid[WIFI_SSID_LEN + 1];
  char    wifiPass[WIFI_PASS_LEN + 1];
  char    victronBmvMac[VICTRON_MAC_LEN];
  uint8_t victronBmvKey[16];
  char    victronMpptMac[VICTRON_MAC_LEN];
  uint8_t victronMpptKey[16];
  char    victronOrionMac[VICTRON_MAC_LEN];
  uint8_t victronOrionKey[16];
};

void loadPersist(PersistState& state, const PersistState& defaults);
void savePersist(PersistState& state, bool& dirty, bool force = false);

static_assert(sizeof(PersistState) <= Persist::EEPROM_BYTES, "Persist too large for EEPROM");
