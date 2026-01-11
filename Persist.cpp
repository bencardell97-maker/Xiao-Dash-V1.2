#include "Persist.h"

#include <EEPROM.h>

namespace {
  unsigned long g_lastSaveMs = 0;

  void applyHeader(PersistState& state){
    state.magic = Persist::EEPROM_MAGIC;
    state.version = Persist::SCHEMA_VERSION;
  }
}

void loadPersist(PersistState& state, const PersistState& defaults){
  EEPROM.begin(Persist::EEPROM_BYTES);
  PersistState stored{};
  EEPROM.get(Persist::EEPROM_ADDR, stored);
  if(stored.magic == Persist::EEPROM_MAGIC && stored.version == Persist::SCHEMA_VERSION){
    state = stored;
  } else {
    state = defaults;
    applyHeader(state);
    EEPROM.put(Persist::EEPROM_ADDR, state);
    EEPROM.commit();
  }
  g_lastSaveMs = millis();
}

void savePersist(PersistState& state, bool& dirty, bool force){
  const unsigned long now = millis();
  if(!force){
    if(!dirty){
      return;
    }
    if(now - g_lastSaveMs < Persist::SAVE_MS){
      return;
    }
  }
  applyHeader(state);
  EEPROM.put(Persist::EEPROM_ADDR, state);
  EEPROM.commit();
  dirty = false;
  g_lastSaveMs = now;
}
