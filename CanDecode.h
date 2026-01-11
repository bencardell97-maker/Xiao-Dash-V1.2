#pragma once
#include <Arduino.h>
#include <mcp2515.h>

// This header expects that the main sketch defines the CFG namespace (IDs, scales).
// Include this AFTER CFG is defined in your .ino.

// --- externs to write decoded values into your existing globals ---
extern float soot_pct, regen_pct, speed_kmh;
extern float rpm, coolantC, trans1C, trans2C, oil_kPa, battV, pedalPct, tqDemandPct;
extern float torqueNm;
extern float egt1C, egt2C, boost_kPa, manifoldC, turboOutC, lambdaVal, iatC, fuelC;
extern uint8_t turboActRaw;
extern bool headlightsOn;
extern int   gear;
extern int   targetgear;   // <-- added
extern bool  lockup;
extern uint8_t g_lockByteRaw3;

enum TCState : uint8_t {
  TC_Unlocked,
  TC_Applying,
  TC_Releasing,
  TC_Flex,
  TC_Full
};
extern volatile TCState g_tcState;

// --- Public entrypoint: call this from your loop() for every frame ---
namespace CanDec {
  void decodeFrame(const can_frame& f);
}
