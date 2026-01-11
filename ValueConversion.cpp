#include "ValueConversion.h"
#include <math.h>
#include "VictronBle.h"

extern float soot_pct;
extern float speed_kmh;
extern float rpm;
extern float coolantC;
extern float trans1C;
extern float trans2C;
extern float oil_kPa;
extern float battV;
extern float torqueNm;
extern float pedalPct;
extern float tqDemandPct;
extern float egt1C;
extern float egt2C;
extern float boost_kPa;
extern float manifoldC;
extern float turboOutC;
extern float lambdaVal;
extern float iatC;
extern float fuelC;

extern uint8_t g_uLambda;

extern float toDisplaySpeed(float v);
extern float toDisplayTemp(float v);
extern float toDisplayPressure(float v);
extern float toDisplayLambda(float v);

float valueRawBase(Channel ch){
  switch(ch){
    case CH_SOOT: return soot_pct; case CH_SPEED: return speed_kmh; case CH_RPM: return rpm;
    case CH_COOLANT: return coolantC; case CH_TRANS1: return trans1C; case CH_TRANS2: return trans2C;
    case CH_OIL: return oil_kPa; case CH_BATTV: return battV;
    case CH_BATT_SOC: return victronReadings().battSocPct; case CH_BATT_CURR: return victronReadings().battCurrentA; case CH_BATT_TTG: return victronReadings().battTimeMin;
    case CH_BATTV2: return victronReadings().battV2;
    case CH_DCDC_OUT_A: return victronReadings().dcdcOutA;
    case CH_DCDC_OUT_V: return victronReadings().dcdcOutV;
    case CH_DCDC_IN_V: return victronReadings().dcdcInV;
    case CH_PV_WATTS: return victronReadings().pvWatts;
    case CH_PV_AMPS: return victronReadings().pvAmps;
    case CH_PV_YIELD: return victronReadings().pvYieldKwh;
    case CH_TORQUE: return torqueNm; case CH_PEDAL: return pedalPct; case CH_TQ_DEMAND: return tqDemandPct;
    case CH_EGT1: return egt1C; case CH_EGT2: return egt2C; case CH_BOOST: return boost_kPa;
    case CH_MANIFOLD: return manifoldC; case CH_TURBO_OUT: return turboOutC; case CH_LAMBDA: return lambdaVal;
    case CH_IAT: return iatC; case CH_FUELT: return fuelC;
    default: return NAN;
  }
}

float valueDisplay(Channel ch){
  float v = valueRawBase(ch);
  switch(ch){
    case CH_SPEED: return toDisplaySpeed(v);
    case CH_COOLANT: case CH_TRANS1: case CH_TRANS2: case CH_IAT: case CH_FUELT:
    case CH_MANIFOLD: case CH_TURBO_OUT: case CH_EGT1: case CH_EGT2:
      return toDisplayTemp(v);
    case CH_BOOST: case CH_OIL: return toDisplayPressure(v);
    case CH_LAMBDA: return toDisplayLambda(v);
    default: return v;
  }
}
