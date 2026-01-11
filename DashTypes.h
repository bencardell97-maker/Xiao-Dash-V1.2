#pragma once

#include <stdint.h>

struct Range { float mn, mx; };
struct PillSpec { int x,y,w,h; };

enum RegenState { REGEN_IDLE, REGEN_ACTIVE, REGEN_PAUSED };
enum Btn  { BTN_NONE, BTN_UP, BTN_DOWN, BTN_LEFT, BTN_RIGHT, BTN_ENTER, BTN_CANCEL };

enum MinMaxMode : uint8_t { MINMAX_NONE=0, MINMAX_MIN, MINMAX_MAX };

enum Channel : uint8_t {
  CH_SOOT, CH_SPEED, CH_RPM, CH_COOLANT, CH_TRANS1, CH_TRANS2, CH_OIL,
  CH_BATTV, CH_GEAR, CH_LOCKUP, CH_TORQUE, CH_PEDAL, CH_TQ_DEMAND,
  CH_EGT1, CH_EGT2, CH_BOOST, CH_MANIFOLD, CH_TURBO_OUT, CH_LAMBDA, CH_IAT, CH_FUELT,
  CH_ACTUATOR, CH_HEADLIGHTS,
  CH_BATT_SOC, CH_BATT_CURR, CH_BATT_TTG, CH_BATTV2,
  CH_DCDC_OUT_A, CH_DCDC_OUT_V, CH_DCDC_IN_V,
  CH_PV_WATTS, CH_PV_AMPS, CH_PV_YIELD,
  CH__COUNT
};

// Units selection
enum UnitsPressure : uint8_t { U_P_kPa=0, U_P_psi=1 };
enum UnitsTemp     : uint8_t { U_T_C=0,   U_T_F=1   };
enum UnitsSpeed    : uint8_t { U_S_kmh=0, U_S_mph=1 };
enum UnitsLambda   : uint8_t { U_L_lambda=0, U_L_AFR=1 };

enum MenuState : uint8_t {
  UI_MAIN=0,
  MENU_ROOT,
  MENU_LAYOUT,             // choose Screen 1..5
  MENU_LAYOUT_PICK_SLOT,   // 2x2 + bar preview
  MENU_LAYOUT_PICK_GAUGE,  // gauge chooser
  MENU_WARN_LIST,          // list of warning-eligible channels
  MENU_WARN_EDIT,          // per-channel edit
  MENU_COLOURS,            // palettes
  MENU_COLOURS_CUSTOM,     // custom palette zones
  MENU_COLOURS_CUSTOM_PICK,// custom palette color picker
  // New system menus
  MENU_SYSTEM,
  MENU_BRIGHTNESS,
  MENU_UNITS,              // <— NEW
  MENU_WIFI,
  MENU_CAN_SNIFF,
  MENU_FACTORY_RESET_CONFIRM,
  MENU_SPEED_TRIM, 
  MENU_OBD2,
  MENU_OBD2_ACTION,
};
