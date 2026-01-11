// ===================== Includes =====================
#include <Arduino.h>
#include <SPI.h>
#include <limits.h>
#include <mcp2515.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <math.h>
#include <ctype.h>
#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>
#include "DashTypes.h"
#include "Persist.h"
#include "Config.h"
#include "UiRenderer.h"
#include "ValueConversion.h"
#include "VictronBle.h"

#ifndef IRAM_ATTR
  #define IRAM_ATTR
#endif

#ifndef TFT_SPI_HZ
  #define TFT_SPI_HZ 80000000UL
#endif
#ifndef RGB565
  #define RGB565(r,g,b) ( (((r)&0xF8)<<8) | (((g)&0xFC)<<3) | ((b)>>3) )
#endif

// Forward declarations for functions referenced before their definitions
void redrawForDimmingChange();
// ==== CAN Sniffer: forward declarations ====
void showCanSniff(bool full = true);
void drawSniffRow(uint8_t row, bool sel, bool blinkHide=false);
void drawSniffLive();
void snifferMaybeCapture(const can_frame& f);

//=====================Xiao Can Expansion Board =================

// === Bring in CAN decoder (expects CFG above) ===
#include "CanDecode.h"

// ===================== Hardware =====================
Adafruit_ILI9341 tft(CFG::TFT_CS, CFG::TFT_DC, CFG::TFT_RST);
MCP2515 mcp(CFG::CAN_CS);
volatile bool can_irq=false; void IRAM_ATTR onCanInt(){ can_irq=true; }
WebServer webServer(80);
static IPAddress g_wifiIp;
static bool g_wifiActive = false;
static bool g_wifiPageActive = false;
static bool g_webServerActive = false;

bool wifiPageActive(){ return g_wifiPageActive; }

// ===================== Live values (extern targets for CanDecode.h) =====================
RegenState regenState=REGEN_IDLE;
float soot_pct=0, regen_pct=0, speed_kmh=0;        // base units: km/h
float rpm=0,coolantC=0,trans1C=0,trans2C=0,oil_kPa=0,battV=0,pedalPct=0,tqDemandPct=0;
float torqueNm=0;
float egt1C=0, egt2C=0, boost_kPa=0, manifoldC=0, turboOutC=0, lambdaVal=1.00f, iatC=0, fuelC=0;
uint8_t turboActRaw=0;
bool headlightsOn=false;
bool prevHeadlightsOn=false;
int gear=0; int targetgear=0; // <- make sure CanDecode.h writes targetgear
volatile TCState g_tcState = TC_Unlocked;
// If these are only declared extern in the header, define them here too:
bool    lockup = false;
uint8_t g_lockByteRaw3 = 0;

// ===== Victron BLE (Instant Readout) =====
static VictronReadings g_victronReadings{};


// ===================== Timing =====================
unsigned long lastMillis=0, lastDraw=0;

// ===================== UI state & persistence =====================
MenuState menuState = UI_MAIN;
bool inSettings() { return menuState != UI_MAIN; }

// ===== Main-UI warnings state & blink =====
unsigned long uiWarnBlinkMs = 0;
bool uiWarnBlinkOn = true;

// ===== Main-UI min/max display =====
bool uiMinMaxActive = false;
static float uiMinValues[CH__COUNT];
static float uiMaxValues[CH__COUNT];
static bool uiMinMaxHas[CH__COUNT];

uint8_t uiHighestWarnLevel = 0;   // 0 = none, 1 = L1, 2 = L2
Channel uiHighestWarnCh = CH__COUNT;

// Highest level on last refresh
uint8_t uiPrevHighestWarnLevel = 0;

// Global L2 detector
bool globalL2ActivePrev = false;
unsigned long lastBeepMs = 0;

// Root selection memory
uint8_t g_lastRootIndex = 0;

// General indices
uint8_t menuIndex = 0;       // general cursor (root; colours; system; etc.)
uint8_t menuIndex2 = 0;      // secondary (gauge list)
uint8_t obd2Sel = 0;         // OBD2 menu selection

// ===== OBD2 scan state =====
static const uint8_t OBD2_MAX_CODES = 6;
static char obd2Codes[OBD2_MAX_CODES][6];
static uint8_t obd2CodeCount = 0;
static bool obd2Awaiting = false;
static bool obd2TimedOut = false;
static bool obd2SendOk = true;
static unsigned long obd2RequestMs = 0;
static bool obd2NeedsRedraw = false;
static bool obd2ClearDone = false;
static bool obd2ClearOk = false;

// Layout editor cursor (row=-1 means BAR, rows 0..1, cols 0..1)
int8_t layoutRow = 0, layoutCol = 0;
uint8_t layoutScreenSel = 0; // 0..4 – which screen being edited

// Gauge picker paging & selection (latched window)
int pickerTop = 0;   // first visible eligible index

// Warnings: list paging & selection
int warnListTop = 0;     // index of first visible eligible item
int warnListSel = 0;     // selected eligible index

// Warnings: field editor
uint8_t warnFieldSel   = 0;        // 0=Mode,1=T1,2=T2
bool    warnFieldEditing = false;  // editing current field?
uint8_t editMode = CFG::WARN_OFF;  // staged
float   editT1=0, editT2=0;        // staged
unsigned long warnBlinkMs = 0; bool warnBlinkOn = true;


// ===================== CAN Sniffer state =====================
// Standard 11-bit CAN IDs (0x000..0x7FF) → 3 hex digits.
// If you later want extended 29-bit, set SNF_HEX_DIGITS=8 and remove the clamp.
static uint32_t snf_id = 0x000;
static const uint8_t SNF_HEX_DIGITS = 3;

static uint8_t  snf_cursor = 0;   // which hex digit (0..SNF_HEX_DIGITS-1) from MSB side
static uint8_t  snf_sel    = 0;   // row select: 0=Address, 1=From, 2=To, 3=Scale, 4=Bias
static bool     snf_editing = false;

static uint8_t  snf_bit_from = 0; // 0..63
static uint8_t  snf_bit_to   = 7; // 0..63

static uint8_t  snf_data[8] = {0}; // last matching frame payload
static uint8_t  snf_dlc = 0;
static bool     snf_has = false;   // true if we’ve captured a frame for the ID

// Calibration: display = raw * scale + bias
static float    snf_scale = 1.0f;
static float    snf_bias  = 0.0f;
// Step for editing floats (LEFT/RIGHT multiplies/divides by 10)
static float    snf_step  = 0.1f;

// ===== Hold-to-accelerate (shared) =====
bool up_now=false, down_now=false; // current press states
unsigned long repeatStartMs=0, lastRepeatMs=0; bool repeating=false;

// Progressive hold state
enum HoldDir { HOLD_NONE=0, HOLD_UP=1, HOLD_DOWN=-1 };
HoldDir holdDir = HOLD_NONE;
float holdStep = 1.0f;  // current step during this hold session (base * 10^holdLevel)
int   holdLevel = 0;    // 0 = base, 1 = ×10, 2 = ×100, ...

// Palettes
struct Palette { const char* name; uint16_t card, frame, ticks, text, accent, yellow, orange, red, bg; };
static const Palette PALETTES[] = {
  {"Cyan",    0x2104, ILI9341_DARKGREY, ILI9341_CYAN,    ILI9341_WHITE,   ILI9341_CYAN,    ILI9341_YELLOW, 0xFD20, ILI9341_RED,   ILI9341_BLACK},
  {"Green",   0x18E3, ILI9341_NAVY,     ILI9341_GREEN,   ILI9341_WHITE,   ILI9341_GREEN,   ILI9341_YELLOW, 0xFBE0, ILI9341_RED,   ILI9341_BLACK},
  {"Magenta", 0x39E7, 0x8410,           ILI9341_MAGENTA, ILI9341_WHITE,   ILI9341_MAGENTA, ILI9341_YELLOW, 0xFCA0, ILI9341_RED,   ILI9341_BLACK},
  {"Amber",   0x4208, ILI9341_DARKGREY, 0xFD20,          ILI9341_WHITE,   0xFDA0,          0xFFE0,         0xFCA0, ILI9341_RED,  ILI9341_BLACK},
  {"Ocean",   0x2104, 0x0011,           0x07FF,          ILI9341_WHITE,   0x05BF,          0xAFE5,         0xFD20, 0xF800,       0x0008},
  {"Forest",  0x0208, ILI9341_DARKGREEN,0x07E0,          ILI9341_WHITE,   0x07E0,          0xFFE0,         0xFD20, 0xF800,       ILI9341_BLACK},
  {"Volt",    0x0000, ILI9341_DARKGREY, 0xAFE5,          ILI9341_WHITE,   0xB7E0,          0xFFE0,         0xFD20, ILI9341_RED,  ILI9341_BLACK},
  {"MonoLight",0x8410, ILI9341_DARKGREY,0xC618,          ILI9341_WHITE,   0xC618,          0xFFE0,         0xFD20, 0xF800,       ILI9341_BLACK},
  {"Sunset",  0x39E7, 0x4208,           0xFCA0,          ILI9341_WHITE,   0xFCA0,          0xFFE0,         0xFD20, 0xF800,       ILI9341_BLACK},
  {"Arctic",  0xDEFB, 0xC618,           0x07FF,          ILI9341_BLACK,   0x04BF,          0xFFE0,         0xFD20, 0xF800,       ILI9341_WHITE},
  {"Sand",    0xEF5B, 0xCE79,           0xFD20,          ILI9341_BLACK,   0xFE80,          0xFFE0,         0xFD20, 0xF800,       ILI9341_WHITE},
  {"Sky",     0xDFFF, 0xAD55,           0x04FF,          ILI9341_BLACK,   0x04DF,          0xFFE0,         0xFD20, 0xF800,       0xFFFF},
  {"Slate",   0xBDF7, 0x9CF3,           0x867D,          ILI9341_BLACK,   0x8C51,          0xFFE0,         0xFD20, 0xF800,       0xE71C},
  {"Cream",   0xF79E, 0xD69A,           0xFDA0,          0x39C4,          0xFCA0,          0xFFE0,         0xFD20, 0xB000,       0xFFFF},
  {"DaylightRacing", 0xFFFF, 0xC618,    0xF800,          ILI9341_BLACK,   0xF800,          0xFFE0,         0xFD20, ILI9341_RED,  0xFFFF},
};

static const char* CUSTOM_PALETTE_NAMES[CUSTOM_PALETTE_COUNT] = { "Custom 1", "Custom 2", "Custom 3" };

static const char* CUSTOM_ZONE_LABELS[] = { "Card", "Frame", "Ticks", "Text", "Accent", "Background" };
static const uint8_t CUSTOM_ZONE_COUNT = 6;

struct NamedColor { const char* name; uint16_t value; };
static const NamedColor CUSTOM_COLOUR_OPTIONS[] = {
  {"White", ILI9341_WHITE},
  {"Light Grey", 0xC618},
  {"Grey", ILI9341_DARKGREY},
  {"Black", ILI9341_BLACK},
  {"Red", ILI9341_RED},
  {"Orange", 0xFD20},
  {"Amber", 0xFCA0},
  {"Yellow", ILI9341_YELLOW},
  {"Green", ILI9341_GREEN},
  {"Dark Green", ILI9341_DARKGREEN},
  {"Cyan", ILI9341_CYAN},
  {"Teal", 0x05BF},
  {"Blue", ILI9341_BLUE},
  {"Navy", ILI9341_NAVY},
  {"Magenta", ILI9341_MAGENTA},
  {"Purple", 0x780F},
};

static inline int basePaletteCount(){
  return (int)(sizeof(PALETTES)/sizeof(PALETTES[0]));
}

uint8_t paletteIndex = 0;

// ---- Colours page scrolling state ----
static int coloursSel = 0;   // selected palette index (cursor)
static int coloursTop = 0;   // first visible row index in window
static uint8_t customPaletteSel = 0;
static uint8_t customZoneSel = 0;
static int customColourSel = 0;
static int customColourTop = 0;

static inline int paletteCount(){
  return basePaletteCount() + CUSTOM_PALETTE_COUNT;
}

static inline bool isCustomPaletteIndex(int idx){
  return idx >= basePaletteCount();
}

static inline uint8_t customPaletteSlot(int idx){
  return (uint8_t)(idx - basePaletteCount());
}

static inline const char* paletteNameForIndex(int idx){
  if (isCustomPaletteIndex(idx)) return CUSTOM_PALETTE_NAMES[customPaletteSlot(idx)];
  return PALETTES[idx].name;
}

static inline uint8_t customColourCount(){
  return (uint8_t)(sizeof(CUSTOM_COLOUR_OPTIONS)/sizeof(CUSTOM_COLOUR_OPTIONS[0]));
}

static inline const char* customColourNameForValue(uint16_t value){
  for (uint8_t i = 0; i < customColourCount(); i++) {
    if (CUSTOM_COLOUR_OPTIONS[i].value == value) return CUSTOM_COLOUR_OPTIONS[i].name;
  }
  return "Custom";
}

static inline uint8_t customColourIndexForValue(uint16_t value){
  for (uint8_t i = 0; i < customColourCount(); i++) {
    if (CUSTOM_COLOUR_OPTIONS[i].value == value) return i;
  }
  return 0;
}

static inline uint16_t customZoneValue(const CustomPalette& palette, uint8_t zone){
  switch(zone){
    case 0: return palette.card;
    case 1: return palette.frame;
    case 2: return palette.ticks;
    case 3: return palette.text;
    case 4: return palette.accent;
    case 5: return palette.bg;
    default: return palette.card;
  }
}

static inline void setCustomZoneValue(CustomPalette& palette, uint8_t zone, uint16_t value){
  switch(zone){
    case 0: palette.card = value; break;
    case 1: palette.frame = value; break;
    case 2: palette.ticks = value; break;
    case 3: palette.text = value; break;
    case 4: palette.accent = value; break;
    case 5: palette.bg = value; break;
    default: break;
  }
}

// Call this when you ENTER the colours page (optional but nice UX)
static inline void enterColoursPage(){
  coloursSel = paletteIndex;                            // focus current
  const int perPage = MENU_PER_PAGE();
  const int total   = paletteCount();
  // Center selection if possible; clamp to valid window
  coloursTop = coloursSel - perPage/2;
  if (coloursTop < 0) coloursTop = 0;
  int maxTop = total - perPage; if (maxTop < 0) maxTop = 0;
  if (coloursTop > maxTop) coloursTop = maxTop;
}

static inline void setColoursSelection(int index){
  const int perPage = MENU_PER_PAGE();
  const int total   = paletteCount();
  coloursSel = index;
  int maxTop = total - perPage; if (maxTop < 0) maxTop = 0;
  if (coloursSel < coloursTop) coloursTop = coloursSel;
  if (coloursSel >= coloursTop + perPage) coloursTop = coloursSel - perPage + 1;
  if (coloursTop < 0) coloursTop = 0;
  if (coloursTop > maxTop) coloursTop = maxTop;
}

// ===================== System settings (NEW) =====================
uint8_t brightOn = 0;   // % brightness with headlights ON
uint8_t brightOff = 0;  // % brightness with headlights OFF
uint8_t brightSel = 0;   // 0=Lights On, 1=Lights Off
bool    brightEditing = false;
unsigned long brightBlinkMs = 0; bool brightBlinkOn = true;

constexpr uint8_t MIN_BRIGHT = 10; // minimum allowed %
// Speed Trim (RAM copy; persisted in 'persist.speedTrimPct')
float speedTrimPct = 0.0f;
static const float SPEED_TRIM_MIN = -20.0f;
static const float SPEED_TRIM_MAX =  20.0f;
static const float SPEED_TRIM_STEP = 0.1f;
bool  speedTrimEditing = false;
unsigned long speedTrimBlinkMs = 0; bool speedTrimBlinkOn = true;

// Units (RAM copies, persisted below)
uint8_t g_uPressure = U_P_kPa;
uint8_t g_uTemp     = U_T_C;
uint8_t g_uSpeed    = U_S_kmh;
uint8_t g_uLambda   = U_L_lambda;

bool     unitsEditing=false;
uint8_t  unitsSel=0;
unsigned long unitsBlinkMs=0; bool unitsBlinkOn=true;

PersistState persist{};
bool dirty=true;

// ===================== UI Layout constants =====================
extern const int APPBAR_H=34, BAR_X=14, BAR_Y=64, BAR_W=292, BAR_H=32, BAR_R=8;
extern const int GRID_TOP=112, GRID_LEFT=12, GRID_W=296, GRID_H=120, CELL_GAP_X=8, CELL_GAP_Y=10;
extern const int CELL_W=(GRID_W-CELL_GAP_X)/2, CELL_H=(GRID_H-CELL_GAP_Y)/2;

Channel currentBarChannel(){ return (Channel)persist.barChannel[persist.currentScreen]; }
Channel currentPillChannel(uint8_t slot){ return (Channel)persist.pillChannel[persist.currentScreen][slot]; }

// Labels (base)
static const char* LBL_BASE[CH__COUNT]={
  "Soot %","Speed","RPM","Coolant","Trans 1","Trans 2","Oil kPa",
  "Battery V","Gear","Converter","Torque","Pedal %","Demand %",
  "EGT 1","EGT 2","Boost","Manifold 2","Manifold 1","Lambda","Intake T","Fuel T",
  "Turbo %","Headlights","Aux Batt %","Aux Batt A","Time Rem","Aux Batt V",
  "DCDC Out A","DCDC Out V","DCDC In V",
  "PV Watts","PV Amps","PV Yield"
};

// ===== Unit helpers =====
const char* unitLabel(Channel ch){
  switch(ch){
    case CH_BATTV: return "V";
    case CH_BATTV2: return "V";
    case CH_BATT_SOC: return "%";
    case CH_BATT_CURR: return "A";
    case CH_BATT_TTG: return "min";
    case CH_DCDC_OUT_A: return "A";
    case CH_DCDC_OUT_V: return "V";
    case CH_DCDC_IN_V: return "V";
    case CH_PV_WATTS: return "W";
    case CH_PV_AMPS: return "A";
    case CH_PV_YIELD: return "kWh";
    case CH_SPEED: return (g_uSpeed==U_S_kmh)?"km/h":"mph";
    case CH_COOLANT: case CH_TRANS1: case CH_TRANS2: case CH_IAT: case CH_FUELT:
    case CH_MANIFOLD: case CH_TURBO_OUT: case CH_EGT1: case CH_EGT2:
      return (g_uTemp==U_T_C)?"C":"F";
    case CH_BOOST: case CH_OIL:
      return (g_uPressure==U_P_kPa)?"kPa":"psi";
    case CH_LAMBDA: return (g_uLambda==U_L_lambda)?"λ":"AFR";
    case CH_TORQUE: return "Nm";
    case CH_PEDAL: case CH_TQ_DEMAND: case CH_SOOT: return "%";
    default: return "";
  }
}
const char* labelText(Channel ch){
  // Change oil label to pressure unit
  if(ch==CH_OIL)
    return (g_uPressure==U_P_kPa) ? "Oil kPa" : "Oil psi";
  if(ch==CH_BOOST)
    return (g_uPressure==U_P_kPa) ? "Boost" : "Boost";
  return LBL_BASE[ch];
}

// Conversions (value in base → display units)
float toDisplayPressure(float kPa){
  return (g_uPressure==U_P_kPa)? kPa : (kPa * 0.1450377f);
}
float toDisplayTemp(float C){
  return (g_uTemp==U_T_C)? C : (C*9.0f/5.0f + 32.0f);
}
float toDisplaySpeed(float kmh){
  // ===== PATCH: apply speed trim in BASE before conversion =====
  // speedTrimPct is a signed percent; clamp huge values just in case:
  float pct = speedTrimPct;
  if (!isfinite(pct)) pct = 0.0f;
  if (pct < -50.0f) pct = -50.0f;  // safety
  if (pct >  50.0f) pct =  50.0f;
  float kmhTrimmed = kmh * (1.0f + pct / 100.0f);
  return (g_uSpeed==U_S_kmh)? kmhTrimmed : (kmhTrimmed * 0.62137119f);
}
float toDisplayLambda(float lambda){
  return (g_uLambda==U_L_lambda)? lambda : (lambda * 14.5f);
}
// Reverse (display → base), used in warnings editor
static inline float fromDisplayPressure(float v){
  return (g_uPressure==U_P_kPa)? v : (v / 0.1450377f);
}
static inline float fromDisplayTemp(float v){
  return (g_uTemp==U_T_C)? v : ((v - 32.0f) * 5.0f/9.0f);
}
static inline float fromDisplaySpeed(float v){
  return (g_uSpeed==U_S_kmh)? v : (v / 0.62137119f);
}
static inline float fromDisplayLambda(float v){
  return (g_uLambda==U_L_lambda)? v : (v / 14.5f);
}

// Ranges in base units (unchanged)
static const Range RNG_BASE[CH__COUNT]={
  {0,100}, {0,200}, {0,5000}, {40,110}, {40,130}, {40,130}, {0,600},
  {10,15}, {0,1}, {0,1}, { -200,600 }, {0,100}, {0,100},
  {0,1000}, {0,1000}, {0,250}, {0,150}, {0,200}, {0,2}, {0,80}, {0,100},
  {0,255}, {0,1}, {0,100}, {-300,300}, {0,1440}, {10,15},
  {0,200}, {0,60}, {0,60},
  {0,2000}, {0,100}, {0,20}
};

// Range in display units
Range rangeFor(Channel c){
  Range r = RNG_BASE[c];
  switch(c){
    case CH_BATTV: break;
    case CH_SPEED: r.mn = toDisplaySpeed(r.mn); r.mx = toDisplaySpeed(r.mx); break;
    case CH_COOLANT: case CH_TRANS1: case CH_TRANS2: case CH_IAT: case CH_FUELT:
    case CH_MANIFOLD: case CH_TURBO_OUT: case CH_EGT1: case CH_EGT2:
      r.mn = toDisplayTemp(r.mn); r.mx = toDisplayTemp(r.mx); break;
    case CH_BOOST: case CH_OIL:
      r.mn = toDisplayPressure(r.mn); r.mx = toDisplayPressure(r.mx); break;
    case CH_LAMBDA:
      r.mn = toDisplayLambda(r.mn); r.mx = toDisplayLambda(r.mx); break;
    default: break;
  }
  if(r.mx <= r.mn) r.mx = r.mn + 1;
  return r;
}

static inline float stepFor(Channel c){
  if(c==CH_BATTV) return 0.1f;
  if(c==CH_BATTV2) return 0.01f;
  if(c==CH_BATT_CURR) return 0.1f;
  if(c==CH_DCDC_OUT_A || c==CH_PV_AMPS) return 0.1f;
  if(c==CH_PV_YIELD) return 0.01f;
  if(c==CH_LAMBDA) return (g_uLambda==U_L_lambda)? 0.01f : 0.1f; // AFR shows tenths
  return 1.0f;
}

MinMaxMode minMaxModeFor(Channel ch){
  switch(ch){
    case CH_GEAR:
    case CH_LOCKUP:
    case CH_HEADLIGHTS:
    case CH_BATT_TTG:
      return MINMAX_NONE;
    case CH_LAMBDA:
      return MINMAX_MIN;
    case CH_BATT_SOC:
      return MINMAX_MIN;
    case CH_COOLANT: case CH_TRANS1: case CH_TRANS2: case CH_IAT: case CH_FUELT:
    case CH_MANIFOLD: case CH_TURBO_OUT: case CH_EGT1: case CH_EGT2:
    case CH_BOOST: case CH_OIL:
      return MINMAX_MAX;
    default:
      return MINMAX_MAX;
  }
}
// ===== PATCH: Decade acceleration helper for hold-to-repeat (warnings) =====
// Increase holdStep by ×10 whenever the value crosses a decade boundary in DISPLAY space.
// baseStep: the channel's display-unit base step (from stepFor).
// prevVal/newVal: DISPLAY values before/after the last step.
// dirSign: +1 for UP, -1 for DOWN.
inline void maybeEscalateHoldStep(float baseStep, float prevVal, float newVal, int dirSign){
  // Normalize by base step so "decade" is 10 ticks at the current level.
  const float scale = 1.0f / (baseStep <= 0 ? 1.0f : baseStep);
  long prevMag = lroundf(fabsf(prevVal) * scale);
  long newMag  = lroundf(fabsf(newVal)  * scale);

  // 10^(holdLevel+1) ticks per next decade boundary
  long decadeSizeTicks = 1;
  for (int i = 0; i < holdLevel + 1; i++) decadeSizeTicks *= 10;

  if (dirSign > 0) {
    // e.g., 9 -> 10
    long boundary = ((prevMag / decadeSizeTicks) + 1) * decadeSizeTicks;
    if (newMag >= boundary) { holdLevel++; holdStep *= 10.0f; }
  } else {
    // e.g., 61 -> 60
    long boundary = (prevMag / decadeSizeTicks) * decadeSizeTicks;
    if (newMag <= boundary) { holdLevel++; holdStep *= 10.0f; }
  }
}
// Eligibility
static inline bool isVictronChannel(Channel ch){
  switch(ch){
    case CH_BATT_SOC:
    case CH_BATT_CURR:
    case CH_BATT_TTG:
    case CH_BATTV2:
    case CH_DCDC_OUT_A:
    case CH_DCDC_OUT_V:
    case CH_DCDC_IN_V:
    case CH_PV_WATTS:
    case CH_PV_AMPS:
    case CH_PV_YIELD:
      return true;
    default:
      return false;
  }
}
static inline bool isGaugeAvailable(Channel ch){
  if(ch == CH_OIL) return false;
  if(!persist.victronEnabled && isVictronChannel(ch)) return false;
  return true;
}
static inline bool isBarEligible(Channel ch){ return isGaugeAvailable(ch) && !(ch==CH_LOCKUP||ch==CH_GEAR||ch==CH_HEADLIGHTS); }
// Warnings: exclude Gear/Lockup/Headlights/Actuator
static inline bool isWarnEligible(Channel ch){
  if(!persist.victronEnabled && isVictronChannel(ch)) return false;
  if(ch == CH_BATT_CURR || ch == CH_BATT_TTG || ch == CH_BATTV2) return false;
  if(ch == CH_DCDC_OUT_A || ch == CH_DCDC_OUT_V || ch == CH_DCDC_IN_V) return false;
  if(ch == CH_PV_WATTS || ch == CH_PV_AMPS || ch == CH_PV_YIELD) return false;
  return isGaugeAvailable(ch) && !(ch==CH_GEAR || ch==CH_LOCKUP || ch==CH_HEADLIGHTS || ch==CH_ACTUATOR);
}

// ===== UI brightness via backlight PWM =====
static inline uint8_t uiBrightnessPct(){
  uint8_t pct = headlightsOn ? brightOn : brightOff;
  if (pct > 100) pct = 100;
  return pct;
}

static uint8_t g_lastBacklightPct = 255;

// Palette accessors
uint16_t COL_BG(){
  if (isCustomPaletteIndex(paletteIndex)) return persist.customPalettes[customPaletteSlot(paletteIndex)].bg;
  return PALETTES[paletteIndex].bg;
}
uint16_t COL_CARD(){
  if (isCustomPaletteIndex(paletteIndex)) return persist.customPalettes[customPaletteSlot(paletteIndex)].card;
  return PALETTES[paletteIndex].card;
}
uint16_t COL_FRAME(){
  if (isCustomPaletteIndex(paletteIndex)) return persist.customPalettes[customPaletteSlot(paletteIndex)].frame;
  return PALETTES[paletteIndex].frame;
}
uint16_t COL_TICKS(){
  if (isCustomPaletteIndex(paletteIndex)) return persist.customPalettes[customPaletteSlot(paletteIndex)].ticks;
  return PALETTES[paletteIndex].ticks;
}
uint16_t COL_TXT(){
  if (isCustomPaletteIndex(paletteIndex)) return persist.customPalettes[customPaletteSlot(paletteIndex)].text;
  return PALETTES[paletteIndex].text;
}
uint16_t COL_ACCENT(){
  if (isCustomPaletteIndex(paletteIndex)) return persist.customPalettes[customPaletteSlot(paletteIndex)].accent;
  return PALETTES[paletteIndex].accent;
}
uint16_t COL_YELLOW(){
  if (isCustomPaletteIndex(paletteIndex)) return ILI9341_YELLOW;
  return PALETTES[paletteIndex].yellow;
}
uint16_t COL_ORANGE(){
  if (isCustomPaletteIndex(paletteIndex)) return 0xFD20;
  return PALETTES[paletteIndex].orange;
}
uint16_t COL_RED(){
  if (isCustomPaletteIndex(paletteIndex)) return ILI9341_RED;
  return PALETTES[paletteIndex].red;
}

// ===================== Helpers =====================
float clampf(float v,float lo,float hi){ if(!isfinite(v)) return lo; return v<lo?lo:(v>hi?hi:v); }
inline uint16_t be16(const uint8_t* d){ return (uint16_t)d[0]<<8 | d[1]; }
void clearRegion(int x,int y,int w,int h,uint16_t col){ if(w>0&&h>0)tft.fillRect(x,y,w,h,col); }
uint16_t barFillColor(){ return COL_ACCENT(); }
PillSpec pillSpec(int idx){ int r=idx/2,c=idx%2; return { GRID_LEFT + c*(CELL_W+CELL_GAP_X), GRID_TOP + r*(CELL_H+CELL_GAP_Y), CELL_W, CELL_H }; }
const char* tcStateText(TCState s) {
  switch (s) {
    case TC_Applying:  return "Applying";
    case TC_Releasing: return "Changing";
    case TC_Flex:      return "Flex";
    case TC_Full:      return "Locked";
    default:           return "Unlocked";
  }
}

uint16_t tcStateColor(TCState s) {
  if (s == TC_Applying)  return ILI9341_CYAN;              // or your INFO color
  if (s == TC_Releasing) return ILI9341_CYAN;              // or AMBER
  if (s == TC_Full)      return ILI9341_GREEN;
  if (s == TC_Flex)      return ILI9341_CYAN;
  return RGB565(160,160,160);                              // Unlocked
}

static inline void copyStringToBuffer(const String& src, char* dest, size_t maxLen){
  if(maxLen == 0) return;
  size_t n = src.length();
  if(n >= maxLen) n = maxLen - 1;
  memcpy(dest, src.c_str(), n);
  dest[n] = '\0';
}

static inline int hexNibble(char c){
  if(c >= '0' && c <= '9') return c - '0';
  if(c >= 'a' && c <= 'f') return 10 + (c - 'a');
  if(c >= 'A' && c <= 'F') return 10 + (c - 'A');
  return -1;
}

static inline bool parseHexBytes(String input, uint8_t* out, size_t len){
  String filtered;
  filtered.reserve(input.length());
  for(size_t i=0;i<input.length();i++){
    char c = input.charAt(i);
    if(isxdigit((unsigned char)c)) filtered += c;
  }
  if(filtered.length() != (int)(len * 2)) return false;
  for(size_t i=0;i<len;i++){
    int hi = hexNibble(filtered.charAt(i*2));
    int lo = hexNibble(filtered.charAt(i*2 + 1));
    if(hi < 0 || lo < 0) return false;
    out[i] = (uint8_t)((hi << 4) | lo);
  }
  return true;
}

static inline void formatHexKey(const uint8_t* key, size_t len, char* out, size_t outLen){
  if(outLen == 0) return;
  size_t needed = len * 2 + 1;
  if(outLen < needed) { out[0] = '\0'; return; }
  for(size_t i=0;i<len;i++){
    snprintf(out + (i*2), outLen - (i*2), "%02X", key[i]);
  }
}

static inline void normalizeMacString(String input, char* dest, size_t maxLen){
  String filtered;
  filtered.reserve(input.length());
  for(size_t i=0;i<input.length();i++){
    char c = input.charAt(i);
    if(isxdigit((unsigned char)c)) filtered += (char)tolower((unsigned char)c);
  }
  if(filtered.length() == 12){
    String formatted;
    formatted.reserve(17);
    for(int i=0;i<6;i++){
      if(i) formatted += ':';
      formatted += filtered.substring(i*2, i*2 + 2);
    }
    copyStringToBuffer(formatted, dest, maxLen);
    return;
  }
  copyStringToBuffer(input, dest, maxLen);
}

static inline String htmlEscape(const String& input){
  String out;
  out.reserve(input.length());
  for(size_t i=0;i<input.length();i++){
    char c = input.charAt(i);
    switch(c){
      case '&': out += F("&amp;"); break;
      case '<': out += F("&lt;"); break;
      case '>': out += F("&gt;"); break;
      case '"': out += F("&quot;"); break;
      case '\'': out += F("&#39;"); break;
      default: out += c; break;
    }
  }
  return out;
}

static inline String wifiPageUrl(){
  if(!g_wifiActive) return String("WiFi offline");
  return String("http://") + g_wifiIp.toString();
}

static inline void sanitizeLayout(){
  for(int s=0;s<SCREEN_COUNT;s++){
    for(int i=0;i<4;i++) {
      if(persist.pillChannel[s][i]>=CH__COUNT) persist.pillChannel[s][i]=CH_SOOT;
      if(!isGaugeAvailable((Channel)persist.pillChannel[s][i])) persist.pillChannel[s][i]=CH_BOOST;
    }
    if(persist.barChannel[s]>=CH__COUNT || !isBarEligible((Channel)persist.barChannel[s])) persist.barChannel[s]=CH_SOOT;
  }
}

static inline void ensureWifiDefaults(){
  if(persist.wifiSsid[0] == '\0'){
    copyStringToBuffer(String(CFG::WIFI_DEFAULT_SSID), persist.wifiSsid, sizeof(persist.wifiSsid));
  }
  if(strlen(persist.wifiPass) < CFG::WIFI_MIN_PASS_LEN){
    copyStringToBuffer(String(CFG::WIFI_DEFAULT_PASS), persist.wifiPass, sizeof(persist.wifiPass));
  }
}

static inline bool isKeyBlank(const uint8_t* key, size_t len){
  bool allZero = true;
  bool allFF = true;
  for(size_t i=0;i<len;i++){
    if(key[i] != 0x00) allZero = false;
    if(key[i] != 0xFF) allFF = false;
  }
  return allZero || allFF;
}

static inline bool isMacBlank(const char* mac){
  return mac[0] == '\0' || strlen(mac) < 11;
}

static inline void ensureVictronDefaults(){
  if(isMacBlank(persist.victronBmvMac)) copyStringToBuffer(String(VictronBle::kBmvMac), persist.victronBmvMac, sizeof(persist.victronBmvMac));
  if(isMacBlank(persist.victronMpptMac)) copyStringToBuffer(String(VictronBle::kMpptMac), persist.victronMpptMac, sizeof(persist.victronMpptMac));
  if(isMacBlank(persist.victronOrionMac)) copyStringToBuffer(String(VictronBle::kOrionMac), persist.victronOrionMac, sizeof(persist.victronOrionMac));
  if(isKeyBlank(persist.victronBmvKey, sizeof(persist.victronBmvKey))) memcpy(persist.victronBmvKey, VictronBle::kBmvKey, sizeof(persist.victronBmvKey));
  if(isKeyBlank(persist.victronMpptKey, sizeof(persist.victronMpptKey))) memcpy(persist.victronMpptKey, VictronBle::kMpptKey, sizeof(persist.victronMpptKey));
  if(isKeyBlank(persist.victronOrionKey, sizeof(persist.victronOrionKey))) memcpy(persist.victronOrionKey, VictronBle::kOrionKey, sizeof(persist.victronOrionKey));
}

bool victronConfigEnabled(){ return persist.victronEnabled; }
const char* victronConfigBmvMac(){ return persist.victronBmvMac; }
const uint8_t* victronConfigBmvKey(){ return persist.victronBmvKey; }
const char* victronConfigMpptMac(){ return persist.victronMpptMac; }
const uint8_t* victronConfigMpptKey(){ return persist.victronMpptKey; }
const char* victronConfigOrionMac(){ return persist.victronOrionMac; }
const uint8_t* victronConfigOrionKey(){ return persist.victronOrionKey; }

static void startWifiAp(){
  ensureWifiDefaults();
  WiFi.mode(WIFI_AP);
  WiFi.softAP(persist.wifiSsid, persist.wifiPass);
  g_wifiIp = WiFi.softAPIP();
  g_wifiActive = true;
}

static void stopWifiAp(){
  if(!g_wifiActive) return;
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  g_wifiIp = IPAddress();
  g_wifiActive = false;
}

static void enterWifiPage(){
  if(g_wifiPageActive) return;
  g_wifiPageActive = true;
  startWifiAp();
  if(!g_webServerActive){
    setupWebServer();
    g_webServerActive = true;
  }
}

static void exitWifiPage(){
  if(!g_wifiPageActive) return;
  g_wifiPageActive = false;
  if(g_webServerActive){
    webServer.stop();
    g_webServerActive = false;
  }
  stopWifiAp();
}

static void appendOption(String& html, int value, int current, const String& label){
  html += F("<option value=\"");
  html += value;
  html += F("\"");
  if(value == current) html += F(" selected");
  html += F(">");
  html += label;
  html += F("</option>");
}

static inline String htmlColorFrom565(uint16_t color){
  uint8_t r = (color >> 11) & 0x1F;
  uint8_t g = (color >> 5) & 0x3F;
  uint8_t b = color & 0x1F;
  r = (r * 255 + 15) / 31;
  g = (g * 255 + 31) / 63;
  b = (b * 255 + 15) / 31;
  char buf[8];
  snprintf(buf, sizeof(buf), "#%02X%02X%02X", r, g, b);
  return String(buf);
}

static void appendPaletteOption(String& html, int idx, int current){
  uint16_t card;
  uint16_t frame;
  uint16_t ticks;
  uint16_t text;
  uint16_t accent;
  uint16_t bg;
  if(isCustomPaletteIndex(idx)){
    const CustomPalette& palette = persist.customPalettes[customPaletteSlot(idx)];
    card = palette.card;
    frame = palette.frame;
    ticks = palette.ticks;
    text = palette.text;
    accent = palette.accent;
    bg = palette.bg;
  }else{
    const Palette& palette = PALETTES[idx];
    card = palette.card;
    frame = palette.frame;
    ticks = palette.ticks;
    text = palette.text;
    accent = palette.accent;
    bg = palette.bg;
  }
  html += F("<option value=\"");
  html += idx;
  html += F("\"");
  if(idx == current) html += F(" selected");
  html += F(" data-card=\"");
  html += htmlColorFrom565(card);
  html += F("\" data-frame=\"");
  html += htmlColorFrom565(frame);
  html += F("\" data-ticks=\"");
  html += htmlColorFrom565(ticks);
  html += F("\" data-text=\"");
  html += htmlColorFrom565(text);
  html += F("\" data-accent=\"");
  html += htmlColorFrom565(accent);
  html += F("\" data-bg=\"");
  html += htmlColorFrom565(bg);
  html += F("\">");
  html += paletteNameForIndex(idx);
  html += F("</option>");
}

static void appendChannelOptions(String& html, uint8_t current, bool barEligible){
  for(uint8_t i=0;i<CH__COUNT;i++){
    Channel ch = (Channel)i;
    if(barEligible && !isBarEligible(ch)) continue;
    if(!barEligible && !isGaugeAvailable(ch)) continue;
    String label = String(labelText(ch));
    String unit = String(unitLabel(ch));
    if(unit.length() > 0){
      label += " (";
      label += unit;
      label += ")";
    }
    appendOption(html, i, current, label);
  }
}

static inline void appendPreviewValue(String& html, Channel ch){
  html += F("0");
  const char* unit = unitLabel(ch);
  if(unit && unit[0]){
    html += F(" ");
    html += unit;
  }
}

static void handleWebConfigPage(){
  String html;
  html.reserve(40000);
  html += F("<!doctype html><html><head><meta charset=\"utf-8\">");
  html += F("<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">");
  html += F("<title>Xiao Dash WiFi Config</title>");
  html += F("<style>body{font-family:Arial,Helvetica,sans-serif;margin:20px;color:#222}");
  html += F("h1{margin-top:0}section{margin-bottom:24px;padding:16px;border:1px solid #ddd;border-radius:8px}");
  html += F("label{display:block;margin:8px 0}input,select{padding:6px;margin-left:6px}");
  html += F("table{width:100%;border-collapse:collapse;margin-top:8px}");
  html += F("th,td{border:1px solid #ddd;padding:6px;text-align:left;font-size:12px}");
  html += F(".row{display:flex;gap:16px;flex-wrap:wrap}.row>div{flex:1 1 260px}");
  html += F(".palette-preview{margin-top:12px;display:flex;flex-wrap:wrap;gap:16px}");
  html += F(".dash-preview{width:320px;max-width:100%;height:240px;border-radius:8px;box-shadow:0 2px 8px rgba(0,0,0,0.2);overflow:hidden}");
  html += F(".dash-screen{position:relative;width:320px;height:240px;font-family:Arial,Helvetica,sans-serif;}");
  html += F(".dash-title{position:absolute;left:0;top:8px;width:100%;text-align:center;font-size:18px;font-weight:bold}");
  html += F(".dash-ticks{position:absolute;left:14px;top:44px;width:292px;display:flex;justify-content:space-between;font-size:16px;font-weight:bold}");
  html += F(".dash-bar{position:absolute;left:14px;top:64px;width:292px;height:32px;border-radius:8px;box-sizing:border-box}");
  html += F(".dash-bar-fill{height:100%;width:45%;border-radius:6px}");
  html += F(".dash-pill{position:absolute;width:144px;height:55px;border-radius:10px;padding:6px 8px;box-sizing:border-box;display:flex;flex-direction:column;justify-content:space-between}");
  html += F(".dash-pill-label{font-size:12px}");
  html += F(".dash-pill-value{font-size:16px;font-weight:bold}</style></head><body>");
  html += F("<h1>Xiao Dash Configuration</h1>");
  html += F("<p>Connect to <strong>");
  html += htmlEscape(persist.wifiSsid);
  html += F("</strong> and open <strong>");
  html += wifiPageUrl();
  html += F("</strong>.</p>");
  html += F("<form method=\"post\" action=\"/save\">");

  html += F("<section><h2>WiFi</h2>");
  html += F("<label>SSID <input name=\"ssid\" value=\"");
  html += htmlEscape(String(persist.wifiSsid));
  html += F("\"></label>");
  html += F("<label>Password <input name=\"pass\" value=\"");
  html += htmlEscape(String(persist.wifiPass));
  html += F("\"></label>");
  html += F("</section>");

  html += F("<section><h2>System</h2>");
  html += F("<label>Brightness (Lights On) <input type=\"number\" min=\"");
  html += MIN_BRIGHT;
  html += F("\" max=\"100\" name=\"brightOn\" value=\"");
  html += brightOn;
  html += F("\"></label>");
  html += F("<label>Brightness (Lights Off) <input type=\"number\" min=\"");
  html += MIN_BRIGHT;
  html += F("\" max=\"100\" name=\"brightOff\" value=\"");
  html += brightOff;
  html += F("\"></label>");
  html += F("<label>Speed Trim (%) <input type=\"number\" step=\"0.1\" min=\"");
  html += SPEED_TRIM_MIN;
  html += F("\" max=\"");
  html += SPEED_TRIM_MAX;
  html += F("\" name=\"speedTrim\" value=\"");
  html += speedTrimPct;
  html += F("\"></label>");
  html += F("<label>Current Screen <select name=\"currentScreen\">");
  for(uint8_t i=0;i<SCREEN_COUNT;i++){
    appendOption(html, i, persist.currentScreen, String(i + 1));
  }
  html += F("</select></label>");

  html += F("<label>Pressure Units <select name=\"uPressure\">");
  appendOption(html, U_P_kPa, g_uPressure, "kPa");
  appendOption(html, U_P_psi, g_uPressure, "psi");
  html += F("</select></label>");
  html += F("<label>Temperature Units <select name=\"uTemp\">");
  appendOption(html, U_T_C, g_uTemp, "C");
  appendOption(html, U_T_F, g_uTemp, "F");
  html += F("</select></label>");
  html += F("<label>Speed Units <select name=\"uSpeed\">");
  appendOption(html, U_S_kmh, g_uSpeed, "km/h");
  appendOption(html, U_S_mph, g_uSpeed, "mph");
  html += F("</select></label>");
  html += F("<label>Lambda Units <select name=\"uLambda\">");
  appendOption(html, U_L_lambda, g_uLambda, "Lambda");
  appendOption(html, U_L_AFR, g_uLambda, "AFR");
  html += F("</select></label>");
  html += F("</section>");

  html += F("<section><h2>Colours</h2>");
  html += F("<label>Palette <select name=\"paletteIndex\">");
  for(int i=0;i<paletteCount();i++){
    appendPaletteOption(html, i, paletteIndex);
  }
  html += F("</select></label>");
  html += F("<div class=\"palette-preview\">");
  html += F("<div class=\"dash-preview\">");
  html += F("<div class=\"dash-screen\" id=\"palettePreviewScreen\">");
  html += F("<div class=\"dash-title\" id=\"palettePreviewTitle\">");
  html += labelText((Channel)persist.barChannel[persist.currentScreen]);
  html += F("</div>");
  html += F("<div class=\"dash-ticks\" id=\"palettePreviewTicks\">");
  html += F("<span>0</span><span>50</span><span>100</span>");
  html += F("</div>");
  html += F("<div class=\"dash-bar\" id=\"palettePreviewBar\">");
  html += F("<div class=\"dash-bar-fill\" id=\"palettePreviewBarFill\"></div>");
  html += F("</div>");
  html += F("<div class=\"dash-pill\" id=\"palettePreviewPill1\" style=\"left:12px;top:112px;\">");
  html += F("<div class=\"dash-pill-label\">");
  html += labelText((Channel)persist.pillChannel[persist.currentScreen][0]);
  html += F("</div>");
  html += F("<div class=\"dash-pill-value\">");
  appendPreviewValue(html, (Channel)persist.pillChannel[persist.currentScreen][0]);
  html += F("</div>");
  html += F("</div>");
  html += F("<div class=\"dash-pill\" id=\"palettePreviewPill2\" style=\"left:164px;top:112px;\">");
  html += F("<div class=\"dash-pill-label\">");
  html += labelText((Channel)persist.pillChannel[persist.currentScreen][1]);
  html += F("</div>");
  html += F("<div class=\"dash-pill-value\">");
  appendPreviewValue(html, (Channel)persist.pillChannel[persist.currentScreen][1]);
  html += F("</div>");
  html += F("</div>");
  html += F("<div class=\"dash-pill\" id=\"palettePreviewPill3\" style=\"left:12px;top:177px;\">");
  html += F("<div class=\"dash-pill-label\">");
  html += labelText((Channel)persist.pillChannel[persist.currentScreen][2]);
  html += F("</div>");
  html += F("<div class=\"dash-pill-value\">");
  appendPreviewValue(html, (Channel)persist.pillChannel[persist.currentScreen][2]);
  html += F("</div>");
  html += F("</div>");
  html += F("<div class=\"dash-pill\" id=\"palettePreviewPill4\" style=\"left:164px;top:177px;\">");
  html += F("<div class=\"dash-pill-label\">");
  html += labelText((Channel)persist.pillChannel[persist.currentScreen][3]);
  html += F("</div>");
  html += F("<div class=\"dash-pill-value\">");
  appendPreviewValue(html, (Channel)persist.pillChannel[persist.currentScreen][3]);
  html += F("</div>");
  html += F("</div>");
  html += F("</div></div></div>");
  html += F("<h3>Custom Palette Editor</h3>");
  html += F("<label>Custom Palette <select name=\"customPaletteSlot\" id=\"customPaletteSlot\">");
  for(uint8_t p=0;p<CUSTOM_PALETTE_COUNT;p++){
    appendOption(html, p, customPaletteSel, String(CUSTOM_PALETTE_NAMES[p]));
  }
  html += F("</select></label>");
  html += F("<label>Zone <select name=\"customZone\" id=\"customZone\">");
  for(uint8_t z=0;z<CUSTOM_ZONE_COUNT;z++){
    appendOption(html, z, customZoneSel, String(CUSTOM_ZONE_LABELS[z]));
  }
  html += F("</select></label>");
  html += F("<select id=\"customZoneValue\" style=\"display:none\">");
  for(uint8_t p=0;p<CUSTOM_PALETTE_COUNT;p++){
    for(uint8_t z=0;z<CUSTOM_ZONE_COUNT;z++){
      html += F("<option data-key=\"");
      html += p;
      html += F("_");
      html += z;
      html += F("\" value=\"");
      html += customZoneValue(persist.customPalettes[p], z);
      html += F("\"></option>");
    }
  }
  html += F("</select>");
  uint16_t currentValue = customZoneValue(persist.customPalettes[customPaletteSel], customZoneSel);
  html += F("<input type=\"hidden\" name=\"customColorValue\" id=\"customColorValue\" value=\"");
  html += currentValue;
  html += F("\">");
  html += F("<div>");
  html += F("<label>Red <input type=\"range\" min=\"0\" max=\"255\" id=\"customColorR\"></label>");
  html += F("<label>Green <input type=\"range\" min=\"0\" max=\"255\" id=\"customColorG\"></label>");
  html += F("<label>Blue <input type=\"range\" min=\"0\" max=\"255\" id=\"customColorB\"></label>");
  html += F("<div id=\"customColorPreview\" style=\"display:inline-block;width:24px;height:24px;border:1px solid #ccc;vertical-align:middle;margin-right:8px;\"></div>");
  html += F("<span id=\"customColorHex\"></span>");
  html += F("</div>");
  html += F("</section>");

  html += F("<section><h2>Layout</h2>");
  for(uint8_t s=0;s<SCREEN_COUNT;s++){
    html += F("<h3>Screen ");
    html += (s + 1);
    html += F("</h3>");
    html += F("<label>Bar <select name=\"bar_s");
    html += s;
    html += F("\">");
    appendChannelOptions(html, persist.barChannel[s], true);
    html += F("</select></label>");
    for(uint8_t i=0;i<4;i++){
      html += F("<label>Pill ");
      html += (i + 1);
      html += F(" <select name=\"pill_s");
      html += s;
      html += F("_");
      html += i;
      html += F("\">");
      appendChannelOptions(html, persist.pillChannel[s][i], false);
      html += F("</select></label>");
    }
  }
  html += F("</section>");

  html += F("<section><h2>Warnings</h2>");
  html += F("<table><tr><th>Channel</th><th>Mode</th><th>Threshold 1</th><th>Threshold 2</th></tr>");
  for(uint8_t i=0;i<CH__COUNT;i++){
    Channel ch = (Channel)i;
    if(!isWarnEligible(ch)) continue;
    html += F("<tr><td>");
    html += labelText(ch);
    html += F("</td><td><select name=\"warnMode_");
    html += i;
    html += F("\">");
    appendOption(html, CFG::WARN_OFF, persist.warnMode[i], "Off");
    appendOption(html, CFG::WARN_HIGH, persist.warnMode[i], "High");
    appendOption(html, CFG::WARN_LOW, persist.warnMode[i], "Low");
    html += F("</select></td><td><input name=\"warnT1_");
    html += i;
    html += F("\" value=\"");
    html += persist.warnT1[i];
    html += F("\"></td><td><input name=\"warnT2_");
    html += i;
    html += F("\" value=\"");
    html += persist.warnT2[i];
    html += F("\"></td></tr>");
  }
  html += F("</table></section>");

  html += F("<section><h2>Victron</h2>");
  html += F("<label><input type=\"checkbox\" name=\"victronEnabled\" value=\"1\"");
  if(persist.victronEnabled) html += F(" checked");
  html += F("> Enable Victron Gauges</label>");
  char keyBuf[33];
  formatHexKey(persist.victronBmvKey, sizeof(persist.victronBmvKey), keyBuf, sizeof(keyBuf));
  html += F("<label>BMV MAC <input name=\"bmvMac\" value=\"");
  html += htmlEscape(String(persist.victronBmvMac));
  html += F("\"></label>");
  html += F("<label>BMV Key (32 hex) <input name=\"bmvKey\" value=\"");
  html += keyBuf;
  html += F("\"></label>");
  formatHexKey(persist.victronMpptKey, sizeof(persist.victronMpptKey), keyBuf, sizeof(keyBuf));
  html += F("<label>MPPT MAC <input name=\"mpptMac\" value=\"");
  html += htmlEscape(String(persist.victronMpptMac));
  html += F("\"></label>");
  html += F("<label>MPPT Key (32 hex) <input name=\"mpptKey\" value=\"");
  html += keyBuf;
  html += F("\"></label>");
  formatHexKey(persist.victronOrionKey, sizeof(persist.victronOrionKey), keyBuf, sizeof(keyBuf));
  html += F("<label>Orion MAC <input name=\"orionMac\" value=\"");
  html += htmlEscape(String(persist.victronOrionMac));
  html += F("\"></label>");
  html += F("<label>Orion Key (32 hex) <input name=\"orionKey\" value=\"");
  html += keyBuf;
  html += F("\"></label>");
  html += F("</section>");

  html += F("<button type=\"submit\">Save Configuration</button></form>");
  html += F("<script>");
  html += F("const paletteSelect=document.querySelector('select[name=\"paletteIndex\"]');");
  html += F("function applyPalette(opt){if(!opt)return;const d=opt.dataset;");
  html += F("const setBg=(id,val)=>{const el=document.getElementById(id);if(el)el.style.backgroundColor=val;};");
  html += F("const setColor=(id,val)=>{const el=document.getElementById(id);if(el)el.style.color=val;};");
  html += F("const setBorder=(id,val)=>{const el=document.getElementById(id);if(el)el.style.borderColor=val;};");
  html += F("const hex2=(v)=>v.toString(16).padStart(2,'0');");
  html += F("const to565=(r,g,b)=>(((r&248)<<8)|((g&252)<<3)|(b>>3));");
  html += F("function updateCustomRgb(){");
  html += F("const rEl=document.getElementById('customColorR');");
  html += F("const gEl=document.getElementById('customColorG');");
  html += F("const bEl=document.getElementById('customColorB');");
  html += F("const out=document.getElementById('customColorValue');");
  html += F("const preview=document.getElementById('customColorPreview');");
  html += F("const hex=document.getElementById('customColorHex');");
  html += F("if(!rEl||!gEl||!bEl||!out)return;");
  html += F("const r=parseInt(rEl.value||'0',10);");
  html += F("const g=parseInt(gEl.value||'0',10);");
  html += F("const b=parseInt(bEl.value||'0',10);");
  html += F("const rgb565=to565(r,g,b);");
  html += F("out.value=rgb565;");
  html += F("if(preview) preview.style.backgroundColor=`#${hex2(r)}${hex2(g)}${hex2(b)}`;");
  html += F("if(hex) hex.textContent=`#${hex2(r)}${hex2(g)}${hex2(b)}`;");
  html += F("}");
  html += F("function syncCustomRgbFromValue(){");
  html += F("const out=document.getElementById('customColorValue');");
  html += F("const rEl=document.getElementById('customColorR');");
  html += F("const gEl=document.getElementById('customColorG');");
  html += F("const bEl=document.getElementById('customColorB');");
  html += F("if(!out||!rEl||!gEl||!bEl)return;");
  html += F("const v=parseInt(out.value||'0',10);");
  html += F("const r=Math.round(((v>>11)&31)*255/31);");
  html += F("const g=Math.round(((v>>5)&63)*255/63);");
  html += F("const b=Math.round((v&31)*255/31);");
  html += F("rEl.value=r; gEl.value=g; bEl.value=b;");
  html += F("updateCustomRgb();");
  html += F("}");
  html += F("function setZoneValueOption(paletteIdx, zoneIdx, value){");
  html += F("const select=document.getElementById('customZoneValue');");
  html += F("if(!select) return;");
  html += F("const key=String(paletteIdx)+'_'+String(zoneIdx);");
  html += F("let opt=select.querySelector(`option[data-key='${key}']`);");
  html += F("if(!opt){opt=document.createElement('option');opt.dataset.key=key;select.appendChild(opt);}");
  html += F("opt.value=value;select.value=value;");
  html += F("}");
  html += F("function refreshCustomColor(){");
  html += F("const slotEl=document.getElementById('customPaletteSlot');");
  html += F("const zoneEl=document.getElementById('customZone');");
  html += F("const valueEl=document.getElementById('customZoneValue');");
  html += F("const out=document.getElementById('customColorValue');");
  html += F("if(!slotEl||!zoneEl||!valueEl||!out) return;");
  html += F("const key=String(slotEl.value)+'_'+String(zoneEl.value);");
  html += F("const opt=valueEl.querySelector(`option[data-key='${key}']`);");
  html += F("const value=opt?parseInt(opt.value,10):parseInt(out.value||'0',10);");
  html += F("out.value=isNaN(value)?0:value;");
  html += F("syncCustomRgbFromValue();");
  html += F("}");
  html += F("['customColorR','customColorG','customColorB'].forEach(id=>{const el=document.getElementById(id);if(el)el.addEventListener('input',updateCustomRgb);});");
  html += F("const paletteSlotEl=document.getElementById('customPaletteSlot');");
  html += F("const zoneEl=document.getElementById('customZone');");
  html += F("if(paletteSlotEl) paletteSlotEl.addEventListener('change',refreshCustomColor);");
  html += F("if(zoneEl) zoneEl.addEventListener('change',refreshCustomColor);");
  html += F("setBg('palettePreviewScreen', d.bg);");
  html += F("setColor('palettePreviewTitle', d.text);");
  html += F("setColor('palettePreviewTicks', d.ticks);");
  html += F("setBg('palettePreviewBar', d.card);");
  html += F("setBg('palettePreviewBarFill', d.accent);");
  html += F("['palettePreviewPill1','palettePreviewPill2','palettePreviewPill3','palettePreviewPill4'].forEach(id=>setBg(id,d.card));");
  html += F("['palettePreviewPill1','palettePreviewPill2','palettePreviewPill3','palettePreviewPill4','palettePreviewBar'].forEach(id=>setBorder(id,d.frame));");
  html += F("document.querySelectorAll('.dash-pill,.dash-bar').forEach(el=>{if(el){el.style.borderWidth='2px';el.style.borderStyle='solid';}});");
  html += F("document.querySelectorAll('.dash-pill-label,.dash-pill-value').forEach(el=>{if(el)el.style.color=d.text;});");
  html += F("}");
  html += F("if(paletteSelect){paletteSelect.addEventListener('change',()=>applyPalette(paletteSelect.selectedOptions[0]));applyPalette(paletteSelect.selectedOptions[0]);}");
  html += F("syncCustomRgbFromValue();");
  html += F("refreshCustomColor();");
  html += F("</script></body></html>");
  webServer.send(200, "text/html", html);
}

static void handleWebConfigSave(){
  bool wifiChanged = false;
  if(webServer.hasArg("ssid")){
    String s = webServer.arg("ssid");
    s.trim();
    if(s.length() > 0){
      copyStringToBuffer(s, persist.wifiSsid, sizeof(persist.wifiSsid));
      wifiChanged = true;
    }
  }
  if(webServer.hasArg("pass")){
    String s = webServer.arg("pass");
    s.trim();
    if(s.length() >= CFG::WIFI_MIN_PASS_LEN){
      copyStringToBuffer(s, persist.wifiPass, sizeof(persist.wifiPass));
      wifiChanged = true;
    }
  }

  if(webServer.hasArg("brightOn")){
    int v = webServer.arg("brightOn").toInt();
    brightOn = (uint8_t)clampf(v, MIN_BRIGHT, 100);
  }
  if(webServer.hasArg("brightOff")){
    int v = webServer.arg("brightOff").toInt();
    brightOff = (uint8_t)clampf(v, MIN_BRIGHT, 100);
  }
  if(webServer.hasArg("speedTrim")){
    speedTrimPct = clampf(webServer.arg("speedTrim").toFloat(), SPEED_TRIM_MIN, SPEED_TRIM_MAX);
  }
  if(webServer.hasArg("currentScreen")){
    int idx = webServer.arg("currentScreen").toInt();
    if(idx >= 0 && idx < SCREEN_COUNT) persist.currentScreen = idx;
  }
  if(webServer.hasArg("uPressure")){
    int v = webServer.arg("uPressure").toInt();
    if(v == U_P_kPa || v == U_P_psi) g_uPressure = v;
  }
  if(webServer.hasArg("uTemp")){
    int v = webServer.arg("uTemp").toInt();
    if(v == U_T_C || v == U_T_F) g_uTemp = v;
  }
  if(webServer.hasArg("uSpeed")){
    int v = webServer.arg("uSpeed").toInt();
    if(v == U_S_kmh || v == U_S_mph) g_uSpeed = v;
  }
  if(webServer.hasArg("uLambda")){
    int v = webServer.arg("uLambda").toInt();
    if(v == U_L_lambda || v == U_L_AFR) g_uLambda = v;
  }

  if(webServer.hasArg("paletteIndex")){
    int idx = webServer.arg("paletteIndex").toInt();
    if(idx >= 0 && idx < paletteCount()) paletteIndex = idx;
  }
  if(webServer.hasArg("customPaletteSlot") && webServer.hasArg("customZone") && webServer.hasArg("customColorValue")){
    int slot = webServer.arg("customPaletteSlot").toInt();
    int zone = webServer.arg("customZone").toInt();
    int value = webServer.arg("customColorValue").toInt();
    if(slot >= 0 && slot < CUSTOM_PALETTE_COUNT && zone >= 0 && zone < CUSTOM_ZONE_COUNT){
      setCustomZoneValue(persist.customPalettes[slot], (uint8_t)zone, (uint16_t)value);
    }
  }

  for(uint8_t s=0;s<SCREEN_COUNT;s++){
    String barName = String("bar_s") + String(s);
    if(webServer.hasArg(barName)){
      int v = webServer.arg(barName).toInt();
      if(v >= 0 && v < CH__COUNT) persist.barChannel[s] = (uint8_t)v;
    }
    for(uint8_t i=0;i<4;i++){
      String pillName = String("pill_s") + String(s) + "_" + String(i);
      if(webServer.hasArg(pillName)){
        int v = webServer.arg(pillName).toInt();
        if(v >= 0 && v < CH__COUNT) persist.pillChannel[s][i] = (uint8_t)v;
      }
    }
  }

  for(uint8_t i=0;i<CH__COUNT;i++){
    Channel ch = (Channel)i;
    if(!isWarnEligible(ch)) continue;
    String modeName = String("warnMode_") + String(i);
    String t1Name = String("warnT1_") + String(i);
    String t2Name = String("warnT2_") + String(i);
    if(webServer.hasArg(modeName)) persist.warnMode[i] = (uint8_t)webServer.arg(modeName).toInt();
    if(webServer.hasArg(t1Name)) persist.warnT1[i] = webServer.arg(t1Name).toFloat();
    if(webServer.hasArg(t2Name)) persist.warnT2[i] = webServer.arg(t2Name).toFloat();
  }

  persist.victronEnabled = webServer.hasArg("victronEnabled") ? 1 : 0;
  if(webServer.hasArg("bmvMac")) normalizeMacString(webServer.arg("bmvMac"), persist.victronBmvMac, sizeof(persist.victronBmvMac));
  if(webServer.hasArg("mpptMac")) normalizeMacString(webServer.arg("mpptMac"), persist.victronMpptMac, sizeof(persist.victronMpptMac));
  if(webServer.hasArg("orionMac")) normalizeMacString(webServer.arg("orionMac"), persist.victronOrionMac, sizeof(persist.victronOrionMac));
  if(webServer.hasArg("bmvKey")) parseHexBytes(webServer.arg("bmvKey"), persist.victronBmvKey, sizeof(persist.victronBmvKey));
  if(webServer.hasArg("mpptKey")) parseHexBytes(webServer.arg("mpptKey"), persist.victronMpptKey, sizeof(persist.victronMpptKey));
  if(webServer.hasArg("orionKey")) parseHexBytes(webServer.arg("orionKey"), persist.victronOrionKey, sizeof(persist.victronOrionKey));

  sanitizeLayout();
  applyBacklight();
  redrawForDimmingChange();
  persist.paletteIndex = paletteIndex;
  persist.brightOn  = brightOn;
  persist.brightOff = brightOff;
  persist.uPressure = g_uPressure;
  persist.uTemp     = g_uTemp;
  persist.uSpeed    = g_uSpeed;
  persist.uLambda   = g_uLambda;
  persist.speedTrimPct = speedTrimPct;
  savePersist(persist, dirty, true);

  if(wifiChanged && g_wifiPageActive) startWifiAp();
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

static void setupWebServer(){
  webServer.on("/", HTTP_GET, handleWebConfigPage);
  webServer.on("/save", HTTP_POST, handleWebConfigSave);
  webServer.begin();
}

// ===================== EEPROM =====================
static PersistState buildDefaultPersistState(){
  PersistState def{};
  def.magic = Persist::EEPROM_MAGIC;
  def.version = Persist::SCHEMA_VERSION;
  uint8_t defP[SCREEN_COUNT][4] = {
    {CH_SPEED, CH_COOLANT, CH_BOOST, CH_BATTV},
    {CH_RPM,   CH_EGT1,    CH_EGT2,  CH_FUELT},
    {CH_SOOT,  CH_IAT,     CH_MANIFOLD, CH_TURBO_OUT},
    {CH_SPEED, CH_COOLANT, CH_BOOST, CH_BATTV},
    {CH_RPM,   CH_EGT1,    CH_EGT2,  CH_FUELT}
  };
  for(int s=0;s<SCREEN_COUNT;s++) for(int i=0;i<4;i++) def.pillChannel[s][i]=defP[s][i];
  def.barChannel[0]=CH_SOOT; def.barChannel[1]=CH_BOOST; def.barChannel[2]=CH_RPM;
  def.barChannel[3]=CH_SOOT; def.barChannel[4]=CH_BOOST;
  def.currentScreen=0;
  for(int i=0;i<CH__COUNT;i++){ def.warnMode[i]=CFG::WARN_OFF; def.warnT1[i]=0; def.warnT2[i]=0; }
  def.warnMode[CH_SOOT]=CFG::WARN_HIGH; def.warnT1[CH_SOOT]=95; def.warnT2[CH_SOOT]=100;
  def.warnMode[CH_COOLANT]=CFG::WARN_HIGH; def.warnT1[CH_COOLANT]=100; def.warnT2[CH_COOLANT]=105;
  def.warnMode[CH_BATTV]=CFG::WARN_LOW;  def.warnT1[CH_BATTV]=11.5f; def.warnT2[CH_BATTV]=10.8f;
  def.warnMode[CH_TRANS1]=CFG::WARN_HIGH; def.warnT1[CH_TRANS1]=105; def.warnT2[CH_TRANS1]=110;
  def.warnMode[CH_TRANS2]=CFG::WARN_HIGH; def.warnT1[CH_TRANS2]=115; def.warnT2[CH_TRANS2]=120;
  def.warnMode[CH_RPM]=CFG::WARN_HIGH; def.warnT1[CH_RPM]=3300; def.warnT2[CH_RPM]=3600;
  def.paletteIndex=0;
  for(uint8_t i=0;i<CUSTOM_PALETTE_COUNT;i++){
    const Palette& src = PALETTES[min<int>(i, basePaletteCount()-1)];
    def.customPalettes[i] = { src.card, src.frame, src.ticks, src.text, src.accent, src.bg };
  }
  def.brightOn=70; def.brightOff=100;
  def.uPressure = U_P_kPa; def.uTemp = U_T_C; def.uSpeed = U_S_kmh; def.uLambda = U_L_lambda;
  def.speedTrimPct = 0.0f;
  def.victronEnabled = 1;
  copyStringToBuffer(String(CFG::WIFI_DEFAULT_SSID), def.wifiSsid, sizeof(def.wifiSsid));
  copyStringToBuffer(String(CFG::WIFI_DEFAULT_PASS), def.wifiPass, sizeof(def.wifiPass));
  copyStringToBuffer(String(VictronBle::kBmvMac), def.victronBmvMac, sizeof(def.victronBmvMac));
  copyStringToBuffer(String(VictronBle::kMpptMac), def.victronMpptMac, sizeof(def.victronMpptMac));
  copyStringToBuffer(String(VictronBle::kOrionMac), def.victronOrionMac, sizeof(def.victronOrionMac));
  memcpy(def.victronBmvKey, VictronBle::kBmvKey, sizeof(def.victronBmvKey));
  memcpy(def.victronMpptKey, VictronBle::kMpptKey, sizeof(def.victronMpptKey));
  memcpy(def.victronOrionKey, VictronBle::kOrionKey, sizeof(def.victronOrionKey));
  return def;
}

static void loadPersistState(){
  PersistState defaults = buildDefaultPersistState();
  loadPersist(persist, defaults);
  sanitizeLayout();
  persist.currentScreen = (persist.currentScreen>=SCREEN_COUNT)?0:persist.currentScreen;
  paletteIndex = (persist.paletteIndex >= paletteCount()) ? 0 : persist.paletteIndex;

  // apply persisted brightness to RAM (clamped)
  brightOn  = max<uint8_t>(persist.brightOn,  MIN_BRIGHT);
  brightOff = max<uint8_t>(persist.brightOff, MIN_BRIGHT);

  // units
  g_uPressure = (persist.uPressure>1)?U_P_kPa:persist.uPressure;
  g_uTemp     = (persist.uTemp>1)?U_T_C:persist.uTemp;
  g_uSpeed    = (persist.uSpeed>1)?U_S_kmh:persist.uSpeed;
  g_uLambda   = (persist.uLambda>1)?U_L_lambda:persist.uLambda;
  if (!isfinite(persist.speedTrimPct)) persist.speedTrimPct = 0.0f;
  speedTrimPct = persist.speedTrimPct;
  if(persist.victronEnabled > 1) persist.victronEnabled = 1;
  ensureWifiDefaults();
  ensureVictronDefaults();
}

// ===================== Backlight helpers (PWM on D0, active high) =====================
inline void applyBacklight(){
  uint8_t pct = uiBrightnessPct();
  if (pct == g_lastBacklightPct) return;
  g_lastBacklightPct = pct;
  uint16_t duty = (uint16_t)((pct * 255 + 50) / 100);
  analogWrite(CFG::BACKLIGHT_PWM, duty);
}

// ===================== Drawing – Main UI =====================

inline void drawAppBar(){
  tft.fillRect(0,0,320,APPBAR_H,COL_CARD()); tft.drawFastHLine(0,APPBAR_H,320,COL_ACCENT());
}
inline void drawTitle(const char* mainTxt){
  clearRegion(0,0,320,APPBAR_H,COL_CARD()); tft.drawFastHLine(0,APPBAR_H,320,COL_ACCENT());
  if(!mainTxt) return;
  tft.setFont(&FreeSans12pt7b); tft.setTextColor(COL_TXT(),COL_CARD());
  int16_t x1,y1; uint16_t w,h; tft.getTextBounds((char*)mainTxt,0,0,&x1,&y1,&w,&h); tft.setCursor((320-(int)w)/2,24); tft.print(mainTxt);
  tft.setFont();
}
void drawPillFrame(const PillSpec& p, bool sel, Channel ch){
  (void)ch;
  tft.drawRoundRect(p.x,p.y,p.w,p.h,10,COL_CARD()); tft.drawRoundRect(p.x+1,p.y+1,p.w-2,p.h-2,9,COL_CARD());
  tft.drawRoundRect(p.x+2, p.y+2, p.w-4, p.h-4,  8,COL_CARD());tft.drawRoundRect(p.x+3,p.y+3,p.w-6,p.h-6,7,COL_CARD());
  uint16_t fc= sel? COL_YELLOW(): COL_FRAME();
  tft.drawRoundRect(p.x,p.y,p.w,p.h,10,fc); if(sel) tft.drawRoundRect(p.x+1,p.y+1,p.w-2,p.h-2,9,fc);
}
void drawPillLabel(const PillSpec& p, const char* s){ tft.setFont(&FreeSans9pt7b); tft.setTextColor(COL_TXT(),COL_CARD()); tft.setCursor(p.x+10,p.y+17); tft.print(s); tft.setFont(); }
void drawPillLabelWithSuffix(const PillSpec& p, const char* s, const char* suffix){
  tft.setFont(&FreeSans9pt7b);
  tft.setTextColor(COL_TXT(),COL_CARD());
  tft.setCursor(p.x+10,p.y+17);
  tft.print(s);
  if(suffix && suffix[0]){
    int16_t x1,y1; uint16_t w,h;
    tft.getTextBounds((char*)s, p.x+10, p.y+17, &x1, &y1, &w, &h);
    tft.setFont(&FreeSans9pt7b);
    tft.setCursor(p.x+10 + (int)w + 4, p.y+16);
    tft.print(suffix);
  }
  tft.setFont();
}
void drawPillLabelForChannel(const PillSpec& p, Channel ch){
  const char* base = labelText(ch);
  if(uiMinMaxActive){
    const char* suffix = minMaxSuffixFor(ch);
    if(suffix[0]){
      drawPillLabelWithSuffix(p, base, suffix);
      return;
    }
  }
  drawPillLabel(p, base);
}
void clearPillValue(const PillSpec& p){ int y=p.y+25; clearRegion(p.x+6,y,p.w-12,max(0,p.y+p.h-y-4),COL_CARD()); }
void drawValueInPill(const PillSpec& p, const char* num, const char* unit){
  clearPillValue(p); tft.setFont(&FreeSans12pt7b); tft.setTextColor(COL_TXT(),COL_CARD()); int nx=p.x+10, ny=p.y+p.h-10; tft.setCursor(nx,ny); tft.print(num);
  if(unit&&unit[0]){ int16_t bx,by; uint16_t bw,bh; tft.getTextBounds(num,nx,ny,&bx,&by,&bw,&bh); tft.setCursor(nx+bw+8,ny); tft.print(unit); }
  tft.setFont();
}
// Draw bar ticks for an arbitrary screen (used in editor preview)
inline void drawBarStaticForScreen(uint8_t screenSel, bool sel){
  Channel ch=(Channel)persist.barChannel[screenSel]; uint16_t fc=sel?COL_YELLOW():COL_FRAME();
  tft.fillRoundRect(BAR_X-2,BAR_Y-2,BAR_W+4,BAR_H+4,BAR_R+2,COL_CARD()); tft.drawRoundRect(BAR_X,BAR_Y,BAR_W,BAR_H,BAR_R,fc);
  auto tick=[&](int x,float v,const char* u){ char b[24];
    if(ch==CH_BATTV) snprintf(b,sizeof(b),"%.1f",v);
    else if(ch==CH_BATTV2 || ch==CH_DCDC_OUT_V || ch==CH_DCDC_IN_V) snprintf(b,sizeof(b),"%.2f",v);
    else if(u&&u[0]=='V') snprintf(b,sizeof(b),"%.2f",v);
    else if(ch==CH_LAMBDA){
      if(g_uLambda==U_L_lambda) snprintf(b,sizeof(b),"%.2f",v);
      else snprintf(b,sizeof(b),"%.1f",v);
    }
    else snprintf(b,sizeof(b),"%.0f",v);
    tft.setFont(&FreeSans12pt7b); tft.setTextColor(COL_TICKS(),COL_BG()); tft.setCursor(x,BAR_Y-6); tft.print(b); tft.setFont(); };
  Range r=rangeFor(ch); float mid=(r.mn+r.mx)/2; clearRegion(0,BAR_Y-20,320,18,COL_BG());
  char tmp[16]; if(ch==CH_LAMBDA && g_uLambda==U_L_lambda) snprintf(tmp,sizeof(tmp),"%.2f", mid); else snprintf(tmp,sizeof(tmp),"%.0f", mid);
  int16_t x1,y1; uint16_t w,h; tft.setFont(&FreeSans12pt7b); tft.getTextBounds(tmp,0,0,&x1,&y1,&w,&h); tft.setFont();
  tick(BAR_X,r.mn,unitLabel(ch)); tick(BAR_X+BAR_W/2-(int)w/2,mid,unitLabel(ch));
  if(ch==CH_LAMBDA && g_uLambda==U_L_lambda) snprintf(tmp,sizeof(tmp),"%.2f", r.mx); else snprintf(tmp,sizeof(tmp),"%.0f", r.mx);
  tft.setFont(&FreeSans12pt7b); tft.getTextBounds(tmp,0,0,&x1,&y1,&w,&h); tft.setFont();
  tick(BAR_X+BAR_W-(int)w-4,r.mx,unitLabel(ch));
}

inline float displayValueForChannel(Channel ch, float baseValue){
  switch(ch){
    case CH_SPEED: return toDisplaySpeed(baseValue);
    case CH_COOLANT: case CH_TRANS1: case CH_TRANS2: case CH_IAT: case CH_FUELT:
    case CH_MANIFOLD: case CH_TURBO_OUT: case CH_EGT1: case CH_EGT2:
      return toDisplayTemp(baseValue);
    case CH_BOOST: case CH_OIL: return toDisplayPressure(baseValue);
    case CH_LAMBDA: return toDisplayLambda(baseValue);
    default: return baseValue;
  }
}

int valueKeyForDisplay(Channel ch, float displayValue){
  switch(ch){
    case CH_BATTV: return (int)lroundf(displayValue * 10.0f);
    case CH_BATTV2: return (int)lroundf(displayValue * 100.0f);
    case CH_BATT_CURR: return (int)lroundf(displayValue * 10.0f);
    case CH_DCDC_OUT_A: return (int)lroundf(displayValue * 10.0f);
    case CH_DCDC_OUT_V: return (int)lroundf(displayValue * 100.0f);
    case CH_DCDC_IN_V: return (int)lroundf(displayValue * 100.0f);
    case CH_PV_AMPS: return (int)lroundf(displayValue * 10.0f);
    case CH_PV_YIELD: return (int)lroundf(displayValue * 100.0f);
    case CH_LAMBDA:
      if(g_uLambda==U_L_lambda) return (int)lroundf(displayValue * 100.0f);
      return (int)lroundf(displayValue * 10.0f);
    default: return (int)lroundf(displayValue);
  }
}

const char* minMaxSuffixFor(Channel ch){
  MinMaxMode mode = minMaxModeFor(ch);
  if(mode == MINMAX_MIN) return "(min)";
  if(mode == MINMAX_MAX) return "(max)";
  return "";
}

float minMaxDisplayValue(Channel ch){
  MinMaxMode mode = minMaxModeFor(ch);
  if(mode == MINMAX_NONE) return valueDisplay(ch);
  float baseValue = (mode == MINMAX_MIN) ? uiMinValues[ch] : uiMaxValues[ch];
  if(!uiMinMaxHas[ch] || !isfinite(baseValue)) baseValue = valueRawBase(ch);
  return displayValueForChannel(ch, baseValue);
}

int valueKey(Channel ch){
  switch(ch){
    case CH_SOOT: return (int)lroundf(soot_pct); 
    case CH_SPEED: return (int)lroundf(valueDisplay(CH_SPEED));
    case CH_RPM: return (int)lroundf(rpm);
    case CH_COOLANT: return (int)lroundf(valueDisplay(CH_COOLANT));
    case CH_TRANS1: return (int)lroundf(valueDisplay(CH_TRANS1));
    case CH_TRANS2: return (int)lroundf(valueDisplay(CH_TRANS2));
    case CH_OIL: return (int)lroundf(valueDisplay(CH_OIL));
    case CH_BATTV: return (int)lroundf(battV*100.0f);
    case CH_BATT_SOC: return isfinite(g_victronReadings.battSocPct) ? (int)lroundf(g_victronReadings.battSocPct) : INT32_MIN;
    case CH_BATT_CURR: return isfinite(g_victronReadings.battCurrentA) ? (int)lroundf(g_victronReadings.battCurrentA * 10.0f) : INT32_MIN;
    case CH_BATT_TTG: return isfinite(g_victronReadings.battTimeMin) ? (int)lroundf(g_victronReadings.battTimeMin) : INT32_MIN;
    case CH_BATTV2: return isfinite(g_victronReadings.battV2) ? (int)lroundf(g_victronReadings.battV2 * 100.0f) : INT32_MIN;
    case CH_DCDC_OUT_A: return isfinite(g_victronReadings.dcdcOutA) ? (int)lroundf(g_victronReadings.dcdcOutA * 10.0f) : INT32_MIN;
    case CH_DCDC_OUT_V: return isfinite(g_victronReadings.dcdcOutV) ? (int)lroundf(g_victronReadings.dcdcOutV * 100.0f) : INT32_MIN;
    case CH_DCDC_IN_V: return isfinite(g_victronReadings.dcdcInV) ? (int)lroundf(g_victronReadings.dcdcInV * 100.0f) : INT32_MIN;
    case CH_PV_WATTS: return isfinite(g_victronReadings.pvWatts) ? (int)lroundf(g_victronReadings.pvWatts) : INT32_MIN;
    case CH_PV_AMPS: return isfinite(g_victronReadings.pvAmps) ? (int)lroundf(g_victronReadings.pvAmps * 10.0f) : INT32_MIN;
    case CH_PV_YIELD: return isfinite(g_victronReadings.pvYieldKwh) ? (int)lroundf(g_victronReadings.pvYieldKwh * 100.0f) : INT32_MIN;
    case CH_GEAR: return ((gear & 0xFF) << 8) | (targetgear & 0xFF);
    case CH_LOCKUP: return (int)g_tcState;
    case CH_TORQUE: return (int)lroundf(torqueNm);
    case CH_PEDAL: return (int)lroundf(pedalPct); case CH_TQ_DEMAND: return (int)lroundf(tqDemandPct);
    case CH_EGT1: return (int)lroundf(valueDisplay(CH_EGT1));
    case CH_EGT2: return (int)lroundf(valueDisplay(CH_EGT2));
    case CH_BOOST: return (int)lroundf(valueDisplay(CH_BOOST));
    case CH_MANIFOLD: return (int)lroundf(valueDisplay(CH_MANIFOLD));
    case CH_TURBO_OUT: return (int)lroundf(valueDisplay(CH_TURBO_OUT));
    case CH_LAMBDA:
      if(g_uLambda==U_L_lambda) return (int)lroundf(valueDisplay(CH_LAMBDA)*100.0f);
      else return (int)lroundf(valueDisplay(CH_LAMBDA)*10.0f);
    case CH_IAT: return (int)lroundf(valueDisplay(CH_IAT));
    case CH_FUELT: return (int)lroundf(valueDisplay(CH_FUELT));
    case CH_ACTUATOR: return (int)turboActRaw;
    case CH_HEADLIGHTS: return headlightsOn?1:0;
    default: return INT32_MIN;
  }
}

inline void resetMinMaxValues(){
  for(int i=0;i<CH__COUNT;i++){
    uiMinValues[i] = NAN;
    uiMaxValues[i] = NAN;
    uiMinMaxHas[i] = false;
  }
}

inline void updateMinMaxValues(){
  for(int i=0;i<CH__COUNT;i++){
    Channel ch = (Channel)i;
    MinMaxMode mode = minMaxModeFor(ch);
    if(mode == MINMAX_NONE) continue;
    float baseValue = valueRawBase(ch);
    if(!isfinite(baseValue)) continue;
    if(!uiMinMaxHas[i]){
      uiMinValues[i] = baseValue;
      uiMaxValues[i] = baseValue;
      uiMinMaxHas[i] = true;
      continue;
    }
    if(mode == MINMAX_MIN){
      if(baseValue < uiMinValues[i]) uiMinValues[i] = baseValue;
    } else if(mode == MINMAX_MAX){
      if(baseValue > uiMaxValues[i]) uiMaxValues[i] = baseValue;
    }
  }
}

void formatDisplayValue(Channel ch, float displayValue, char* out, size_t outSize){
  if((ch == CH_BATT_SOC || ch == CH_BATT_CURR || ch == CH_BATT_TTG || ch == CH_BATTV2
      || ch == CH_DCDC_OUT_A || ch == CH_DCDC_OUT_V || ch == CH_DCDC_IN_V
      || ch == CH_PV_WATTS || ch == CH_PV_AMPS || ch == CH_PV_YIELD)
      && !isfinite(displayValue)){
    snprintf(out, outSize, "--");
    return;
  }
  if(ch == CH_BATTV){
    snprintf(out, outSize, "%.1f", displayValue);
  } else if(ch == CH_BATTV2){
    snprintf(out, outSize, "%.2f", displayValue);
  } else if(ch == CH_BATT_CURR || ch == CH_DCDC_OUT_A || ch == CH_PV_AMPS){
    snprintf(out, outSize, "%.1f", displayValue);
  } else if(ch == CH_DCDC_OUT_V || ch == CH_DCDC_IN_V){
    snprintf(out, outSize, "%.2f", displayValue);
  } else if(ch == CH_PV_YIELD){
    snprintf(out, outSize, "%.2f", displayValue);
  } else if(ch == CH_LAMBDA){
    if(g_uLambda==U_L_lambda) snprintf(out, outSize, "%.2f", displayValue);
    else snprintf(out, outSize, "%.1f", displayValue);
  } else {
    snprintf(out, outSize, "%.0f", displayValue);
  }
}

// ===================== WARNINGS: helpers & UI overlays =====================
// --- rotating title warning state ---

// Evaluate current warning level for a channel (0=none, 1=L1, 2=L2)
// Warnings are stored/compared in BASE units
uint8_t warnLevelFor(Channel ch){
  if(!isWarnEligible(ch)) return 0;
  uint8_t m = persist.warnMode[ch];
  if(m == CFG::WARN_OFF) return 0;

  float v = valueRawBase(ch);
  if(!isfinite(v)) return 0;

  float t1 = persist.warnT1[ch];
  float t2 = persist.warnT2[ch];

  if(m == CFG::WARN_HIGH){
    if(v >= t2) return 2;
    if(v >= t1) return 1;
  } else { // WARN_LOW
    if(v <= t2) return 2;
    if(v <= t1) return 1;
  }
  return 0;
}

// Thin overlay
inline void overlayPillWarnOutline(const PillSpec& p, uint16_t col){
  tft.drawRoundRect(p.x,   p.y,   p.w,   p.h,   10, col);
  tft.drawRoundRect(p.x+1, p.y+1, p.w-2, p.h-2,  9, col);
}
// Thick overlay
void overlayPillWarnOutlineThick(const PillSpec& p, uint16_t col){
  tft.drawRoundRect(p.x,   p.y,   p.w,   p.h,   10, col);
  tft.drawRoundRect(p.x+1, p.y+1, p.w-2, p.h-2,  9, col);
  tft.drawRoundRect(p.x+2, p.y+2, p.w-4, p.h-4,  8, col);
  tft.drawRoundRect(p.x+3, p.y+3, p.w-6, p.h-6,  7, col);
}


// ===================== Menu – shared drawing =====================
static const int MENU_ROW_H = 26;
static const int MENU_TOP   = APPBAR_H + 28;
static inline int MENU_PER_PAGE(){ return 7; }

inline void drawMenuPill(int y, bool sel){
  tft.fillRoundRect(8,y-18,304,22,8,COL_CARD());
  uint16_t fc = sel? COL_YELLOW(): COL_FRAME();
  tft.drawRoundRect(8,y-18,304,22,8,fc);
}
inline void drawMenuTitle(const char* title){ drawTitle(title); }
inline void drawMenuItemText(int y, const char* left, const char* right, bool sel){
  drawMenuPill(y, sel);
  tft.setFont(&FreeSans12pt7b);
  tft.setTextColor(COL_TXT(),COL_CARD());
  tft.setCursor(16,y); tft.print(left);
  if(right && right[0]){
    int16_t x1,y1; uint16_t w,h; tft.getTextBounds((char*)right,0,0,&x1,&y1,&w,&h);
    tft.setCursor(312 - (int)w - 2, y); tft.print(right);
  }
  tft.setFont();
}
inline void redrawMenuRowAtLogical(int logicalRow, const char* left, const char* right, bool sel){
  int y = MENU_TOP + (logicalRow*MENU_ROW_H);
  drawMenuItemText(y,left,right,sel);
}
inline void fullScreenMenuFrame(const char* title){
  tft.fillScreen(COL_BG());
  drawAppBar();
  drawMenuTitle(title);
}

// ===================== Root Menu =====================
const char* MENU_ROOT_ITEMS[] = { "Screen Layout", "Gauge Warnings", "Colours", "OBD2 Scan Tool", "System" };
const int   MENU_ROOT_COUNT   = 5;

void showRootMenu(bool full=true){
  if(full) fullScreenMenuFrame("Settings");
  for(int i=0;i<MENU_ROOT_COUNT;i++)
    redrawMenuRowAtLogical(i,MENU_ROOT_ITEMS[i], "", i==menuIndex);
}
void updateRootSel(uint8_t prev, uint8_t now){
  if(prev==now) return;
  redrawMenuRowAtLogical(prev, MENU_ROOT_ITEMS[prev], "", false);
  redrawMenuRowAtLogical(now,  MENU_ROOT_ITEMS[now],  "", true);
}

// ===================== OBD2 Menu =====================
const char* MENU_OBD2_ITEMS[] = { "Read Codes", "Clear Codes" };
const int   MENU_OBD2_COUNT   = 2;

static void clearObd2Codes(){
  for(uint8_t i=0;i<OBD2_MAX_CODES;i++){
    obd2Codes[i][0] = '\0';
  }
  obd2CodeCount = 0;
}

static bool decodeObd2Dtc(uint8_t a, uint8_t b, char out[6]){
  if(a == 0 && b == 0){
    return false;
  }
  static const char typeMap[4] = {'P','C','B','U'};
  uint8_t typeIdx = (a >> 6) & 0x03;
  uint8_t digit1 = (a >> 4) & 0x03;
  uint8_t digit2 = a & 0x0F;
  uint8_t digit3 = (b >> 4) & 0x0F;
  uint8_t digit4 = b & 0x0F;
  snprintf(out, 6, "%c%u%X%X%X", typeMap[typeIdx], digit1, digit2, digit3, digit4);
  return true;
}

static void sendObd2ReadRequest(){
  clearObd2Codes();
  obd2TimedOut = false;
  obd2SendOk = true;
  obd2Awaiting = true;
  obd2RequestMs = millis();
  obd2NeedsRedraw = false;
  obd2ClearDone = false;
  obd2ClearOk = false;

  struct can_frame out{};
  out.can_id = 0x7DF;
  out.can_dlc = 8;
  out.data[0] = 0x02;
  out.data[1] = 0x03;
  for(uint8_t i=2;i<8;i++) out.data[i] = 0x00;
  if(mcp.sendMessage(&out) != MCP2515::ERROR_OK){
    obd2SendOk = false;
    obd2Awaiting = false;
    obd2NeedsRedraw = true;
  }
}

static void sendObd2ClearRequest(){
  clearObd2Codes();
  obd2TimedOut = false;
  obd2SendOk = true;
  obd2Awaiting = true;
  obd2RequestMs = millis();
  obd2NeedsRedraw = false;
  obd2ClearDone = false;
  obd2ClearOk = false;

  struct can_frame out{};
  out.can_id = 0x7DF;
  out.can_dlc = 8;
  out.data[0] = 0x01;
  out.data[1] = 0x04;
  for(uint8_t i=2;i<8;i++) out.data[i] = 0x00;
  if(mcp.sendMessage(&out) != MCP2515::ERROR_OK){
    obd2SendOk = false;
    obd2Awaiting = false;
    obd2NeedsRedraw = true;
  }
}

static void obd2MaybeCapture(const can_frame& f){
  if(!obd2Awaiting || menuState != MENU_OBD2_ACTION) return;
  if(f.can_id < 0x7E8 || f.can_id > 0x7EF) return;
  if(f.can_dlc < 3) return;

  if(obd2Sel == 0){
    if(f.data[1] != 0x43) return;

    for(uint8_t i=2;i+1<f.can_dlc;i+=2){
      if(obd2CodeCount >= OBD2_MAX_CODES) break;
      if(decodeObd2Dtc(f.data[i], f.data[i+1], obd2Codes[obd2CodeCount])){
        obd2CodeCount++;
      }
    }
  } else if(obd2Sel == 1){
    if(f.data[1] != 0x44) return;
    obd2ClearDone = true;
    obd2ClearOk = true;
  } else {
    return;
  }

  obd2Awaiting = false;
  obd2NeedsRedraw = true;
}

static void updateObd2Timeout(unsigned long now){
  if(obd2Awaiting && (now - obd2RequestMs > 20000)){
    obd2Awaiting = false;
    obd2TimedOut = true;
    obd2NeedsRedraw = true;
  }
}

void showObd2Menu(bool full=true){
  if(full) fullScreenMenuFrame("OBD2 Scan Tool");
  for(int i=0;i<MENU_OBD2_COUNT;i++)
    redrawMenuRowAtLogical(i, MENU_OBD2_ITEMS[i], "", i==obd2Sel);
}
void updateObd2Sel(uint8_t prev, uint8_t now){
  if(prev==now) return;
  redrawMenuRowAtLogical(prev, MENU_OBD2_ITEMS[prev], "", false);
  redrawMenuRowAtLogical(now,  MENU_OBD2_ITEMS[now],  "", true);
}
void showObd2Action(bool full=true){
  if(full){
    char title[32];
    snprintf(title, sizeof(title), "OBD2 > %s", MENU_OBD2_ITEMS[obd2Sel]);
    fullScreenMenuFrame(title);
  }
  if(obd2Sel == 0){
    if(obd2Awaiting){
      redrawMenuRowAtLogical(0, "Scanning...", "", true);
      redrawMenuRowAtLogical(1, "Waiting for ECU", "", false);
    } else if(!obd2SendOk){
      redrawMenuRowAtLogical(0, "Send failed", "", true);
      redrawMenuRowAtLogical(1, "Check CAN wiring", "", false);
    } else if(obd2TimedOut){
      redrawMenuRowAtLogical(0, "No response", "", true);
      redrawMenuRowAtLogical(1, "Try again", "", false);
    } else if(obd2CodeCount == 0){
      redrawMenuRowAtLogical(0, "No codes found", "", true);
      redrawMenuRowAtLogical(1, "Press CANCEL to return", "", false);
    } else {
      for(uint8_t i=0;i<OBD2_MAX_CODES;i++){
        const char* code = (i < obd2CodeCount) ? obd2Codes[i] : "";
        redrawMenuRowAtLogical(i, code, "", i==0);
      }
    }
  } else {
    if(obd2Awaiting){
      redrawMenuRowAtLogical(0, "Clearing codes...", "", true);
      redrawMenuRowAtLogical(1, "Waiting for ECU", "", false);
    } else if(!obd2SendOk){
      redrawMenuRowAtLogical(0, "Send failed", "", true);
      redrawMenuRowAtLogical(1, "Check CAN wiring", "", false);
    } else if(obd2TimedOut){
      redrawMenuRowAtLogical(0, "No response", "", true);
      redrawMenuRowAtLogical(1, "Try again", "", false);
    } else if(obd2ClearDone && obd2ClearOk){
      redrawMenuRowAtLogical(0, "Codes cleared", "", true);
      redrawMenuRowAtLogical(1, "Press CANCEL to return", "", false);
    } else {
      redrawMenuRowAtLogical(0, "Clear request sent", "", true);
      redrawMenuRowAtLogical(1, "Press CANCEL to return", "", false);
    }
  }
}

// ===================== Layout: Screen picker =====================
void showLayoutScreenPick(bool full=true){
  if(full) fullScreenMenuFrame("Settings > Screen Layout");
  const char* items[]={"Screen 1","Screen 2","Screen 3","Screen 4","Screen 5"};
  for(int i=0;i<SCREEN_COUNT;i++) redrawMenuRowAtLogical(i, items[i], "", i==layoutScreenSel);
}
void updateLayoutScreenSel(uint8_t prev, uint8_t now){
  const char* items[]={"Screen 1","Screen 2","Screen 3","Screen 4","Screen 5"};
  redrawMenuRowAtLogical(prev, items[prev], "", false);
  redrawMenuRowAtLogical(now,  items[now],  "", true);
}

// ===================== Layout: Slot navigator =====================
inline uint8_t slotFromRowCol(int8_t r,int8_t c){ return (r<0)? 4 : (uint8_t)(r*2 + c); }

void showLayoutSlots(bool full=true){
  if(full){
    char ttl[32];
    snprintf(ttl, sizeof(ttl), "Layout: Screen %d", (int)layoutScreenSel+1);
    fullScreenMenuFrame(ttl);
  }
  // draw 2x2 + bar preview
  for(int i=0;i<4;i++){
    PillSpec p = pillSpec(i);
    tft.fillRoundRect(p.x,p.y,p.w,p.h,10,COL_CARD());
    drawPillFrame(p,false,(Channel)persist.pillChannel[layoutScreenSel][i]);
    drawPillLabel(p, labelText((Channel)persist.pillChannel[layoutScreenSel][i]));
    clearPillValue(p);
    tft.setFont(&FreeSans12pt7b); tft.setTextColor(COL_TXT(),COL_CARD());
    tft.setCursor(p.x+10,p.y+p.h-10); tft.print("--");
    const char* unit = unitLabel((Channel)persist.pillChannel[layoutScreenSel][i]);
    if(unit[0]){
      int16_t bx,by; uint16_t bw,bh; tft.getTextBounds("--", p.x+10, p.y+p.h-10, &bx,&by,&bw,&bh);
      tft.setCursor(p.x+10 + bw + 8, p.y+p.h-10); tft.print(unit);
    }
    tft.setFont();
  }
  drawBarStaticForScreen(layoutScreenSel, false);
  uint8_t slot = slotFromRowCol(layoutRow,layoutCol);
  if(slot==4) drawBarStaticForScreen(layoutScreenSel, true); else drawPillFrame(pillSpec(slot),true,(Channel)persist.pillChannel[layoutScreenSel][slot]);
}
void updateLayoutCursor(uint8_t prevSlot, uint8_t nowSlot){
  if(prevSlot==nowSlot) return;
  if(prevSlot==4) drawBarStaticForScreen(layoutScreenSel, false);
  else drawPillFrame(pillSpec(prevSlot),false,(Channel)persist.pillChannel[layoutScreenSel][prevSlot]);

  if(nowSlot==4) drawBarStaticForScreen(layoutScreenSel, true);
  else drawPillFrame(pillSpec(nowSlot),true,(Channel)persist.pillChannel[layoutScreenSel][nowSlot]);
}

// ===================== Gauge picker (latched window like warning list) =====================
inline bool pickingBarSlot(){ return (layoutRow<0); }
inline bool isEligibleForPicker(Channel ch){
  if(!isGaugeAvailable(ch)) return false;
  return pickingBarSlot()? isBarEligible(ch) : true;
}
inline int eligibleCount(){ int n=0; for(int i=0;i<CH__COUNT;i++) if(isEligibleForPicker((Channel)i)) n++; return n; }
inline uint8_t channelFromEligibleIndex(int idx){ for(int i=0;i<CH__COUNT;i++) if(isEligibleForPicker((Channel)i)){ if(idx==0) return (uint8_t)i; idx--; } return 0; }
inline int eligibleIndexFromChannel(uint8_t ch){ int pos=0; for(int i=0;i<CH__COUNT;i++) if(isEligibleForPicker((Channel)i)){ if(i==ch) return pos; pos++; } return 0; }

void showLayoutGaugePicker(uint8_t current, bool full=true){
  if(full) fullScreenMenuFrame("Pick Gauge");
  const int perPage = MENU_PER_PAGE();
  int total = eligibleCount();
  for(int i=0;i<perPage;i++){
    int idx = pickerTop + i;
    int y = MENU_TOP + (i*MENU_ROW_H);
    if(idx >= total){ clearRegion(8,y-20,304,26,COL_BG()); continue; }
    uint8_t ch = channelFromEligibleIndex(idx);
    bool isCurr = pickingBarSlot() ? (eligibleIndexFromChannel(current) == idx) : (current == ch);
    const char* right = isCurr ? "current" : "";
    redrawMenuRowAtLogical(i, labelText((Channel)ch), right, idx==menuIndex2);
  }
}
void updateGaugePickerSel(uint8_t prev, uint8_t now, uint8_t current){
  const int perPage = MENU_PER_PAGE();

  // Adjust window
  int prevTopWin = pickerTop;
  if(now < pickerTop) pickerTop = now;
  if(now >= pickerTop + perPage) pickerTop = now - perPage + 1;

  if(prevTopWin != pickerTop){
    // OLD: showLayoutGaugePicker(current,true);
    repaintGaugePickerWindow(current);   // <-- minimal repaint, avoids flicker
    return;
  }

  int rowPrev = prev - pickerTop;
  int rowNow  = now  - pickerTop;

  if(rowPrev >= 0 && rowPrev < perPage){
    uint8_t chPrev = channelFromEligibleIndex(prev);
    bool isCurrPrev = pickingBarSlot() ? (eligibleIndexFromChannel(current) == prev) : (current == chPrev);
    redrawMenuRowAtLogical(rowPrev, labelText((Channel)chPrev), isCurrPrev ? "current" : "", false);
  }
  if(rowNow >= 0 && rowNow < perPage){
    uint8_t chNow = channelFromEligibleIndex(now);
    bool isCurrNow = pickingBarSlot() ? (eligibleIndexFromChannel(current) == now) : (current == chNow);
    redrawMenuRowAtLogical(rowNow, labelText((Channel)chNow), isCurrNow ? "current" : "", true);
  }
}
// ===================== Warnings list (eligible channels only) =====================
int warnEligibleCount(){ int n=0; for(int i=0;i<CH__COUNT;i++) if(isWarnEligible((Channel)i)) n++; return n; }
uint8_t warnChFromIdx(int idx){ for(int i=0;i<CH__COUNT;i++) if(isWarnEligible((Channel)i)){ if(idx==0) return (uint8_t)i; idx--; } return 0; }
int warnIdxFromCh(uint8_t ch){ int pos=0; for(int i=0;i<CH__COUNT;i++) if(isWarnEligible((Channel)i)){ if(i==ch) return pos; pos++; } return 0; }

void showWarnList(bool full=true){
  if(full) fullScreenMenuFrame("Settings > Gauge Warnings");
  const int perPage=MENU_PER_PAGE();
  for(int i=0;i<perPage;i++){
    int gi = warnListTop + i;
    int y = MENU_TOP + (i*MENU_ROW_H);
    if(gi >= warnEligibleCount()){ clearRegion(8,y-20,304,26,COL_BG()); continue; }
    uint8_t ch = warnChFromIdx(gi);
    redrawMenuRowAtLogical(i, labelText((Channel)ch), "", gi==warnListSel);
  }
}
// ===== Minimal repaint: WARNING LIST (no full frame) =====
static inline void repaintWarnListWindow(){
  const int perPage = MENU_PER_PAGE();
  // Clear only the list body area
  clearRegion(8, MENU_TOP-20, 304, perPage*MENU_ROW_H + 26, COL_BG());

  for(int i=0;i<perPage;i++){
    int gi = warnListTop + i;
    int y = MENU_TOP + (i*MENU_ROW_H);
    if(gi >= warnEligibleCount()){ clearRegion(8,y-20,304,26,COL_BG()); continue; }
    uint8_t ch = warnChFromIdx(gi);
    redrawMenuRowAtLogical(i, labelText((Channel)ch), "", gi==warnListSel);
  }
}
// ===== Minimal repaint: GAUGE PICKER (no full frame) =====
static inline void repaintGaugePickerWindow(uint8_t current){
  const int perPage = MENU_PER_PAGE();
  // Clear only the list body area
  clearRegion(8, MENU_TOP-20, 304, perPage*MENU_ROW_H + 26, COL_BG());

  int total = eligibleCount();
  for(int i=0;i<perPage;i++){
    int idx = pickerTop + i;
    int y = MENU_TOP + (i*MENU_ROW_H);
    if(idx >= total){ clearRegion(8,y-20,304,26,COL_BG()); continue; }
    uint8_t ch = channelFromEligibleIndex(idx);
    bool isCurr = pickingBarSlot() ? (eligibleIndexFromChannel(current) == idx) : (current == ch);
    const char* right = isCurr ? "current" : "";
    redrawMenuRowAtLogical(i, labelText((Channel)ch), right, idx==menuIndex2);
  }
}
void updateWarnListSel(int prevSel, int newSel, int prevTop, int newTop){
  const int perPage=MENU_PER_PAGE();
  if(prevTop!=newTop){
    // OLD: showWarnList(true);
    repaintWarnListWindow();         // <-- minimal repaint, avoids flicker
    return;
  }
  int rowPrev = prevSel - newTop;
  int rowNow  = newSel  - newTop;
  if(rowPrev>=0 && rowPrev<perPage){
    uint8_t chPrev = warnChFromIdx(prevSel);
    redrawMenuRowAtLogical(rowPrev, labelText((Channel)chPrev), "", false);
  }
  if(rowNow>=0 && rowNow<perPage){
    uint8_t chNow = warnChFromIdx(newSel);
    redrawMenuRowAtLogical(rowNow, labelText((Channel)chNow), "", true);
  }
}

// ===================== Warnings field editor (detail page) =====================
void drawWarnFieldRow(int row, uint8_t ch, bool sel, bool blinkHide=false, bool useStaged=false){
  char left[24]; char right[24];
  float t1 = useStaged? editT1 : persist.warnT1[ch];
  float t2 = useStaged? editT2 : persist.warnT2[ch];
  // convert to display units for presentation
  auto toDisplayForCh = [&](float v)->float{
    switch((Channel)ch){
      case CH_BATTV: return v;
      case CH_LAMBDA: return toDisplayLambda(v);
      case CH_COOLANT: case CH_TRANS1: case CH_TRANS2: case CH_IAT: case CH_FUELT:
      case CH_EGT1: case CH_EGT2: case CH_MANIFOLD: case CH_TURBO_OUT: return toDisplayTemp(v);
      case CH_BOOST: case CH_OIL: return toDisplayPressure(v);
      case CH_SPEED: return toDisplaySpeed(v);
      default: return v;
    }
  };

  if(row==0){
    snprintf(left,sizeof(left),"Mode");
    uint8_t m = useStaged? editMode : persist.warnMode[ch];
    const char* ms = (m==CFG::WARN_HIGH?"High":(m==CFG::WARN_LOW?"Low":"Off"));
    snprintf(right,sizeof(right),"%s", ms);
  }else if(row==1 || row==2){
    snprintf(left,sizeof(left), row==1?"Level 1":"Level 2");
    float v = (row==1)? t1 : t2;
    v = toDisplayForCh(v);
    if(blinkHide) snprintf(right,sizeof(right),"      ");
    else{
      if((Channel)ch==CH_BATTV) snprintf(right,sizeof(right),"%0.2f", v);
      else if((Channel)ch==CH_LAMBDA) {
        if(g_uLambda==U_L_lambda) snprintf(right,sizeof(right),"%0.2f", v);
        else snprintf(right,sizeof(right),"%0.1f", v);
      }
      else snprintf(right,sizeof(right),"%0.0f", v);
    }
  }else{ left[0]=right[0]=0; }
  redrawMenuRowAtLogical(row, left, right, sel);
}
void showWarnFieldEditor(uint8_t ch, bool full=true){
  if(full){ char title[48]; snprintf(title,sizeof(title),"Warn: %s (%s)", labelText((Channel)ch), unitLabel((Channel)ch)); fullScreenMenuFrame(title); }
  // staged = current (in BASE units)
  editMode = persist.warnMode[ch];
  editT1   = persist.warnT1[ch];
  editT2   = persist.warnT2[ch];
  warnFieldEditing=false; warnBlinkOn=true; warnBlinkMs=millis();
  for(int r=0;r<3;r++) drawWarnFieldRow(r,ch, r==warnFieldSel, false, false);
}

// Taps: base step in DISPLAY units, then convert back when committing edits
inline void stepWarnValueOnce(uint8_t ch, bool up){
  // Work in DISPLAY units to feel natural, then convert back to staged base values
  float dispT = (warnFieldSel==1) ? editT1 : editT2; // still base currently
  // Convert base -> display
  auto baseToDisp = [&](float v)->float{
    switch((Channel)ch){
      case CH_LAMBDA: return toDisplayLambda(v);
      case CH_COOLANT: case CH_TRANS1: case CH_TRANS2: case CH_IAT: case CH_FUELT:
      case CH_EGT1: case CH_EGT2: case CH_MANIFOLD: case CH_TURBO_OUT: return toDisplayTemp(v);
      case CH_BOOST: case CH_OIL: return toDisplayPressure(v);
      case CH_SPEED: return toDisplaySpeed(v);
      default: return v;
    }
  };
  auto dispToBase = [&](float v)->float{
    switch((Channel)ch){
      case CH_LAMBDA: return fromDisplayLambda(v);
      case CH_COOLANT: case CH_TRANS1: case CH_TRANS2: case CH_IAT: case CH_FUELT:
      case CH_EGT1: case CH_EGT2: case CH_MANIFOLD: case CH_TURBO_OUT: return fromDisplayTemp(v);
      case CH_BOOST: case CH_OIL: return fromDisplayPressure(v);
      case CH_SPEED: return fromDisplaySpeed(v);
      default: return v;
    }
  };
  float curDisp = baseToDisp(dispT);
  float st = stepFor((Channel)ch);
  if(!up) st = -st;
  float newDisp = curDisp + st;

  // clamp to display range
  Range rd = rangeFor((Channel)ch);
  newDisp = clampf(newDisp, rd.mn, rd.mx);

  // write back into staged BASE variable
  float newBase = dispToBase(newDisp);
  if(warnFieldSel==1) editT1 = newBase; else editT2 = newBase;

  // Keep ordering visually sane in BASE
  if(editMode==CFG::WARN_HIGH){ if(editT2 < editT1) editT2 = editT1; }
  else if(editMode==CFG::WARN_LOW){ if(editT2 > editT1) editT1 = editT2; }

  // Clamp to BASE ranges
  Range rb = RNG_BASE[(Channel)ch];
  editT1=clampf(editT1,rb.mn,rb.mx); editT2=clampf(editT2,rb.mn,rb.mx);
}
void commitWarnField(uint8_t ch){
  if(warnFieldSel==0) persist.warnMode[ch]=editMode;
  else if(warnFieldSel==1) persist.warnT1[ch]=editT1;
  else persist.warnT2[ch]=editT2;

  if(persist.warnMode[ch]==CFG::WARN_HIGH){ if(persist.warnT2[ch] < persist.warnT1[ch]) persist.warnT2[ch] = persist.warnT1[ch]; }
  else if(persist.warnMode[ch]==CFG::WARN_LOW){ if(persist.warnT2[ch] > persist.warnT1[ch]) persist.warnT1[ch] = persist.warnT2[ch]; }

  Range r=RNG_BASE[(Channel)ch];
  persist.warnT1[ch]=clampf(persist.warnT1[ch],r.mn,r.mx); persist.warnT2[ch]=clampf(persist.warnT2[ch],r.mn,r.mx);
  dirty=true;
}
void discardWarnField(uint8_t ch){
  if(warnFieldSel==0) editMode = persist.warnMode[ch];
  else if(warnFieldSel==1) editT1 = persist.warnT1[ch];
  else editT2 = persist.warnT2[ch];
}

// Redraw whole visible window (used when the window scrolls)
static inline void repaintColoursWindow(){
  const int perPage = MENU_PER_PAGE();
  const int total   = paletteCount();

  // Clear only the list body area
  clearRegion(8, MENU_TOP-20, 304, perPage*MENU_ROW_H + 26, COL_BG());

  for(int i=0;i<perPage;i++){
    int gi = coloursTop + i;
    int y  = MENU_TOP + (i*MENU_ROW_H);
    if (gi >= total) { clearRegion(8, y-20, 304, 26, COL_BG()); continue; }
    const char* right = (gi == paletteIndex) ? "current" : "";
    redrawMenuRowAtLogical(i, paletteNameForIndex(gi), right, gi==coloursSel);
  }
}

// Incremental selection update (no window scroll)
static inline void updateColoursSel(int prevSel, int curSel, int prevTop, int curTop){
  const int perPage = MENU_PER_PAGE();
  const int total   = paletteCount();

  // If window moved, redraw the visible slice
  if (prevTop != curTop) { repaintColoursWindow(); return; }

  // Unhighlight previous row (if visible)
  if (prevSel >= curTop && prevSel < curTop + perPage && prevSel < total) {
    int i = prevSel - curTop;                       // logical row
    const char* rightPrev = (prevSel == paletteIndex) ? "current" : "";
    redrawMenuRowAtLogical(i, paletteNameForIndex(prevSel), rightPrev, false);
  }

  // Highlight current row (if visible)
  if (curSel >= curTop && curSel < curTop + perPage && curSel < total) {
    int i = curSel - curTop;                        // logical row
    const char* rightNow = (curSel == paletteIndex) ? "current" : "";
    redrawMenuRowAtLogical(i, paletteNameForIndex(curSel), rightNow, true);
  }
}

// ===================== Colours =====================
void showColoursPage(bool full=true){
  if(full) fullScreenMenuFrame("Settings > Colours");

  const int perPage = MENU_PER_PAGE();
  const int total   = paletteCount();

  for(int i=0;i<perPage;i++){
    int gi = coloursTop + i;                         // global index
    int y  = MENU_TOP + (i*MENU_ROW_H);              // same layout as warn list

    if (gi >= total) {                               // clear trailing rows
      clearRegion(8, y-20, 304, 26, COL_BG());
      continue;
    }

    const char* right = (gi == paletteIndex) ? "current" : "";
    // i is the logical row index (0..perPage-1), just like warn list
    redrawMenuRowAtLogical(i, paletteNameForIndex(gi), right, gi==coloursSel);
  }
}

// ===================== Custom palette =====================
static inline void drawCustomPaletteRow(uint8_t row, bool sel){
  const char* left = CUSTOM_ZONE_LABELS[row];
  const CustomPalette& palette = persist.customPalettes[customPaletteSel];
  uint16_t value = customZoneValue(palette, row);
  const char* right = customColourNameForValue(value);
  redrawMenuRowAtLogical(row, left, right, sel);
}

void showCustomPaletteMenu(bool full=true){
  if(full){
    char title[32];
    snprintf(title, sizeof(title), "Custom %u > Zones", (unsigned)(customPaletteSel + 1));
    fullScreenMenuFrame(title);
  }
  for(uint8_t r = 0; r < CUSTOM_ZONE_COUNT; r++){
    drawCustomPaletteRow(r, r == customZoneSel);
  }
}

void updateCustomPaletteSel(uint8_t prev, uint8_t now){
  if(prev==now) return;
  drawCustomPaletteRow(prev, false);
  drawCustomPaletteRow(now, true);
}

static inline void repaintCustomColourWindow(){
  const int perPage = MENU_PER_PAGE();
  const int total = customColourCount();
  const CustomPalette& palette = persist.customPalettes[customPaletteSel];
  uint16_t currentValue = customZoneValue(palette, customZoneSel);
  uint8_t currentIndex = customColourIndexForValue(currentValue);

  clearRegion(8, MENU_TOP-20, 304, perPage*MENU_ROW_H + 26, COL_BG());

  for(int i = 0; i < perPage; i++){
    int gi = customColourTop + i;
    int y  = MENU_TOP + (i * MENU_ROW_H);
    if (gi >= total) { clearRegion(8, y-20, 304, 26, COL_BG()); continue; }
    const char* right = (gi == currentIndex) ? "current" : "";
    redrawMenuRowAtLogical(i, CUSTOM_COLOUR_OPTIONS[gi].name, right, gi == customColourSel);
  }
}

static inline void updateCustomColourSel(int prevSel, int curSel, int prevTop, int curTop){
  const int perPage = MENU_PER_PAGE();
  const int total = customColourCount();

  if (prevTop != curTop) { repaintCustomColourWindow(); return; }

  if (prevSel >= curTop && prevSel < curTop + perPage && prevSel < total) {
    int i = prevSel - curTop;
    const CustomPalette& palette = persist.customPalettes[customPaletteSel];
    uint8_t currentIndex = customColourIndexForValue(customZoneValue(palette, customZoneSel));
    const char* rightPrev = (prevSel == currentIndex) ? "current" : "";
    redrawMenuRowAtLogical(i, CUSTOM_COLOUR_OPTIONS[prevSel].name, rightPrev, false);
  }

  if (curSel >= curTop && curSel < curTop + perPage && curSel < total) {
    int i = curSel - curTop;
    const CustomPalette& palette = persist.customPalettes[customPaletteSel];
    uint8_t currentIndex = customColourIndexForValue(customZoneValue(palette, customZoneSel));
    const char* rightNow = (curSel == currentIndex) ? "current" : "";
    redrawMenuRowAtLogical(i, CUSTOM_COLOUR_OPTIONS[curSel].name, rightNow, true);
  }
}

static inline void enterCustomColourPicker(){
  const CustomPalette& palette = persist.customPalettes[customPaletteSel];
  uint16_t currentValue = customZoneValue(palette, customZoneSel);
  customColourSel = customColourIndexForValue(currentValue);
  const int perPage = MENU_PER_PAGE();
  const int total = customColourCount();
  customColourTop = customColourSel - perPage / 2;
  if (customColourTop < 0) customColourTop = 0;
  int maxTop = total - perPage; if (maxTop < 0) maxTop = 0;
  if (customColourTop > maxTop) customColourTop = maxTop;
}

void showCustomColourPicker(bool full=true){
  if(full){
    char title[32];
    snprintf(title, sizeof(title), "Custom %u > %s", (unsigned)(customPaletteSel + 1), CUSTOM_ZONE_LABELS[customZoneSel]);
    fullScreenMenuFrame(title);
  }
  repaintCustomColourWindow();
}

// ===================== System Menus (NEW) =====================

// ---- System root ----
const char* MENU_SYSTEM_ITEMS[] = { "Brightness", "Units", "WiFi", "CAN Sniff", "Speed Trim", "Factory Reset" };
const int   MENU_SYSTEM_COUNT   = 6;
void showSystemMenu(bool full=true){
  if(full) fullScreenMenuFrame("Settings > System");
  for(int i=0;i<MENU_SYSTEM_COUNT;i++)
    redrawMenuRowAtLogical(i, MENU_SYSTEM_ITEMS[i], "", i==menuIndex);
}
void updateSystemSel(uint8_t prev, uint8_t now){
  if(prev==now) return;
  redrawMenuRowAtLogical(prev, MENU_SYSTEM_ITEMS[prev], "", false);
  redrawMenuRowAtLogical(now,  MENU_SYSTEM_ITEMS[now],  "", true);
}

// ---- Brightness page ----
static inline void drawBrightnessRow(int row, bool sel, bool blinkHide=false){
  const char* left = (row==0) ? "Lights On" : "Lights Off";
  uint8_t v = (row==0) ? brightOn : brightOff;
  char right[16];
  if(blinkHide) snprintf(right,sizeof(right),"   ");
  else snprintf(right,sizeof(right), "%u%%", (unsigned)v);
  redrawMenuRowAtLogical(row, left, right, sel);
}
void showBrightness(bool full=true){
  if(full) fullScreenMenuFrame("System > Brightness");
  drawBrightnessRow(0, brightSel==0, false);
  drawBrightnessRow(1, brightSel==1, false);
}
void updateBrightnessSel(uint8_t prev, uint8_t now){
  if(prev==now) return;
  drawBrightnessRow(prev, false, false);
  drawBrightnessRow(now,  true,  false);
}

// ---- Units page ----
static inline void drawUnitsRow(int row, bool sel, bool blinkHide=false){
  const char* left = (row==0) ? "Pressure" : (row==1) ? "Temp" : (row==2) ? "Speed" : "Lambda";
  char right[12]="";
  if(!blinkHide){
    if(row==0) strcpy(right, (g_uPressure==U_P_kPa)?"kPa":"psi");
    else if(row==1) strcpy(right, (g_uTemp==U_T_C)?"C":"F");
    else if(row==2) strcpy(right, (g_uSpeed==U_S_kmh)?"km/h":"mph");
    else strcpy(right, (g_uLambda==U_L_lambda)?"λ":"AFR");
  }
  redrawMenuRowAtLogical(row, left, right, sel);
}
void showUnitsPage(bool full=true){
  if(full) fullScreenMenuFrame("System > Units");
  for(int r=0;r<4;r++) drawUnitsRow(r, r==unitsSel, false);
}
// ---- WiFi page ----
static inline void drawWifiRow(int row, bool sel){
  const char* left = (row==0) ? "SSID" : (row==1) ? "Password" : "Config URL";
  String rightStr;
  if(row==0) rightStr = String(persist.wifiSsid);
  else if(row==1) rightStr = String(persist.wifiPass);
  else rightStr = wifiPageUrl();
  char right[40];
  copyStringToBuffer(rightStr, right, sizeof(right));
  redrawMenuRowAtLogical(row, left, right, sel);
}
void showWifiMenu(bool full=true){
  if(full) fullScreenMenuFrame("System > WiFi");
  drawWifiRow(0, true);
  drawWifiRow(1, false);
  drawWifiRow(2, false);
}
// ===== Speed Trim page =====
static inline void drawSpeedTrimRow(bool sel, bool blinkHide=false){
  const char* left = "Speed Trim";
  char right[24];
  if (blinkHide) {
    snprintf(right,sizeof(right), "      ");
  } else {
    // Show signed percentage with one decimal (e.g., +1.5%)
    char sign = (speedTrimPct >= 0.0f) ? '+' : '-';
    float mag = fabsf(speedTrimPct);
    snprintf(right,sizeof(right), "%c%.1f%%", sign, mag);
  }
  redrawMenuRowAtLogical(0, left, right, sel);
}

void showSpeedTrim(bool full=true){
  if(full) fullScreenMenuFrame("System > Speed Trim");
  drawSpeedTrimRow(true, false);
}
void updateUnitsSel(uint8_t prev, uint8_t now){
  if(prev==now) return;
  drawUnitsRow(prev, false, false);
  drawUnitsRow(now,  true,  false);
}

// ===================== Input handling & navigation =====================
static bool sw_cancel_pressed=false;
static unsigned long sw_cancel_t0=0;
static bool suppressNextCancelRelease=false;
static bool sw_enter_pressed=false;
static unsigned long sw_enter_t0=0;
static bool suppressNextEnterRelease=false;

// Double-tap cancel for page cycling (live UI)
static uint8_t cancelTapCount = 0;
static unsigned long lastCancelTapMs = 0;
constexpr uint32_t DOUBLE_TAP_MS = 450;

// Triple-tap enter for min/max reset (live UI)
static uint8_t enterTapCount = 0;
static unsigned long lastEnterTapMs = 0;
constexpr uint32_t ENTER_TAP_MS = 450;
constexpr uint32_t ENTER_HOLD_MS = 2000;

// Buttons (edge detect)
static bool sw_up_prev=false, sw_down_prev=false, sw_left_prev=false, sw_right_prev=false, sw_enter_prev=false;

inline bool bitSetSafe(const can_frame& f,uint8_t byteIdx,uint8_t bit){ if(bit>7||f.can_dlc<=byteIdx) return false; return (f.data[byteIdx]&(1u<<bit))!=0; }

// Enter/exit settings
void navEnterSettings(){
  menuState = MENU_ROOT;
  menuIndex = g_lastRootIndex;
  menuIndex2 = 0;
  fullScreenMenuFrame("Settings");
  showRootMenu(false);
}
// ===================== Navigation =====================
void navExitSettings(){
  menuState = UI_MAIN;

  // Make sure regenState reflects the latest regen_pct before drawing the title
  updateRegenState();

  tft.fillScreen(COL_BG());
  drawAppBar();
  renderStatic();
  renderDynamic();
  dirty = true;
  savePersist(persist, dirty, true);
}

void setMinMaxActive(bool active){
  if(uiMinMaxActive == active) return;
  uiMinMaxActive = active;
  renderStatic();
  renderDynamic();
}

// Wrap helpers
template<typename T> inline void wrapInc(T& v, T maxIncl){ v = (v<maxIncl)? (T)(v+1) : (T)0; }
template<typename T> inline void wrapDec(T& v, T maxIncl){ v = (v>0)? (T)(v-1) : maxIncl; }

// ---- CAN Sniffer: helpers ----
static inline char hexNib(uint8_t v){ return (v<10)?('0'+v):('A'+(v-10)); }

// Render one row (Address / From / To / Scale / Bias)
void drawSniffRow(uint8_t row, bool sel, bool blinkHide){
  char left[16], right[40]; left[0]=right[0]=0;

  if(row==0){
    strcpy(left, "Address");
    // Build "0x" + SNF_HEX_DIGITS nibbles (MSB..LSB). While editing & selected,
    // we only blink the *selected nibble* (snf_cursor); others stay visible.
    char tmp[16]; tmp[0]=0; strcat(tmp,"0x");
    for(uint8_t i=0;i<SNF_HEX_DIGITS;i++){
      // i=0 is MSB, i=SNF_HEX_DIGITS-1 is LSB
      uint8_t nibIndexFromMSB = i;
      uint8_t shiftFromMSB = (SNF_HEX_DIGITS-1 - nibIndexFromMSB)*4; // convert to shift-from-LSB
      uint8_t nib = (snf_id >> shiftFromMSB) & 0xF;

      bool blinkThisNib = (snf_editing && snf_sel==0 && blinkHide && (snf_cursor == nibIndexFromMSB));
      char c = blinkThisNib ? ' ' : ((nib<10)?('0'+nib):('A'+(nib-10)));
      size_t len = strlen(tmp); tmp[len] = c; tmp[len+1] = '\0';
    }
    strncpy(right,tmp,sizeof(right));
  }
  else if(row==1){
    strcpy(left,"From bit");
    if(blinkHide && snf_editing && snf_sel==1) strcpy(right,"  ");
    else snprintf(right,sizeof(right), "%u", (unsigned)snf_bit_from);
  }
  else if(row==2){
    strcpy(left,"To bit");
    if(blinkHide && snf_editing && snf_sel==2) strcpy(right,"  ");
    else snprintf(right,sizeof(right), "%u", (unsigned)snf_bit_to);
  }
  else if(row==3){
    strcpy(left,"Scale");
    if(blinkHide && snf_editing && snf_sel==3) strcpy(right,"     ");
    else snprintf(right,sizeof(right), "%.3f  (step %.3g)", snf_scale, snf_step);
  }
  else { // row==4
    strcpy(left,"Bias");
    if(blinkHide && snf_editing && snf_sel==4) strcpy(right,"     ");
    else snprintf(right,sizeof(right), "%.3f  (step %.3g)", snf_bias, snf_step);
  }

  redrawMenuRowAtLogical(row, left, right, sel);
}

// Pack payload little-endian into 64-bit lane:
// data[0]→bits0..7, data[1]→8..15, data[2]→16..23, etc.
static inline uint64_t snfPackLE(){
  uint64_t v=0;
  for(uint8_t i=0;i<snf_dlc && i<8;i++)
    v |= ((uint64_t)snf_data[i]) << (8*i);
  return v;
}
// Draw the live area (raw + scaled)
void drawSniffLive(){
  // We print 3 lines: DATA, RAW, SCALED
  // Use a safe baseline so last line never exceeds the 240px panel.
  const int LH = 20;                     // line height for FreeSans9 (roughly)
  const int lines = 3;
  const int yMaxBaseline = 236;          // a couple of pixels above bottom
  int baseSuggested = MENU_TOP + 5*MENU_ROW_H + 6;  // below 5 rows of controls
  int liveTop = baseSuggested;
  int maxTop  = yMaxBaseline - (lines-1)*LH;
  if(liveTop > maxTop) liveTop = maxTop; // pull up if it would clip

  static unsigned long lastDrawMs = 0;
  static char prevDATA[64]   = "";
  static char prevRAW[64]    = "";
  static char prevSCALED[48] = "";

  // Build current strings
  char DATA[64]="", RAW[64]="", SCALED[48]="--";

  // DATA line
  if(!snf_has){
    snprintf(DATA, sizeof(DATA), "DATA: (waiting...)");
  } else {
    char bytes[48] = "";
    for(uint8_t i=0;i<snf_dlc;i++){
      char b[4]; snprintf(b,sizeof(b), "%02X", snf_data[i]);
      strcat(bytes, b);
      if(i+1<snf_dlc) strcat(bytes, " ");
    }
    snprintf(DATA, sizeof(DATA), "DATA: %s", bytes);
  }

  // RAW / SCALED
  uint32_t rawVal = 0;
  if(!snf_has){
    snprintf(RAW, sizeof(RAW), "RAW %u..%u: --", (unsigned)snf_bit_from, (unsigned)snf_bit_to);
    snprintf(SCALED, sizeof(SCALED), "--");
  } else if(snf_bit_from > snf_bit_to){
    snprintf(RAW, sizeof(RAW), "RAW %u..%u: (invalid)", (unsigned)snf_bit_from, (unsigned)snf_bit_to);
    snprintf(SCALED, sizeof(SCALED), "--");
  } else {
    uint8_t width = (uint8_t)(snf_bit_to - snf_bit_from + 1);
    if(width>32) width = 32;
    uint64_t lane = snfPackLE();
    uint64_t mask = (width==64)?~(uint64_t)0 : (((uint64_t)1<<width) - 1);
    rawVal  = (uint32_t)((lane >> snf_bit_from) & mask);
    snprintf(RAW, sizeof(RAW), "RAW %u..%u: 0x%X  (%u)",
             (unsigned)snf_bit_from, (unsigned)snf_bit_to, rawVal, rawVal);
    float scaled = (float)rawVal * snf_scale + snf_bias;
    snprintf(SCALED, sizeof(SCALED), "SCALED: %.3f", scaled);
  }

  // Throttle + change-detect to reduce flicker
  unsigned long now = millis();
  bool changed = (strcmp(DATA, prevDATA)!=0) || (strcmp(RAW, prevRAW)!=0) || (strcmp(SCALED, prevSCALED)!=0);
  if(!changed && (now - lastDrawMs < 100)) return;  // 10 Hz max if nothing changed
  lastDrawMs = now;

  // Clear only the live block (once per redraw), not every frame
  clearRegion(8, liveTop-18, 304, LH*lines + 8, COL_BG());

  tft.setFont(&FreeSans9pt7b);
  tft.setTextColor(COL_TXT(), COL_BG());

  // DATA
  tft.setCursor(16, liveTop);
  tft.print(DATA);

  // RAW
  tft.setCursor(16, liveTop + LH);
  tft.print(RAW);

  // SCALED
  tft.setCursor(16, liveTop + 2*LH);
  tft.print(SCALED);

  tft.setFont();

  // cache
  strncpy(prevDATA,   DATA,   sizeof(prevDATA));
  strncpy(prevRAW,    RAW,    sizeof(prevRAW));
  strncpy(prevSCALED, SCALED, sizeof(prevSCALED));
}

// Build whole page
void showCanSniff(bool full){
  if(full){
    fullScreenMenuFrame("System > CAN Sniff");
    snf_sel = 0; snf_editing = false;
  }
  for(uint8_t r=0;r<5;r++) drawSniffRow(r, r==snf_sel, false);
  drawSniffLive();
}
// Record frames that match the selected ID and refresh live view if on page
void snifferMaybeCapture(const can_frame& f){
  // Standard 11-bit only. If using extended frames, adapt this check
  // according to your MCP2515 lib’s EFF flag handling.
  if (f.can_id != snf_id) return;

  snf_dlc = min<uint8_t>(f.can_dlc, 8);
  for(uint8_t i=0;i<snf_dlc;i++) snf_data[i] = f.data[i];
  snf_has = true;

  if(menuState == MENU_CAN_SNIFF){
    drawSniffLive();
  }
}

// ---- Factory reset confirm ----
void showFactoryResetConfirm(bool full /*=true*/) {
  if (full) {
    fullScreenMenuFrame("System > Factory Reset");
    // Single-row instruction, no cursor movement/selection needed
    redrawMenuRowAtLogical(0, "Press ENTER to reset", "", true);
  }
}

// ===== UI redraw for palette changes =====
void redrawForDimmingChange(){
  switch(menuState){
    case UI_MAIN: {
      tft.fillScreen(COL_BG());
      drawAppBar();
      updateRegenState();
      renderStatic();
      renderDynamic();
    } break;
    case MENU_ROOT:
      showRootMenu(true);
      break;
    case MENU_LAYOUT:
      showLayoutScreenPick(true);
      break;
    case MENU_LAYOUT_PICK_SLOT:
      showLayoutSlots(true);
      break;
    case MENU_LAYOUT_PICK_GAUGE: {
      uint8_t current = pickingBarSlot()
        ? persist.barChannel[layoutScreenSel]
        : persist.pillChannel[layoutScreenSel][slotFromRowCol(layoutRow, layoutCol)];
      showLayoutGaugePicker(current, true);
    } break;
    case MENU_WARN_LIST:
      showWarnList(true);
      break;
    case MENU_WARN_EDIT:
      showWarnFieldEditor(warnChFromIdx(warnListSel), true);
      break;
    case MENU_COLOURS:
      showColoursPage(true);
      break;
    case MENU_COLOURS_CUSTOM:
      showCustomPaletteMenu(true);
      break;
    case MENU_COLOURS_CUSTOM_PICK:
      showCustomColourPicker(true);
      break;
    case MENU_SYSTEM:
      showSystemMenu(true);
      break;
    case MENU_BRIGHTNESS:
      showBrightness(true);
      break;
    case MENU_UNITS:
      showUnitsPage(true);
      break;
    case MENU_WIFI:
      showWifiMenu(true);
      break;
    case MENU_CAN_SNIFF:
      fullScreenMenuFrame("System > CAN Sniff");
      for(uint8_t r=0;r<5;r++) drawSniffRow(r, r==snf_sel, false);
      drawSniffLive();
      break;
    case MENU_FACTORY_RESET_CONFIRM:
      showFactoryResetConfirm(true);
      break;
    case MENU_SPEED_TRIM:
      showSpeedTrim(true);
      break;
    case MENU_OBD2:
      showObd2Menu(true);
      break;
    case MENU_OBD2_ACTION:
      showObd2Action(true);
      break;
    default:
      break;
  }
}

// ===================== Buttons handler and nav =====================
void handleButton(Btn b){
  switch(menuState){

    case MENU_ROOT:{
      uint8_t prev = menuIndex;
      if(b==BTN_UP){ wrapDec(menuIndex,(uint8_t)(MENU_ROOT_COUNT-1)); updateRootSel(prev,menuIndex); }
      else if(b==BTN_DOWN){ wrapInc(menuIndex,(uint8_t)(MENU_ROOT_COUNT-1)); updateRootSel(prev,menuIndex); }
      else if(b==BTN_ENTER){
        g_lastRootIndex = menuIndex;
        if(menuIndex==0){ menuState=MENU_LAYOUT; layoutScreenSel=persist.currentScreen; showLayoutScreenPick(true); }
        else if(menuIndex==1){ menuState=MENU_WARN_LIST; warnListTop=0; warnListSel=0; showWarnList(true); }
        else if (menuIndex == 2) {
          menuState = MENU_COLOURS;
          enterColoursPage();        // <-- init coloursSel/coloursTop (centers on current)
           showColoursPage(true);     // full draw of the scrolling window
          }
        else if(menuIndex==3){ menuState=MENU_OBD2; obd2Sel=0; showObd2Menu(true); }
        else if(menuIndex==4){ menuState=MENU_SYSTEM; menuIndex=0; showSystemMenu(true); }
      } else if(b==BTN_CANCEL){ navExitSettings(); }
    } break;

    case MENU_LAYOUT:{
      uint8_t prev = layoutScreenSel;
      if(b==BTN_UP){ wrapDec(layoutScreenSel,(uint8_t)(SCREEN_COUNT - 1)); updateLayoutScreenSel(prev,layoutScreenSel); }
      else if(b==BTN_DOWN){ wrapInc(layoutScreenSel,(uint8_t)(SCREEN_COUNT - 1)); updateLayoutScreenSel(prev,layoutScreenSel); }
      else if(b==BTN_ENTER){
        layoutRow=0; layoutCol=0; menuState=MENU_LAYOUT_PICK_SLOT; showLayoutSlots(true);
      } else if(b==BTN_CANCEL){ menuState=MENU_ROOT; menuIndex=g_lastRootIndex; showRootMenu(true); }
    } break;

    case MENU_LAYOUT_PICK_SLOT:{
      uint8_t prevSlot = slotFromRowCol(layoutRow,layoutCol);
      if(b==BTN_LEFT){ if(layoutRow>=0) layoutCol = (layoutCol==0)?1:0; updateLayoutCursor(prevSlot,slotFromRowCol(layoutRow,layoutCol)); }
      else if(b==BTN_RIGHT){ if(layoutRow>=0) layoutCol = (layoutCol==0)?1:0; updateLayoutCursor(prevSlot,slotFromRowCol(layoutRow,layoutCol)); }
      else if(b==BTN_UP){ if(layoutRow<0) layoutRow=1; else if(layoutRow==0) layoutRow=-1; else layoutRow=0; updateLayoutCursor(prevSlot,slotFromRowCol(layoutRow,layoutCol)); }
      else if(b==BTN_DOWN){ if(layoutRow<0) layoutRow=0; else if(layoutRow==1) layoutRow=-1; else layoutRow=1; updateLayoutCursor(prevSlot,slotFromRowCol(layoutRow,layoutCol)); }
      else if(b==BTN_ENTER){
        uint8_t current = (layoutRow<0)? persist.barChannel[layoutScreenSel]
                                       : persist.pillChannel[layoutScreenSel][slotFromRowCol(layoutRow,layoutCol)];
        menuIndex2 = eligibleIndexFromChannel(current);
        const int perPage = MENU_PER_PAGE();
        pickerTop = (menuIndex2 / perPage) * perPage;
        menuState = MENU_LAYOUT_PICK_GAUGE; showLayoutGaugePicker(current,true);
      } else if(b==BTN_CANCEL){ menuState=MENU_LAYOUT; showLayoutScreenPick(true); }
    } break;

    case MENU_LAYOUT_PICK_GAUGE:{
      uint8_t &tgt = (layoutRow<0)? persist.barChannel[layoutScreenSel]
                                  : persist.pillChannel[layoutScreenSel][slotFromRowCol(layoutRow,layoutCol)];
      int total = eligibleCount();
      uint8_t prev = menuIndex2;
      if(b==BTN_UP){ menuIndex2 = (menuIndex2>0)? menuIndex2-1 : (uint8_t)(total-1); updateGaugePickerSel(prev,menuIndex2,tgt); }
      else if(b==BTN_DOWN){ menuIndex2 = (menuIndex2<(total-1))? menuIndex2+1 : 0; updateGaugePickerSel(prev,menuIndex2,tgt); }
      else if(b==BTN_ENTER){
        uint8_t chosen = channelFromEligibleIndex(menuIndex2);
        tgt = chosen; dirty=true; menuState=MENU_LAYOUT_PICK_SLOT; showLayoutSlots(true);
      } else if(b==BTN_CANCEL){ menuState=MENU_LAYOUT_PICK_SLOT; showLayoutSlots(true); }
    } break;

    case MENU_WARN_LIST:{
      int total = warnEligibleCount();
      int prevSel = warnListSel, prevTop = warnListTop;
      const int perPage=MENU_PER_PAGE();
      if(b==BTN_UP){
        if(total > 0){
          warnListSel = (warnListSel>0) ? (warnListSel-1) : (total-1);
        }
        if(warnListSel < warnListTop) warnListTop = warnListSel;
        if(warnListSel >= warnListTop+perPage) warnListTop = warnListSel - perPage + 1;
        updateWarnListSel(prevSel,warnListSel,prevTop,warnListTop);
      } else if(b==BTN_DOWN){
        if(total > 0){
          warnListSel = (warnListSel < total-1) ? (warnListSel+1) : 0;
        }
        if(warnListSel < warnListTop) warnListTop = warnListSel;
        if(warnListSel >= warnListTop+perPage) warnListTop = warnListSel - perPage + 1;
        updateWarnListSel(prevSel,warnListSel,prevTop,warnListTop);
      } else if(b==BTN_ENTER){
        warnFieldSel=0; menuState=MENU_WARN_EDIT; showWarnFieldEditor(warnChFromIdx(warnListSel), true);
      } else if(b==BTN_CANCEL){
        menuState=MENU_ROOT; menuIndex=g_lastRootIndex; showRootMenu(true);
      }
    } break;

    case MENU_WARN_EDIT:{
      uint8_t ch = warnChFromIdx(warnListSel);
      if(!warnFieldEditing){
        if(b==BTN_UP){ wrapDec(warnFieldSel,(uint8_t)2); for(int r=0;r<3;r++) drawWarnFieldRow(r,ch, r==warnFieldSel,false,false); }
        else if(b==BTN_DOWN){ wrapInc(warnFieldSel,(uint8_t)2); for(int r=0;r<3;r++) drawWarnFieldRow(r,ch, r==warnFieldSel,false,false); }
        else if(b==BTN_ENTER){
          editMode = persist.warnMode[ch]; editT1 = persist.warnT1[ch]; editT2 = persist.warnT2[ch];
          warnFieldEditing=true; warnBlinkMs=millis(); warnBlinkOn=true; repeating=false;
          holdDir = HOLD_NONE; holdLevel = 0; holdStep = stepFor((Channel)ch);
          drawWarnFieldRow(warnFieldSel,ch,true,false,true);
        } else if(b==BTN_CANCEL){
          menuState=MENU_WARN_LIST; showWarnList(true);
        }
      }else{
        if(warnFieldSel==0){
          if(b==BTN_UP){
            editMode = (uint8_t)((editMode+2)%3);
            if(editMode==CFG::WARN_HIGH){ if(editT2 < editT1) editT2 = editT1; }
            else if(editMode==CFG::WARN_LOW){ if(editT2 > editT1) editT1 = editT2; }
            drawWarnFieldRow(0,ch,true,false,true);
            drawWarnFieldRow(1,ch,false,false,true);
            drawWarnFieldRow(2,ch,false,false,true);
          }
          else if(b==BTN_DOWN){
            editMode = (uint8_t)((editMode+1)%3);
            if(editMode==CFG::WARN_HIGH){ if(editT2 < editT1) editT2 = editT1; }
            else if(editMode==CFG::WARN_LOW){ if(editT2 > editT1) editT1 = editT2; }
            drawWarnFieldRow(0,ch,true,false,true);
            drawWarnFieldRow(1,ch,false,false,true);
            drawWarnFieldRow(2,ch,false,false,true);
          }
          else if(b==BTN_ENTER){
            commitWarnField(ch); warnFieldEditing=false; for(int r=0;r<3;r++) drawWarnFieldRow(r,ch, r==warnFieldSel,false,false);
          } else if(b==BTN_CANCEL){
            discardWarnField(ch); warnFieldEditing=false; for(int r=0;r<3;r++) drawWarnFieldRow(r,ch, r==warnFieldSel,false,false);
          }
        } else {
          // T1/T2 numeric editing with hold escalation (in DISPLAY units)
          if(b==BTN_ENTER){
            commitWarnField(ch); warnFieldEditing=false; for(int r=0;r<3;r++) drawWarnFieldRow(r,ch, r==warnFieldSel,false,false);
          } else if(b==BTN_CANCEL){
            discardWarnField(ch); warnFieldEditing=false; for(int r=0;r<3;r++) drawWarnFieldRow(r,ch, r==warnFieldSel,false,false);
          } else if(b==BTN_UP || b==BTN_DOWN){
            stepWarnValueOnce(ch, b==BTN_UP);
            drawWarnFieldRow(1,ch, warnFieldSel==1, false, true);
            drawWarnFieldRow(2,ch, warnFieldSel==2, false, true);
          }
        }
      }
    } break;

    case MENU_COLOURS: {
      const int total   = paletteCount();
      const int perPage = MENU_PER_PAGE();

     int prevSel = coloursSel;
     int prevTop = coloursTop;

     if (b == BTN_UP) {
     if (total > 0) {
       coloursSel = (coloursSel > 0) ? (coloursSel - 1) : (total - 1);
     }
     if (coloursSel < coloursTop) coloursTop = coloursSel;
     if (coloursSel >= coloursTop + perPage) coloursTop = coloursSel - perPage + 1;
     updateColoursSel(prevSel, coloursSel, prevTop, coloursTop);

     } else if (b == BTN_DOWN) {
      if (total > 0) {
        coloursSel = (coloursSel < total - 1) ? (coloursSel + 1) : 0;
      }
      if (coloursSel < coloursTop) coloursTop = coloursSel;
      if (coloursSel >= coloursTop + perPage) coloursTop = coloursSel - perPage + 1;
      updateColoursSel(prevSel, coloursSel, prevTop, coloursTop);

   } else if (b == BTN_ENTER) {
     if (isCustomPaletteIndex(coloursSel)) {
       paletteIndex = coloursSel;
       dirty = true;
       customPaletteSel = customPaletteSlot(coloursSel);
       customZoneSel = 0;
       menuState = MENU_COLOURS_CUSTOM;
       showCustomPaletteMenu(true);
     } else {
       paletteIndex = coloursSel;                 // apply chosen palette
       dirty = true;
       showColoursPage(true);                      // full redraw to reflect theme
     }

   } else if (b == BTN_CANCEL) {
      menuState       = MENU_ROOT;
        g_lastRootIndex = 2;
       menuIndex       = 2;
       showRootMenu(true);
     }
    } break;

    case MENU_COLOURS_CUSTOM: {
      uint8_t prev = customZoneSel;
      if (b == BTN_UP) { wrapDec(customZoneSel, (uint8_t)(CUSTOM_ZONE_COUNT - 1)); updateCustomPaletteSel(prev, customZoneSel); }
      else if (b == BTN_DOWN) { wrapInc(customZoneSel, (uint8_t)(CUSTOM_ZONE_COUNT - 1)); updateCustomPaletteSel(prev, customZoneSel); }
      else if (b == BTN_ENTER) {
        enterCustomColourPicker();
        menuState = MENU_COLOURS_CUSTOM_PICK;
        showCustomColourPicker(true);
      } else if (b == BTN_CANCEL) {
        menuState = MENU_COLOURS;
        setColoursSelection(paletteIndex);
        showColoursPage(true);
      }
    } break;

    case MENU_COLOURS_CUSTOM_PICK: {
      const int total   = customColourCount();
      const int perPage = MENU_PER_PAGE();
      int prevSel = customColourSel;
      int prevTop = customColourTop;

      if (b == BTN_UP) {
        if (total > 0) {
          customColourSel = (customColourSel > 0) ? (customColourSel - 1) : (total - 1);
        }
        if (customColourSel < customColourTop) customColourTop = customColourSel;
        if (customColourSel >= customColourTop + perPage) customColourTop = customColourSel - perPage + 1;
        updateCustomColourSel(prevSel, customColourSel, prevTop, customColourTop);
      } else if (b == BTN_DOWN) {
        if (total > 0) {
          customColourSel = (customColourSel < total - 1) ? (customColourSel + 1) : 0;
        }
        if (customColourSel < customColourTop) customColourTop = customColourSel;
        if (customColourSel >= customColourTop + perPage) customColourTop = customColourSel - perPage + 1;
        updateCustomColourSel(prevSel, customColourSel, prevTop, customColourTop);
      } else if (b == BTN_ENTER) {
        CustomPalette& palette = persist.customPalettes[customPaletteSel];
        setCustomZoneValue(palette, customZoneSel, CUSTOM_COLOUR_OPTIONS[customColourSel].value);
        dirty = true;
        menuState = MENU_COLOURS_CUSTOM;
        showCustomPaletteMenu(true);
      } else if (b == BTN_CANCEL) {
        menuState = MENU_COLOURS_CUSTOM;
        showCustomPaletteMenu(true);
      }
    } break;


    case MENU_SYSTEM:{
      uint8_t prev = menuIndex;
      if(b==BTN_UP){ wrapDec(menuIndex,(uint8_t)(MENU_SYSTEM_COUNT-1)); updateSystemSel(prev,menuIndex); }
      else if(b==BTN_DOWN){ wrapInc(menuIndex,(uint8_t)(MENU_SYSTEM_COUNT-1)); updateSystemSel(prev,menuIndex); }
      else if(b==BTN_ENTER){
        if(menuIndex==0){ menuState=MENU_BRIGHTNESS; brightSel=0; brightEditing=false; showBrightness(true); }
        else if(menuIndex==1){ menuState=MENU_UNITS; unitsSel=0; unitsEditing=false; showUnitsPage(true); }
        else if(menuIndex==2){ menuState=MENU_WIFI; enterWifiPage(); showWifiMenu(true); }
        else if(menuIndex==3){ menuState=MENU_CAN_SNIFF; showCanSniff(true); }
        else if(menuIndex==4){ menuState=MENU_SPEED_TRIM; speedTrimEditing=false; showSpeedTrim(true); }
        else if(menuIndex==5){ menuState=MENU_FACTORY_RESET_CONFIRM; showFactoryResetConfirm(true); }
      } else if(b==BTN_CANCEL){
        menuState=MENU_ROOT; menuIndex=g_lastRootIndex; showRootMenu(true);
      }
    } break;

    case MENU_OBD2:{
      uint8_t prev = obd2Sel;
      if(b==BTN_UP){ wrapDec(obd2Sel,(uint8_t)(MENU_OBD2_COUNT-1)); updateObd2Sel(prev,obd2Sel); }
      else if(b==BTN_DOWN){ wrapInc(obd2Sel,(uint8_t)(MENU_OBD2_COUNT-1)); updateObd2Sel(prev,obd2Sel); }
      else if(b==BTN_ENTER){
        menuState = MENU_OBD2_ACTION;
        if(obd2Sel == 0){
          sendObd2ReadRequest();
        } else if(obd2Sel == 1){
          sendObd2ClearRequest();
        }
        showObd2Action(true);
      } else if(b==BTN_CANCEL){
        menuState=MENU_ROOT; menuIndex=g_lastRootIndex; showRootMenu(true);
      }
    } break;

    case MENU_OBD2_ACTION:{
      if(b==BTN_CANCEL){
        obd2Awaiting = false;
        obd2NeedsRedraw = false;
        menuState = MENU_OBD2;
        showObd2Menu(true);
      }
    } break;

    case MENU_BRIGHTNESS:{
      if(!brightEditing){
        if(b==BTN_UP){ uint8_t prev=brightSel; wrapDec(brightSel,(uint8_t)1); updateBrightnessSel(prev,brightSel); }
        else if(b==BTN_DOWN){ uint8_t prev=brightSel; wrapInc(brightSel,(uint8_t)1); updateBrightnessSel(prev,brightSel); }
        else if(b==BTN_ENTER){
          brightEditing = true;
          brightBlinkMs = millis(); brightBlinkOn = true;
          repeating=false; holdDir=HOLD_NONE; holdLevel=0; holdStep=1.0f;
          drawBrightnessRow(brightSel, true, false);
        } else if(b==BTN_CANCEL){
          menuState=MENU_SYSTEM; menuIndex=0; showSystemMenu(true);
        }
      } else {
        // editing
        if(b==BTN_ENTER){
          brightEditing=false;
          drawBrightnessRow(brightSel, true, false);
          applyBacklight();   // ensure applied
          dirty=true;
        } else if(b==BTN_UP || b==BTN_DOWN){
          // single step (±1), clamped to MIN_BRIGHT..100
          int delta = (b==BTN_UP)? +1 : -1;
          if(brightSel==0) brightOn  = (uint8_t)clampf((int)brightOn + delta, MIN_BRIGHT,100);
          else             brightOff = (uint8_t)clampf((int)brightOff + delta, MIN_BRIGHT,100);
          drawBrightnessRow(brightSel, true, false);
          applyBacklight();   // live update
          dirty = true;
        } else if(b==BTN_CANCEL){
          brightEditing=false;
          drawBrightnessRow(brightSel, true, false);
          applyBacklight();
        }
      }
    } break;

    case MENU_UNITS:{
      if(!unitsEditing){
        if(b==BTN_UP){ uint8_t prev=unitsSel; wrapDec(unitsSel,(uint8_t)3); updateUnitsSel(prev,unitsSel); }
        else if(b==BTN_DOWN){ uint8_t prev=unitsSel; wrapInc(unitsSel,(uint8_t)3); updateUnitsSel(prev,unitsSel); }
        else if(b==BTN_ENTER){
          unitsEditing=true; unitsBlinkMs=millis(); unitsBlinkOn=true;
          drawUnitsRow(unitsSel, true, false);
        } else if(b==BTN_CANCEL){
          menuState=MENU_SYSTEM; menuIndex=1; showSystemMenu(true);
        }
      } else {
        bool changed=false;
        if(b==BTN_UP || b==BTN_DOWN){
          switch(unitsSel){
            case 0: g_uPressure = (g_uPressure==U_P_kPa)? U_P_psi:U_P_kPa; changed=true; break;
            case 1: g_uTemp     = (g_uTemp==U_T_C)? U_T_F:U_T_C; changed=true; break;
            case 2: g_uSpeed    = (g_uSpeed==U_S_kmh)? U_S_mph:U_S_kmh; changed=true; break;
            case 3: g_uLambda   = (g_uLambda==U_L_lambda)? U_L_AFR:U_L_lambda; changed=true; break;
          }
          if(changed){
            // Redraw whole preview/static to refresh unit labels and ticks
            showUnitsPage(true);
            // Update live UI frames if we’re not in settings
            dirty=true;
          }
        } else if(b==BTN_ENTER){
          unitsEditing=false; drawUnitsRow(unitsSel, true, false); dirty=true;
        } else if(b==BTN_CANCEL){
          unitsEditing=false; drawUnitsRow(unitsSel, true, false);
        }
      }
    } break;

    case MENU_WIFI:{
      if(b==BTN_CANCEL){
        exitWifiPage();
        menuState=MENU_SYSTEM; menuIndex=2; showSystemMenu(true);
      }
    } break;

   case MENU_CAN_SNIFF:{
   if(!snf_editing){
    if(b==BTN_UP){ uint8_t prev=snf_sel; wrapDec(snf_sel,(uint8_t)4); drawSniffRow(prev,false,false); drawSniffRow(snf_sel,true,false); }
    else if(b==BTN_DOWN){ uint8_t prev=snf_sel; wrapInc(snf_sel,(uint8_t)4); drawSniffRow(prev,false,false); drawSniffRow(snf_sel,true,false); }
    else if(b==BTN_ENTER){
      snf_editing = true;
      drawSniffRow(snf_sel, true, false);
    } else if(b==BTN_CANCEL){
      menuState=MENU_SYSTEM; menuIndex=3; showSystemMenu(true); // return to System menu at "CAN Sniff"
    }
   } else {
    if(snf_sel==0){
      // Address hex editor per nibble
      if(b==BTN_LEFT){
        if(snf_cursor>0) snf_cursor--;
        drawSniffRow(0, true, false);
      } else if(b==BTN_RIGHT){
        if(snf_cursor+1 < SNF_HEX_DIGITS) snf_cursor++;
        drawSniffRow(0, true, false);
      } else if(b==BTN_UP || b==BTN_DOWN){
        uint8_t posFromMSB = snf_cursor;
        int8_t  bitShift   = (SNF_HEX_DIGITS-1 - posFromMSB) * 4;
        uint32_t mask      = (uint32_t)0xF << bitShift;
        uint8_t oldNib     = (snf_id >> bitShift) & 0xF;
        int8_t newNib      = oldNib + ((b==BTN_UP)? +1 : -1);
        if(newNib < 0) newNib = 15;
        if(newNib > 15) newNib = 0;
        snf_id = (snf_id & ~mask) | ((uint32_t)newNib << bitShift);
        if(snf_id > 0x7FF) snf_id = 0x7FF; // keep standard 11-bit
        snf_has=false; // force wait until next match
        drawSniffRow(0, true, false);
        drawSniffLive();
      } else if(b==BTN_ENTER || b==BTN_CANCEL){
        snf_editing = false;
        drawSniffRow(0, true, false);
      }
    } else if(snf_sel==1 || snf_sel==2){
      // From/To bit editors
      uint8_t &val = (snf_sel==1)? snf_bit_from : snf_bit_to;
      if(b==BTN_UP){ if(val<63) val++; drawSniffRow(snf_sel, true, false); drawSniffLive(); }
      else if(b==BTN_DOWN){ if(val>0) val--; drawSniffRow(snf_sel, true, false); drawSniffLive(); }
      else if(b==BTN_LEFT || b==BTN_RIGHT){
        uint8_t prev = snf_sel;
        snf_sel = (snf_sel==1)? 2 : 1;
        drawSniffRow(prev, false, false);
        drawSniffRow(snf_sel, true, false);
      } else if(b==BTN_ENTER || b==BTN_CANCEL){
        snf_editing = false;
        drawSniffRow(snf_sel, true, false);
      }
    } else if(snf_sel==3 || snf_sel==4){
      // Scale / Bias (float) with adjustable step via LEFT/RIGHT
      float &val = (snf_sel==3)? snf_scale : snf_bias;
      if(b==BTN_UP){ val += snf_step; drawSniffRow(snf_sel, true, false); drawSniffLive(); }
      else if(b==BTN_DOWN){ val -= snf_step; drawSniffRow(snf_sel, true, false); drawSniffLive(); }
      else if(b==BTN_LEFT){
        snf_step /= 10.0f; if(snf_step < 0.001f) snf_step = 0.001f;
        drawSniffRow(snf_sel, true, true);
      } else if(b==BTN_RIGHT){
        snf_step *= 10.0f; if(snf_step > 1000.0f) snf_step = 1000.0f;
        drawSniffRow(snf_sel, true, true);
      } else if(b==BTN_ENTER || b==BTN_CANCEL){
        snf_editing = false;
        drawSniffRow(snf_sel, true, false);
      }
    }
   }
  } break;

  case MENU_SPEED_TRIM:{
  if(!speedTrimEditing){
    // Start editing on any action except CANCEL
    if(b==BTN_ENTER || b==BTN_UP || b==BTN_DOWN || b==BTN_LEFT || b==BTN_RIGHT){
      speedTrimEditing = true; speedTrimBlinkMs = millis(); speedTrimBlinkOn = true;
      // reset hold state
      repeating=false; holdDir=HOLD_NONE; holdLevel=0; holdStep=1.0f;
      drawSpeedTrimRow(true, false);
    } else if(b==BTN_CANCEL){
      menuState=MENU_SYSTEM; menuIndex=4; showSystemMenu(true);
    }
  } else {
    // Editing
    if(b==BTN_CANCEL || b==BTN_ENTER){
      speedTrimEditing = false;
      drawSpeedTrimRow(true, false);
    } else if(b==BTN_UP){
      speedTrimPct += SPEED_TRIM_STEP;
      if(speedTrimPct > SPEED_TRIM_MAX) speedTrimPct = SPEED_TRIM_MAX;
      drawSpeedTrimRow(true, false); dirty = true;
    } else if(b==BTN_DOWN){
      speedTrimPct -= SPEED_TRIM_STEP;
      if(speedTrimPct < SPEED_TRIM_MIN) speedTrimPct = SPEED_TRIM_MIN;
      drawSpeedTrimRow(true, false); dirty = true;
    }
  }
 } break;

    case MENU_FACTORY_RESET_CONFIRM:{
      if(b==BTN_ENTER){
        // Erase EEPROM and reload defaults
        EEPROM.begin(Persist::EEPROM_BYTES);
        for(size_t i=0;i<Persist::EEPROM_BYTES;++i) EEPROM.write(i,0xFF);
        EEPROM.commit();
        loadPersistState();
        // Visual feedback
        fullScreenMenuFrame("Reset complete");
        redrawMenuRowAtLogical(0, "Defaults restored", "", true);

        menuState=MENU_SYSTEM; menuIndex=5; showSystemMenu(true);
      } else if(b==BTN_CANCEL){
        menuState=MENU_SYSTEM; menuIndex=5; showSystemMenu(true);
      }
    } break;

    default: break;
  }
}

// Buttons from CAN + Double-tap Cancel + Press-hold to enter settings
void updateButtonsFromFrame(const can_frame& f){
  if(f.can_id!=CFG::ID_SWBTN) return;

  // Press states
  bool cancel_now=bitSetSafe(f,6,4);
  up_now   = bitSetSafe(f,6,2);
  down_now = bitSetSafe(f,6,0);
  bool left = bitSetSafe(f,7,4), right=bitSetSafe(f,7,2), enter=bitSetSafe(f,7,6);

  // Press & hold (2s) enter/exit Settings WHILE holding
  if(cancel_now && !sw_cancel_pressed){ sw_cancel_pressed=true; sw_cancel_t0=millis(); }
  if(cancel_now && sw_cancel_pressed){
    if(millis()-sw_cancel_t0 >= 2000){
      sw_cancel_pressed=false;
      if(inSettings()) navExitSettings(); else navEnterSettings();
      cancelTapCount=0; suppressNextCancelRelease=true; return;
    }
  }

  // Short-press release handling (double-tap only in live UI)
  if(!cancel_now && sw_cancel_pressed){
    unsigned long held=millis()-sw_cancel_t0;
    sw_cancel_pressed=false;

    if(suppressNextCancelRelease){ suppressNextCancelRelease=false; return; }

    // if(held < 2000) {
    if (inSettings()) {
      handleButton(BTN_CANCEL); // Back via Cancel inside menus
    } else {
      unsigned long now = millis();
      if (cancelTapCount == 0) {
        cancelTapCount = 1; 
        lastCancelTapMs = now;
      } else {
        if (now - lastCancelTapMs <= DOUBLE_TAP_MS) {
          // Go to next screen
          persist.currentScreen = (persist.currentScreen + 1) % SCREEN_COUNT;       //  No. of screen in rotation
          dirty = true;

          // Redraw chrome
          tft.fillScreen(COL_BG());
          drawAppBar();

          updateRegenState();
          renderStatic();
          renderDynamic();

          cancelTapCount = 0;
        } else {
          cancelTapCount = 1; 
          lastCancelTapMs = now;
        }
      }
    }
  }

  // Expire single tap window
  if(cancelTapCount==1 && (millis() - lastCancelTapMs > DOUBLE_TAP_MS)) cancelTapCount = 0;

  // Enter hold for min/max display (main UI only)
  if(menuState == UI_MAIN){
    if(enter && !sw_enter_pressed){
      sw_enter_pressed = true;
      sw_enter_t0 = millis();
    }
    if(enter && sw_enter_pressed){
      if(!uiMinMaxActive && (millis() - sw_enter_t0 >= ENTER_HOLD_MS)){
        setMinMaxActive(true);
        enterTapCount = 0;
        suppressNextEnterRelease = true;
      }
    }
    if(!enter && sw_enter_pressed){
      sw_enter_pressed = false;
      if(uiMinMaxActive){
        setMinMaxActive(false);
        suppressNextEnterRelease = true;
      }
      if(suppressNextEnterRelease){
        suppressNextEnterRelease = false;
      } else {
        unsigned long now = millis();
        if(enterTapCount == 0){
          enterTapCount = 1;
          lastEnterTapMs = now;
        } else if(now - lastEnterTapMs <= ENTER_TAP_MS){
          enterTapCount++;
          lastEnterTapMs = now;
          if(enterTapCount >= 3){
            resetMinMaxValues();
            enterTapCount = 0;
          }
        } else {
          enterTapCount = 1;
          lastEnterTapMs = now;
        }
      }
    }

    if(enterTapCount > 0 && (millis() - lastEnterTapMs > ENTER_TAP_MS)) enterTapCount = 0;
  }

  // Edge-triggered dispatch
  if(up_now && !sw_up_prev) handleButton(BTN_UP);
  if(down_now && !sw_down_prev) handleButton(BTN_DOWN);
  if(left && !sw_left_prev) handleButton(BTN_LEFT);
  if(right && !sw_right_prev) handleButton(BTN_RIGHT);
  if(enter && !sw_enter_prev) handleButton(BTN_ENTER);
  sw_up_prev=up_now; sw_down_prev=down_now; sw_left_prev=left; sw_right_prev=right; sw_enter_prev=enter;
}

// ===================== Regen banner =====================
void updateRegenState(){
  // --- ACTIVE/IDLE hysteresis using percent thresholds in CFG:: ---
  static RegenState baseState = REGEN_IDLE;

  const bool active_now =
      (baseState == REGEN_ACTIVE)
        ? (regen_pct > CFG::REGEN_OFF_PCT)   // stay active until below OFF
        : (regen_pct > CFG::REGEN_ON_PCT);   // turn on once above ON

  baseState = active_now ? REGEN_ACTIVE : REGEN_IDLE;

  // --- Simple % change probe (1 Hz) to detect "paused" ---
  static uint32_t probe_ms  = 0;
  static float    probe_pct = 0.0f;
  static uint8_t  pause_conf = 0;  // tiny hysteresis (0..2)

  const uint32_t now = millis();
  if (probe_ms == 0) {
    probe_ms  = now;
    probe_pct = regen_pct;
  } else if (now - probe_ms >= 1000) {
    const float diff = fabsf(regen_pct - probe_pct);
    const bool paused_now = (regen_pct > 0.0f) && (diff <= CFG::REGEN_PAUSE_EPS_PCT);

    pause_conf = paused_now ? (pause_conf < 2 ? pause_conf + 1 : 2) : 0;

    probe_pct = regen_pct;
    probe_ms  = now;
  }

  // --- Final state selection (prefer PAUSED over ACTIVE) ---
  if (baseState == REGEN_ACTIVE && pause_conf >= 2) {
    regenState = REGEN_PAUSED;
  } else {
    regenState = baseState; // ACTIVE or IDLE
  }
}

// ===================== Cluster Beep =====================
inline void triggerClusterBeep(){
  if (!CFG::BEEP_ENABLED) return; // toggle kills the send
  struct can_frame out{};
  out.can_id  = CFG::ID_CLUSTER_BEEP;
  out.can_dlc = CFG::CLUSTER_BEEP_DLC;
  for(uint8_t i=0;i<out.can_dlc && i<8;i++) out.data[i] = CFG::CLUSTER_BEEP_PAYLOAD[i];
  mcp.sendMessage(&out);
}

// ===================== Setup / Loop =====================
void setup(){
  loadPersistState();
  resetMinMaxValues();
  g_lastBacklightPct = 255;

  // Ensure CS lines idle high before SPI
  pinMode(CFG::TFT_CS,OUTPUT); digitalWrite(CFG::TFT_CS,HIGH);
  pinMode(CFG::CAN_CS,OUTPUT); digitalWrite(CFG::CAN_CS,HIGH);
  pinMode(CFG::BACKLIGHT_PWM, OUTPUT);
  analogWrite(CFG::BACKLIGHT_PWM, 0);

  SPI.begin(CFG::SPI_SCK,CFG::SPI_MISO,CFG::SPI_MOSI); SPI.setFrequency(TFT_SPI_HZ);

  // --- TFT boot & first paint ---
  tft.begin(); tft.setRotation(1);
  initUi(tft, nullptr);
  tft.fillScreen(COL_BG());
  drawAppBar();
  renderStatic();
  renderDynamic();

  // After first frame is ready, apply backlight PWM
  brightOn  = max<uint8_t>(brightOn,  MIN_BRIGHT);
  brightOff = max<uint8_t>(brightOff, MIN_BRIGHT);
  applyBacklight();

  // --- CAN init ---
  pinMode(CFG::CAN_INT,INPUT_PULLUP); mcp.reset(); mcp.setBitrate(CFG::CAN_SPEED_SEL,CFG::CAN_CLOCK_SEL);
  mcp.setFilterMask(MCP2515::MASK0,false,0x000); mcp.setFilterMask(MCP2515::MASK1,false,0x000);
  mcp.setFilter(MCP2515::RXF0,false,0x000); mcp.setFilter(MCP2515::RXF1,false,0x000); mcp.setFilter(MCP2515::RXF2,false,0x000);
  mcp.setFilter(MCP2515::RXF3,false,0x000); mcp.setFilter(MCP2515::RXF4,false,0x000); mcp.setFilter(MCP2515::RXF5,false,0x000);
  mcp.setNormalMode(); attachInterrupt(digitalPinToInterrupt(CFG::CAN_INT), onCanInt, FALLING);

  // --- BLE scan for Victron Instant Readout ---
  victronInit();

  unsigned long now=millis(); lastMillis=lastDraw=now;
}

void loop(){
  unsigned long now=millis();
  g_victronReadings = victronLoop();
  if(g_webServerActive){
    webServer.handleClient();
  }

  if(can_irq){
    can_irq=false; struct can_frame f;
    while(mcp.readMessage(&f)==MCP2515::ERROR_OK){
      CanDec::decodeFrame(f);
      updateButtonsFromFrame(f);
      snifferMaybeCapture(f);
      obd2MaybeCapture(f);

    }
  } else {
    struct can_frame f;
    if(mcp.readMessage(&f)==MCP2515::ERROR_OK){
      CanDec::decodeFrame(f);
      updateButtonsFromFrame(f);
      snifferMaybeCapture(f);
      obd2MaybeCapture(f);

    }
  }

  // Regen banner update
  RegenState old = regenState; updateRegenState();
  if(old!=regenState && !inSettings() && !uiMinMaxActive)
    renderDynamic();

  updateMinMaxValues();
  if(menuState == MENU_OBD2_ACTION){
    updateObd2Timeout(now);
    if(obd2NeedsRedraw){
      showObd2Action(true);
      obd2NeedsRedraw = false;
    }
  }

  if(menuState==UI_MAIN){
    // blink tick for warning overlays/title
    if(now - uiWarnBlinkMs >= 500){
      uiWarnBlinkMs = now; uiWarnBlinkOn = !uiWarnBlinkOn;
      renderDynamic(); // flip overlays/title immediately
      lastDraw = now;
    }
    // regular dynamic refresh
    if(now-lastDraw>=CFG::SCREEN_REFRESH_MS){ renderDynamic(); lastDraw=now; }
  }

  // ===== Units page blink while editing =====
  if(menuState==MENU_UNITS && unitsEditing){
    if(now - unitsBlinkMs >= 400){
      unitsBlinkMs = now; unitsBlinkOn = !unitsBlinkOn;
      drawUnitsRow(unitsSel, true, !unitsBlinkOn);
    }
  }
   // ===== CAN Sniffer: blink active row while editing =====
   if(menuState==MENU_CAN_SNIFF && snf_editing){
     static unsigned long snfBlinkMs = 0;
     static bool snfBlinkOn = true;
     unsigned long nowMs = millis();
     if(nowMs - snfBlinkMs >= 400){
     snfBlinkMs = nowMs; snfBlinkOn = !snfBlinkOn;
     drawSniffRow(snf_sel, true, !snfBlinkOn);
   }
  }
// ===== PATCH: Warning editor: blink & hold-to-repeat with decade acceleration =====
if (menuState == MENU_WARN_EDIT) {
  uint8_t ch = warnChFromIdx(warnListSel);

  // Blink the active field while editing
  if (warnFieldEditing) {
    if (now - warnBlinkMs >= 400) {
      warnBlinkMs = now; warnBlinkOn = !warnBlinkOn;
      // Repaint the currently selected field (0=Mode, 1=T1, 2=T2)
      drawWarnFieldRow(warnFieldSel, ch, true, !warnBlinkOn, true);
    }
  }

  // Hold-to-repeat for numeric fields only (T1/T2)
  if (warnFieldEditing && (warnFieldSel == 1 || warnFieldSel == 2)) {
    bool pressedUp   = up_now;
    bool pressedDown = down_now;
    bool pressed     = pressedUp || pressedDown;

    // Start a new repeat session
    if (pressed && !repeating) {
      repeating      = true;
      repeatStartMs  = now;
      lastRepeatMs   = now;
      holdDir        = pressedUp ? HOLD_UP : HOLD_DOWN;
      holdLevel      = 0;
      holdStep       = stepFor((Channel)ch);  // base step in DISPLAY units
    }
    // End the repeat session
    if (!pressed && repeating) {
      repeating      = false;
      holdDir        = HOLD_NONE;
      holdLevel      = 0;
      holdStep       = 1.0f;
    }

    // Allow direction change mid-hold
    if (repeating) {
      HoldDir currentDir = pressedUp ? HOLD_UP : (pressedDown ? HOLD_DOWN : HOLD_NONE);
      if (currentDir != HOLD_NONE && currentDir != holdDir) {
        holdDir       = currentDir;
        holdLevel     = 0;
        holdStep      = stepFor((Channel)ch);
        repeatStartMs = now;
        lastRepeatMs  = now;
      }
    }
    // Timed repeat ticks
    if (repeating) {
      const unsigned long firstDelay     = 350;
      const unsigned long repeatInterval = 350;
      if ((now - repeatStartMs) >= firstDelay && (now - lastRepeatMs) >= repeatInterval) {
        lastRepeatMs = now;

        // --- Apply one step in DISPLAY units, then write back to BASE ---
        // Convert staged BASE -> DISPLAY for the field being edited
        float dispT = (warnFieldSel == 1) ? editT1 : editT2;

        auto baseToDisp = [&](float v)->float{
          switch ((Channel)ch) {
            case CH_LAMBDA: return toDisplayLambda(v);
            case CH_COOLANT: case CH_TRANS1: case CH_TRANS2: case CH_IAT: case CH_FUELT:
            case CH_EGT1: case CH_EGT2: case CH_MANIFOLD: case CH_TURBO_OUT: return toDisplayTemp(v);
            case CH_BOOST: case CH_OIL: return toDisplayPressure(v);
            case CH_SPEED: return toDisplaySpeed(v);
            default: return v;
          }
        };
        auto dispToBase = [&](float v)->float{
          switch ((Channel)ch) {
            case CH_LAMBDA: return fromDisplayLambda(v);
            case CH_COOLANT: case CH_TRANS1: case CH_TRANS2: case CH_IAT: case CH_FUELT:
            case CH_EGT1: case CH_EGT2: case CH_MANIFOLD: case CH_TURBO_OUT: return fromDisplayTemp(v);
            case CH_BOOST: case CH_OIL: return fromDisplayPressure(v);
            case CH_SPEED: return fromDisplaySpeed(v);
            default: return v;
          }
        };

        float curDisp  = baseToDisp(dispT);
        float stepDisp = holdStep * ((holdDir == HOLD_UP) ? +1.0f : -1.0f);
        float prevDisp = curDisp;
        float newDisp  = curDisp + stepDisp;

        // Clamp to display range
        Range rd = rangeFor((Channel)ch);
        newDisp = clampf(newDisp, rd.mn, rd.mx);

        // Write back to staged BASE value
        float newBase = dispToBase(newDisp);
        if (warnFieldSel == 1) editT1 = newBase; else editT2 = newBase;

        // Keep ordering sane in BASE
        if (editMode == CFG::WARN_HIGH) { if (editT2 < editT1) editT2 = editT1; }
        else if (editMode == CFG::WARN_LOW) { if (editT2 > editT1) editT1 = editT2; }

        // Decade acceleration in DISPLAY space
        int dirSign = (holdDir == HOLD_UP ? +1 : -1);
        maybeEscalateHoldStep(stepFor((Channel)ch), prevDisp, newDisp, dirSign);

        // Redraw rows
        drawWarnFieldRow(1, ch, (warnFieldSel == 1), false, true);
        drawWarnFieldRow(2, ch, (warnFieldSel == 2), false, true);
      }
    }
  }
}
// ===== Speed Trim: blink & hold-repeat =====
if(menuState==MENU_SPEED_TRIM){
  if(speedTrimEditing){
    // Blink the value
    if(now - speedTrimBlinkMs >= 400){
      speedTrimBlinkMs = now; speedTrimBlinkOn = !speedTrimBlinkOn;
      drawSpeedTrimRow(true, !speedTrimBlinkOn);
    }

    // Hold-to-repeat
    bool pressedUp   = up_now;
    bool pressedDown = down_now;
    bool pressed = pressedUp || pressedDown;

    if(pressed && !repeating){
      repeating=true; repeatStartMs=now; lastRepeatMs=now;
      holdDir = pressedUp ? HOLD_UP : HOLD_DOWN;
      holdLevel = 0;
      holdStep = SPEED_TRIM_STEP;   // base step in percentage points
    }
    if(!pressed && repeating){
      repeating=false; holdDir=HOLD_NONE; holdLevel=0; holdStep=1.0f;
    }
    if(repeating){
      HoldDir currentDir = pressedUp ? HOLD_UP : (pressedDown ? HOLD_DOWN : HOLD_NONE);
      if(currentDir != HOLD_NONE && currentDir != holdDir){
        holdDir = currentDir; holdLevel=0; holdStep=SPEED_TRIM_STEP; repeatStartMs=now; lastRepeatMs=now;
      }
    }
    if(repeating){
      const unsigned long firstDelay = 350;
      const unsigned long repeatInterval = 350;
      if(now - repeatStartMs >= firstDelay && now - lastRepeatMs >= repeatInterval){
        lastRepeatMs = now;
        float st = (holdDir==HOLD_UP)? +holdStep : -holdStep;
        float prevVal = speedTrimPct;

        // Same bounds as page handler
        speedTrimPct += st;
        if(speedTrimPct < SPEED_TRIM_MIN) speedTrimPct = SPEED_TRIM_MIN;
        if(speedTrimPct > SPEED_TRIM_MAX) speedTrimPct = SPEED_TRIM_MAX;

        int dirSign = (holdDir == HOLD_UP ? +1 : -1);
        maybeEscalateHoldStep(SPEED_TRIM_STEP, prevVal, speedTrimPct, dirSign);

        drawSpeedTrimRow(true, false);
        dirty = true;
      }
    }
  }
}
  // ===== Brightness editor: blink & progressive hold logic =====
  if(menuState==MENU_BRIGHTNESS){
    if(brightEditing){
      if(now - brightBlinkMs >= 400){
        brightBlinkMs = now; brightBlinkOn = !brightBlinkOn;
        drawBrightnessRow(brightSel, true, !brightBlinkOn);
      }

      bool pressedUp   = up_now;
      bool pressedDown = down_now;
      bool pressed = pressedUp || pressedDown;

      if(pressed && !repeating){
        repeating=true; repeatStartMs=now; lastRepeatMs=now;
        holdDir = pressedUp ? HOLD_UP : HOLD_DOWN;
        holdLevel = 0;
        holdStep  = 1.0f; // base step 1%
      }
      if(!pressed && repeating){
        repeating=false; holdDir = HOLD_NONE; holdLevel = 0; holdStep=1.0f;
      }
      if(repeating){
        HoldDir currentDir = pressedUp ? HOLD_UP : (pressedDown ? HOLD_DOWN : HOLD_NONE);
        if(currentDir != HOLD_NONE && currentDir != holdDir){
          holdDir = currentDir; holdLevel=0; holdStep=1.0f; repeatStartMs=now; lastRepeatMs=now;
        }
      }
      if(repeating){
        const unsigned long firstDelay = 350;
        const unsigned long repeatInterval = 350;
        if(now - repeatStartMs >= firstDelay && now - lastRepeatMs >= repeatInterval){
          lastRepeatMs = now;
          uint8_t &cur = (brightSel==0)? brightOn : brightOff;
          int st = (holdDir==HOLD_UP)? (int)holdStep : -(int)holdStep;
          float prevF = (float)cur;
          int ni = (int)cur + st;
          if(ni<MIN_BRIGHT) ni=MIN_BRIGHT; if(ni>100) ni=100;
          cur = (uint8_t)ni;

          int dirSign = (holdDir == HOLD_UP ? +1 : -1);
          maybeEscalateHoldStep(1.0f, prevF, (float)cur, dirSign);

          drawBrightnessRow(brightSel, true, false);
          applyBacklight();   // live
          dirty = true;
        }
      }
    }
  }

  // ===== Global Level-2 detection → cluster beep (edge-triggered) =====
  bool globalL2ActiveNow = false;
  for(int i=0;i<CH__COUNT;i++){
    if(isWarnEligible((Channel)i) && warnLevelFor((Channel)i) == 2){
      globalL2ActiveNow = true; break;
    }
  }
  if(globalL2ActiveNow && !globalL2ActivePrev){
    if(now - lastBeepMs >= CFG::BEEP_COOLDOWN_MS){
      triggerClusterBeep();
      lastBeepMs = now;
    }
  }
  globalL2ActivePrev = globalL2ActiveNow;

  // --- Backlight: react to headlights state changes ---
  if (headlightsOn != prevHeadlightsOn) {
    prevHeadlightsOn = headlightsOn;
    applyBacklight();
  }

  // Save if dirty
  if(dirty){
    persist.paletteIndex=paletteIndex;
    persist.brightOn=brightOn; persist.brightOff=brightOff;
    persist.uPressure=g_uPressure; persist.uTemp=g_uTemp; persist.uSpeed=g_uSpeed; persist.uLambda=g_uLambda;
    persist.speedTrimPct = speedTrimPct;
    savePersist(persist, dirty);
  }
  lastMillis=now;
}
