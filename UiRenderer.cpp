#include "UiRenderer.h"

#include <Arduino.h>
#include <Adafruit_ILI9341.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <math.h>
#include <string.h>

#include "DashTypes.h"
#include "ValueConversion.h"

enum TCState : uint8_t;

extern const int APPBAR_H;
extern const int BAR_X;
extern const int BAR_Y;
extern const int BAR_W;
extern const int BAR_H;
extern const int BAR_R;

extern uint8_t g_uLambda;
extern float battV;
extern int gear;
extern int targetgear;
extern bool headlightsOn;
extern volatile TCState g_tcState;
extern bool uiMinMaxActive;
extern bool uiWarnBlinkOn;
extern uint8_t uiHighestWarnLevel;
extern Channel uiHighestWarnCh;
extern RegenState regenState;

extern Channel currentBarChannel();
extern Channel currentPillChannel(uint8_t slot);

extern const char* labelText(Channel ch);
extern const char* unitLabel(Channel ch);
extern Range rangeFor(Channel c);
extern PillSpec pillSpec(int idx);
extern const char* minMaxSuffixFor(Channel ch);
extern MinMaxMode minMaxModeFor(Channel ch);
extern float minMaxDisplayValue(Channel ch);
extern int valueKey(Channel ch);
extern int valueKeyForDisplay(Channel ch, float displayValue);
extern void formatDisplayValue(Channel ch, float displayValue, char* out, size_t outSize);
extern uint8_t warnLevelFor(Channel ch);
extern void overlayPillWarnOutlineThick(const PillSpec& p, uint16_t col);
extern uint16_t barFillColor();
extern float clampf(float v,float lo,float hi);
extern void clearRegion(int x,int y,int w,int h,uint16_t col);
extern void drawPillFrame(const PillSpec& p, bool sel, Channel ch);
extern void drawPillLabelForChannel(const PillSpec& p, Channel ch);
extern void clearPillValue(const PillSpec& p);
extern void drawValueInPill(const PillSpec& p, const char* num, const char* unit);
static const char* gearText(int g){
  switch(g){
    case -3: return "P";
    case -2: return "R";
    case -1: return "N";
    case 0:  return "--";
    default: break;
  }
  static char b[4];
  snprintf(b,sizeof(b),"%d",g);
  return b;
}
extern const char* tcStateText(TCState s);
extern uint16_t tcStateColor(TCState s);
extern uint16_t COL_CARD();
extern uint16_t COL_ACCENT();
extern uint16_t COL_BG();
extern uint16_t COL_FRAME();
extern uint16_t COL_TICKS();
extern uint16_t COL_TXT();
extern uint16_t COL_YELLOW();
extern uint16_t COL_ORANGE();
extern uint16_t COL_RED();

static Adafruit_ILI9341* s_tft = nullptr;
static const Palette* s_palette = nullptr;

static Channel prevBarChannel = CH__COUNT;
static int prevBarFillW = -1;
static int prevPillValueKey[4] = {INT32_MIN,INT32_MIN,INT32_MIN,INT32_MIN};
static Channel prevPillChannel[4] = {CH__COUNT,CH__COUNT,CH__COUNT,CH__COUNT};
static uint8_t prevPillWarnLevel[4] = {0,0,0,0};

static char g_prevTitle[64] = "";
static uint16_t g_prevTitleColor = 0;
static char g_prevTitleSuffix[16] = "";

static void resetTitleCache(){
  g_prevTitle[0] = '\0';
  g_prevTitleSuffix[0] = '\0';
  g_prevTitleColor = 0;
}

static void setTitleIfChanged(const char* text, uint16_t color){
  if(!text) text = "";
  if(strcmp(text, g_prevTitle) == 0 && color == g_prevTitleColor) return;

  strncpy(g_prevTitle, text, sizeof(g_prevTitle)-1);
  g_prevTitle[sizeof(g_prevTitle)-1] = 0;
  g_prevTitleColor = color;

  clearRegion(0,0,320,APPBAR_H,COL_CARD());
  s_tft->drawFastHLine(0,APPBAR_H,320,COL_ACCENT());
  s_tft->setFont(&FreeSans12pt7b);
  s_tft->setTextColor(color, COL_CARD());
  int16_t x1,y1; uint16_t w,h;
  s_tft->getTextBounds((char*)text,0,0,&x1,&y1,&w,&h);
  s_tft->setCursor((320-(int)w)/2,24);
  s_tft->print(text);
  s_tft->setFont();
}

static void setTitleWithSuffixIfChanged(const char* text, const char* suffix, uint16_t color){
  if(!text) text = "";
  if(!suffix) suffix = "";
  if(strcmp(text, g_prevTitle) == 0 && strcmp(suffix, g_prevTitleSuffix) == 0 && color == g_prevTitleColor) return;

  strncpy(g_prevTitle, text, sizeof(g_prevTitle)-1);
  g_prevTitle[sizeof(g_prevTitle)-1] = 0;
  strncpy(g_prevTitleSuffix, suffix, sizeof(g_prevTitleSuffix)-1);
  g_prevTitleSuffix[sizeof(g_prevTitleSuffix)-1] = 0;
  g_prevTitleColor = color;

  clearRegion(0,0,320,APPBAR_H,COL_CARD());
  s_tft->drawFastHLine(0,APPBAR_H,320,COL_ACCENT());

  int16_t x1,y1; uint16_t wMain,hMain;
  s_tft->setFont(&FreeSans12pt7b);
  s_tft->getTextBounds((char*)text,0,0,&x1,&y1,&wMain,&hMain);

  uint16_t wSuffix = 0;
  if(suffix[0]){
    s_tft->setFont(&FreeSans9pt7b);
    s_tft->getTextBounds((char*)suffix,0,0,&x1,&y1,&wSuffix,&hMain);
  }

  uint16_t totalW = wMain + (suffix[0] ? (wSuffix + 6) : 0);
  int16_t startX = (320 - (int)totalW) / 2;

  s_tft->setFont(&FreeSans12pt7b);
  s_tft->setTextColor(color, COL_CARD());
  s_tft->setCursor(startX, 24);
  s_tft->print(text);

  if(suffix[0]){
    s_tft->setFont(&FreeSans9pt7b);
    s_tft->setTextColor(color, COL_CARD());
    s_tft->setCursor(startX + (int)wMain + 6, 24);
    s_tft->print(suffix);
  }

  s_tft->setFont();
}

static void drawBarStatic(bool sel){
  Channel ch = currentBarChannel();
  uint16_t fc = sel ? COL_YELLOW() : COL_FRAME();
  s_tft->fillRoundRect(BAR_X-2,BAR_Y-2,BAR_W+4,BAR_H+4,BAR_R+2,COL_CARD());
  s_tft->drawRoundRect(BAR_X,BAR_Y,BAR_W,BAR_H,BAR_R,fc);
  auto tick=[&](int x,float v,const char* u){ char b[24];
    if(ch==CH_BATTV) snprintf(b,sizeof(b),"%.1f",v);
    else if(ch==CH_BATTV2 || ch==CH_DCDC_OUT_V || ch==CH_DCDC_IN_V) snprintf(b,sizeof(b),"%.2f",v);
    else if(u&&u[0]=='V') snprintf(b,sizeof(b),"%.2f",v);
    else if(ch==CH_LAMBDA){
      if(g_uLambda==U_L_lambda) snprintf(b,sizeof(b),"%.2f",v);
      else snprintf(b,sizeof(b),"%.1f",v);
    }
    else if(u && (u[0]=='k' || u[0]=='p')) snprintf(b,sizeof(b),"%.0f",v);
    else snprintf(b,sizeof(b),"%.0f",v);
    s_tft->setFont(&FreeSans12pt7b); s_tft->setTextColor(COL_TICKS(),COL_BG()); s_tft->setCursor(x,BAR_Y-6); s_tft->print(b); s_tft->setFont(); };
  Range r=rangeFor(ch); float mid=(r.mn+r.mx)/2; clearRegion(0,BAR_Y-20,320,18,COL_BG());
  char tmp[16]; if(ch==CH_LAMBDA && g_uLambda==U_L_lambda) snprintf(tmp,sizeof(tmp),"%.2f", mid); else snprintf(tmp,sizeof(tmp),"%.0f", mid);
  int16_t x1,y1; uint16_t w,h; s_tft->setFont(&FreeSans12pt7b); s_tft->getTextBounds(tmp,0,0,&x1,&y1,&w,&h); s_tft->setFont();
  tick(BAR_X,r.mn,unitLabel(ch)); tick(BAR_X+BAR_W/2-(int)w/2,mid,unitLabel(ch));
  if(ch==CH_LAMBDA && g_uLambda==U_L_lambda) snprintf(tmp,sizeof(tmp),"%.2f", r.mx); else snprintf(tmp,sizeof(tmp),"%.0f", r.mx);
  s_tft->setFont(&FreeSans12pt7b); s_tft->getTextBounds(tmp,0,0,&x1,&y1,&w,&h); s_tft->setFont();
  tick(BAR_X+BAR_W-(int)w-4,r.mx,unitLabel(ch)); prevBarFillW=-1;
}

static void drawGridStatic(){
  for(int i=0;i<4;i++){
    PillSpec p = pillSpec(i);
    s_tft->fillRoundRect(p.x, p.y, p.w, p.h, 10, COL_CARD());
  }

  for(int i=0;i<4;i++){
    Channel ch = currentPillChannel(i);
    drawPillFrame(pillSpec(i), false, ch);
    drawPillLabelForChannel(pillSpec(i), ch);
    prevPillChannel[i]  = ch;
    prevPillValueKey[i] = INT32_MIN;
    prevPillWarnLevel[i]= 0;
  }

  prevBarChannel = currentBarChannel();
  prevBarFillW   = -1;
  drawBarStatic(false);
}

static void fmtValueForTitle(Channel ch, char* out, size_t n){
  if(ch==CH_BATTV) {
    snprintf(out, n, "%.1f %s", battV, unitLabel(ch));
  } else if(ch==CH_LAMBDA){
    float v = valueDisplay(CH_LAMBDA);
    if(g_uLambda==U_L_lambda) snprintf(out,n,"%.2f %s", v, unitLabel(ch));
    else snprintf(out,n,"%.1f %s", v, unitLabel(ch));
  } else {
    float vv = valueDisplay(ch);
    if(isfinite(vv)) {
      if(ch==CH_BATTV) snprintf(out,n,"%.2f %s", vv, unitLabel(ch));
      else snprintf(out, n, "%.0f %s", vv, unitLabel(ch));
    } else snprintf(out, n, "--");
  }
}

static void refreshPillsDynamic(){
  static int prevRenderedTargetGear[4] = {INT32_MIN,INT32_MIN,INT32_MIN,INT32_MIN};

  for(int i=0;i<4;i++){
    PillSpec p = pillSpec(i);
    Channel ch = currentPillChannel(i);

    if(ch!=prevPillChannel[i]){
      clearRegion(p.x+6,p.y+4,p.w-12,24,COL_CARD());
      drawPillLabelForChannel(p, ch);
      prevPillChannel[i]=ch;
      prevPillValueKey[i]=INT32_MIN;
      prevRenderedTargetGear[i]=INT32_MIN;
      drawPillFrame(p,false,ch);
    }

    int key = valueKey(ch);

    if(uiMinMaxActive && minMaxModeFor(ch) != MINMAX_NONE){
      float dispValue = minMaxDisplayValue(ch);
      int mmKey = valueKeyForDisplay(ch, dispValue);
      if(mmKey == prevPillValueKey[i]) continue;

      char nb[16];
      formatDisplayValue(ch, dispValue, nb, sizeof(nb));
      drawValueInPill(p, nb, unitLabel(ch));

      prevPillValueKey[i] = mmKey;
      continue;
    }

    if(ch==CH_GEAR){
      bool targetChanged = (targetgear != prevRenderedTargetGear[i]);
      if(!targetChanged && key==prevPillValueKey[i]){
      } else {
        clearPillValue(p);
        s_tft->setFont(&FreeSans12pt7b);
        s_tft->setTextColor(COL_TXT(),COL_CARD());
        s_tft->setCursor(p.x+10,p.y+p.h-10);
        if(gear>0 && gear<=9 && targetgear>0 && targetgear<=9 && gear!=targetgear){
          char b[8]; snprintf(b,sizeof(b), "%d>%d", gear, targetgear); s_tft->print(b);
        }else{
          s_tft->print(gearText(gear));
        }
        s_tft->setFont();

        prevPillValueKey[i]        = key;
        prevRenderedTargetGear[i]  = targetgear;
      }
      continue;
    }

    if(key==prevPillValueKey[i]) continue;

    if (ch == CH_LOCKUP) {
      clearPillValue(p);
      s_tft->setFont(&FreeSans12pt7b);
      s_tft->setTextColor(tcStateColor(g_tcState), COL_CARD());
      s_tft->setCursor(p.x + 10, p.y + p.h - 10);
      s_tft->print(tcStateText(g_tcState));
      s_tft->setFont();
    }

    else if(ch==CH_HEADLIGHTS){
      clearPillValue(p);
      s_tft->setFont(&FreeSans12pt7b);
      s_tft->setTextColor(COL_TXT(),COL_CARD());
      s_tft->setCursor(p.x+10,p.y+p.h-10);
      s_tft->print(headlightsOn?"On":"Off");
      s_tft->setFont();
    }
    else{
      float v = valueDisplay(ch);
      char nb[16];
      formatDisplayValue(ch, v, nb, sizeof(nb));
      drawValueInPill(p,nb,unitLabel(ch));
    }

    prevPillValueKey[i] = key;
  }

  Channel barCh = currentBarChannel();
  if (barCh != prevBarChannel) { drawBarStatic(false); prevBarChannel = barCh; }

  static uint16_t s_prevBarColor = 0;
  {
    uint8_t barLvl = warnLevelFor(barCh);
    uint16_t barCol =
      (barLvl == 2) ? COL_RED() :
      (barLvl == 1) ? COL_ORANGE() :
                     barFillColor();

    Range r = rangeFor(barCh); if (r.mx <= r.mn) r.mx = r.mn + 1;
    float v = uiMinMaxActive ? minMaxDisplayValue(barCh) : valueDisplay(barCh);
    float t = clampf((v - r.mn) / (r.mx - r.mn), 0, 1);
    int innerW = BAR_W - 4, innerH = BAR_H - 4, x0 = BAR_X + 2, y0 = BAR_Y + 2;
    int fillW = (int)roundf(t * innerW);
    if (fillW < 0) fillW = 0; else if (fillW > innerW) fillW = innerW;

    if (prevBarFillW < 0) {
      s_tft->fillRect(x0, y0, innerW, innerH, COL_CARD());
      if (fillW > 0) s_tft->fillRect(x0, y0, fillW, innerH, barCol);
      prevBarFillW = fillW;
      s_prevBarColor = barCol;
    } else {
      if (fillW > prevBarFillW) {
        int dx = fillW - prevBarFillW;
        if (dx > 0) s_tft->fillRect(x0 + prevBarFillW, y0, dx, innerH, barCol);
      }
      else if (fillW < prevBarFillW) {
        int dx = prevBarFillW - fillW;
        if (dx > 0) s_tft->fillRect(x0 + fillW, y0, dx, innerH, COL_CARD());
      }

      if (barCol != s_prevBarColor && fillW > 0) {
        s_tft->fillRect(x0, y0, fillW, innerH, barCol);
      }

      prevBarFillW = fillW;
      s_prevBarColor = barCol;
    }
  }

  if(uiMinMaxActive){
    setTitleWithSuffixIfChanged(labelText(barCh), minMaxSuffixFor(barCh), COL_TXT());
  } else {
    uiHighestWarnLevel = 0;
    uiHighestWarnCh    = CH__COUNT;

    uint8_t critList[CH__COUNT];  uint8_t critCnt = 0;
    uint8_t warnList[CH__COUNT];  uint8_t warnCnt = 0;

    uint64_t critMask = 0, warnMask = 0;

    for (int ch = 0; ch < CH__COUNT; ++ch) {
      uint8_t lvl = warnLevelFor((Channel)ch);
      if (!lvl) continue;

      if (lvl == 2) { critList[critCnt++] = (uint8_t)ch; critMask |= (1ULL << ch); }
      else          { warnList[warnCnt++] = (uint8_t)ch; warnMask |= (1ULL << ch); }

      if (lvl > uiHighestWarnLevel) uiHighestWarnLevel = lvl;
    }

    const uint8_t total = critCnt + warnCnt;

    unsigned long now = millis();
    uint64_t sig = (critMask << 32) ^ warnMask;
    static uint8_t rrIndex = 0;
    static unsigned long lastCycleMs = 0;
    static uint64_t prevSig = 0;
    if (sig != prevSig) { prevSig = sig; rrIndex = 0; lastCycleMs = now; }

    if (total > 0) {
      if (now - lastCycleMs >= 1500) { lastCycleMs = now; ++rrIndex; }
      uint8_t i = rrIndex % total;

      uint8_t showChIdx, showLvl;
      if (i < critCnt) { showChIdx = critList[i];           showLvl = 2; }
      else             { showChIdx = warnList[i - critCnt]; showLvl = 1; }

      uiHighestWarnCh = (Channel)showChIdx;

      char val[24]; fmtValueForTitle((Channel)uiHighestWarnCh, val, sizeof(val));
      char ttl[64];
      snprintf(ttl, sizeof(ttl), "WARNING: %s %s",
               labelText((Channel)uiHighestWarnCh), val);

      setTitleIfChanged(ttl, (showLvl == 2) ? COL_RED() : COL_ORANGE());
    }
    else {
      if (regenState == REGEN_PAUSED) {
        setTitleIfChanged("REGEN INCOMPLETE", COL_TXT());
      } else if (regenState == REGEN_ACTIVE) {
        setTitleIfChanged("REGEN ACTIVE", COL_TXT());
      } else {
        setTitleIfChanged(labelText(barCh), COL_TXT());
      }
    }
  }

  static bool lastBlinkPill = true;
  for (int i = 0; i < 4; i++) {
    Channel ch = currentPillChannel(i);
    uint8_t lvl = warnLevelFor(ch);

    if (lvl != prevPillWarnLevel[i] || lastBlinkPill != uiWarnBlinkOn) {
      if (lvl > 0 && uiWarnBlinkOn) {
        overlayPillWarnOutlineThick(
          pillSpec(i),
          (lvl == 2) ? COL_RED() : COL_ORANGE()
        );
      }
      else {
        drawPillFrame(pillSpec(i), false, ch);
      }
      prevPillWarnLevel[i] = lvl;
    }
  }
  lastBlinkPill = uiWarnBlinkOn;
}

void initUi(Adafruit_ILI9341& tft, const Palette* palette){
  s_tft = &tft;
  s_palette = palette;
  (void)s_palette;
  resetTitleCache();
  prevBarChannel = CH__COUNT;
  prevBarFillW = -1;
  for(int i=0;i<4;i++){
    prevPillChannel[i] = CH__COUNT;
    prevPillValueKey[i] = INT32_MIN;
    prevPillWarnLevel[i] = 0;
  }
}

void renderStatic(){
  if(!s_tft) return;
  resetTitleCache();
  drawGridStatic();
}

void renderDynamic(){
  if(!s_tft) return;
  refreshPillsDynamic();
}