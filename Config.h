#pragma once

#include <Arduino.h>
#include <mcp2515.h>

//=====================Xiao Can Expansion Board 2 =================
// ===================== Config / CAN IDs =====================
namespace CFG {
  constexpr uint8_t TFT_CS= D1, TFT_DC= D3, TFT_RST= D2;
  constexpr int SPI_SCK= D8, SPI_MISO= D9, SPI_MOSI= D10;
  constexpr uint8_t CAN_CS= D7, CAN_INT= D6;
  constexpr uint8_t BACKLIGHT_PWM=D0;
  constexpr CAN_SPEED CAN_SPEED_SEL = CAN_500KBPS;
  constexpr CAN_CLOCK CAN_CLOCK_SEL = MCP_16MHZ;
  
  // Core frames
  constexpr uint32_t ID_SPEED=0x141, ID_RPM_SPEED=0x160, ID_TRANS_T=0x050, ID_GEAR_LOCK=0x161;
  constexpr uint32_t ID_BATTV=0x4A3, ID_COOLANT_ETC=0x2C0, ID_TORQUE=0x150, ID_SWBTN=0x402;

  // New / revised frames
  constexpr uint32_t ID_SOOT=0x4AC;        // soot bytes 4-5 (raw/14.1=%)
  constexpr uint32_t ID_REGEN=0x4AB;       // regen + EGT2 byte 0 (raw*10=°C)
  constexpr uint32_t ID_EGT1=0x4B0;        // bytes 4-5 raw=°C (per your decode)
  constexpr uint32_t ID_BOOST=0x4A4;       // byte 3 (raw*2=kPa abs)
  constexpr uint32_t ID_MAP_T=0x4CC;       // [1]=turbo out (raw-40), [7]=manifold (raw-40)
  constexpr uint32_t ID_LAMBDA=0x4B2;      // bytes 1-2 (raw/1000)
  constexpr uint32_t ID_ACTUATOR=0x4CD;    // byte 3 (raw)
  constexpr uint32_t ID_HEADLIGHTS=0x401;  // byte 1 (0x50 on)

  constexpr uint32_t SCREEN_REFRESH_MS=16.66;

  // Soot scaling (no learning)
  constexpr float SOOT_DIV = 14.1f;

  // Regen banner hysteresis from regen%
  constexpr uint16_t REGEN_RAW_MAX=65535;
  constexpr float REGEN_ON_PCT=1.0f, REGEN_OFF_PCT=0.2f;
  constexpr float RAW_TO_PCT           = 100.0f / 65535.0f;   // ≈ 0.001526%
  constexpr float REGEN_PAUSE_EPS_PCT  = RAW_TO_PCT * 1.2f;   // ≈ 0.00183%
  // Atmosphere for boost (kPa)
  constexpr float ATM_KPA = 101.0f;

  enum WarnMode: uint8_t { WARN_OFF=0, WARN_HIGH=1, WARN_LOW=2 };

  // ===== Instrument cluster beep (placeholder — set to your vehicle) =====
  constexpr uint32_t ID_CLUSTER_BEEP = 0x5A0; // TODO: set real ID
  constexpr uint8_t  CLUSTER_BEEP_DLC = 1;
  constexpr uint8_t  CLUSTER_BEEP_PAYLOAD[8] = { 0x01,0,0,0,0,0,0,0 };
  constexpr uint32_t BEEP_COOLDOWN_MS = 1500; // rate limit new L2 beeps
  constexpr bool BEEP_ENABLED = false;

  // ===== Victron BLE (Instant Readout) =====
  constexpr char WIFI_DEFAULT_SSID[] = "Xiao-Dash";
  constexpr char WIFI_DEFAULT_PASS[] = "dashconfig";
  constexpr uint8_t WIFI_MIN_PASS_LEN = 8;
}