#include "CanDecode.h"
#include "Config.h"

namespace {
uint16_t CD_be16(const uint8_t* d) { return static_cast<uint16_t>(d[0]) << 8 | d[1]; }

float CD_clampf(float v, float lo, float hi) {
  if (!isfinite(v)) {
    return lo;
  }
  return v < lo ? lo : (v > hi ? hi : v);
}

void h_141_speed(const can_frame& f) {
  if (f.can_dlc < 3) {
    return;
  }
  uint16_t raw = (static_cast<uint16_t>(f.data[1]) << 8) | static_cast<uint16_t>(f.data[2]);
  speed_kmh = static_cast<float>(raw) / 64.0f;
}

void h_160_rpm_pedal(const can_frame& f) {
  if (f.can_dlc >= 2) {
    rpm = static_cast<float>(CD_be16(&f.data[0])) / 8.0f;
  }
  if (f.can_dlc >= 3) {
    pedalPct = static_cast<float>(f.data[2]) / 2.5f;
  }
  if (f.can_dlc >= 5) {
    tqDemandPct = static_cast<float>(f.data[4]) / 2.55f;
  }
}

void h_050_transT(const can_frame& f) {
  if (f.can_dlc >= 3) {
    trans1C = static_cast<float>(f.data[2]) - 40.0f;
  }
  if (f.can_dlc >= 4) {
    trans2C = static_cast<float>(f.data[3]) - 40.0f;
  }
}

// Exact meanings: 0x00 = Unlocked, 0x20 = Transition, 0x40 = Flex, 0x60 = Full
void h_161_gear_lock(const can_frame& f) {
  if (f.can_dlc >= 2) {
    const uint8_t lb = f.data[1];
    const uint8_t mask = lb & 0x60;  // 00 / 20 / 40 / 60
    g_lockByteRaw3 = lb;

    static uint8_t prev_steady = 0x00;  // remember last steady only (00/40/60)

    if (mask == 0x20) {
      // Show Applying if we came from 00, else Releasing
      g_tcState = (prev_steady == 0x00) ? TC_Applying : TC_Releasing;

      // Choose your preferred behavior during 0x20:
      lockup = false;
    } else {
      if (mask == 0x60) {
        g_tcState = TC_Full;
        lockup = true;
        prev_steady = 0x60;
      } else if (mask == 0x40) {
        g_tcState = TC_Flex;
        lockup = true;
        prev_steady = 0x40;
      } else {
        g_tcState = TC_Unlocked;
        lockup = false;
        prev_steady = 0x00;
      }
    }
  }

  // Byte 2: current gear
  if (f.can_dlc >= 3) {
    uint8_t g = f.data[2];
    switch (g) {
      case 251:
        gear = -3;
        break;
      case 123:
        gear = -2;
        break;
      case 125:
        gear = -1;
        break;
      case 126:
        gear = 1;
        break;
      case 127:
        gear = 2;
        break;
      case 128:
        gear = 3;
        break;
      case 129:
        gear = 4;
        break;
      case 130:
        gear = 5;
        break;
      case 131:
        gear = 6;
        break;
      default:
        gear = 0;
    }
  }

  // Byte 0: target gear (NEW)
  if (f.can_dlc >= 1) {
    uint8_t tg = f.data[0];
    switch (tg) {
      case 251:
        targetgear = -3;
        break;
      case 123:
        targetgear = -2;
        break;
      case 125:
        targetgear = -1;
        break;
      case 126:
        targetgear = 1;
        break;
      case 127:
        targetgear = 2;
        break;
      case 128:
        targetgear = 3;
        break;
      case 129:
        targetgear = 4;
        break;
      case 130:
        targetgear = 5;
        break;
      case 131:
        targetgear = 6;
        break;
      default:
        targetgear = 0;
    }
  }
}

void h_4A3_batt(const can_frame& f) {
  if (f.can_dlc >= 1) {
    battV = static_cast<float>(f.data[0]) * 0.1f;
  }
}

void h_2C0_coolant_iat_fuel(const can_frame& f) {
  if (f.can_dlc >= 1) {
    coolantC = static_cast<float>(f.data[0]) - 40.0f;
  }
  if (f.can_dlc >= 2) {
    iatC = static_cast<float>(f.data[1]) - 40.0f;
  }
  if (f.can_dlc >= 3) {
    fuelC = static_cast<float>(f.data[2]) - 40.0f;
  }
}

void h_150_torque(const can_frame& f) {
  if (f.can_dlc < 2) {
    return;
  }
  uint16_t raw = CD_be16(&f.data[0]);
  float nm = 0.5f * (static_cast<float>(raw) - 1696.0f);
  if (!isfinite(nm)) {
    nm = 0;
  }
  if (nm < -200.0f) {
    nm = -200.0f;
  }
  torqueNm = nm;
}

void h_4AC_soot(const can_frame& f) {
  if (f.can_dlc < 6) {
    return;
  }
  uint16_t raw = CD_be16(&f.data[4]);
  soot_pct = CD_clampf(static_cast<float>(raw) / CFG::SOOT_DIV, 0.0f, 100.0f);
}

void h_4AB_regen_egt2(const can_frame& f) {
  if (f.can_dlc >= 1) {
    egt2C = static_cast<float>(f.data[0]) * 10.0f;  // raw*10 = °C
  }
  if (f.can_dlc >= 7) {
    uint16_t raw = (static_cast<uint16_t>(f.data[5]) << 8) | static_cast<uint16_t>(f.data[6]);
    regen_pct = (raw == 0) ? 0.0f : CD_clampf(static_cast<float>(raw) * 100.0f / CFG::REGEN_RAW_MAX, 0, 100);
  }
}

void h_4B0_egt1(const can_frame& f) {
  if (f.can_dlc >= 6) {
    // extract 10-bit value from byte 4 (lowest 2 bits) and byte 5 (all bits)
    uint16_t raw = ((f.data[4] & 0x03) << 8) | f.data[5];
    egt1C = static_cast<float>(raw);
  }
}

void h_4A4_boost_abs(const can_frame& f) {
  if (f.can_dlc >= 4) {
    float abs_kPa = static_cast<float>(f.data[3]) * 2.0f;
    boost_kPa = CD_clampf(abs_kPa - CFG::ATM_KPA, 0.0f, 250.0f);
  }
}

void h_4CC_manifold_turboOut(const can_frame& f) {
  if (f.can_dlc >= 2) {
    turboOutC = static_cast<float>(f.data[1]) - 40.0f;
  }
  if (f.can_dlc >= 8) {
    manifoldC = static_cast<float>(f.data[7]) - 40.0f;
  }
}

void h_4B2_lambda(const can_frame& f) {
  if (f.can_dlc >= 3) {
    uint16_t raw = CD_be16(&f.data[1]);
    lambdaVal = static_cast<float>(raw) / 1000.0f;
  }
}

void h_4CD_actuator(const can_frame& f) {
  if (f.can_dlc >= 4) {
    turboActRaw = f.data[3];
  }
}

void h_401_headlights(const can_frame& f) {
  if (f.can_dlc >= 2) {
    headlightsOn = (f.data[1] == 0x50);
  }
}
}  // namespace

namespace CanDec {
void decodeFrame(const can_frame& f) {
  switch (f.can_id) {
    case CFG::ID_SPEED:
      h_141_speed(f);
      break;
    case CFG::ID_RPM_SPEED:
      h_160_rpm_pedal(f);
      break;
    case CFG::ID_TRANS_T:
      h_050_transT(f);
      break;
    case CFG::ID_GEAR_LOCK:
      h_161_gear_lock(f);
      break;
    case CFG::ID_BATTV:
      h_4A3_batt(f);
      break;
    case CFG::ID_COOLANT_ETC:
      h_2C0_coolant_iat_fuel(f);
      break;
    case CFG::ID_TORQUE:
      h_150_torque(f);
      break;

    case CFG::ID_SOOT:
      h_4AC_soot(f);
      break;
    case CFG::ID_REGEN:
      h_4AB_regen_egt2(f);
      break;
    case CFG::ID_EGT1:
      h_4B0_egt1(f);
      break;
    case CFG::ID_BOOST:
      h_4A4_boost_abs(f);
      break;
    case CFG::ID_MAP_T:
      h_4CC_manifold_turboOut(f);
      break;
    case CFG::ID_LAMBDA:
      h_4B2_lambda(f);
      break;
    case CFG::ID_ACTUATOR:
      h_4CD_actuator(f);
      break;
    case CFG::ID_HEADLIGHTS:
      h_401_headlights(f);
      break;

    // NOTE: CFG::ID_SWBTN is handled in the main sketch (buttons)
    default:
      break;
  }
}
}  // namespace CanDec
