#include "VictronBle.h"

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <math.h>
#include <string>
#include <string.h>
#include "mbedtls/aes.h"

namespace VictronBle {
const char kBmvMac[] = "e2:0e:ab:c7:49:5b";
const uint8_t kBmvKey[16] = {
  0xA5,0x85,0x12,0x40,0x7B,0x4C,0x5C,0x4A,
  0xD1,0xFA,0xFB,0x30,0x6E,0x05,0x69,0x12
};
const char kMpptMac[] = "c6:c8:c5:a4:88:8a";
const uint8_t kMpptKey[16] = {
  0x3D,0x67,0xAB,0x2A,0xBB,0x21,0xC7,0xFD,
  0x4C,0x31,0xEC,0x8F,0xAF,0x30,0xE0,0x2D
};
const char kOrionMac[] = "cf:96:cf:8c:28:5b";
const uint8_t kOrionKey[16] = {
  0x03,0xB3,0x85,0xE8,0x3E,0x1C,0x7A,0xF5,
  0x7B,0x98,0x7B,0xED,0x1B,0x43,0xBD,0x59
};
}

extern bool wifiPageActive();

extern bool victronConfigEnabled();
extern const char* victronConfigBmvMac();
extern const uint8_t* victronConfigBmvKey();
extern const char* victronConfigMpptMac();
extern const uint8_t* victronConfigMpptKey();
extern const char* victronConfigOrionMac();
extern const uint8_t* victronConfigOrionKey();

namespace {
constexpr uint16_t kVictronCompanyId = 0x02E1;
constexpr uint8_t kVictronRecordInstant = 0x10;
constexpr uint32_t kVictronStaleMs = 60000;
constexpr uint16_t kBleScanInterval = 160;
constexpr uint16_t kBleScanWindow = 160;

struct VictronDevCfg {
  const char* name;
  const char* mac;
  const uint8_t* key;
};

NimBLEScan* victronScan = nullptr;
VictronReadings g_readings = {
  NAN, NAN, NAN, NAN,
  NAN, NAN, NAN,
  NAN, NAN, NAN,
  0, 0, 0
};

static inline void resetReadings() {
  g_readings.battV2 = NAN;
  g_readings.battSocPct = NAN;
  g_readings.battCurrentA = NAN;
  g_readings.battTimeMin = NAN;
  g_readings.dcdcOutA = NAN;
  g_readings.dcdcOutV = NAN;
  g_readings.dcdcInV = NAN;
  g_readings.pvWatts = NAN;
  g_readings.pvAmps = NAN;
  g_readings.pvYieldKwh = NAN;
  g_readings.lastBmvUpdateMs = 0;
  g_readings.lastMpptUpdateMs = 0;
  g_readings.lastDcdcUpdateMs = 0;
}

static inline void clearStaleVictronData(){
  unsigned long now = millis();
  if(!victronConfigEnabled()){
    resetReadings();
    return;
  }
  if(g_readings.lastBmvUpdateMs > 0 && now - g_readings.lastBmvUpdateMs > kVictronStaleMs){
    g_readings.battV2 = NAN;
    g_readings.battSocPct = NAN;
    g_readings.battCurrentA = NAN;
    g_readings.battTimeMin = NAN;
    g_readings.lastBmvUpdateMs = 0;
  }
  if(g_readings.lastMpptUpdateMs > 0 && now - g_readings.lastMpptUpdateMs > kVictronStaleMs){
    g_readings.pvWatts = NAN;
    g_readings.pvAmps = NAN;
    g_readings.pvYieldKwh = NAN;
    g_readings.lastMpptUpdateMs = 0;
  }
  if(g_readings.lastDcdcUpdateMs > 0 && now - g_readings.lastDcdcUpdateMs > kVictronStaleMs){
    g_readings.dcdcOutA = NAN;
    g_readings.dcdcOutV = NAN;
    g_readings.dcdcInV = NAN;
    g_readings.lastDcdcUpdateMs = 0;
  }
}

static inline uint32_t getBitsLE(const uint8_t* buf, uint32_t startBit, uint32_t bitLen){
  uint32_t v = 0;
  for(uint32_t i = 0; i < bitLen; i++){
    uint32_t bitIndex = startBit + i;
    uint8_t b = (buf[bitIndex / 8] >> (bitIndex % 8)) & 0x01;
    v |= (uint32_t)b << i;
  }
  return v;
}

static inline int32_t signExtend(uint32_t v, uint32_t bits){
  uint32_t m = 1UL << (bits - 1);
  return (int32_t)((v ^ m) - m);
}

static void aesCtrDecrypt(const uint8_t key[16], const uint8_t iv[16], const uint8_t* in,
                          uint8_t* out, size_t len){
  mbedtls_aes_context ctx;
  mbedtls_aes_init(&ctx);
  mbedtls_aes_setkey_enc(&ctx, key, 128);
  uint8_t nonce_counter[16];
  uint8_t stream_block[16];
  memcpy(nonce_counter, iv, sizeof(nonce_counter));
  memset(stream_block, 0, sizeof(stream_block));
  size_t nc_off = 0;
  mbedtls_aes_crypt_ctr(&ctx, len, &nc_off, nonce_counter, stream_block, in, out);
  mbedtls_aes_free(&ctx);
}

static bool matchDevice(const std::string& macStr, VictronDevCfg& out){
  if(macStr == victronConfigBmvMac()){
    out = {"BMV-712", victronConfigBmvMac(), victronConfigBmvKey()};
    return true;
  }
  if(macStr == victronConfigMpptMac()){
    out = {"MPPT100/30", victronConfigMpptMac(), victronConfigMpptKey()};
    return true;
  }
  if(macStr == victronConfigOrionMac()){
    out = {"OrionXS", victronConfigOrionMac(), victronConfigOrionKey()};
    return true;
  }
  return false;
}

static inline void decodeBMV(const uint8_t* plain, size_t plainLen){
  if(plainLen < 14) return;
  uint16_t ttg_min = (uint16_t)getBitsLE(plain, 0, 16);
  int16_t v_cV = (int16_t)getBitsLE(plain, 16, 16);
  float volts = (v_cV == (int16_t)0x7FFF) ? NAN : (v_cV / 100.0f);
  int32_t i_mA = signExtend(getBitsLE(plain, 66, 22), 22);
  float amps = (i_mA == (int32_t)0x3FFFFF) ? NAN : (i_mA / 1000.0f);
  uint16_t soc_raw = (uint16_t)getBitsLE(plain, 108, 10);
  float soc = (soc_raw == 0x3FF) ? NAN : (soc_raw / 10.0f);
  bool updated = false;
  if(isfinite(volts)) { g_readings.battV2 = volts; updated = true; }
  if(isfinite(amps)) { g_readings.battCurrentA = amps; updated = true; }
  if(isfinite(soc)) { g_readings.battSocPct = soc; updated = true; }
  if(ttg_min != 0xFFFF) { g_readings.battTimeMin = ttg_min; updated = true; }
  if(updated) g_readings.lastBmvUpdateMs = millis();
}

static inline void decodeMPPT(const uint8_t* plain, size_t plainLen){
  if(plainLen < 12) return;
  int16_t i_dA = (int16_t)getBitsLE(plain, 32, 16);
  float battI = (i_dA == (int16_t)0x7FFF) ? NAN : (i_dA / 10.0f);
  uint16_t yield_cWh = (uint16_t)getBitsLE(plain, 48, 16);
  float yield_kWh = (yield_cWh == 0xFFFF) ? NAN : (yield_cWh / 100.0f);
  uint16_t pvW = (uint16_t)getBitsLE(plain, 64, 16);
  float pvPowerW = (pvW == 0xFFFF) ? NAN : (float)pvW;
  bool updated = false;
  if(isfinite(pvPowerW)) { g_readings.pvWatts = pvPowerW; updated = true; }
  if(isfinite(yield_kWh)) { g_readings.pvYieldKwh = yield_kWh; updated = true; }
  if(isfinite(battI)) { g_readings.pvAmps = battI; updated = true; }
  if(updated) g_readings.lastMpptUpdateMs = millis();
}

static inline void decodeDcdc04(const uint8_t* plain, size_t plainLen){
  if(plainLen < 12) return;
  uint16_t in_cV_u = (uint16_t)getBitsLE(plain, 16, 16);
  float inV = (in_cV_u == 0xFFFF) ? NAN : (in_cV_u / 100.0f);
  int16_t out_cV = (int16_t)getBitsLE(plain, 32, 16);
  float outV = (out_cV == (int16_t)0x7FFF) ? NAN : (out_cV / 100.0f);
  bool updated = false;
  if(isfinite(inV)) { g_readings.dcdcInV = inV; updated = true; }
  if(isfinite(outV)) { g_readings.dcdcOutV = outV; updated = true; }
  if(updated) g_readings.lastDcdcUpdateMs = millis();
}

static inline void decodeOrion0F(const uint8_t* plain, size_t plainLen){
  if(plainLen < 14) return;
  int16_t out_cV = (int16_t)getBitsLE(plain, 16, 16);
  float outV = (out_cV == (int16_t)0x7FFF) ? NAN : (out_cV / 100.0f);
  int16_t out_dA = (int16_t)getBitsLE(plain, 32, 16);
  float outA = (out_dA == (int16_t)0x7FFF) ? NAN : (out_dA / 10.0f);
  uint16_t in_cV_u = (uint16_t)getBitsLE(plain, 48, 16);
  float inV = (in_cV_u == 0xFFFF) ? NAN : (in_cV_u / 100.0f);
  bool updated = false;
  if(isfinite(inV)) { g_readings.dcdcInV = inV; updated = true; }
  if(isfinite(outV)) { g_readings.dcdcOutV = outV; updated = true; }
  if(isfinite(outA)) { g_readings.dcdcOutA = outA; updated = true; }
  if(updated) g_readings.lastDcdcUpdateMs = millis();
}

class VictronScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* dev) override {
    if(!dev->haveManufacturerData()) return;
    VictronDevCfg cfg{};
    std::string addr = dev->getAddress().toString();
    if(!matchDevice(addr, cfg)) return;
    const std::string& mfg = dev->getManufacturerData();
    if(mfg.size() < 12) return;
    const uint8_t* data = reinterpret_cast<const uint8_t*>(mfg.data());
    if(!(data[0] == (kVictronCompanyId & 0xFF) && data[1] == (kVictronCompanyId >> 8))) return;
    const size_t o = 2;
    if(data[o + 0] != kVictronRecordInstant) return;
    const uint8_t recordType = data[o + 4];
    const uint8_t nonce0 = data[o + 5];
    const uint8_t nonce1 = data[o + 6];
    const uint8_t key0_in_msg = data[o + 7];
    if(key0_in_msg != cfg.key[0]){
      return;
    }
    const size_t enc_off = o + 8;
    if(mfg.size() <= enc_off) return;
    size_t enc_len = mfg.size() - enc_off;
    if(enc_len == 0) return;
    uint8_t enc[16] = {0};
    uint8_t plain[16] = {0};
    size_t take = enc_len > sizeof(enc) ? sizeof(enc) : enc_len;
    memcpy(enc, data + enc_off, take);
    uint8_t iv[16] = {0};
    iv[0] = nonce0;
    iv[1] = nonce1;
    aesCtrDecrypt(cfg.key, iv, enc, plain, take);
    switch(recordType){
      case 0x02: decodeBMV(plain, take); break;
      case 0x01: decodeMPPT(plain, take); break;
      case 0x04: decodeDcdc04(plain, take); break;
      case 0x0F: decodeOrion0F(plain, take); break;
      default:
        break;
    }
  }
};

VictronScanCallbacks g_victronCallbacks;

static void updateVictronScanState(){
  if(!victronScan) return;
  if(wifiPageActive()){
    if(victronScan->isScanning()){
      victronScan->stop();
    }
    return;
  }
  if(victronConfigEnabled()){
    if(!victronScan->isScanning()){
      victronScan->start(0, true, true);
    }
  } else {
    if(victronScan->isScanning()){
      victronScan->stop();
    }
  }
}
}

void victronInit(){
  NimBLEDevice::init("XIAO-BMV");
  victronScan = NimBLEDevice::getScan();
  victronScan->setScanCallbacks(&g_victronCallbacks, true);
  victronScan->setDuplicateFilter(false);
  victronScan->setActiveScan(false);
  victronScan->setInterval(kBleScanInterval);
  victronScan->setWindow(kBleScanWindow);
  updateVictronScanState();
}

VictronReadings victronLoop(){
  clearStaleVictronData();
  updateVictronScanState();
  return g_readings;
}

const VictronReadings& victronReadings(){
  return g_readings;
}
