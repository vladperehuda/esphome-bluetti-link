#pragma once
// Plain Modbus-RTU over BLE for the Bluetti AC180P.
// Values and command setters are consumed by the template entities in bluetti-ac180p.yaml
// (ESPHome native API; no MQTT).

#include <esp_coexist.h>

#include <cmath>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace bluetti {

static const char *const TAG = "bluetti";

static const uint32_t POLL_MS = 20000;     // one full read cycle per period
static const uint32_t TIMEOUT_MS = 3000;  // per request
static const uint32_t SETTLE_MS = 1500;   // let the notify subscription finish after connect
static const uint8_t MAX_FAILS = 3;       // consecutive failed core requests before dropping the link
static const size_t WRITE_Q = 4;

static const uint16_t REG_AC_OUT = 2011;
static const uint16_t REG_DC_OUT = 2012;
static const uint16_t REG_CHARGE_MODE = 2020;
static const uint16_t REG_POWER_LIFTING = 2021;

struct Step {
  uint16_t reg;
  uint16_t qty;
  bool core;  // failures of core requests count towards dropping the link
};

static const Step STEPS[] = {
    {102, 1, true},   // SoC %
    {140, 7, true},   // dc_out, ac_out, dc_in, ac_in at +0, +2, +4, +6 (W)
    {1314, 1, true},  // AC input voltage, /10
    {REG_AC_OUT, 1, true},
    {REG_DC_OUT, 1, true},
    {104, 1, false},  // time remaining, minutes
    {REG_CHARGE_MODE, 1, false},
    {REG_POWER_LIFTING, 1, false},
};
static const uint8_t N_STEPS = sizeof(STEPS) / sizeof(STEPS[0]);

struct Mode {
  int id;
  const char *label;
};
static const Mode MODES[] = {{1, "Silent"}, {0, "Standard"}, {2, "Turbo"}};

inline uint16_t crc16(const uint8_t *d, size_t n) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    crc ^= d[i];
    for (int b = 0; b < 8; b++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
  }
  return crc;
}

class Client {
 public:
  std::vector<uint8_t> tx;  // frame handed to ble_client.ble_write

  // ---- link events -------------------------------------------------------------------------
  void on_connect(uint32_t now) {
    reset_();
    connected_ = true;
    connected_ms_ = now;
    step_ = 0;
    cycle_ms_ = now;
  }
  void on_disconnect() {
    reset_();
    connected_ = false;
    has_data_ = false;
  }
  bool take_disconnect() {
    if (!want_disconnect_ || disconnect_sent_) return false;
    disconnect_sent_ = true;
    return true;
  }

  // ---- request scheduling ------------------------------------------------------------------
  // Returns true when `tx` holds a frame to write. One request in flight at a time.
  bool prepare_next(uint32_t now) {
    if (!connected_ || want_disconnect_ || now - connected_ms_ < SETTLE_MS) return false;
    if (busy_) {
      if (now - sent_ms_ < TIMEOUT_MS) return false;
      ESP_LOGW(TAG, "timeout waiting for fn %02x reg %u", pend_fn_, pend_reg_);
      busy_ = false;
      fail_(pend_core_);
      if (want_disconnect_) return false;
    }
    if (!writes_.empty()) {
      auto w = writes_.front();
      writes_.erase(writes_.begin());
      return send_(0x06, w.first, w.second, now, false);
    }
    if (step_ < N_STEPS) {
      const Step &s = STEPS[step_++];
      return send_(0x03, s.reg, s.qty, now, s.core);
    }
    if (now - cycle_ms_ >= POLL_MS) {
      step_ = 0;
      cycle_ms_ = now;
    }
    return false;
  }

  bool queue_write(uint16_t reg, uint16_t value) {
    if (!connected_ || writes_.size() >= WRITE_Q) return false;
    writes_.push_back({reg, value});
    return true;
  }

  // ---- response handling -------------------------------------------------------------------
  // Called with every notification fragment from ff01. Always returns NAN (the ESPHome sensor
  // that hosts this lambda is a dummy; the data is kept here).
  float on_notify(const std::vector<uint8_t> &x) {
    if (!busy_) return NAN;
    rx_.insert(rx_.end(), x.begin(), x.end());
    if (rx_.size() < 2) return NAN;
    const bool exc = rx_[1] & 0x80;
    const size_t need = exc ? 5 : (pend_fn_ == 0x03 ? 2u * pend_qty_ + 5u : 8u);
    if (rx_.size() < need) return NAN;  // wait for the remaining fragments
    busy_ = false;

    const uint16_t crc = (uint16_t) (rx_[need - 2] | (rx_[need - 1] << 8));
    if (rx_[0] != 0x01 || crc != crc16(rx_.data(), need - 2)) {
      ESP_LOGW(TAG, "bad frame for reg %u (len %u)", pend_reg_, (unsigned) rx_.size());
      fail_(pend_core_);
      return NAN;
    }
    if (exc) {
      ESP_LOGW(TAG, "modbus exception %02x for reg %u", rx_[2], pend_reg_);
      fail_(pend_core_);
      return NAN;
    }
    if (pend_fn_ == 0x03) {
      if (rx_[2] != 2 * pend_qty_) {
        fail_(pend_core_);
        return NAN;
      }
      store_(&rx_[3]);
    }
    if (pend_core_) fails_ = 0;
    return NAN;
  }

  // ---- values for the template entities ----------------------------------------------------
  // Numeric getters return NAN while the link is down or no full cycle has completed yet, which
  // Home Assistant shows as "unknown" instead of a stale or zero reading.
  bool live() const { return connected_ && has_data_; }
  float soc() const { return live() ? (float) soc_ : NAN; }
  float ac_in() const { return live() ? (float) ac_in_ : NAN; }
  float dc_in() const { return live() ? (float) dc_in_ : NAN; }
  float ac_out() const { return live() ? (float) ac_out_ : NAN; }
  float dc_out() const { return live() ? (float) dc_out_ : NAN; }
  float in_w() const { return live() ? (float) (ac_in_ + dc_in_) : NAN; }
  float out_w() const { return live() ? (float) (ac_out_ + dc_out_) : NAN; }
  float net_w() const { return live() ? (float) (ac_in_ + dc_in_ - ac_out_ - dc_out_) : NAN; }
  float ac_in_v() const { return live() ? ac_in_v_ : NAN; }
  float time_h() const { return live() ? time_h_ : NAN; }
  bool ac_on() const { return ac_on_; }
  bool dc_on() const { return dc_on_; }
  bool lift_on() const { return lift_; }
  // Empty when the mode register has not been read yet (or holds an unknown value).
  std::string mode_label() const {
    for (const Mode &m : MODES)
      if (m.id == mode_) return m.label;
    return "";
  }

  // ---- commands from the template switches / select ----------------------------------------
  void set_ac(bool on) { log_cmd_("ac", on ? "ON" : "OFF", queue_write(REG_AC_OUT, on)); }
  void set_dc(bool on) { log_cmd_("dc", on ? "ON" : "OFF", queue_write(REG_DC_OUT, on)); }
  void set_lift(bool on) { log_cmd_("lift", on ? "ON" : "OFF", queue_write(REG_POWER_LIFTING, on)); }
  void set_mode(const std::string &label) {
    bool ok = false;
    for (const Mode &m : MODES)
      if (label == m.label) ok = queue_write(REG_CHARGE_MODE, m.id);
    log_cmd_("mode", label.c_str(), ok);
  }

 private:
  // link
  bool connected_ = false;
  bool has_data_ = false;
  bool want_disconnect_ = false;
  bool disconnect_sent_ = false;
  uint32_t connected_ms_ = 0;
  // request state
  std::vector<uint8_t> rx_;
  std::vector<std::pair<uint16_t, uint16_t>> writes_;
  bool busy_ = false;
  uint32_t sent_ms_ = 0;
  uint8_t pend_fn_ = 0;
  uint16_t pend_reg_ = 0, pend_qty_ = 0;
  bool pend_core_ = false;
  uint8_t step_ = N_STEPS;
  uint32_t cycle_ms_ = 0;
  uint8_t fails_ = 0;
  // values
  uint16_t soc_ = 0, dc_out_ = 0, ac_out_ = 0, dc_in_ = 0, ac_in_ = 0;
  float ac_in_v_ = 0, time_h_ = NAN;
  bool ac_on_ = false, dc_on_ = false, lift_ = false;
  int mode_ = -1;

  void log_cmd_(const char *target, const char *value, bool ok) {
    ESP_LOGI(TAG, "cmd %s=%s %s", target, value, ok ? "queued" : "rejected");
  }

  void reset_() {
    rx_.clear();
    writes_.clear();
    busy_ = false;
    fails_ = 0;
    want_disconnect_ = false;
    disconnect_sent_ = false;
    step_ = N_STEPS;
  }

  void fail_(bool core) {
    if (core && ++fails_ >= MAX_FAILS) {
      ESP_LOGW(TAG, "%u failed requests in a row, dropping the BLE link", fails_);
      want_disconnect_ = true;
    }
  }

  bool send_(uint8_t fn, uint16_t addr, uint16_t val, uint32_t now, bool core) {
    tx = {0x01, fn, (uint8_t) (addr >> 8), (uint8_t) addr, (uint8_t) (val >> 8), (uint8_t) val};
    const uint16_t crc = crc16(tx.data(), tx.size());
    tx.push_back(crc & 0xFF);
    tx.push_back(crc >> 8);
    rx_.clear();
    busy_ = true;
    sent_ms_ = now;
    pend_fn_ = fn;
    pend_reg_ = addr;
    pend_qty_ = val;
    pend_core_ = core;
    return true;
  }

  void store_(const uint8_t *d) {
    auto r = [d](int i) { return (uint16_t) ((d[2 * i] << 8) | d[2 * i + 1]); };
    switch (pend_reg_) {
      case 102: soc_ = r(0); break;
      case 104: time_h_ = r(0) / 60.0f; break;
      case 140:
        dc_out_ = r(0);
        ac_out_ = r(2);
        dc_in_ = r(4);
        ac_in_ = r(6);
        break;
      case 1314: ac_in_v_ = r(0) / 10.0f; break;
      case REG_AC_OUT: ac_on_ = r(0) == 1; break;
      case REG_DC_OUT:
        dc_on_ = r(0) == 1;
        has_data_ = true;  // last core register of a cycle
        break;
      case REG_CHARGE_MODE: mode_ = r(0); break;
      case REG_POWER_LIFTING: lift_ = r(0) == 1; break;
    }
  }
};

inline Client client;

}  // namespace bluetti
