#pragma once
// NimBLE central for the Bluetti AC180P (esp-nimble-cpp). It connects, subscribes to
// ff01, pumps one read cycle of the Modbus requests of bluetti_ac180p.h and disconnects again (or
// stays connected, see PERSISTENT_LINK), from its own task on core 1, off the WiFi/main-loop core.

#include <NimBLEDevice.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <algorithm>
#include <string>
#include <vector>

#include "bluetti_ac180p.h"
#include "esphome/components/wifi/wifi_component.h"

namespace bluetti {

static const char *const BLE_TAG = "bluetti_ble";

static const char *const SERVICE_UUID = "0000ff00-0000-1000-8000-00805f9b34fb";
static const char *const NOTIFY_UUID = "0000ff01-0000-1000-8000-00805f9b34fb";
static const char *const WRITE_UUID = "0000ff02-0000-1000-8000-00805f9b34fb";

// TEST: bring the NimBLE controller up but never scan or connect. Tells whether the WiFi stall needs
// BLE radio activity or just the BLE controller running next to WiFi. Sensors stay unknown.
static const bool CONTROLLER_ONLY_TEST = false;

// An open BLE connection wedges WiFi's TX path on this board after 1-6 minutes, so the link is only
// opened for one read cycle (~0.5 s of requests) per CYCLE_MS and dropped again.
// PERSISTENT_LINK keeps one connection open and reads every POLL_MS; otherwise the link is opened per
// cycle, every CYCLE_MS.
static const bool PERSISTENT_LINK = false;
static const uint32_t POLL_MS = 5000;
static const uint32_t CYCLE_MS = 30000;         // time between read cycles (burst mode)
static const uint32_t CYCLE_TIMEOUT_MS = 20000; // give up on a connection that never completes a cycle
static const uint32_t PUMP_MS = 50;             // request pump period while connected
static const uint32_t RECONNECT_MS = 10000;     // pause after a broken link, so a flapping one can't churn the radio
static const uint32_t BACKOFF_MIN_MS = 2000;    // failed connect attempts back off up to BACKOFF_MAX_MS
static const uint32_t BACKOFF_MAX_MS = 30000;

class Link : public NimBLEClientCallbacks {
 public:
  void begin(const char *mac) {
    if (task_) return;
    addr_ = NimBLEAddress(std::string(mac), BLE_ADDR_PUBLIC);
    xTaskCreatePinnedToCore(&Link::task_entry_, "bluetti_ble", 8192, this, 1, &task_, 1);
  }

  // The "BLE connection" switch: off releases the station (e.g. for the phone app).
  bool enabled() const { return enabled_; }
  void set_enabled(bool on) {
    ESP_LOGI(BLE_TAG, "link %s", on ? "enabled" : "released");
    client.set_accept_writes(on);
    enabled_ = on;
  }

  void onConnectFail(NimBLEClient *, int reason) override { ESP_LOGW(BLE_TAG, "connect failed, reason %d", reason); }
  void onDisconnect(NimBLEClient *, int reason) override { ESP_LOGW(BLE_TAG, "disconnected, reason %d", reason); }

 private:
  TaskHandle_t task_ = nullptr;
  NimBLEAddress addr_;
  NimBLEClient *client_ = nullptr;
  NimBLERemoteCharacteristic *write_ = nullptr;
  volatile bool enabled_ = true;
  bool have_db_ = false;  // GATT database cached: later connects skip service discovery

  static void task_entry_(void *self) { static_cast<Link *>(self)->run_(); }

  static bool wifi_up_() {
    return esphome::wifi::global_wifi_component != nullptr && esphome::wifi::global_wifi_component->is_connected();
  }

  void run_() {
    NimBLEDevice::init("");
    if (CONTROLLER_ONLY_TEST) {
      ESP_LOGW(BLE_TAG, "controller-only test: BLE is initialised but never connects");
      for (;;) vTaskDelay(pdMS_TO_TICKS(60000));
    }
    client_ = NimBLEDevice::createClient();
    client_->setClientCallbacks(this, false);
    client_->setConnectTimeout(10000);
    // Relaxed connection events (60-120 ms, 4 s supervision timeout) and a light scan while
    // connecting, to leave airtime for WiFi on the shared radio.
    client_->setConnectionParams(48, 96, 0, 400, 320, 48);

    uint32_t backoff = BACKOFF_MIN_MS;
    uint32_t next_ms = 0;  // when the next read cycle is due
    for (;;) {
      // Keep the radio free for WiFi association: no BLE activity until WiFi is up.
      if (!enabled_ || !wifi_up_()) {
        vTaskDelay(pdMS_TO_TICKS(500));
        continue;
      }
      // A queued command (switch, charge mode) opens the link right away.
      if ((int32_t) (millis() - next_ms) < 0 && !client.has_pending_writes()) {
        vTaskDelay(pdMS_TO_TICKS(200));
        continue;
      }
      const uint32_t start = millis();
      if (!connect_()) {
        vTaskDelay(pdMS_TO_TICKS(backoff));
        backoff = std::min(backoff * 2, BACKOFF_MAX_MS);
        continue;
      }
      backoff = BACKOFF_MIN_MS;
      const bool ok = serve_();
      drop_();
      next_ms = start + (ok ? CYCLE_MS : RECONNECT_MS);
    }
  }

  bool connect_() {
    if (!client_->connect(addr_, !have_db_)) {
      ESP_LOGW(BLE_TAG, "connect to the Bluetti failed (error %d)", client_->getLastError());
      return false;
    }
    NimBLERemoteService *svc = client_->getService(SERVICE_UUID);
    NimBLERemoteCharacteristic *notify = svc ? svc->getCharacteristic(NOTIFY_UUID) : nullptr;
    write_ = svc ? svc->getCharacteristic(WRITE_UUID) : nullptr;
    if (!notify || !write_) {
      ESP_LOGW(BLE_TAG, "characteristics ff01/ff02 not found");
      have_db_ = false;
      drop_();
      return false;
    }
    const bool ok = notify->subscribe(true, [](NimBLERemoteCharacteristic *, uint8_t *d, size_t n, bool) {
      client.on_notify(std::vector<uint8_t>(d, d + n));
    });
    if (!ok) {
      ESP_LOGW(BLE_TAG, "subscribe to ff01 failed");
      drop_();
      return false;
    }
    have_db_ = true;
    ESP_LOGI(BLE_TAG, "connected, notify subscribed (rssi %d dBm, heap %u)", client_->getRssi(),
             (unsigned) esp_get_free_heap_size());
    client.on_connect(millis());
    return true;
  }

  // Runs while the link is up: one Modbus request in flight at a time. In burst mode it returns true
  // once a cycle completed; in persistent mode it keeps cycling every POLL_MS until the link breaks.
  bool serve_() {
    uint32_t cycle_start = millis();
    while (enabled_ && client_->isConnected() && millis() - cycle_start < CYCLE_TIMEOUT_MS) {
      if (client.take_disconnect()) return false;
      if (client.prepare_next(millis()) && !write_->writeValue(client.tx.data(), client.tx.size(), false)) {
        ESP_LOGW(BLE_TAG, "write to ff02 failed");  // the request timeout counts the failure
      }
      if (client.cycle_done()) {
        ESP_LOGD(BLE_TAG, "cycle done in %u ms", (unsigned) (millis() - cycle_start));
        if (!PERSISTENT_LINK) return true;
        // Wait for the next period, still serving queued commands.
        while (enabled_ && client_->isConnected() && millis() - cycle_start < POLL_MS) {
          if (client.take_disconnect()) return false;
          if (client.prepare_next(millis()) && !write_->writeValue(client.tx.data(), client.tx.size(), false)) {
            ESP_LOGW(BLE_TAG, "write to ff02 failed");
          }
          vTaskDelay(pdMS_TO_TICKS(PUMP_MS));
        }
        cycle_start = millis();
        client.begin_cycle();
        continue;
      }
      vTaskDelay(pdMS_TO_TICKS(PUMP_MS));
    }
    return false;
  }

  void drop_() {
    if (client_->isConnected()) client_->disconnect();
    write_ = nullptr;
    client.on_disconnect();
  }
};

inline Link ble;

}  // namespace bluetti
