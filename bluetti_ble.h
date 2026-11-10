#pragma once
// NimBLE central for the Bluetti AC180P (esp-nimble-cpp). It connects, subscribes to ff01 and keeps
// the link open, pumping one read cycle of the Modbus requests of bluetti_ac180p.h every POLL_MS,
// from its own task on core 1, off the WiFi/main-loop core. A broken link is re-established.

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

// The link stays open. On some boards an open BLE connection has wedged WiFi's TX path after 1-6
// minutes; the coexistence/connection-parameter settings and the reboot timeouts in the YAML mitigate that.
static const uint32_t POLL_MS = 15000;          // time between read cycles
static const uint32_t CYCLE_TIMEOUT_MS = 20000; // give up on a connection that never completes a cycle
static const uint32_t PUMP_MS = 50;             // request pump period while connected
static const uint32_t RECONNECT_MS = 3000;      // pause after a broken link, so a flapping one can't churn the radio
static const uint32_t WIFI_SETTLE_MS = 15000;   // no BLE until WiFi has been up this long (DHCP/mDNS/API handshake)
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
    client_ = NimBLEDevice::createClient();
    client_->setClientCallbacks(this, false);
    client_->setConnectTimeout(10000);
    // Relaxed connection events (60-120 ms, 8 s supervision timeout) and a light scan while
    // connecting, to leave airtime for WiFi on the shared radio.
    client_->setConnectionParams(48, 96, 0, 800, 320, 48);

    uint32_t backoff = BACKOFF_MIN_MS;
    uint32_t wifi_since = 0;  // when WiFi last came up (0 = down)
    for (;;) {
      // Keep the radio free for WiFi association: no BLE activity until WiFi is up and settled.
      if (!wifi_up_()) wifi_since = 0;
      else if (!wifi_since) wifi_since = millis() | 1;
      if (!enabled_ || !wifi_since || millis() - wifi_since < WIFI_SETTLE_MS) {
        vTaskDelay(pdMS_TO_TICKS(500));
        continue;
      }
      if (!connect_()) {
        vTaskDelay(pdMS_TO_TICKS(backoff));
        backoff = std::min(backoff * 2, BACKOFF_MAX_MS);
        continue;
      }
      backoff = BACKOFF_MIN_MS;
      serve_();
      drop_();
      vTaskDelay(pdMS_TO_TICKS(RECONNECT_MS));
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

  void pump_() {
    if (client.prepare_next(millis()) && !write_->writeValue(client.tx.data(), client.tx.size(), false)) {
      ESP_LOGW(BLE_TAG, "write to ff02 failed");  // the request timeout counts the failure
    }
  }

  // Runs while the link is up: one Modbus request in flight at a time, a read cycle every POLL_MS.
  // Queued commands are served during the idle wait. Returns when the link breaks, is released or stalls.
  void serve_() {
    uint32_t cycle_start = millis();
    while (enabled_ && client_->isConnected() && millis() - cycle_start < CYCLE_TIMEOUT_MS) {
      if (client.take_disconnect()) return;
      pump_();
      if (client.cycle_done()) {
        ESP_LOGD(BLE_TAG, "cycle done in %u ms", (unsigned) (millis() - cycle_start));
        while (enabled_ && client_->isConnected() && millis() - cycle_start < POLL_MS) {
          if (client.take_disconnect()) return;
          pump_();
          vTaskDelay(pdMS_TO_TICKS(PUMP_MS));
        }
        cycle_start = millis();
        client.begin_cycle();
        continue;
      }
      vTaskDelay(pdMS_TO_TICKS(PUMP_MS));
    }
  }

  void drop_() {
    if (client_->isConnected()) client_->disconnect();
    write_ = nullptr;
    client.on_disconnect();
  }
};

inline Link ble;

}  // namespace bluetti
