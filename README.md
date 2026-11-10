# Bluetti AC180P on ESPHome

ESPHome config for the Bluetti AC180P. Data goes to Home Assistant over the ESPHome native API (no MQTT
broker, no web server).

- `bluetti-ac180p.yaml`: the ESPHome config
- `bluetti_ble.h`: NimBLE (esp-nimble-cpp) central running in its own task; it holds a permanent connection and triggers a read cycle every 15 s (`POLL_MS`)
- `bluetti_ac180p.h`: Modbus-RTU over BLE (framing, CRC16, polling) and the values/setters used by the template entities
- `secrets.yaml.example`: copy to `secrets.yaml`

No external component is needed; the two headers are pulled in through `esphome: includes:`.

## Home Assistant

The device is discovered by the ESPHome integration; enter `bluetti__api_key` from `secrets.yaml`
when asked. Numeric sensors show "unknown" while the Bluetti link is down, and the switches/select
report no state. Use the `BLE connection` switch to release the station for the phone app.

## Build with the ESPHome desktop app

1. Install the app from https://desktop.esphome.io/.
2. `cp secrets.yaml.example secrets.yaml` and fill it in.
3. Open this folder in the app, Validate, then Install (USB the first time, OTA afterwards).

## First-boot checks

- The log shows the BLE connection, and then values flowing. If requests time out, the GATT service
  UUID may not be `0xFF00`; the log of the connect lists the real services.
- Entities in HA show unknown/unavailable until the `Bluetti link` binary sensor is on.
- The BLE link is permanent. If WiFi stalls after a few minutes with the link open, check the coexistence and
  connection-parameter settings in `bluetti-ac180p.yaml` / `bluetti_ble.h`.
