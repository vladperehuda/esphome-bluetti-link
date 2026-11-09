# Bluetti AC180P on ESPHome

Pure-ESPHome replacement for the PlatformIO firmware in the repo root (which is left untouched, so
you can roll back by flashing it again). No dashboard and no REST API: data goes to Home Assistant
over the ESPHome native API (no MQTT broker, no web server).

- `bluetti-ac180p.yaml`: the ESPHome config
- `bluetti_ac180p.h`: Modbus-RTU over BLE (framing, CRC16, polling) and the values/setters used by the template entities
- `secrets.yaml.example`: copy to `secrets.yaml`

No external component is needed. The EB3A project you found uses a fork of ESPHome only to feed BLE
bytes into `modbus_controller`; here stock `ble_client` plus the header does the same job.

## Home Assistant

The device is discovered by the ESPHome integration; enter `bluetti__api_key` from `secrets.yaml`
when asked. Numeric sensors show "unknown" while the Bluetti link is down, and the switches/select
report no state. Use the `BLE connection` switch to release the station for the phone app.

Migrating from the MQTT firmware: stop the old firmware first, delete the old MQTT `Bluetti AC180P`
device in HA (otherwise the new entities get a `_2` suffix), and clear the retained
`homeassistant/+/<mac6>/+/config` and `bluetti/<mac6>/#` topics from the broker.

## Build with the ESPHome desktop app

1. Install the app from https://desktop.esphome.io/.
2. `cp secrets.yaml.example secrets.yaml` and fill it in.
3. Open this folder in the app, Validate, then Install (USB the first time, OTA afterwards).

## First-boot checks

- The log shows the BLE connection, and then values flowing. If requests time out, the GATT service
  UUID may not be `0xFF00`; the log of the connect lists the real services.
- Entities in HA show unknown/unavailable until the `Bluetti link` binary sensor is on.
- If WiFi drops while BLE is connected, lower the scan duty in `esp32_ble_tracker`.
