# Pita-flow Firmware

ESP-IDF firmware for the Pita-flow reflow oven controller, targeting the
**ESP32-C3 Super Mini** board. See [`docs/firmware-plan.md`](../docs/firmware-plan.md)
for the full architecture, safety model, and roadmap.

## Layout

```
firmware/
  main/                 App entry point, Wi-Fi bring-up, built-in profile, task wiring
  components/
    core/               Hardware-independent logic: profile engine, safety
                         evaluation, NTC math, relay timing, heater control.
                         No ESP-IDF/FreeRTOS dependency - buildable/testable
                         with a plain host C toolchain.
    drivers/             ESP-IDF hardware adapters: MAX31855 thermocouple (SPI),
                         plate NTC (ADC), relay outputs (GPIO). Thin wrappers
                         around `core`.
    config_store/        NVS-backed persistent configuration.
    web_api/             esp_http_server based `/api/*` endpoints.
  tests/core/            Host-buildable unit tests for the `core` component.
```

## Building the firmware (requires ESP-IDF)

This firmware requires the [ESP-IDF](https://github.com/espressif/esp-idf) SDK
and toolchain, which are not part of this repository. Once ESP-IDF is
installed and sourced (`. $IDF_PATH/export.sh`):

```sh
cd firmware
idf.py set-target esp32c3
idf.py menuconfig   # set Pita-flow Configuration -> Wi-Fi SSID/Password
idf.py build
idf.py -p <PORT> flash monitor
```

## Running the core unit tests (no ESP-IDF required)

The `core` component has no hardware or ESP-IDF dependency, so its logic can
be exercised with a plain C toolchain:

```sh
firmware/tests/core/run_tests.sh
```
