# Pita-flow Firmware

Firmware for the Pita-flow reflow oven controller, with the control/safety core
kept hardware-independent and the local operator workflow being adapted around a
**Wio Terminal** front panel:

- built-in LCD for status and profile selection,
- built-in buttons for local run control,
- microSD-backed YAML profile catalogs,
- built-in speaker for audible status/fault notifications.

See [`docs/firmware-plan.md`](../docs/firmware-plan.md) for the architecture,
safety model, and roadmap.

## Layout

```
firmware/
  main/                 App entry point, Wi-Fi bring-up, built-in profile, task wiring
  components/
    core/               Hardware-independent logic: profile engine, safety
                         evaluation, NTC math, relay timing, heater control,
                         YAML profile catalog parsing, and local UI state/control.
                         No ESP-IDF/FreeRTOS dependency - buildable/testable
                         with a plain host C toolchain.
    drivers/             ESP-IDF hardware adapters: MAX31855 thermocouple (SPI),
                         plate NTC (ADC), relay outputs (GPIO). Thin wrappers
                         around `core`.
    config_store/        NVS-backed persistent configuration.
    web_api/             esp_http_server based `/api/*` endpoints.
  profiles/             Example microSD YAML profile catalog for Wio Terminal.
  tests/core/            Host-buildable unit tests for the `core` component.
```

## Board-specific runtime status

The heater/sensor runtime in `main/`, `drivers/`, `config_store/`, and
`web_api/` is still the earlier ESP-IDF reference implementation. The Wio
Terminal adaptation in this change set focuses on hardware-independent pieces
that can be shared by the eventual board bring-up: local button control, screen
view state, and YAML profile catalog parsing.

## Building the current reference runtime (requires ESP-IDF)

The current reference runtime still requires the
[ESP-IDF](https://github.com/espressif/esp-idf) SDK and toolchain, which are
not part of this repository. Once ESP-IDF is installed and sourced
(`. $IDF_PATH/export.sh`):

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

The core test suite now covers:

- thermal/safety logic,
- profile engine behavior,
- constrained YAML parsing for the microSD profile catalog,
- local Wio Terminal button-control state and notification events.
