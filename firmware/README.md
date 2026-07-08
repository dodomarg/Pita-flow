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
  components/
    core/               Hardware-independent logic: profile engine, safety
                         evaluation, NTC math, relay timing, heater control,
                         YAML profile catalog parsing, and local UI state/control.
                         No hardware/RTOS dependency - buildable/testable
                         with a plain host C toolchain.
  profiles/             Example microSD YAML profile catalog for Wio Terminal.
  tests/core/            Host-buildable unit tests for the `core` component.
  wio_terminal/         PlatformIO project for the Wio Terminal board
                        (Arduino framework): reuses `components/core`
                        unmodified and adds Arduino-based drivers for the
                        screen, buttons, speaker, microSD, external
                        MAX31855, plate NTC, and heater relays.
```

## Board-specific runtime status

The Wio Terminal board port in `wio_terminal/` is the single supported
runtime: it wires the shared `core` control/safety logic to real screen/
button/speaker/microSD/relay/sensor drivers on the Wio Terminal's SAMD51 MCU.
Hardware pin assignments there are bring-up placeholders pending the external
sensor/relay board finalization described in docs/firmware-plan.md.

## Building the Wio Terminal firmware (PlatformIO)

The Wio Terminal board port requires [PlatformIO](https://platformio.org/):

```sh
cd firmware/wio_terminal
pio run                     # build
pio run -t upload           # flash over USB
```

## Running the core unit tests

The `core` component has no hardware or RTOS dependency, so its logic can
be exercised with a plain C toolchain:

```sh
firmware/tests/core/run_tests.sh
```

The core test suite now covers:

- thermal/safety logic,
- profile engine behavior,
- constrained YAML parsing for the microSD profile catalog,
- local Wio Terminal button-control state and notification events.
