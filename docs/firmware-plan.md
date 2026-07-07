# Firmware Plan

This document describes the initial firmware architecture for Pita-flow, an ESP32-C3 based reflow oven controller.

The firmware should be designed around deterministic thermal control first. Wi-Fi, web UI, logging, calibration, and profile editing are important, but they must never compromise the heater-control loop or safety shutdown paths.

## Goals

- Maintain stable and repeatable reflow profiles.
- Keep heater control deterministic even while Wi-Fi and web clients are active.
- Use lightweight server-side behavior; push rendering and interaction to client devices where possible.
- Support configurable reflow profiles.
- Support future auto-calibration and PID tuning.
- Fail safe: any critical sensor, control, or software fault disables both heaters.

## Platform

- Microcontroller: ESP32-C3
- Board/module: **ESP32-C3 Super Mini** (ESP32-C3FH4-based dev board)
- Suggested framework: ESP-IDF
- Runtime: FreeRTOS
- Connectivity: Wi-Fi
- UI: lightweight web API first; optional local LCD/buttons later

### ESP32-C3 Super Mini Pin Assignment

The Super Mini breaks out GPIO0-10, 18, and 19. Suggested assignment (arbitrary GPIO SPI/ADC
pin mapping is supported on ESP32-C3, so these can be adjusted in `config_store` defaults
without firmware redesign):

| Signal | GPIO | Notes |
|---|---|---|
| MAX31855 SPI CLK | GPIO6 | HSPI default CLK |
| MAX31855 SPI MISO | GPIO2 | HSPI default MISO (read-only device, MOSI unused) |
| MAX31855 SPI CS | GPIO10 | HSPI default CS |
| Plate NTC ADC input | GPIO0 | ADC1_CH0 |
| Top heater (SSR) output | GPIO4 | Digital output |
| Bottom heater (mechanical relay) output | GPIO5 | Digital output |
| Status LED (optional) | GPIO8 | Onboard LED on most Super Mini boards |
| Reserved for `ui_local` | GPIO1, GPIO3, GPIO7, GPIO9, GPIO18, GPIO19 | Available for future buttons/LCD |

GPIO0 is a strapping pin; it is safe to use as an analog input but must not be pulled low by
external circuitry at boot.

### Confirmed Hardware Interfaces

The following hardware decisions are confirmed and replace the corresponding entries in
"Open Firmware Decisions":

**Bottom heater — mechanical relay module**

- Contact type: normally open (NO).
- Control input: active-high (input logic-high energizes the relay/heater).
- `relay_output` drives `gpio_set_level(pin, 1)` to energize and defaults to `0` (de-energized)
  at boot and on any fault, consistent with this active-high, normally-open wiring.

**Top heater — solid state relay, COMUS WG480D25Z**

- Control input: 3–32 VDC, opto-isolated.
- Switching behavior: **zero-cross switching** (not random-fire/phase-angle). This is compatible
  with the firmware's slow (nominal 1 second) time-proportional windowing: at 50/60 Hz there are
  far more zero crossings than relay-window transitions, so the SSR always switches cleanly at
  the next zero crossing regardless of exactly when the GPIO changes state. Phase-angle/dimming
  control is not possible with this part and is not required by this design.
- Load rating: 25 A at rated voltage, but this requires proper heatsinking (hockey-puck package);
  size the heatsink for the actual quartz heater current draw, not just worst case.
- Because the ESP32-C3 GPIO output is 3.3 V and its recommended source current (~20 mA/pin) may
  be marginal for reliably driving the SSR's opto-input at full brightness/response across
  temperature and unit variation, drive the SSR input through a small NPN/MOSFET driver stage
  (or a logic-level driver such as a low-side N-channel MOSFET with a pull-down and series gate
  resistor) rather than directly from the GPIO. This also preserves clean electrical separation
  between the logic-level control board and the mains-adjacent SSR input.

**Plate NTC divider**

- NTC: 100 KΩ NTC (nominal at 25 °C), Beta ≈ 3950 (adjust to the actual part's datasheet Beta
  once known).
- Wiring: 3.3 V — NTC — ADC node — **pulldown resistor** — GND (NTC on the high side, fixed
  resistor pulling the ADC node down to GND). With this wiring, higher temperature (lower NTC
  resistance) produces a *higher* ADC reading, which is the more intuitive mapping for logging
  and calibration.
- **Recommended pulldown resistor: 4.7 KΩ** (standard E12 value; a 4.99 KΩ 1% resistor is an
  equally good precision alternative). This value is chosen to maximize ADC resolution across
  the plate's actual assisted-heating operating range (roughly 100–150 °C, where the 100 K
  NTC's resistance has already dropped to roughly 2–7 KΩ), rather than around the NTC's 25 °C
  nominal value, since that hot operating band is where control precision and over-temperature
  detection both matter most. The tradeoff is reduced resolution near room temperature, which is
  acceptable since the plate is not precision-controlled while idle/cold.
- ADC attenuation: 12 dB (`ADC_ATTEN_DB_12`), giving the full 0–3.3 V input range needed to read
  the divider's node voltage across its whole swing.
- Filtering: oversample (average) 8 raw ADC samples per reading in `plate_ntc_read()` to reduce
  quantization/noise jitter; no additional hardware RC filtering is required for this
  slow-moving thermal signal, but a small (e.g. 100 nF) decoupling capacitor from the ADC node to
  GND is recommended for noise immunity given the mechanical relay sharing the same enclosure.
- Calibration method: two-point calibration is recommended once hardware is available — measure
  the raw ADC reading at two known, stable reference temperatures (e.g. room temperature and a
  boiling-water or oven-controlled reference near the plate's operating range), solve for an
  effective Beta and/or nominal-resistance correction, and store the corrected `ntc_params_t`
  values in `config_store`. Until calibrated, firmware defaults use the NTC datasheet nominal
  values (100 KΩ @ 25 °C, Beta 3950) as a reasonable starting point.

**Local UI / auxiliary inputs**

- No LCD/local display is planned at this time; `ui_local` remains unimplemented and the GPIOs
  reserved for it are simply unused for now.
- Buzzer, fan, door/cover switch, and emergency-stop input are explicitly deferred to a later
  hardware revision. The safety model and GPIO map should leave room for them, but no firmware
  work is being done for these in this milestone.


## Hardware Abstraction

Firmware should isolate hardware-specific code behind small drivers/modules:

| Module | Responsibility |
|---|---|
| `thermocouple_max31855` | Read chamber K-type thermocouple and decode MAX31855 fault bits |
| `plate_ntc` | Read ADC, filter samples, convert 100K NTC reading to plate temperature |
| `relay_output` | Drive mechanical relay outputs using slow time-proportional control |
| `heater_control` | Own PID/state logic for top and bottom heaters |
| `profile_engine` | Track current reflow profile phase, target temperature, and ramp/soak timing |
| `safety` | Enforce sensor faults, over-temperature limits, watchdogs, and emergency shutdown |
| `config_store` | Persist profiles, calibration values, and configurable limits |
| `web_api` | Expose status, profile selection, configuration, and logs through lightweight endpoints |
| `ui_local` | Optional LCD/buttons integration, currently TBD |

## FreeRTOS Task Model

The control path should have higher priority than networking and UI tasks.

| Task | Priority | Suggested period | Responsibilities |
|---|---:|---:|---|
| Safety/control task | High | 100 ms logic tick; 1 s relay window | Read sensors, update filtered temperatures, run safety checks, update heater demands, drive relay windows |
| Profile task | Medium-high | 100–500 ms | Advance reflow state machine and compute process targets |
| Telemetry/logging task | Medium | 500 ms–1 s | Record samples, expose recent history, prepare status snapshots |
| Web/API task | Low | Event-driven | Serve status/config/profile endpoints with minimal processing |
| Optional UI task | Low-medium | 50–200 ms | Read buttons, update display, avoid blocking control |

The implementation can combine some of these initially, but the architecture should preserve the same separation of concerns.

## Control Loop

### Timing

Mechanical relays are controlled using slow time-proportional PWM with a nominal **1 second window**.

Example: if the controller requests 35% heater power during a 1 second window, the relay is energized for approximately 350 ms and de-energized for the remaining 650 ms.

Because these are mechanical relays, this is intentionally much slower than normal PWM. Relay longevity is not the primary concern for this project, but the firmware should still avoid unnecessarily rapid chatter around thresholds.

### Sensor Mapping

| Heater | Primary sensor | Role |
|---|---|---|
| Bottom plate heater | 100K NTC in plate | Preheat/soak assist; constrained by configurable plate ceiling |
| Top quartz heater | K-type thermocouple via MAX31855 | Primary chamber/profile control |

### Heater Roles by Phase

| Phase | Bottom heater behavior | Top heater behavior |
|---|---|---|
| Idle | Off | Off |
| Preheat | Enabled as needed, plate-limited | Enabled |
| Soak | Enabled as needed, plate-limited | Enabled |
| Reflow / liquidus | Normally disabled or heavily limited | Enabled, primary control source |
| Cooldown | Off | Off |
| Fault | Off | Off |

The bottom plate limit must be configurable. It should be stored as part of system configuration and may eventually be overridden per profile if needed.

## Reflow Profile Engine

Profiles should be represented as phase-based data instead of hardcoded logic.

A profile phase should eventually support fields such as:

```json
{
  "name": "soak",
  "duration_s": 90,
  "target_chamber_c": 170,
  "ramp_rate_c_per_s": 0.5,
  "bottom_heater_mode": "limited",
  "bottom_plate_limit_c": 150,
  "top_heater_enabled": true
}
```

Initial firmware can start with one built-in profile, but the data model should allow loading custom profiles from persistent storage later.

## Safety Model

Any critical fault should immediately force both relays off.

Initial safety checks should include:

- MAX31855 thermocouple fault bit set.
- Thermocouple open/short fault reported by MAX31855.
- Plate NTC ADC reading outside plausible range.
- Chamber temperature above configured absolute maximum.
- Plate temperature above configured absolute maximum.
- Control task watchdog timeout.
- Relay output commanded on while system state is idle/fault.
- Invalid or corrupt active profile.

Safety values should be configurable where appropriate, but firmware should also include conservative compile-time defaults.

## Configuration

Configuration should include at minimum:

| Setting | Description |
|---|---|
| Bottom plate temperature ceiling | Configurable maximum plate target/limit during assisted heating |
| Chamber absolute maximum | Safety cutoff for chamber/process temperature |
| Plate absolute maximum | Safety cutoff below the hardware 250 °C cutoff |
| Relay window duration | Nominally 1 second |
| PID constants | Separate control constants for chamber/top heater and plate/bottom heater |
| Active profile | Selected reflow profile |
| Sensor calibration values | ADC calibration, NTC parameters, thermocouple offsets if required |

ESP-IDF NVS is a reasonable first persistent storage backend.

## Web/API Design

The ESP32-C3 should do as little web rendering as possible.

Preferred approach:

- Serve static files if needed, but keep them small.
- Provide lightweight JSON endpoints for status, profiles, config, and commands.
- Let the browser/client render charts and controls.
- Avoid expensive server-side string building during active reflow.
- Rate-limit telemetry endpoints so polling clients cannot starve control tasks.

Possible initial endpoints:

| Endpoint | Method | Purpose |
|---|---|---|
| `/api/status` | GET | Current state, temperatures, heater outputs, active phase, fault state |
| `/api/profile` | GET | Active profile |
| `/api/profiles` | GET | List stored profiles |
| `/api/profile/select` | POST | Select a profile |
| `/api/run/start` | POST | Start selected profile |
| `/api/run/stop` | POST | Stop run and disable heaters |
| `/api/config` | GET/POST | Read/update configurable limits and PID constants |
| `/api/log/recent` | GET | Recent temperature/control history |

## Auto-Calibration Roadmap

Auto-calibration is not part of the first firmware milestone, but the design should allow it.

Possible calibration features:

1. **Sensor calibration**
   - ADC calibration for the NTC voltage divider.
   - NTC model parameters or lookup-table correction.
   - Optional thermocouple offset correction.

2. **Thermal response characterization**
   - Apply controlled heater pulses.
   - Log chamber and plate response curves.
   - Estimate lag, overshoot, and heater effectiveness.

3. **PID tuning support**
   - Assist in selecting PID constants for top and bottom loops.
   - Store tuned constants in configuration.
   - Keep conservative fallback constants in firmware.

4. **Profile validation**
   - Compare measured ramp rates with profile requirements.
   - Warn if a profile demands impossible heating/cooling behavior.

## Initial Firmware Milestones

The following milestones are implemented in [`firmware/`](../firmware/) (see its README for
build instructions); 11 and 12 remain future work.

1. Create ESP-IDF project skeleton. ✅
2. Implement GPIO relay outputs with forced-safe default-off behavior. ✅
3. Implement MAX31855 readout and fault detection. ✅
4. Implement NTC ADC readout and basic temperature conversion. ✅
5. Implement fixed-period control task and telemetry snapshot. ✅
6. Implement 1 second relay time-proportional output windows. ✅
7. Implement basic profile state machine with one built-in test profile. ✅
8. Implement safety/fault state that disables both heaters. ✅
9. Add lightweight `/api/status` endpoint. ✅
10. Add configuration persistence for plate limit and safety thresholds. ✅
11. Add editable/custom profile support.
12. Add calibration/tuning routines.

## Testability & Host-Side Unit Testing

To keep the safety-critical logic verifiable without hardware or the ESP-IDF toolchain, the
firmware separates **hardware-independent core logic** from **ESP-IDF hardware adapters**:

- The `core` component (profile engine state machine, safety fault evaluation, NTC
  temperature conversion math, relay time-proportional window calculation, and heater
  bang-bang/PID control) contains no ESP-IDF, FreeRTOS, or hardware I/O calls. It only depends
  on the C standard library, so it can be compiled and unit tested with a plain host
  toolchain (e.g. gcc + CMake/CTest) independent of ESP-IDF.
- ESP-IDF specific components (`thermocouple_max31855`, `plate_ntc`, `relay_output`,
  `config_store`, `web_api`) are thin adapters that perform hardware I/O and call into `core`
  for all decision logic.
- This keeps the control and safety logic fast to test in CI and reviewable independent of
  board bring-up, while the adapters remain small enough to review by inspection.

## Open Firmware Decisions

- Exact ESP-IDF version.
- Whether to use PID immediately or begin with bang-bang/time-proportional control and evolve to PID.
- Initial default values for safety limits and PID constants.
- JSON schema for custom profiles.
- How much telemetry history to retain in RAM versus persistent logs.
- Whether to add mDNS/captive portal behavior for first-time Wi-Fi setup.
- Buzzer, fan, door/cover switch, and emergency-stop input: deferred to a later hardware revision.
- Local LCD/display: not planned at this time (see "Confirmed Hardware Interfaces").
