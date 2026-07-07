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
- Suggested framework: ESP-IDF
- Runtime: FreeRTOS
- Connectivity: Wi-Fi
- UI: lightweight web API first; optional local LCD/buttons later

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

1. Create ESP-IDF project skeleton.
2. Implement GPIO relay outputs with forced-safe default-off behavior.
3. Implement MAX31855 readout and fault detection.
4. Implement NTC ADC readout and basic temperature conversion.
5. Implement fixed-period control task and telemetry snapshot.
6. Implement 1 second relay time-proportional output windows.
7. Implement basic profile state machine with one built-in test profile.
8. Implement safety/fault state that disables both heaters.
9. Add lightweight `/api/status` endpoint.
10. Add configuration persistence for plate limit and safety thresholds.
11. Add editable/custom profile support.
12. Add calibration/tuning routines.

## Open Firmware Decisions

- Exact ESP-IDF version.
- Exact ESP32-C3 board/module target.
- Whether to use PID immediately or begin with bang-bang/time-proportional control and evolve to PID.
- Initial default values for safety limits and PID constants.
- JSON schema for custom profiles.
- How much telemetry history to retain in RAM versus persistent logs.
- Whether to add mDNS/captive portal behavior for first-time Wi-Fi setup.
