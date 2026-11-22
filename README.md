# Doppler Radar Collision Avoidance & Braking Estimation System

A Doppler radar-based collision avoidance and braking estimation system for a trailing vehicle in a vehicle-platoon prototype.

## Overview
- **Relative Velocity Sensing:** HB100 X-band Doppler radar module (10.525 GHz) detects motion along the radar axis with frequency scaling according to relative speed ($k \approx 70.42\text{ Hz/(m/s)}$).
- **Hybrid Hardware + Microcontroller Architecture:**
  - Time-critical safety functions (emergency latching, dead-time insertion, reverse braking pulse, and drive gating) are handled deterministically in hardware using `74HCT74` (`U5`), dual `NE555` timers (`U9`, `U10`), and `74HC08` AND gating (`U6`).
  - **ESP32** measures Doppler pulse frequency (`GPIO32`), monitors latch/trigger status (`GPIO34`, `GPIO35`), adjusts drive command (`GPIO26`), and provides software/V2V triggers (`GPIO33`).
- **Power Rails:**
  - 11.1V Battery (`BAT_SW`) $\rightarrow$ LM2596 buck converter $\rightarrow$ 5V rail.
  - 5VA analog rail (~4.7V) via 1N4007 diode drop for radar & analog conditioning.
  - Virtual ground ($V_G \approx 2.35\text{V}$) reference and 3.3V logic level shifters ($1\text{ k}\Omega / 2\text{ k}\Omega$).

---

## Files in Repository

| File | Description |
| --- | --- |
| `doppler-sim.html` | Real-time interactive architecture simulator, visual top-view platoon emulator, dynamic oscilloscope/timing charts, and tuning console. |
| `doppler_platoon.ino` | Production ESP32 Arduino firmware featuring ISR edge-frequency measurement, moving average filter, proportional PWM braking, and serial telemetry. |

---

## ESP32 Pin Mapping

| Pin | Direction | Signal | Connected Node / Role |
| --- | --- | --- | --- |
| `GPIO32` | INPUT | `SQ` | Conditioned Doppler square wave from comparator `U3` Pin 1 (ISR freq measurement) |
| `GPIO35` | INPUT | `TRIG` | Envelope threshold trigger from comparator `U3` Pin 7 |
| `GPIO34` | INPUT | `Q_3V3` | Hardware latch state from `U5` Pin 5 ($1\text{Q}$) scaled via $1\text{ k}\Omega / 2\text{ k}\Omega$ divider |
| `GPIO26` | OUTPUT | `CMD_FWD` | Microcontroller forward drive enable into AND gate `U6` Pin 1 |
| `GPIO33` | OUTPUT | `V2V_TRIG`| Software/V2V collision test trigger into `CLK_NODE` via 1N4148 diode |
| `GPIO25` | OUTPUT | `PWM` | Motor speed modulation into TB6612FNG driver |
