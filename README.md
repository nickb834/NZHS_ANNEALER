# NZHS Annealer

This is a fork of [MJGNZ/NZHS_ANNEALER](https://github.com/MJGNZ/NZHS_ANNEALER).

Arduino Uno firmware for the NZHS Annealer Shield. For the annealer build and
hardware guide, see [MGNZ Makes](http://www.mgnz-makes.com/).

This branch is firmware version **4.0.0**. Test changes with the machine
attended before relying on automatic operation. Menu changes can be safely
tested with the included [SimulIDE simulation](simulide/).

## Controls

| Button | Normal stopped screen | Menus |
| --- | --- | --- |
| START | Start the selected run mode |
| MODE | Move through the home-screen selections | Move the selection or scroll information |
| UP | Change the selected time/mode, or open the selected item | Change, confirm, or enter the selected item |

The OLED screen retains its original layout: anneal time on the left;
fan state, operating mode, case count, and temperature on the right.

Press MODE to move through `TIME`, `MODE`, `SETTINGS`, `PROFILES`, `ANALYSE`,
`INFO`, and `DIAGNOSTICS`; press UP to open the highlighted menu. Representative
home screens are shown below.

| TIME | MODE | SETTINGS | PROFILES |
| --- | --- | --- | --- |
| ![TIME selected](docs/screenshots/home-time.png) | ![MODE selected](docs/screenshots/home-mode.png) | ![SETTINGS selected](docs/screenshots/home-settings.png) | ![PROFILES selected](docs/screenshots/home-profiles.png) |

| ANALYSE | INFO | DIAGNOSTICS |
| --- | --- | --- |
| ![ANALYSE selected](docs/screenshots/home-analyse.png) | ![INFO selected](docs/screenshots/home-info.png) | ![DIAGNOSTICS selected](docs/screenshots/home-diagnostics.png) |

`DIAGNOSTICS >` then shows its sensor and reset information:

![Diagnostics menu](docs/screenshots/diagnostics-menu.png)

### Time adjustment

With TIME selected, UP normally adds 0.1 seconds and wraps from 8.0 to 2.0
seconds. After five UP presses no more than one second apart, it adds 0.5
seconds instead. Pause for more than one second to return to 0.1-second steps.
Holding UP resets the time to 2.0 seconds.

## Running cases

Select a mode, then press START.

- **Single-shot** anneals and drops one case, then stops.
- **Free-run** anneals and drops a case, shows a five-second `LOAD` countdown,
  then starts the next cycle. Load each case manually.
- **Auto-feed** operates the stepper feeder between cycles.

A current sensor is strongly recommended for free-run and auto-feed, and is
required for the firmware's under-current and over-current safety features.
Do not use unattended automatic operation without one.

On first use after power-up, START first displays the existing case-height/time
warning. Press START again to acknowledge it, then START once more to begin a
run.

## Settings

Open `SETTINGS >` with UP. MODE selects `RESTART`, `DUMP`, or `BACK >`; UP
changes the selected item.

- **RESTART** resumes free-run or auto-feed after a cooldown. It is saved in
  EEPROM.
- **DUMP** forces free-run and disables the feeder. In DUMP mode you can hold
  MODE to open the drop gate and release it to close the drop gate.

RESTART and DUMP are mutually exclusive: enabling one turns the other off.
Changing operating mode also turns DUMP off.

The gate still opens automatically at the end of every anneal cycle. DUMP adds
a manual gate override; it does not remove the normal drop sequence.

## Cooldown and faults

With a temperature sensor fitted, cooldown begins above 55 C and a run may
resume below 40 C.

- With RESTART enabled, free-run and auto-feed resume automatically after
  cooldown. Single-shot does not.
- START during cooldown cancels the pending restart and returns to the stopped
  screen. `COOL!` remains beside the temperature and prevents another run until
  the temperature is below 40 C.
- If a sensor detected at boot later returns an invalid reading, annealing stops,
  the fan stays on, and `TEMP ERROR` is displayed. Restore a valid reading and
  press MODE to clear it.
- If no temperature sensor is detected at boot, operation follows upstream
  v3.8.0 behaviour: temperature display, cooldown, and thermal protection are
  unavailable.
- With a current sensor detected, the low-current safety check ignores the
  first anneal cycle. From the second cycle, an average current at or below
  0.1 A indicates no usable annealing current: it stops before the gate opens
  and displays `CHECK CASE` / `CURRENT LO!`.
- After one ignored cycle and five normal cycles, the firmware also stops when
  a cycle falls below 85% of the learned moving current baseline.

The current checks help detect an empty coil or missing cases. They cannot
detect every mechanical jam.

## Cartridge profiles

`PROFILES >` stores up to eight named profiles in EEPROM. Each profile saves:

- Anneal time
- Operating mode
- RESTART setting
- DUMP setting

MODE selects a profile or action; UP opens or confirms it. Empty slots are
shown as `PROFILE 1` through `PROFILE 8`. A profile name has ten characters;
hold UP for one second while editing the profile name to cycle characters
automatically. Loading a profile applies its settings immediately. The learned
current baseline is per-run and is not saved in a profile.

## Analyse mode

`ANALYSE >` records the annealer's current profile over a fixed eight-second
run. It is intended for attended testing with sacrificial cases while working
out the current and energy profile for a new case type. An analysis run may
overheat and ruin the case; it is not a normal annealing cycle.

| Analyse menu | Completed result |
| --- | --- |
| ![Analyse menu with NEW, REVIEW, and BACK selections](docs/screenshots/analyse-menu.png) | ![Completed Analyse current graph](docs/screenshots/analyse-result.png) |

A current sensor is required. Select `ANALYSE >`, load a case when prompted,
then press START. During the run the firmware:

- Targets one current sample every 25 ms.
- Draws the current trace across the 128-pixel OLED. The horizontal scale is
  zero to eight seconds and the vertical scale is zero to 12.5 A.
- Shows the running current and estimated input energy.
- Sends each sample to the USB serial port at 115200 baud for external logging
  and graphing.
- Retains the latest graph, peak current, and total input-energy estimate in RAM
  so it can be reviewed until another analysis starts or the Uno is reset.
- Turns the annealing output off after eight seconds, opens the drop gate for
  five seconds, then shows the completed graph with `BACK >`.

After one result has been retained, opening Analyse shows `NEW >`, `REVIEW >`,
and `BACK >`. MODE moves between these items and UP selects one. The retained
result is session-only and is not written to EEPROM or a cartridge profile.

Holding MODE for at least 300 ms during an analysis is the manual safety abort.
It immediately turns the annealing output off and opens the drop gate for five
seconds. The 12.3 A over-current cutoff remains active throughout the run.
Analysis cannot start while cooldown is active or while the last valid
temperature reading is above 55 C. Temperature conversion is paused during the
eight-second run to avoid disrupting the 25 ms current-sampling interval.

Serial output uses records of the following form:

```text
ANALYSE,START
ANALYSE,SAMPLE,t_ms=25,current_ma=6500,input_energy_J=7.800
ANALYSE,END,t_ms=8000,input_energy_J=400.000,peak_ma=12000
ANALYSE,GATE_OPEN
```

The energy value is calculated from measured current, elapsed time, and the
configured 48 V supply voltage. It estimates electrical energy entering the ZVS
board, not the thermal energy absorbed by the case; conversion losses and energy
heating the coil and surrounding hardware are included.

## Information and diagnostics

`INFO >` shows the current-safety threshold (85% of the learned baseline for
current during annealing), the learned baseline current from the last normal
cases, the estimated Uno 5V rail, and the firmware version. MODE scrolls
through these values; UP returns to the stopped screen.

`DIAGNOSTICS >` shows temperature-sensor/current-sensor status and `RST:c|s`,
a best-effort reset record: `c` is the reset cause and `s` is the state
recorded before the reset. UP returns to the stopped screen.

| Record | Values |
| --- | --- |
| `c` cause | `W` watchdog; `B` brown-out; `E` external reset; `P` power-on; `-` unavailable |
| `s` state | `A` annealing; `D` dropping; `R` reloading; `C` cooldown; `S` stopped; `?` unknown |

The record is best-effort because the Uno bootloader can clear the reset cause
before the firmware reads it.

## Build and upload

### Arduino IDE

1. Install the current [Arduino IDE](https://www.arduino.cc/en/software).
2. In **Library Manager**, install:
   - `Adafruit SSD1306`
   - `Adafruit GFX Library`
   - `OneWire`
   - `DallasTemperature`
3. Open [`NZHS_ANNEALER_128x32_OLED.ino`](NZHS_ANNEALER_128x32_OLED/NZHS_ANNEALER_128x32_OLED.ino).
4. Select **Tools → Board → Arduino AVR Boards → Arduino Uno**.
5. Select the Uno's serial port under **Tools → Port**, then use **Verify** or
   **Upload**.

Only flash the firmware while the unit is idle, not annealing or in cooldown.

### Arduino CLI

From the repository root:

```sh
arduino-cli compile --fqbn arduino:avr:uno --export-binaries NZHS_ANNEALER_128x32_OLED
```

The tracked release artifacts are:

- `NZHS_ANNEALER_128x32_OLED.ino.standard.hex` — standard Uno image
- `NZHS_ANNEALER_128x32_OLED.ino.with_bootloader.standard.hex` — image that
  includes the bootloader

When firmware source changes, rebuild both artifacts from the exact commit
before distributing or pushing them.

## Future Uno R4 hardware support

Future hardware work to move beyond the ATmega328P's 32 KB flash limit will
focus on the [Uno R4 Minima](https://docs.arduino.cc/hardware/uno-r4-minima) and
[Uno R4 WiFi](https://docs.arduino.cc/hardware/uno-r4-wifi). Both retain the
Arduino Uno shield layout and 5 V I/O while providing 256 KB flash and 32 KB RAM. The
Minima is the simpler initial target; the WiFi board uses the same Renesas
RA4M1 main processor and can follow the same port if its wireless features are
useful.

The R4 is a 32-bit Arm board, not an AVR board, so the current firmware cannot
be compiled for it unchanged. R4 support will require:

- Replacing the AVR Timer1 configuration used for drop-gate servo control.
- Replacing the Timer2 compare interrupt used to generate feeder step pulses.
- Porting AVR watchdog and reset-cause diagnostics.
- Replacing the ATmega328P ADC/band-gap implementation used to estimate the Uno
  5V rail.
- Verifying EEPROM behaviour and the SSD1306, OneWire, DallasTemperature, and
  servo support against the Arduino R4 core.
- Revalidating pin behaviour, feeder and gate timing, current measurements,
  cooldown handling, over-current protection, and every other hardware safety
  path on an installed annealer shield.
- Producing R4-specific firmware artifacts and deciding how R4 builds will be
  tested, because the present SimulIDE harness models an Arduino Uno.

Until that port and hardware validation are complete, release firmware and HEX
artifacts remain for Arduino Uno-compatible AVR boards only.

## Simulator

The [`simulide/`](simulide/) harness uses SimulIDE 1.1.0 SR2 on macOS (as
tested, but should work on Windows/Linux) and loads the normal production HEX;
it is not a simulator-specific firmware build.
See [simulide/README.md](simulide/README.md) for setup and limits.

![SimulIDE Uno harness](simulide/nzhs-annealer-simulide.png)

## Testing and development notes

The detailed release/hardware validation checklist is in
[docs/TESTING.md](docs/TESTING.md).

## Version history

SW version 4.0.0

- Added home screen menus for settings, profiles, diagnostics, and info.
- Manual Dump and Cooldown auto restart are configurable directly in Settings.
- Added a low-current safety stop.
- Added Analyse mode with an OLED current trace, USB serial data, an input-energy
  estimate, retained result review, and a manual safety abort.

SW version 3.8.0
- Fixed bug relating to case feeder home position drift

SW version 3.7.0
- Added case counter feature. Can be enabled/disabled
- changed OLED I2C clock speed to improve main loop speed

SW version 3.6.0
- Utilise CURRENT_SENSOR_SCALE define in current measurement function

SW version 3.5.0
- bug fix for scaling error in current measurement function

SW version 3.4.0
- Merged changes from stro to reduce RAM usage for fixed strings

SW version 3.3.0
- Bug fix for stepper interrupt duration
- Improvement to Anneal timing precision and repeatabilty

SW version 3.2.0
- Resolved bugs relating to stepper motor drifting after multiple cases
- UI cleanup

SW version v3.0.0
- Support for REV C hardware with stepper motor control for automatic case feeder
- v3.0.0 software is backward compatible with REV A & REV B hardware

SW version v2.1.0
- Added provision for reassigning the mode key as a case dump button for stuck cases. Unit runs in free run mode if this option is selected.
