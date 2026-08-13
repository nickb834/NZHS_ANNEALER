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
| MODE | Move through TIME, MODE, SETTINGS, PROFILES, INFO, and DIAGNOSTICS | Move the selection or scroll information |
| UP | Change the selected time/mode, or open the selected item | Change, confirm, or enter the selected item |

The OLED screen retains its original layout: anneal time on the left;
fan state, operating mode, case count, and temperature on the right.

Press MODE to move through the home-screen selections shown below; press UP to
open the highlighted menu.

| TIME | MODE | SETTINGS |
| --- | --- | --- |
| ![TIME selected](docs/screenshots/home-time.png) | ![MODE selected](docs/screenshots/home-mode.png) | ![SETTINGS selected](docs/screenshots/home-settings.png) |

| PROFILES | INFO | DIAGNOSTICS |
| --- | --- | --- |
| ![PROFILES selected](docs/screenshots/home-profiles.png) | ![INFO selected](docs/screenshots/home-info.png) | ![DIAGNOSTICS selected](docs/screenshots/home-diagnostics.png) |

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
