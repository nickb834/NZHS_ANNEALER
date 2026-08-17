# Hardware validation checklist

Use this checklist before unattended use or publishing a firmware release.

## Build and artifacts

- Compile the same sketch for both `arduino:avr:uno` and
  `arduino:renesas_uno:unor4wifi` with OneWire and DallasTemperature installed,
  plus Servo for the R4.
- Record the compiler flash and RAM report for both targets.
- Regenerate and test both tracked R3 HEX artifacts and the tracked R4 WiFi BIN
  artifact from the exact source commit.
- Repeat the behavioural sections below on each physical board; a successful
  cross-compile is not hardware validation.

## Uno R4 platform bring-up

- With the annealing/high-current output disconnected, confirm D6 remains LOW
  throughout boot and stopped operation.
- Measure the D9 servo waveform and confirm approximately 640 us open and
  1920 us closed pulses in a 20 ms servo frame.
- Measure D12 feeder pulses in each speed region; confirm D13 direction and the
  active-low D5 enable state match the R3 installation.
- Confirm A0 defaults to ten-bit readings and recalibrate current against known
  measurements before testing the 12.3 A trip.
- Confirm the D8 DS18B20, A4/A5 OLED bus, R4 5V estimate, EEPROM profile and
  reference persistence, watchdog reset and reset-cause display.
- Only reconnect the high-current output after all safe-state, gate and feeder
  checks pass.

## Controls and menus

- Check TIME → MODE → SETTINGS → PROFILES → INFO → DIAGNOSTICS → TIME
  navigation on the stopped screen.
- Confirm UP opens each selected menu and every `BACK >` returns to the
  originating stopped-screen item.
- Confirm INFO scrolls with MODE and returns with UP.
- Confirm Diagnostics returns with UP.
- Confirm normal UP adjustment, rapid 0.5-second adjustment after five quick
  taps, and the UP-hold reset to 2.0 seconds.

## Modes, feeder, and gate

- Test single-shot: anneal, drop, stop.
- Test free-run: anneal, drop, manual `LOAD` countdown, next cycle.
- Test auto-feed: feeder enable, direction, step pulses, and case timing.
- Test DUMP ON: free-run is selected, feeder is disabled, and MODE opens the
  gate only while held.
- Confirm enabling DUMP turns RESTART off; enabling RESTART turns DUMP off.

## Cooldown and temperature safety

- Confirm cooldown begins above 55 C and a run cannot resume until below 40 C.
- With RESTART enabled, confirm free-run and auto-feed resume after cooldown;
  confirm single-shot does not.
- Press START during cooldown: confirm pending restart is cancelled, `COOL!` is
  displayed, menus remain available, and a new run is blocked until below 40 C.
- Boot without a temperature sensor and confirm no `TEMP ERROR` is displayed,
  and that temperature display, cooldown, and thermal protection are unavailable.
- Boot with a temperature sensor, then disconnect it while stopped and while
  annealing. Confirm annealer off, fan on, `TEMP ERROR`, and that a valid sensor
  reading plus MODE acknowledgement are required to continue.

## Current safety

- Fit a current sensor before testing free-run or auto-feed. It is required for
  the firmware's under-current and over-current safety protections.
- Safely verify the over-current stop at approximately 12.3 A.
- Confirm the first no-current cycle completes as the intentional sensing leap
  of faith.
- From the second cycle, verify an average at or below 0.1 A stops before the
  gate opens, shows the low-current fault, and clears RESTART across reboot.
- Establish one ignored cycle then five known-good cases. Verify a following
  cycle below 85% of the learned baseline stops before dropping.

## Profiles, information, and stability

- Create, rename, save, load, and delete profiles. Confirm time, mode,
  RESTART, and DUMP persist across a power cycle.
- Save an Analyse reference to a profile, load it, run a comparison case, and
  confirm the live/dropping/reloading comparison graph and PERFORMANCE review.
- Confirm a DUMP-enabled profile loads into free-run with the feeder disabled
  and RESTART off.
- In INFO, scroll through LOW, BASE, 5V, and firmware version; confirm `BASE`
  stays blank until five accepted baseline cases exist.
- Inspect diagnostics after any reset.
- Run multiple continuous free-run and auto-feed cycles and watch for resets or
  unexpected output changes during annealing, dropping, and feeding.
