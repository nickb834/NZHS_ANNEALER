# Hardware validation checklist

Use this checklist before unattended use or publishing a firmware release.

## Build and artifacts

- Compile for an Arduino Uno with Adafruit SSD1306, Adafruit GFX, OneWire, and
  DallasTemperature installed.
- Record the compiler flash and RAM report.
- Regenerate and test both tracked HEX artifacts from the exact source commit.

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
- Confirm a DUMP-enabled profile loads into free-run with the feeder disabled
  and RESTART off.
- In INFO, scroll through LOW, BASE, 5V, and firmware version; confirm `BASE`
  stays blank until five accepted baseline cases exist.
- Inspect diagnostics after any reset.
- Run multiple continuous free-run and auto-feed cycles and watch for resets or
  unexpected output changes during annealing, dropping, and feeding.
