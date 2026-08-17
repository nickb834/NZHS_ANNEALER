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

The matrix page legend and bench procedure are documented in
[R4_MATRIX_DEBUG.md](R4_MATRIX_DEBUG.md). WiFi monitor operation is documented
in [R4_WIFI_MONITOR.md](R4_WIFI_MONITOR.md).

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
- Confirm Analyse serial records are visible at 115200 baud through the R4
  WiFi board's normal USB/ESP32-S3 bridge.
- Confirm a forced timer, servo, or watchdog initialisation failure blocks a run
  with `R4 HW` / `INIT ERR` and identifies the failed backend over serial.
- Confirm the onboard LED matrix remains dark and uninitialised during normal
  operation with the shield fitted.
- With the shield/high-current output disconnected and firmware stopped, send
  `M` at 115200 baud. Confirm matrix debug starts, D6 remains LOW, the gate is
  closed, the feeder is disabled, and START is blocked until reset.
- Confirm RDY/ERR, reset-cause, OUT, SNS, TMP/value, and HBT pages rotate every
  four seconds; confirm M or N advances one page immediately.
- Confirm each page change prints its three-letter identifier and full values
  over Serial. Force or simulate a platform failure and confirm `ERR` flashes.
- Confirm RDY bottom indicators match feeder timer, Servo, watchdog and EEPROM.
- Check OUT bottom indicators and Serial values against D6, D7, D10, D5, D13,
  and step activity.
- Connect the bench DS18B20 (`G` to GND, `R` to 5V, `Y` to D8) before reset and
  confirm the SNS bottom indicators and Serial output report a detected device
  and valid temperature.
- Confirm the TMP page alternates after two seconds to the rounded temperature
  (for example `50C`) and Serial reports the one-decimal reading. Disconnect or
  invalidate the sensor and confirm the value becomes `ERR`.
- Confirm HBT remains readable while its bottom-row dot moves continuously.
- Reset and confirm the matrix turns off, normal START behaviour returns, and
  no matrix timer remains allocated.
- On the R4 WiFi, confirm Settings scrolls through RESTART, DUMP, WIFI and BACK;
  confirm UP on WIFI opens MONITOR, SETUP and BACK and nested BACK returns to
  WIFI in Settings.
- With MONITOR off, select SETUP. Confirm `SETUP: ACTIVE`, connect to the open
  `NZHS-Annealer-Setup` AP, browse to `http://192.168.4.1`, and submit valid LAN
  credentials without exposing the password in Serial output.
- On a bare R4 without the shield/OLED/buttons, send `S` at 115200 baud while
  stopped and confirm it starts the same setup AP and saves the same persistent
  configuration without requiring a special firmware build.
- Send `I` before, during and after connection. Confirm it reports OFF,
  CONNECTING or LAN as applicable and never presents `0.0.0.0` as a usable IP.
- Send `O` while stopped and confirm WiFi stops while saved credentials remain;
  confirm matrix diagnostics work both before and after this command and reset
  reconnects the saved network.
- On the OLED, select WIFI `RESET >`; confirm the reset screen defaults to
  `BACK >`, BACK preserves credentials, and `CONFIRM >` clears credentials,
  disables monitoring and stops WiFi. Repeat on a bare board with stopped-state
  Serial `X`, then send `I` and confirm `WIFI STATUS: OFF IP=--`.
- Confirm the setup page acknowledges the save, the setup AP closes, the R4
  joins the normal network, and Info eventually displays `WIFI: LAN` and a
  reachable IP. Reset and confirm the connection and monitor-enabled setting
  persist.
- Turn MONITOR off and confirm the radio stops while the credentials remain;
  turn it on and confirm the saved LAN is reused without another setup.
- Submit deliberately invalid/unreachable credentials. After 15 seconds,
  confirm the open setup AP returns and valid replacement credentials can be
  saved. Confirm malformed or oversized HTTP setup requests fail closed.
- Disconnect the normal access point after a successful connection, restore it,
  and confirm reconnection. Repeat during Analyse or auto-feed operation and
  confirm reconfiguration is deferred until the timing-critical state ends.
- Reset, wait for the stopped screen, and send `W` at 115200 baud. Confirm the
  non-persistent open `NZHS-Annealer` direct monitor AP starts without changing
  the saved monitor setting or credentials.
- Connect a phone or computer and confirm `/`, `/api/status`, and `/api/curve`
  load. Confirm there are no browser controls for START, gate, feeder, settings,
  profiles, upload, or firmware update.
- Confirm `/favicon.ico`, `/apple-touch-icon.png`, `/icon-192.png`,
  `/icon-512.png`, and `/manifest.webmanifest` load with the expected content
  types. Add the dashboard to an iOS Home Screen and confirm the MGNZ-derived
  icon and standalone dark presentation are used.
- With history empty, confirm the dashboard and `/api/history` report no
  records. Complete Analyse, user-aborted Analyse, profiled and ordinary timed
  runs and confirm newest-first rows, stop reasons, elapsed time, peak, energy
  and optional match values.
- Select retained rows and LIVE repeatedly; confirm the graph changes without
  affecting the physical state machine. Download each CSV and verify its
  summary, sample times and current values against the corresponding graph and
  Analyse Serial output.
- Create more than 16 results and confirm the oldest record is replaced without
  heap growth or EEPROM writes. Reset and confirm history is empty.
- Exercise stopped, Analyse, normal anneal, dropping, reloading, cooldown and a
  fault state. Confirm state, sensor values, case count, remaining time and
  actual/reference curves update without stale live energy values.
- Generate browser traffic during Analyse and auto-feed operation. Compare the
  25 ms Serial sample timestamps, feeder timing and safety trips with WiFi off;
  confirm no material timing regression.
- With the shield fitted, repeat D6 safe boot, D9 gate, D5/D12/D13 feeder, A0
  current, D8 temperature, OLED/button and current/temperature fault checks
  while the dashboard and history polling are active. Confirm enclosure/shield
  placement still provides usable WiFi reception.
- With LAN monitoring active, send `M` and confirm matrix pages and the browser
  remain responsive together while D6 stays LOW, the feeder stays disabled and
  START remains blocked. Repeat by entering matrix mode first and starting the
  direct monitor with `W`. Reset and confirm matrix mode exits and persistent
  LAN monitoring reconnects.
- Only reconnect the high-current output after all safe-state, gate and feeder
  checks pass.

## Controls and menus

- Check TIME → MODE → SETTINGS → PROFILES → ANALYSE → INFO → DIAGNOSTICS → TIME
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

## Analyse and configuration

- Without a current sensor, confirm Analyse shows `CUR SENSOR` / `REQUIRED` and
  cannot energise the annealer.
- With a current sensor, select Analyse, confirm `LOAD CASE` / `PRESS START`,
  and verify START begins one attended eight-second capture.
- Confirm current is sampled at the 25 ms target rate, the graph fills from left
  to right, and live amps and joules use their final result positions.
- Confirm serial output contains `ANALYSE,START`, timestamped SAMPLE records,
  END with elapsed time/energy/peak current, and `ANALYSE,GATE_OPEN`.
- Confirm the annealer switches off at eight seconds, the gate opens for five
  seconds, the DUMPING message is brief, and the completed graph has `BACK >`.
- Hold MODE for at least 300 ms during a capture: confirm immediate annealer off,
  `reason=USER ABORT`, and the same five-second gate-open safety action.
- After a capture, confirm the scrolling menu contains NEW, REVIEW, CONFIG and
  BACK with no blank rows or unexpected START shortcut.
- In CONFIG, test TIME adjustment including rapid 0.5-second increments and the
  two-second reset; save and reload the resulting profile.
- Configure ENERGY with each four-digit editor position, including a leading
  zero such as `0300`, and verify it is stored as 300 J.
- Configure PEAK DROP across its percentage range and verify the separate MAX
  TIME safety limit is stored and enforced.
- Select every profile destination, save, and confirm the Analyse working copy
  is removed while the saved reference remains under profile PERFORMANCE.

## Profile references and performance

- Confirm every profile action list opens on LOAD, including an empty slot.
- Confirm valid LOAD shows `LOADED` then returns to TIME; empty LOAD shows
  `EMPTY`; SAVE and confirmed DELETE show their corresponding acknowledgement.
- Select PERFORMANCE before saving a reference and confirm `NO DATA`.
- Save an Analyse reference, power-cycle, reload the profile, and confirm its
  reference graph, peak current and total joules persist.
- Delete that profile and confirm its reference is invalidated.
- Run a case that closely follows the saved curve and confirm the solid case
  trace overlays the dotted reference with a high match percentage.
- Repeat with safely simulated higher and lower current. Confirm match decreases
  and the energy percentage moves above or below 100% in the expected direction.
- Confirm comparison is observational: TIME, ENERGY or PEAK DROP still controls
  when the annealer switches off.
- Confirm the completed graph remains visible during the existing DROP period.
- In auto-feed, confirm it remains visible during reload with a live
  `NEXT 2.0s` to `NEXT 0.1s` countdown and no added feeder delay.
- In free-run, confirm the normal manual LOAD countdown remains visible. In
  single-shot, confirm the latest result remains available under PERFORMANCE.
- From the second case, confirm a no-current/low-current safety fault takes
  precedence over performance reporting and prevents the gate opening.

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

## SimulIDE scope

- Use SimulIDE 1.1.0 SR2 only for the production Uno R3 HEX; do not treat it as
  an Uno R4 timer, watchdog, servo or virtual-EEPROM test.
- Confirm the normal tracked HEX is used with no simulator-specific firmware
  flags or alternate sketch.
- When testing saved references across simulator restarts or firmware reloads,
  use the MCU's Save EEPROM Data and Load EEPROM Data actions.
- Exercise all shared menu paths, profile acknowledgements, Analyse CONFIG,
  reference review and comparison screens before capturing documentation images.
