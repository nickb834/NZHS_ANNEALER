# NZHS_ANNEALER
Arduino Uno source code for NZHS Annealer Shield

Annealer build and use guide
http://www.mgnz-makes.com/

## Version 4.0.0 operation and safety

Version 4.0.0 changes the stopped-screen controls. Test the firmware with the
machine attended before relying on any automatic operation.

### Stopped-screen controls

The left side of the OLED continues to show the anneal time. The normal
right-hand list remains `FAN`, operating mode, case count, and temperature.
MODE moves a visible selection through TIME, mode, `SETTINGS >`, `PROFILES >`,
`INFO >`, and `DIAGNOSTICS>`; the list scrolls upward to reveal each menu
item. The top-right row always shows `FAN ON` or `FAN OFF` in the normal view.

| Button | Stopped-screen action |
| --- | --- |
| MODE | Select the next item: `TIME`, `MODE`, `SETTINGS >`, `PROFILES >`, `INFO >`, or `DIAGNOSTICS>` |
| UP | Change TIME or MODE, or enter the selected menu |
| START | Start the selected operating mode |

When TIME is selected, UP normally advances by 0.1 seconds. After five
consecutive presses no more than one second apart, it advances by 0.5 seconds;
pause briefly to return to 0.1-second adjustment. Holding UP still resets TIME
to 2.0 seconds.

`RESTART` is a user-selected setting and is honoured at boot, before the first
anneal-cycle current measurement.

| Selected item | UP action |
| --- | --- |
| TIME | Add 0.1 seconds; wrap from 8.0 s to 2.0 s |
| MODE | Cycle single-shot, free-run, and auto-feed |
| SETTINGS > | Open the Restart/Dump settings screen |
| PROFILES > | Open the cartridge-profile selector |
| INFO > | Open the low-current guard and learned-baseline screen |
| DIAGNOSTICS> | Open the sensor and reset-diagnostics screen |

Within **Settings**, MODE selects `RESTART`, `DUMP`, or `BACK >`; UP toggles
the selected setting or returns at `BACK >`. Diagnostics displays a visible
highlighted `BACK >`; UP returns to the stopped screen. Info keeps `BACK >`
visible while MODE scrolls its detail rows; UP returns to the stopped screen.
START does not leave menus. Every `BACK >` returns to the home-list item that
opened it, while `LOAD >` deliberately returns to `TIME`, ready to run.

On the first startup, press START once to display the existing case-height and
time warning, press it again to acknowledge the warning, then press START a
third time to begin the first anneal cycle.

### Operating modes and dump button

- **Single-shot:** anneal and drop one case, then stop.
- **Free-run:** anneal and drop a case, then show the five-second `LOAD`
  countdown for manual loading before the next cycle.
- **Auto-feed:** use the stepper feeder to run continuously.

`DUMP: ON` selects free-run and disables the feeder. It also forces `RESTART`
off, because manual case handling must not resume unattended after cooldown.
Conversely, enabling `RESTART` turns `DUMP` off. While an active free-run cycle
is in progress, hold MODE to open the drop gate and release it to close the
gate. Changing to another operating mode turns `DUMP` off.

During annealing, the OLED retains its original full-size current and remaining
time layout. During the dropping interval, it shows a full-size `DROPPING`
message and the live temperature in the lower row.

### Cartridge profiles

`PROFILES >` stores up to eight named cartridge profiles in EEPROM. A profile
contains anneal time, operating mode, automatic-restart preference, and the
dump-button preference. The learned current baseline is deliberately not
saved: it is specific to the brass being processed and is rebuilt for each
run.

All profile slots are blank on first use, but the selector labels them `PROFILE 1` through `PROFILE 8` until they are
renamed. MODE selects profile slots 1–8 or `BACK >`; UP opens the selected slot. In the
profile action menu, MODE selects `LOAD`, `SAVE`, `RENAME`, `DELETE`, or
`BACK >`; UP performs the highlighted action. Saving an empty slot creates
`PROFILE 1`–`PROFILE 8` and opens the name editor. In the editor, UP cycles the
selected character through `A-Z`, `0-9`, `-`, `.`, and space; MODE moves the
cursor through every character and then the visible `SAVE >` item; UP at
`SAVE >` saves and briefly shows `SAVED`. Saving an existing profile also
briefly shows `SAVED`. One more MODE press selects `BACK >`; UP there discards name
changes. While a name character is selected, holding UP for one second starts
automatic character cycling; a normal UP press still
advances only one character. Delete confirmation has visible `DELETE >` and `BACK >` items; MODE
selects either and UP performs the selected action. START does not leave menus:
every menu exits through its visible `BACK >` item and UP.

Loading a profile applies all settings together and also updates the normal
saved settings. If `DUMP` is saved on, loading preserves the existing safety
rule: dump mode forces free-run and keeps the feeder disabled.

When cooldown begins, START cancels a pending automatic restart and returns to
the normal home/menu UI. Settings, profiles, information, and diagnostics
remain available while cooling. The temperature is suffixed with `COOL!`, and
START cannot begin another run until it drops below the 40 °C hysteresis
threshold.

### Diagnostics and information

The diagnostics screen retains the normal left-side time panel and divider.
Its right-side rows show the temperature-sensor count/current status,
`RST:<cause>|<state>` best-effort reset record, and `BACK >`. The separate
Info screen shows `LOW: 85% N5`, the configured 85% threshold and five-case
baseline window, plus the learned `BASE` current in amps. MODE scrolls
upward through the estimated regulated `5V` rail and firmware version while
keeping `BACK >` visible. The `5V` value is derived from the Uno's nominal
1.1 V internal band-gap and is best used for trend monitoring unless
calibrated. `BASE: --` is shown until five accepted cases establish the
baseline. Reset causes are
`W` watchdog, `B` brown-out, `E` external reset, and `P` power-on; the Uno
bootloader may clear the cause before the firmware reads it. The final state is
`A` annealing, `D` dropping, `R` reloading, `C` cooldown, or `S` stopped. With
`DEBUG` enabled, the full reset flags
and previous state are also printed to serial at 115200 baud during boot.

### Automatic cooldown restart

Automatic restart is stored across power cycles and is honoured before the
first current measurement. Cooldown starts above 55 C and ends below 40 C.

- Free-run returns to its normal `LOAD` countdown after cooldown.
- Auto-feed resumes its automatic sequence after cooldown.
- Single-shot does not automatically restart.
- START during cooldown cancels a pending automatic restart.
- An invalid or missing temperature reading immediately stops annealing,
  cancels a pending restart, keeps the cooling fan on, and shows the
  `TEMP ERROR` / `CHECK TEMP` fault. A valid reading and MODE acknowledgement
  are required before another run; the saved `RESTART` preference is retained.
- START from stopped enters cooldown rather than annealing if a valid reading
  is still above 55 C.

### Low-current guard

The existing A0 mid-rail check continues to detect current-sensor presence for
the display and over-current protection. The first anneal cycle is a deliberate
current-sensing leap of faith; from the second cycle onward, every average must
exceed 0.1 A. A cycle at or below that floor stops before opening the gate,
displays the low-current fault, and clears the saved `RESTART` preference. The
cooldown screen always shows `AUTO ON` when a restart is pending, otherwise
`AUTO OFF`.

After verification, the first anneal cycle of each run is ignored. The next
five accepted anneal-cycle averages establish a moving normal-current window.
After that, the first cycle average below 85% of that window's average stops
the machine before that suspected case is dropped and displays `CHECK CASE` /
`CURRENT LO!`. Suspected low-current cycles are not added to the window, so an
empty run cannot progressively lower the reference value.

This guard helps detect an empty coil or an out-of-cases condition. It cannot
detect every mechanical jam: a case that remains in the coil may still draw a
normal current. A MODE press acknowledges the fault and returns to stopped;
starting a new run resets the learned baseline.

### Hardware validation checklist

Before unattended use or publication of a release:

- Compile for the Arduino Uno with the required Adafruit, DallasTemperature,
  and OneWire libraries; record the compiler's program-storage and dynamic-RAM
  report.
- Verify stopped-screen navigation: TIME → MODE → SETTINGS → PROFILES → INFO →
  DIAGNOSTICS → TIME. At each menu item, press UP and confirm the intended
  submenu opens; confirm Settings `BACK >`, Diagnostics `BACK >`, and Info
  UP/START each return to the normal stopped screen.
- In Info, press MODE through all three rows and confirm `LOW`, `BASE`, `5V`,
  and firmware version appear in order while `BACK >` remains visible. Confirm
  the 5 V value refreshes no more frequently than once per second.
- Verify the first no-current cycle completes as the intentional sensing leap
  of faith, then confirm the second cycle at or below 0.1 A stops before
  dropping, shows the low-current fault, and `RESTART` remains OFF after
  reboot; the other current-sensor displays and protections retain their
  existing A0 detection behaviour.
- Verify restart preference persistence with a current sensor fitted.
- Exercise free-run, auto-feed, dump OFF, and dump ON behavior.
- Save, rename, load, and delete a profile. Confirm loaded time, mode,
  restart, and dump settings survive a power cycle; confirm a dump-enabled
  profile loads in free-run with the feeder disabled.
- Verify cooldown entry above 55 C, restart only below 40 C, and START cancellation.
- Disconnect the temperature sensor while stopped and while annealing: verify
  `TEMP ERROR` / `CHECK TEMP`, annealer-off behavior, fan operation, and that
  a valid reading plus MODE acknowledgement is required to continue.
- With a valid temperature above 55 C, press START from stopped and confirm
  that the firmware enters cooldown rather than annealing.
- Confirm one ignored cycle followed by five known-good cycles establishes a
  representative current baseline, then validate the low-current fault using a
  safe test method.
- Generate and test fresh `.hex` firmware artifacts before distributing them.

SW version 4.0.0
- Added a scrolling stopped-screen menu: TIME and operating mode retain their original direct adjustment, while Restart/Dump, Diagnostics, and Info are opened through visible menu entries
- Automatic restart is saved across power cycles and resumes free-run or auto-feed after the existing cooldown threshold is reached; START cancels a pending restart during cooldown
- Automatic restart is honoured before the first anneal-cycle current measurement, then any saved restart preference is cleared when no current is detected
- Uses the existing A0 current-sensor detection for current monitoring; independently clears automatic restart after an anneal-cycle average at or below 0.1 A, then stops before the first cycle below 85% of the accepted five-cycle moving baseline is dropped
- Added a persisted dump-button setting: enabling it selects free-run, and MODE opens the drop gate while active in free-run
- Added a visible firmware and sensor diagnostics screen

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
