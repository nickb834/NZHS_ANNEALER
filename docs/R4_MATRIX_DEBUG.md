# Uno R4 WiFi matrix bench diagnostics

The Uno R4 WiFi LED matrix is hidden when the NZHS Annealer Shield is fitted.
The production firmware therefore leaves it uninitialised during normal use.
This avoids reserving another Renesas timer and running the matrix's 10 kHz
refresh interrupt when the display cannot be seen.

Matrix diagnostics use the normal production firmware. No special sketch or
build option is required. The feature is compiled only for the Uno R4 WiFi; it
adds no flash or RAM to Uno R3 or R4 Minima builds.

## Safety behaviour

Use matrix diagnostics only on a bare board or with the shield/high-current
output disconnected.

When diagnostics start, the firmware:

- forces D6 LOW;
- closes the drop gate;
- disables the feeder driver;
- blocks START;
- does not save the mode in EEPROM.

Reset the board to exit. If matrix timer allocation fails, heating remains
blocked until reset.

## Starting and navigating

1. Connect the Uno R4 WiFi over USB and open Serial Monitor at 115200 baud.
2. Wait until the firmware reaches the stopped state.
3. Send `M` to initialise the matrix and enter diagnostics.
4. Pages advance automatically every four seconds.
5. Send `M` or `N` to advance immediately.
6. Press RESET to exit and restore normal operation.

Every page change prints its identifier and full decoded values over Serial.

## Page legend

### `RDY` or flashing `ERR`

The four bottom-row indicators are, left to right:

1. feeder timer;
2. drop Servo;
3. watchdog;
4. EEPROM capacity.

`RDY` is steady when all four are ready. `ERR` flashes if any item failed; the
Serial line identifies which one.

### Reset cause

| Matrix | Meaning |
| --- | --- |
| `PWR` | Power-on reset |
| `WDT` | Watchdog reset |
| `BRN` | Brown-out reset |
| `EXT` | External, upload, or other reset |
| `UNK` | Cause unavailable |

### `OUT`

The six bottom-row indicators are, left to right:

1. D6 annealer output;
2. D7 cooling fan;
3. D10 gate/solenoid output;
4. D5 feeder enabled (active-low hardware converted to logical enabled);
5. D13 feeder direction;
6. feeder step activity.

In the normal safe idle state, only the direction indicator is expected to be
lit. Serial prints each logical value.

### `SNS`

The four bottom-row indicators are, left to right:

1. temperature device detected at boot;
2. current temperature reading valid;
3. current sensor detected;
4. profile reference loaded.

An unconnected A0 input can float and produce a false current-sensor indication;
use the Serial value and known bench wiring when interpreting this page.

### `TMP` and temperature value

The page first shows `TMP`, then alternates after two seconds to the rounded
DS18B20 reading, for example `25C`. Serial retains one decimal place:

```text
PAGE TMP: temperature=25.0C
```

Formatting rules:

- one- and two-digit positive values include `C`;
- a single-digit negative value displays like `-5C`;
- three-digit or two-digit-negative values omit `C` to fit;
- invalid or disconnected readings display `ERR`.

For the three-pin DS18B20 breakout used during development, connect it before
reset:

| Breakout | Uno R4 WiFi |
| --- | --- |
| `G` | GND |
| `R` | 5V |
| `Y` | D8 data |

The breakout must include a pull-up between `Y` and `R`; otherwise add 4.7 kOhm.

### `HBT`

`HBT` remains visible while one dot moves across the bottom row. The dot is
updated by the main loop. A frozen dot indicates a stalled loop; the page
restarting from its initial position indicates a reset.

## Expected Serial example

```text
MATRIX DEBUG ACTIVE - RESET TO EXIT
START IS BLOCKED; ANNEALER OUTPUT FORCED OFF
Pages: RDY/ERR, reset cause, OUT, SNS, TMP/value, HBT
PAGE RDY: feeder=Y servo=Y watchdog=Y eeprom=Y
PAGE EXT: reset=external/other
PAGE OUT: annealer=0 fan=0 gate=0 feeder_enabled=0 direction=1 stepping=0
PAGE SNS: temp_device=Y temp_valid=Y current=Y reference=N
PAGE TMP: temperature=25.0C
PAGE HBT: main-loop heartbeat
```

## Troubleshooting

- No matrix output: confirm an Uno R4 WiFi is selected and send `M` only after
  the stopped state is reached.
- Initialisation error over Serial: reset before attempting a normal run; timer
  allocation may be incomplete.
- Unexpected `CUR`/current indication: A0 may be floating when the shield is
  absent.
- `ERR` temperature: connect and power the DS18B20 before reset; confirm the
  data pull-up and D8 wiring.
- START remains blocked: this is intentional after matrix initialisation; reset
  the board to return to production operation.

