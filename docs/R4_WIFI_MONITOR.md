# Uno R4 WiFi read-only monitor

The Uno R4 WiFi build contains an experimental local browser monitor. It uses
the normal production sketch and is compiled out of Uno R3 and R4 Minima
builds.

The first implementation is intentionally read-only. It cannot start an
anneal, operate the gate or feeder, change a profile or setting, upload a file,
or update firmware.

## Starting the monitor

1. Boot the Uno R4 WiFi and wait for the stopped screen.
2. Open Serial Monitor at 115200 baud.
3. Send `W`.
4. Connect the viewing device to the open `NZHS-Annealer` WiFi network.
5. Open the address printed over Serial. The default is normally
   `http://192.168.4.1`.

Expected Serial output:

```text
WIFI MONITOR: STARTING OPEN READ-ONLY ACCESS POINT
WIFI MONITOR ACTIVE - RESET TO EXIT
SSID: NZHS-Annealer (open, read-only)
Open http://192.168.4.1
```

The monitor is non-persistent. Resetting the board turns it off. Send `W` again
after reset when it is needed.

## Displayed data

The browser page polls the firmware without reloading and displays:

- state and operating mode;
- current and capacitor temperature when their sensors are available;
- accumulated input-energy estimate and peak current when a curve is active or
  retained;
- session case count and current state time remaining;
- profile number, match percentage and energy percentage when available;
- cooldown-lock and fault state;
- the 0-8 second actual-current curve and active profile reference curve.

The API uses compact 0-250 graph samples, corresponding to 0-12.5 A in 50 mA
steps. The browser expands them to the labelled current scale.

## HTTP endpoints

| Endpoint | Content |
| --- | --- |
| `/` | Embedded read-only dashboard |
| `/api/status` | Current state and aggregate values as JSON |
| `/api/curve` | Actual and reference graph samples as JSON |

The browser polls status every 500 ms and curves every second. Requests are
handled from the main loop with a fixed request buffer; there is no dynamic
request `String` and no waiting loop for a slow client.

## Safety and network scope

- The access point is open because this version exposes no control operations.
- Treat all displayed values as monitoring aids, not independent safety
  interlocks. The physical firmware safety paths remain authoritative.
- Do not expose the access point through an Internet router or bridge.
- Matrix diagnostics and the WiFi monitor are mutually exclusive. Reset before
  changing between them.
- WiFi startup is accepted only while the annealer is stopped. Once active, it
  can remain available while the physical controls operate the annealer.
- Network handling does not run in the feeder timer callback or current-sample
  function. Physical testing must still confirm that browser traffic does not
  disturb 25 ms Analyse sampling or feeder timing.

## Current limitations

- There is no password, station-mode network configuration, hostname discovery,
  HTTPS, authentication, or user management.
- There is no WiFi on/off setting in the OLED menus and no EEPROM persistence.
- The page has not yet been tested across a large set of phones and browsers.
- SimulIDE cannot emulate the Uno R4 WiFi radio; use a physical R4 WiFi.
- The current profile number reflects the firmware's selected profile slot; a
  future revision may track a separately named loaded-profile identity.

