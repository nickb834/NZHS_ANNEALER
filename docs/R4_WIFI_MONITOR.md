# Uno R4 WiFi read-only monitor

The Uno R4 WiFi build contains an experimental local browser monitor. It uses
the normal production sketch and is compiled out of Uno R3 and R4 Minima
builds.

The monitor is intentionally read-only. It cannot start an anneal, operate the
gate or feeder, change a profile or annealing setting, upload a file, or update
firmware.

## Configure the normal WiFi network

1. Boot the Uno R4 WiFi and wait for the stopped screen.
2. Use MODE to select `SETTINGS >`, then press UP.
3. Use MODE to select `WIFI >`, then press UP.
4. Select `SETUP >` and press UP. The row changes to `SETUP: ACTIVE`.
5. Connect a phone or computer to the open `NZHS-Annealer-Setup` network.
6. Open `http://192.168.4.1`.
7. Enter the normal network name and password, then select **Save and
   connect**.
8. Reconnect the phone or computer to the normal network.
9. Open the annealer's Info menu and scroll to `WIFI: LAN` and `IP:`. Browse to
   that IP address.

The setup page enables the monitor when credentials are saved. The
`MONITOR: ON/OFF` item controls whether the saved network is joined after
future resets. Turning it off retains the credentials but stops the radio.

To erase the saved network rather than replace it, select `RESET >`, then move
from the default `BACK >` choice to `CONFIRM >` and press UP. This clears the
SSID and password, disables monitoring and stops the radio.

Credentials are stored unencrypted in the R4 EEPROM-backed storage at the
unused tail beginning at address 768. They are protected against accidental
partial/corrupt reads by a magic value, format version and checksum; this is
integrity checking, not encryption.

### Bare-board setup without the OLED or buttons

The same production firmware supports a bench-only Serial entry path. Open
Serial Monitor at 115200 baud, wait until the firmware reaches its stopped
state, then send `S`. This starts `NZHS-Annealer-Setup`; continue from step 5
above. No separate test build is required, and the browser saves exactly the
same persistent configuration used by the OLED menu.

Send `I` at any time to print the current WiFi mode and usable address over
USB. The firmware does not announce LAN readiness until both `WL_CONNECTED`
and a non-zero DHCP address are present, so this also works when the development
computer is isolated from the device LAN by a VPN.

Send `X` while stopped to perform the same guarded reset operation without the
OLED/buttons. It erases the saved SSID/password, disables monitoring and prints
the resulting OFF status. Send `S` afterwards to configure a different network.

Send `O` while stopped to stop the WiFi runtime without changing the saved
credentials or persistent MONITOR setting. This is useful when comparing bench
timing with and without network activity; it is not required for matrix mode.
Resetting reconnects the saved network.

Send `B` to toggle raw START/MODE/UP transition reporting during normal
operation. Sending `B` again turns the optional trace off. This reporting does
not block or alter normal button actions; use matrix mode when START must remain
safely blocked. Matrix diagnostics always report button transitions regardless
of the optional `B` setting.

## Connection and fallback behaviour

- LAN connection runs asynchronously; the firmware does not wait in a blocking
  connection loop.
- A successful connection starts the monitor and prints its LAN address over
  Serial only after DHCP supplies a non-zero address.
- If the saved network cannot be reached within 15 seconds,
  `NZHS-Annealer-Setup` starts again so the credentials can be corrected.
- A lost LAN connection is retried automatically.
- Connection, reconnection and access-point transitions are deferred while the
  firmware is annealing, analysing, dropping a case or reloading the feeder.
- `WIFI: CONNECT`, `WIFI: LAN`, `WIFI: SETUP AP`, `WIFI: DIRECT AP`,
  `WIFI: OFF` and `WIFI: ERROR` describe the current state in Info.
- The next Info row shows the active IP address or `--` when unavailable.

Because the fallback setup network is open, anyone within radio range could
replace the stored credentials while it is active. Turn `MONITOR` off when WiFi
is not required and do not expose or bridge the monitor to the Internet.

## Direct monitor access point

For temporary bench use, send `W` at 115200 baud while the firmware is stopped.
This starts the original open `NZHS-Annealer` read-only access point without
changing the persistent monitor setting or credentials. Its default address is
normally `http://192.168.4.1`; reset exits this bench-only mode.

Matrix diagnostics may run alongside the direct AP or persistent LAN monitor.
The matrix remains a run-blocking bench mode: D6 is forced LOW, the feeder is
disabled and START remains blocked even though the dashboard is reachable.

## Displayed data

The browser page polls the firmware without reloading and displays:

- state and operating mode;
- current and capacitor temperature when their sensors are available;
- accumulated 80%-efficiency energy estimate and peak current when a curve is
  active or retained;
- session case count and current state time remaining;
- profile number, match percentage and energy percentage when available;
- cooldown-lock and fault state;
- the 0-8 second actual-current curve and active profile reference curve.

The API uses compact 0-250 graph samples, corresponding to 0-12.5 A in 50 mA
steps. The browser expands them to the labelled current scale.

## Session history and CSV

The R4 retains the latest 16 completed current traces in RAM, newest first.
Each row shows its sequence number, stop reason, elapsed time, peak current,
estimated energy and profile match when available. Select the sequence
number to replace the live graph with that retained trace; select **LIVE** to
resume the current graph. Each row also provides a CSV download containing its
summary and 0-8 second current samples.

Records include:

- completed and user-aborted Analyse runs;
- Analyse or normal-run overcurrent stops when a graph exists;
- normal profile-reference comparisons, even if WiFi is temporarily off;
- ordinary TIME, ENERGY and PEAK DROP runs when WiFi monitoring (persistent or
  direct AP) is enabled and a current sensor is present;
- low-current and target-timeout results when a graph was being captured.

History is deliberately session-only: it clears on reset and does not write to
EEPROM. Ordinary timed runs use the existing 25 ms sampler while history is
enabled, allowing energy and curve data to be retained without changing the
anneal output deadline. Status/history JSON and CSV retain raw input energy
alongside the 80% estimate.

## HTTP endpoints

| Endpoint | Content |
| --- | --- |
| `/` | Setup form while the setup AP is active; otherwise the read-only dashboard |
| `/setup` | Setup form while the setup AP is active |
| `POST /setup/save` | Save credentials and schedule a LAN connection |
| `/api/status` | Current state, network state and aggregate values as JSON |
| `/api/curve` | Actual and reference graph samples as JSON |
| `/api/history` | Up to 16 retained result summaries as JSON |
| `/api/history/<id>` | One retained actual/reference graph as JSON |
| `/history/<id>.csv` | Download one retained trace and summary as CSV |

`/api/status` reports the primary 80% estimate as `energy_j`, raw electrical
input as `input_energy_j`, and the applied factor as
`energy_efficiency_pct`. It also reports the central configured supply value as
`supply_voltage_mV`. History summaries use explicit `input_energy_mj` and
`estimated_energy_mj` fields; CSV exports include both, the factor and the
configured voltage.

The browser polls status every 500 ms and curves every second. Requests use a
fixed 1536-byte buffer and bounded per-loop reads; the firmware does not use a
dynamic request `String` or wait in a client loop. Oversized and malformed
requests fail with HTTP 400.

## Home Screen shortcut and icons

The dashboard advertises and serves:

- a multi-size `/favicon.ico` for browser tabs and bookmarks;
- a 180x180 `/apple-touch-icon.png` for iOS Web Clips;
- 192x192 and 512x512 PNG icons through `/manifest.webmanifest`;
- standalone-display and dark theme metadata.

On iOS, open the monitor in Safari, use **Share**, then **Add to Home Screen**.
The icon is derived from the MGNZ Makes maker mark with a simplified induction
coil ring and no small text. The source and generated sizes are retained in
[`docs/web-icons/`](web-icons/).

The shortcut records the current numeric IP address. Configure a DHCP
reservation for the R4 if the shortcut must survive router address changes.

## Safety and network scope

- Treat all displayed values as monitoring aids, not independent safety
  interlocks. The physical firmware safety paths remain authoritative.
- There is no remote actuator or configuration API apart from submitting WiFi
  credentials while the setup AP is active.
- WiFi setup can only be entered through the physical OLED controls while the
  firmware is stopped.
- Network handling does not run in the feeder timer callback or current-sample
  function.
- Physical testing must confirm that dashboard traffic does not disturb 25 ms
  Analyse sampling, feeder timing, current trips or temperature safety.

## Current limitations

- Credentials are stored in plaintext and the setup AP is open.
- There is no hostname discovery, HTTPS, authentication or user management.
- There is no browser facility to clear credentials; use the physical OLED
  reset confirmation or stopped-state Serial `X` command.
- Session history is RAM-only and clears whenever the R4 resets.
- SimulIDE cannot emulate the Uno R4 WiFi radio; use a physical R4 WiFi.
- The current profile number reflects the firmware's selected profile slot; a
  future revision may track a separately named loaded-profile identity.
