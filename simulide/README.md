# SimulIDE harness

`nzhs-annealer.sim1` targets **SimulIDE 1.1.0 SR2** on macOS. Download
SimulIDE 1 from the [official SimulIDE downloads page](https://simulide.com/p/downloads/).
This is a software-only harness for exercising the Uno firmware's UI and
state-machine paths. It loads the tracked standard Uno HEX using a relative
path: it is deliberately not a simulator-specific firmware build.

![SimulIDE Uno harness](nzhs-annealer-simulide.png)

## What is connected

| Harness control | Firmware connection | Use |
| --- | --- | --- |
| `START` push button | D2 | Start/stop and acknowledge the first-start warning |
| `MODE` push button | D3 | Move the visible selection or a selected menu item |
| `UP` push button | A2 | Enter, toggle, edit, or confirm the selected item |
| `CURRENT 0-12.5A A0` potentiometer | A0 | Simulated current-sensor signal; full travel represents approximately 12.5 A from its boot-time centre position |
| `CASE TEMP (D8)` DS18B20 | D8 | Use its `+` / `-` controls to change simulated temperature; the harness includes its required 4.7 kΩ one-wire pull-up to 5 V |
| `ANALYSIS SERIAL (115200)` terminal | D1 (TX) | Open the terminal to inspect `Serial.print` analysis output from the production firmware |
| OLED | A4 (SDA), A5 (SCL) | Inspect all rendered UI states |
| `ANNEAL D6`, `FAN D7`, `GATE D10` LEDs | matching Uno outputs | Observe logical output changes |
| `FEED EN D5`, `FEED DIR D13`, `FEED STEP D12` LEDs | matching Uno outputs | Observe the three STEP/DIR/ENABLE signals sent to the external feeder driver |

The three buttons use the firmware's `INPUT_PULLUP` configuration: pressing a
button grounds its input.

## Starting a simulation

1. Build the normal firmware and refresh its tracked standard HEX whenever the
   firmware changes. The simulator must use this exact production artifact:

   ```sh
   arduino-cli compile --fqbn arduino:avr:uno --export-binaries NZHS_ANNEALER_128x32_OLED
   cp NZHS_ANNEALER_128x32_OLED/build/arduino.avr.uno/NZHS_ANNEALER_128x32_OLED.ino.hex \
      NZHS_ANNEALER_128x32_OLED/NZHS_ANNEALER_128x32_OLED.ino.standard.hex
   ```

2. Open `simulide/nzhs-annealer.sim1` in SimulIDE 1.1.0 SR2 and start the
   circuit. The `CASE TEMP (D8)` DS18B20 starts at 50 °C; its visible `+` /
   `-` controls change the simulated temperature by 1 °C per press.
   Open `ANALYSIS SERIAL (115200)` to view firmware serial output; it is
   connected to the Uno's D1/TX UART output, equivalent to viewing that data
   through the physical Uno's USB serial port.
3. Keep `CURRENT 0-12.5A A0` centred during boot. The firmware measures that
   position as its ACS712 zero-current offset. A 10 kΩ series resistor narrows
   the simulation input to approximately 12.5 A across either half of the
   potentiometer travel, including the 12.3 A over-current threshold. Move it
   after boot to imitate current.

## Recommended checks

- Navigate all home and submenu paths; confirm each visible `BACK >` returns
  to the originating home selection.
- Create, rename, load, and delete profiles, including the one-second
  `SAVED` acknowledgement.
- Use the potentiometer to create an accepted-current baseline and then a
  lower reading to exercise the low-current stop path.
- Observe the LEDs during annealing, dropping, cooldown, and free-run reload.
- During an automatic feeder move, `FEED DIR D13` shows the selected direction
  and `FEED STEP D12` flashes with the step train. The feeder enable is
  active-low, so `FEED EN D5` is dark while the driver is enabled (`LOW=RUN`).

## Deliberate limits

This harness does **not** simulate the real ZVS/coil, PSU, ACS712 electrical
characteristics, gate mechanics, or feeder mechanics. Hardware testing remains
required for electrical safety, feeder/gate timing, and reset/noise behaviour.
