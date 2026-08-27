## What changed

Describe the purpose and behaviour of this change.

## Affected targets

- [ ] Uno R3 / ATmega328P
- [ ] Uno R4 Minima
- [ ] Uno R4 WiFi
- [ ] SimulIDE model
- [ ] Documentation only

## Areas affected

- [ ] Pin assignments, timers or interrupts
- [ ] EEPROM layout or saved settings
- [ ] Annealing, relay, feeder, fan or drop-gate outputs
- [ ] Temperature or current safety behaviour
- [ ] OLED, LED matrix or button controls
- [ ] WiFi dashboard or serial protocol
- [ ] Firmware artifacts
- [ ] None of the above

Explain any checked item that could affect existing hardware or stored data.

## Validation

- [ ] Firmware CI passes for Uno R3, R4 Minima and R4 WiFi
- [ ] Flash and SRAM remain within the configured limits
- [ ] Tracked R3 HEX and R4 WiFi BIN artifacts were regenerated if required
- [ ] SimulIDE behaviour was checked, where applicable
- [ ] OLED, buttons and outputs were checked on hardware, where applicable
- [ ] Temperature and current safety paths were checked, where applicable
- [ ] WiFi dashboard and API were checked, where applicable
- [ ] Documentation and screenshots were updated, where applicable

Hardware or simulator tested:

<!-- State the board, shield, sensors and firmware commit used. -->

## Safety and compatibility

Describe any effect on automatic operation, high-current output, cooldown,
current detection, case feeding or the drop gate. State explicitly if there is
no safety-related change.

## Not tested or still uncertain

List anything reviewers should not assume has been validated.
