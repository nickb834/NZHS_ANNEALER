# Contributing

Changes are welcome through pull requests. This firmware controls high-current
induction-heating hardware, so a successful compile is only the first part of
validation.

## Before making a change

- Open or link an issue when the behaviour or scope needs discussion.
- Branch from the intended base branch; do not work directly on `master`.
- Keep commits focused and use short imperative commit messages.
- Avoid mixing firmware, generated artifacts and unrelated documentation
  changes unless they must change together.

## Build environment

Use the board packages and libraries listed in the
[README build instructions](README.md#build-and-upload). The sketch must remain
directly buildable in the Arduino IDE for the selected board; do not require a
special wrapper or test-only firmware build.

Before opening a pull request, build:

- `arduino:avr:uno`
- `arduino:renesas_uno:minima`
- `arduino:renesas_uno:unor4wifi`

GitHub Actions repeats these builds with pinned dependencies and enforces the
configured flash and SRAM ceilings.

## Firmware artifacts

The tracked Uno R3 HEX files and Uno R4 WiFi BIN must represent the exact source
tree being proposed. Follow the canonical artifact procedure in the
[README](README.md#arduino-cli); the R4 BIN must be built at the documented fixed
path because the R4 core embeds the build path.

For a deliberately R4-only change, compile the Uno R3 and confirm both R3 HEX
files remain byte-identical. Do not replace them merely to make the artifact
check pass. For a shared R3/R4 change, regenerate and commit every affected
tracked artifact.

## Validation

Use the pull-request template to distinguish what compiled from what was
actually tested. At minimum:

- Exercise shared menus and state-machine changes in SimulIDE where applicable.
- Start physical testing with the high-current output disconnected.
- Verify changed pins, timers, servo/feeder outputs, sensors and fault paths on
  every affected physical board.
- Repeat complete-machine tests before claiming that annealing, automatic
  feeding or safety behaviour is validated.
- List anything not tested instead of assuming CI covered it.

The full checklist is in [docs/TESTING.md](docs/TESTING.md). Only flash firmware
while the annealer is idle, not while annealing or in cooldown.

## Pull requests

- Keep the scope reviewable and explain any EEPROM or stored-data compatibility
  change.
- Describe effects on the annealing output, fan, feeder, drop gate, cooldown and
  current/temperature protection.
- Update user documentation and screenshots when controls or visible behaviour
  change.
- Wait for both required Firmware CI jobs to pass and resolve review
  conversations before merging.
- Use squash merge so `master` receives one coherent commit for the pull
  request.
