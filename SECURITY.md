# Security and safety policy

This project controls induction-heating hardware capable of hazardous voltage,
current and temperature. Firmware safeguards supplement correctly designed,
earthed and tested hardware; they are not a substitute for electrical
protection or attended commissioning.

## Reporting a vulnerability

Do not publish credentials, exploitable details or a safety bypass in a normal
issue. Use GitHub's **Security → Report a vulnerability** option for this
repository to start a private security advisory. If that option is unavailable,
open an issue containing only a request for private contact and no sensitive
details.

Include the affected board and firmware commit, hardware configuration, impact,
reproduction conditions and any safe evidence available. Remove WiFi passwords,
network details and other secrets from logs and screenshots.

Ordinary reproducible firmware bugs and completed hardware-validation results
can use the repository issue forms. Make the equipment electrically safe before
inspection or rewiring.

## Supported versions

This fork is maintained on a best-effort basis. Security and safety fixes target
the latest code on the default branch; older firmware may require upgrading.

## Safety-sensitive changes

Changes involving remote control, annealing output, automatic feeding, the drop
gate, timers, watchdogs, EEPROM safety state, or current/temperature protection
require explicit safety review and physical hardware validation.

The R4 WiFi interface is intentionally read-only. Adding browser or network
control of START, profiles, the relay, feeder or gate is outside that safety
boundary and must be proposed and reviewed separately before implementation.

Never energise the high-current hardware solely to reproduce a software report.
Begin with the high-current output disconnected and follow
[docs/TESTING.md](docs/TESTING.md) before complete-machine testing.
