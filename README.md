# Pleasc

Pleasc is the pyro/deployment board in the Ogma stack. Its job is to keep high-current deployment switching hardware away from the core flight computer while still accepting deployment commands from the stack.

## Role In Ogma

- Provides pyro/deployment output hardware.
- Uses the common STM32F072 + TJA1051 CAN architecture.
- Receives deployment commands over CAN from the flight stack.
- Reports deployment status/acknowledgement frames back to the stack.

## Hardware Summary

Current hardware files include:

- STM32F072 MCU.
- TJA1051 CAN transceiver.
- Protected CAN bus input/output.
- Pyro connector section.
- Multiple MOSFET switching stages.
- Common Ogma local power architecture.

## Firmware State

Initial bringup firmware now exists under `firmware/`.

Implemented first slice:

- `ogma_board_identity` for SWD detection.
- continuity/status readout.
- guarded arm/fire command path with inert default and explicit Rev1 release target.
- CAN frame handling for `PYRO_ARM`, `PYRO_FIRE`, `PYRO_ACK`, and `PYRO_STATUS`.

## Ogma Console Support

Ogma Console can identify, build, flash, and read the Pleasc status block over SWD. It has no pyro-fire control.

## Safety Note

`stm32f072c8t6` is inert: arm/fire frames return `FIRE_LOCKED`. `stm32f072c8t6_rev1_pyro` requires `REV1_ACCEPTED_RISK=1` and can fire only after a fresh Croi heartbeat, valid flight state, mission-bound sequenced arm command, 100 ms arm settle, continuity, and a fresh fire command.

Rev1 has no MCU-readable arm input. External RBF/pyro-power disconnect is therefore mandatory. The command tag prevents accidental cross-talk; it is not cryptographic security against hostile firmware on CAN.

Status reports release gate, armed/continuity/fired masks, mission tag, sequence, Croi state, and rejection counters.

## Dependency Lock

Use the exact shared-library pins in `../dependencies.lock.json`:

- `comheadan`: `ogma/flight-hardening`

Ogma Console doctor fails a board when these submodule SHAs do not match the lock file.
