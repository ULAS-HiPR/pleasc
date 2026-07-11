# Pleasc Firmware

Safety-locked bringup for STM32F072 + TJA1051.

## Build

```sh
pio run -e stm32f072c8t6
pio run -e stm32f072c8t6_rev1_pyro
```

## Pin Map

- CAN RX/TX: PA11/PA12.
- Ch0 Drogue fire: PB3.
- Ch1 Main fire: PB4.
- Ch2 Aux1 fire: PB5.
- Ch3 Aux2 fire: PB6.
- Pyro master enable: PB7.
- Continuity inputs: PB10/PB11/PB12/PB13, active high.

Continuity active-high is inferred from the optocoupler transistor side: collector to 3.3 V, emitter to `*_SEAT`, 10 k pulldown.

## CAN

Uses Croi's current CAN IDs and payload structs:

- `PYRO_ARM` `0x050`
- `PYRO_FIRE` `0x060`
- `PYRO_STATUS` `0x200`
- `PYRO_ACK` `0x210`
- `HEARTBEAT` `0x420`

Guards:

- standard build is fire-locked.
- Rev1 live build requires explicit `REV1_ACCEPTED_RISK=1`.
- command tag binds command, channel/mask, sequence, and mission tag.
- rolling sequence rejects stale/replayed commands.
- first valid arm latches the mission tag for the boot.
- Croi heartbeat must remain fresh within 5 s.
- arm is accepted only in powered/coasting/drogue/main states.
- fire is accepted only in drogue/main states.
- channel must remain armed for at least 100 ms.
- fire requires channel armed.
- fire requires continuity present.
- each channel may fire once per boot.
- pulse is bounded to 250 ms by default.
- arm lease expires after 2 s without renewal.
- all fire outputs default low before GPIO mode changes.

## Required Before Firing Is Enabled

Rev1 requires an external RBF/pyro-power disconnect and the acceptance tests in `../../HIL_TEST_CAMPAIGN.md`. Never test the live image with energetic devices attached before inert-load acceptance.
