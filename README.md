# Simple RS485 stepper controller for ULM2003 driver (ATMEGA328P)

This is a simple stepper controller for cheap steppers that are driven by
ULM2003 driver chips that is controlled by an RS485 communication interface.
RS485 is realized with an external MAX485 level shifter.

## Wiring

### Steppers

| Port and pin | Connection                       |
| ------------ | -------------------------------- |
| PB0          | Stepper 1, In 2                  |
| PB1          | Stepper 1, In 1                  |
| PD6          | stepper 1, In 4                  |
| PD7          | Stepper 1, In 3                  |
| PB2          | Stepper 2, In 1                  |
| PB3          | Stepper 2, In 2                  |
| PB4          | Stepper 2, In 3                  |
| PB5          | Stepper 2, In 4                  |

### RS485

| Port and pin | Connection                       |
| ------------ | -------------------------------- |
| PD0          | MCU TX: MAX485 DI                |
| PD1          | MCU RX: MAX485 RO                |
| PD3          | Data direction: MAX485 DE and RE |

## Protocol

The protocol is documented in ```doc/serialproto.md```

Currently supported / tested commands:

| Command        | Status      |
| -------------  | ----------- |
| Identify       | ok          |
| Get boundaries | implemented |
| Set boundaries | implemented |
| Get position   | implemented |
| Set position   | ok          |
| Get speed      | implemented |
| Set speed      | ok          |
| Get status     | ok          |
| Set cur. pos.  | implemented |

## License

The license can be found in ```LICENSE.md```. It's a simple BSD style license.
