# Simple RS485 stepper controller for ULM2003 driver (ATMEGA328P)

This is a simple stepper controller for cheap steppers that are driven by
ULM2003 driver chips that is controlled by an RS485 communication interface.
RS485 is realized with an external MAX485 level shifter.

## Protocol

The protocol is documented in ```doc/serialproto.md```

Currently supported / tested commands:

| Command        | Status      |
| -------------  | ----------- |
| Identify       | ok          |
| Get boundaries |             |
| Set boundaries | implemented |
| Get position   |             |
| Set position   | ok          |
| Get speed      |             |
| Set speed      | ok          |
| Get status     | ok          |
| Set cur. pos.  | implemented |


## License

The license can be found in ```LICENSE.md```. It's a simple BSD style license.
