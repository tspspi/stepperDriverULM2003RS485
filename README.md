# Simple RS485 stepper controller for ULM2003 driver (ATMEGA328P)

This is a simple stepper controller for cheap steppers that are driven by
ULM2003 driver chips that is controlled by an RS485 communication interface.
RS485 is realized with an external MAX485 level shifter.

## Protocol

The protocol is documented in [doc/serialproto.md](/blob/master/doc/serialproto.md).
