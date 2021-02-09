# Serial protocol

## Introduction

Since RS485 is a multi device bus with a single master all commands are prefixed
by an address as well the command length. Note these are not compatible with the
Thyracont devices so they cannot share the same RS485 bus unfortunately.

Requests are always initiated by the master. They are _always_ prefixed by a
single byte address byte as well as a single byte length field. The length
does not include the address as well as the length field.

The bus operates at a baud rate of 57600.

All multi-byte integers are encoded in _little endian_ format.

## Overview

All commands and responses are sent via RS485. The device is usually in listening mode

| Code | Command        | Arguments      | Description                                                                                     |
| ---- | -------------- |--------------- | ----------------------------------------------------------------------------------------------- |
| 0x00 | Identify       |                | Returns a 16 byte UUID as well as 2 bytes identifying firmware version and revision             |
| 0x01 | Get boundaries |                | Returns the minimum and maximum position for each channel                                       |
| 0x02 | Set boundaries | 4 boundaries   | Sets the boundaries for both channels                                                           |
| 0x03 | Get position   |                | Gets the current position                                                                       |
| 0x04 | Set position   | 2 positions    | Sets the position TARGET for both motors. They start moving on the next tick towards the target |
| 0x05 | Get speed      |                | Returns a "speed" setting - this sets the delay between steps in timer ticks                    |
| 0x06 | Set speed      | 2 delay values | Sets a "speed" setting - this sets the delay between steps in timer ticks                       |
| 0x07 | Get status     |                | Returns status flags. Currently only defined if the motor is moving or not                      |

## Commands in detail

### Identify

Request:

| Offset | Length | Content                                         |
| ------ | ------ | ----------------------------------------------- |
| 0      | 1      | Address byte                                    |
| 1      | 1      | Length (3)                                      |
| 2      | 1      | Command ```0x00```                              |

Response:

| Offset | Length | Content                                                                                                       |
| ------ | ------ | ------------------------------------------------------------------------------------------------------------- |
| 0      | 1      | Address 0 (response)                                                                                          |
| 1      | 1      | Length (20)                                                                                                   |
| 2      | 16     | UUID identifying the device: ```{ 0xe1729ab7, 0x6a03, 0x11eb, 0x8045, 0xb4, 0x99, 0xba, 0xdf, 0x00, 0xa1 }``` |
| 18     | 2      | Version number (```0x00000001```)                                                                             |

### Get boundaries

Request:

| Offset | Length | Content                                         |
| ------ | ------ | ----------------------------------------------- |
| 0      | 1      | Address byte                                    |
| 1      | 1      | Length (3)                                      |
| 2      | 1      | Command ```0x01```                              |

Response:

| Offset | Length | Content                                         |
| ------ | ------ | ----------------------------------------------- |
| 0      | 1      | Address 0 (response)                            |
| 1      | 1      | Length (18)                                     |
| 2      | 4      | Max position in positive x direction (unsigned) |
| 6      | 4      | Max position in negative x direction (unsigned) |
| 10     | 4      | Max position in positive y direction (unsigned) |
| 14     | 4      | Max position in negative y direction (unsigned) |

### Set boundaries

Request:

| Offset | Length | Content                                         |
| ------ | ------ | ----------------------------------------------- |
| 0      | 1      | Address byte                                    |
| 1      | 1      | Length (19)                                     |
| 2      | 1      | Command ```0x02```                              |
| 3      | 4      | Max position in positive x direction (unsigned) |
| 7      | 4      | Max position in negative x direction (unsigned) |
| 11     | 4      | Max position in positive y direction (unsigned) |
| 15     | 4      | Max position in negative y direction (unsigned) |

Response:

None

### Get position

Request:

| Offset | Length | Content                                         |
| ------ | ------ | ----------------------------------------------- |
| 0      | 1      | Address byte                                    |
| 1      | 1      | Length (3)                                      |
| 2      | 1      | Command ```0x03```                              |

Response:

| Offset | Length | Content                                         |
| ------ | ------ | ----------------------------------------------- |
| 0      | 1      | Address 0 (response)                            |
| 1      | 1      | Length (10)                                     |
| 2      | 4      | Current position X (signed int)                 |
| 6      | 4      | Current position Y (signed int)                 |

### Set position

Request:

| Offset | Length | Content                                         |
| ------ | ------ | ----------------------------------------------- |
| 0      | 1      | Address byte                                    |
| 1      | 1      | Length (11)                                     |
| 2      | 1      | Command ```0x04```                              |
| 3      | 4      | Target position X (signed int)                  |
| 7      | 4      | Target position Y (signed int)                  |

Response:

None

### Get speed

Request:

| Offset | Length | Content                                         |
| ------ | ------ | ----------------------------------------------- |
| 0      | 1      | Address byte                                    |
| 1      | 1      | Length (3)                                      |
| 2      | 1      | Command ```0x05```                              |

Response:

| Offset | Length | Content                                         |
| ------ | ------ | ----------------------------------------------- |
| 0      | 1      | Address byte (0, response)                      |
| 1      | 1      | Length (10)                                     |
| 3      | 4      | Delay count X (unsigned int)                    |
| 7      | 4      | Delay count Y (unsigned int)                    |

### Set speed

Request:

| Offset | Length | Content                                         |
| ------ | ------ | ----------------------------------------------- |
| 0      | 1      | Address byte                                    |
| 1      | 1      | Length (11)                                     |
| 2      | 1      | Command ```0x06```                              |
| 3      | 4      | Delay count X (unsigned int)                    |
| 7      | 4      | Delay count Y (unsigned int)                    |

Response:

None

### Get status

Request:

| Offset | Length | Content                                         |
| ------ | ------ | ----------------------------------------------- |
| 0      | 1      | Address byte                                    |
| 1      | 1      | Length (3)                                      |
| 2      | 1      | Command ```0x06```                              |

Response:

| Offset | Length | Content                                         |
| ------ | ------ | ----------------------------------------------- |
| 0      | 1      | Address byte (0, response)                      |
| 1      | 1      | Length (3)                                      |
| 2      | 1      | Status flags                                    |

The status flags consist of the following bits:

| Bit  | Meaning                                                    |
| ---- | ---------------------------------------------------------- |
| 0    | Motor X is moving                                          |
| 1    | Motor Y is moving                                          |
| 2    |                                                            |
| 3    |                                                            |
| 4    |                                                            |
| 5    |                                                            |
| 6    |                                                            |
| 7    |                                                            |
