# KOGGER Serial Binary Protocol (SBP)

## DVL Integration and Parsing Guide

Version: Based on KS_SBP_100 Revision 4.0.10

---

# Overview

The KOGGER Serial Binary Protocol (SBP) is a framed binary protocol used for communication between a host computer and KOGGER sonar/DVL devices.

All multi-byte values are transmitted in:

* Little-endian byte order
* IEEE754 floating point representation
* Fletcher-16 checksum protection

---

# Packet Format

Every packet uses the following frame structure:

| Field   | Type   | Size   |
| ------- | ------ | ------ |
| SYNC1   | U1     | 1      |
| SYNC2   | U1     | 1      |
| ROUTE   | U1     | 1      |
| MODE    | U1     | 1      |
| ID      | U1     | 1      |
| LENGTH  | U1     | 1      |
| PAYLOAD | BYTE[] | LENGTH |
| CHECK1  | U1     | 1      |
| CHECK2  | U1     | 1      |

## Sync Bytes

```text
SYNC1 = 0xBB
SYNC2 = 0x55
```

---

# Checksum

Checksum is Fletcher-16.

Pseudo-code:

```c
uint8_t check1 = 0;
uint8_t check2 = 0;

for each byte in:
{
    ROUTE,
    MODE,
    ID,
    LENGTH,
    PAYLOAD
}
{
    check1 += byte;
    check2 += check1;
}
```

Verify:

```text
computed_check1 == packet.check1
computed_check2 == packet.check2
```

---

# MODE Field

Bits:

```text
bit0-1 : TYPE
bit2   : reserved
bit3-5 : VERSION
bit6   : MARK
bit7   : RESPONSE
```

TYPE values:

```text
0 Reserved
1 CONTENT
2 SETTING
3 GETTING
```

---

# Common Data Types

| Name | Type   | Bytes |
| ---- | ------ | ----- |
| U1   | uint8  | 1     |
| U2   | uint16 | 2     |
| U4   | uint32 | 4     |
| S2   | int16  | 2     |
| S4   | int32  | 4     |
| F4   | float  | 4     |
| D8   | double | 8     |

---

# Parser State Machine

```text
WAIT_SYNC1
    ↓
WAIT_SYNC2
    ↓
READ_HEADER
    ↓
READ_PAYLOAD
    ↓
VERIFY_CHECKSUM
    ↓
DECODE_MESSAGE
    ↓
WAIT_SYNC1
```

---

# DVL Messages

## Message ID

```text
ID_DVL_VEL = 0x79
```

Current DVL implementation uses:

```text
TYPE    = CONTENT
VERSION = 2
```

Payload length:

```text
68 bytes
```

---

# DVL Payload Layout

| Offset | Type | Name           |
| ------ | ---- | -------------- |
| 0      | U4   | FLAGS          |
| 4      | U4   | TIMESTAMP      |
| 8      | F4   | DELTA_TIME     |
| 12     | F4   | LATENCY        |
| 16     | F4   | VELOCITY_X     |
| 20     | F4   | VELOCITY_Y     |
| 24     | F4   | VELOCITY_Z     |
| 28     | F4   | VELOCITY_Z1    |
| 32     | F4   | VELOCITY_Z2    |
| 36     | F4   | UNCERTAINTY_X  |
| 40     | F4   | UNCERTAINTY_Y  |
| 44     | F4   | UNCERTAINTY_Z  |
| 48     | F4   | UNCERTAINTY_Z1 |
| 52     | F4   | UNCERTAINTY_Z2 |
| 56     | F4   | DISTANCE_Z     |
| 60     | F4   | DISTANCE_Z1    |
| 64     | F4   | DISTANCE_Z2    |

---

# DVL Field Descriptions

## FLAGS

Status bitfield.

Use to indicate:

* valid solution
* bottom lock state
* sensor health
* beam status

(The exact bit definitions are not specified in the current SBP document.)

---

## TIMESTAMP

```text
Unit: milliseconds
```

Device timestamp.

---

## DELTA_TIME

```text
Unit: seconds
```

Time since previous DVL update.

Useful for dead reckoning.

---

## LATENCY

```text
Unit: seconds
```

Measurement processing latency.

---

## Velocity Components

```text
VELOCITY_X
VELOCITY_Y
VELOCITY_Z
```

Units:

```text
m/s
```

These represent the measured DVL velocity vector.

Example:

```json
{
  "vx": 0.12,
  "vy": -0.04,
  "vz": 0.01
}
```

---

## Additional Vertical Solutions

```text
VELOCITY_Z1
VELOCITY_Z2
```

Alternative vertical estimates.

Can be used for:

* multi-beam solutions
* beam-pair solutions
* confidence checking

---

## Uncertainty Estimates

```text
UNCERTAINTY_X
UNCERTAINTY_Y
UNCERTAINTY_Z
UNCERTAINTY_Z1
UNCERTAINTY_Z2
```

Units:

```text
m/s
```

Represent estimated velocity error.

Lower values indicate better quality.

Example:

```json
{
  "uncertainty_x": 0.005,
  "uncertainty_y": 0.007,
  "uncertainty_z": 0.012
}
```

---

## Range Measurements

```text
DISTANCE_Z
DISTANCE_Z1
DISTANCE_Z2
```

Units:

```text
meters
```

Bottom distance estimates.

Example:

```json
{
  "bottom_distance": 4.72
}
```

---

# Python Decoder Example

```python
import struct

DVL_FORMAT = "<II15f"

def decode_dvl(payload):

    values = struct.unpack(DVL_FORMAT, payload)

    return {
        "flags": values[0],
        "timestamp_ms": values[1],
        "delta_time": values[2],
        "latency": values[3],

        "velocity": {
            "x": values[4],
            "y": values[5],
            "z": values[6]
        },

        "velocity_alt": {
            "z1": values[7],
            "z2": values[8]
        },

        "uncertainty": {
            "x": values[9],
            "y": values[10],
            "z": values[11],
            "z1": values[12],
            "z2": values[13]
        },

        "distance": {
            "z": values[14],
            "z1": values[15],
            "z2": values[16]
        }
    }
```

---

# Recommended Output Object

```json
{
  "timestamp_ms": 102345,
  "velocity": {
    "x": 0.18,
    "y": -0.03,
    "z": 0.01
  },
  "uncertainty": {
    "x": 0.004,
    "y": 0.006,
    "z": 0.011
  },
  "distance": {
    "z": 5.24
  }
}
```

---

# Other Useful Messages

| ID   | Name      |
| ---- | --------- |
| 0x01 | TIMESTAMP |
| 0x02 | DIST      |
| 0x03 | CHART     |
| 0x04 | ATTITUDE  |
| 0x05 | TEMP      |
| 0x64 | NAV       |
| 0x79 | DVL_VEL   |

For navigation fusion, combine:

* ID_NAV
* ID_ATTITUDE
* ID_DVL_VEL
* ID_TIMESTAMP

to produce a full vehicle state estimate.
