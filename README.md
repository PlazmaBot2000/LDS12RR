# LDS12RR

| Script                               | Purpose                                                          | Arguments                                                                                                                                                                                                                                                                                               |
| ------------------------------------ | ---------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [`LDS12RR-scan.py`](LDS12RR-scan.py) | capture N revolutions and write them to an ASCII PCD point cloud | -h, --help         show this help message and exit<br>-n, --scans N      number of revolutions to capture (1)<br>--hz HZ            motor speed, 2-8 Hz (5)<br>-o, --output FILE  output file, .pcd is appended if missing (scan.pcd)<br>-p, --port DEVICE  serial port of the lidar (/dev/ttyUSB0)<br> |
| [`LDS12RR-view.py`](LDS12RR-view.py) | minimal live on-screen viewer                                    | -h, --help         show this help message and exit<br>-p, --port PORT    serial port (/dev/ttyUSB0)<br>--hz HZ            motor speed, Hz (5)<br>-r, --range RANGE  view range, m (1.5)                                                                                                                 |
## Requirements

```
python3 -m pip install pyserial numpy matplotlib
```

## Protocol

### Motor commands

The LDS12RR motor is driven with the LDROBOT **LD14P** command layout: every
command is the 7-byte envelope `54 <code> 04 <payload:4> <crc8>`, where the
payload is little endian and zero padded, and the CRC-8 covers bytes 0..6.

| Command            | Code   | Payload                                   |
| ------------------ | ------ | ----------------------------------------- |
| Start spinning     | `0xA0` | none                                      |
| Stop spinning      | `0xA1` | none                                      |
| Set rotation speed | `0xA2` | `uint16 = speed_hz * 360` (little endian) |

### Scan frames

Each measurement frame is **36 bytes** and starts with the fixed header
`55 AA 03 08`:

| Offset | Size | Field | Notes |
| --- | --- | --- | --- |
| 0 | 4 | header | `0x55 0xAA 0x03 0x08` |
| 4 | 2 | rotation speed | `uint16 LE`, `speed_hz = value / 3840` (= Hz * 64 * 60) |
| 6 | 2 | start angle | `uint16 LE`, first sample of the frame |
| 8 | 24 | 8 samples | each: `int16 LE` distance in mm, then `uint8` quality |
| 32 | 2 | end angle | `uint16 LE`, last sample of the frame |
| 34 | 2 | checksum | 15-bit, see below |

The 8 samples are interleaved as `distance, quality` triples: sample *i* sits at offset `8 + i * 3`.

Angles are stored as `angle_q6 + 0xA000`.
Distances are **signed** 16-bit values in millimetres. A negative distance marks an invalid return; quality `0` means the same and should be filtered out.
