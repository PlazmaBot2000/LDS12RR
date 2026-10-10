#!/usr/bin/env python3
import argparse
import struct
import time

import matplotlib.pyplot as plt
import numpy as np
import serial
from matplotlib.animation import FuncAnimation

HEADER = b"\x55\xaa\x03\x08"
PACKET_LEN = 36
SAMPLES = 8
SAMPLE_OFF = 8
END_ANGLE_OFF = 32
ANGLE_MIN = 0xA000
CMD_START, CMD_STOP, CMD_SET_SPEED = 0xA0, 0xA1, 0xA2


def crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x4D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def command(cmd, payload=b""):
    body = bytes([0x54, cmd, 0x04]) + (payload + b"\x00\x00\x00\x00")[:4]
    return body + bytes([crc8(body)])


def checksum(data):
    chk = 0
    for i in range(0, len(data) - 1, 2):
        chk = ((chk << 1) + (data[i] | (data[i + 1] << 8))) & 0xFFFFFFFF
    return ((chk & 0x7FFF) + (chk >> 15)) & 0x7FFF


def parse(frame):
    if len(frame) != PACKET_LEN or frame[:4] != HEADER:
        return None
    (start_raw,) = struct.unpack_from("<H", frame, 6)
    end_raw, stored = struct.unpack_from("<HH", frame, END_ANGLE_OFF)
    if checksum(frame[: END_ANGLE_OFF + 2]) != stored or start_raw < ANGLE_MIN or end_raw < ANGLE_MIN:
        return None
    start = (start_raw - ANGLE_MIN) / 64.0
    end = (end_raw - ANGLE_MIN) / 64.0
    span = end - start + (360.0 if start < end else 0.0)
    raw = np.frombuffer(frame, dtype=np.uint8, count=SAMPLES * 3, offset=SAMPLE_OFF).reshape(SAMPLES, 3)
    dist = ((raw[:, 1].astype(np.int32) << 8) | raw[:, 0]).astype(np.int32)
    dist[dist >= 0x8000] -= 0x10000
    good = (dist > 0) & (raw[:, 2] > 0)
    angles = (start + span * np.arange(SAMPLES) / (SAMPLES - 1))[good]
    return np.radians(angles), dist[good] / 1000.0


def main():
    parser = argparse.ArgumentParser(description="Minimal live viewer for an LDS12RR lidar.")
    parser.add_argument("-p", "--port", default="/dev/ttyUSB0", help="serial port (/dev/ttyUSB0)")
    parser.add_argument("--hz", type=float, default=5.0, help="motor speed, Hz (5)")
    parser.add_argument("-r", "--range", type=float, default=1.5, help="view range, m (1.5)")
    args = parser.parse_args()

    ser = serial.Serial(args.port, 115200, timeout=0)
    ser.write(command(CMD_START))
    time.sleep(0.2)
    ser.write(command(CMD_SET_SPEED, struct.pack("<H", int(round(args.hz * 360)))))
    time.sleep(0.5)
    ser.reset_input_buffer()

    fig, ax = plt.subplots()
    ax.set_aspect("equal")
    ax.set_xlim(-args.range, args.range)
    ax.set_ylim(-args.range, args.range)
    ax.grid(True, alpha=0.3)
    scat = ax.scatter([], [], s=8)

    buf = bytearray()
    chunks = []
    keep = 1.0 / args.hz

    def update(_):
        if ser.in_waiting:
            buf.extend(ser.read(ser.in_waiting))
        while True:
            i = buf.find(HEADER)
            if i < 0:
                del buf[:-3]
                break
            if len(buf) - i < PACKET_LEN:
                del buf[:i]
                break
            frame = parse(bytes(buf[i:i + PACKET_LEN]))
            if frame is None:
                del buf[:i + 1]
                continue
            del buf[:i + PACKET_LEN]
            chunks.append((time.monotonic(), frame[0], frame[1]))
        now = time.monotonic()
        while chunks and chunks[0][0] < now - keep:
            chunks.pop(0)
        if chunks:
            rad = np.concatenate([c[1] for c in chunks])
            dist = np.concatenate([c[2] for c in chunks])
            scat.set_offsets(np.column_stack((dist * np.cos(rad), dist * np.sin(rad))))
        return [scat]

    try:
        ani = FuncAnimation(fig, update, interval=40, blit=True, cache_frame_data=False)
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        ser.write(command(CMD_STOP))
        ser.close()


if __name__ == "__main__":
    main()