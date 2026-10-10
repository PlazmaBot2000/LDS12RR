#!/usr/bin/env python3
import argparse
import math
import os
import struct
import sys
import time

import serial

PORT = "/dev/ttyUSB0"
BAUD = 115200
TIMEOUT = 5

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


def parse(data):
    if len(data) != PACKET_LEN or data[:4] != HEADER:
        return None
    (start_raw,) = struct.unpack_from("<H", data, 6)
    end_raw, stored = struct.unpack_from("<HH", data, END_ANGLE_OFF)
    if checksum(data[: END_ANGLE_OFF + 2]) != stored or start_raw < ANGLE_MIN or end_raw < ANGLE_MIN:
        return None
    start = (start_raw - ANGLE_MIN) / 64.0
    end = (end_raw - ANGLE_MIN) / 64.0
    span = end - start + (360.0 if start < end else 0.0)
    step = span / (SAMPLES - 1)
    samples = []
    for i in range(SAMPLES):
        distance, quality = struct.unpack_from("<hB", data, SAMPLE_OFF + i * 3)
        samples.append(((start + i * step) % 360.0, distance, quality))
    return start, end, samples


def read_packet(ser):
    header = ser.read(4)
    if header != HEADER:
        return None
    return parse(header + ser.read(PACKET_LEN - 4))


def run_scan(ser, scans, log=print, stall=10.0):
    points, start_idx, done, collecting = [], 0, 0, False
    log("waiting for the first full revolution ...")
    last = time.monotonic()
    try:
        while True:
            packet = read_packet(ser)
            if packet is None:
                if time.monotonic() - last > stall:
                    log(f"no scan data for {stall:g} s, is the motor running?")
                    break
                continue
            last = time.monotonic()
            start_angle, end_angle, samples = packet
            if start_angle < end_angle:
                if collecting:
                    done += 1
                    log(f"revolution {done}/{scans}: {len(points) - start_idx} points")
                    start_idx = len(points)
                    if done >= scans:
                        break
                else:
                    collecting = True
                    log(f"start of revolution 1/{scans}")
            if not collecting:
                continue
            for angle, distance, quality in samples:
                if distance > 0 and quality > 0:
                    radians = math.radians(angle)
                    radius = distance / 1000.0
                    points.append((radius * math.cos(radians), radius * math.sin(radians), 0.0))
    except KeyboardInterrupt:
        log("interrupted, saving captured data")
    except serial.SerialException as error:
        log(f"serial error: {error}, saving captured data")
    return points, done


def write_pcd(path, points, scans):
    directory = os.path.dirname(os.path.abspath(path))
    if not os.path.isdir(directory):
        raise IOError(f"output directory does not exist: {directory}")
    with open(path, "w") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write(f"# LDS12RR-scan.py revolutions={scans}\n")
        f.write("VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
        f.write(f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points)}\nDATA ascii\n")
        for x, y, z in points:
            f.write(f"{x:.4f} {y:.4f} {z:.4f}\n")


def main(argv=None):
    parser = argparse.ArgumentParser(description="Capture revolutions from an LDS12RR lidar into a PCD file.")
    parser.add_argument("-n", "--scans", type=int, default=1, metavar="N",
                        help="number of revolutions to capture (1)")
    parser.add_argument("--hz", type=float, default=5.0, metavar="HZ", help="motor speed, 2-8 Hz (5)")
    parser.add_argument("-o", "--output", default="scan.pcd", metavar="FILE",
                        help="output file, .pcd is appended if missing (scan.pcd)")
    parser.add_argument("-p", "--port", default=PORT, metavar="DEVICE",
                        help=f"serial port of the lidar ({PORT})")
    args = parser.parse_args(argv)
    if args.scans < 1:
        parser.error("--scans must be >= 1")
    if not 2.0 <= args.hz <= 8.0:
        parser.error("--hz must be between 2 and 8")
    output = args.output if args.output.lower().endswith(".pcd") else args.output + ".pcd"

    def log(message):
        print(message, file=sys.stderr)

    try:
        ser = serial.Serial(args.port, BAUD, timeout=TIMEOUT)
    except serial.SerialException as error:
        log(f"error: cannot open {args.port}: {error}")
        log("hint: check `ls /dev` and dialout group membership")
        return 1

    try:
        ser.write(command(CMD_START))
        time.sleep(0.2)
        ser.write(command(CMD_SET_SPEED, struct.pack("<H", int(round(args.hz * 360)))))
        time.sleep(0.5)
        ser.reset_input_buffer()
        log(f"motor started at {args.hz:g} Hz")
        points, done = run_scan(ser, args.scans, log=log)
    finally:
        try:
            ser.write(command(CMD_STOP))
            log("motor stopped")
        except Exception:
            pass
        ser.close()

    if not points:
        log("error: no valid points captured, nothing written")
        return 1
    try:
        write_pcd(output, points, done)
    except (IOError, OSError) as error:
        log(f"error: cannot write {output}: {error}")
        return 1
    log(f"saved {len(points)} points ({done}/{args.scans} revolutions) to {output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())