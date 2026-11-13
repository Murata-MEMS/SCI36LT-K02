#!/usr/bin/env python3
# Copyright (c) 2024-2026 Murata Electronics Oy.
# SPDX-License-Identifier: BSD-3-Clause
# See LICENSE at the repository root.

"""Dump K02 registers and stream sensor data over the UART interface."""

from __future__ import annotations

import argparse
import math
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ModuleNotFoundError as exc:
    if exc.name == "serial" or (exc.name or "").startswith("serial."):
        print(
            "pyserial is required. Install it with: python -m pip install pyserial",
            file=sys.stderr,
        )
        raise SystemExit(2) from exc
    raise


BAUD_RATE = 460_800
FRAME_SIZE = 6
CONF = 0
SENSOR_STATUS = 33
FWVER_REG = 43
ODR_BIT = 5
DATA_READY_BIT = 31
FRACTIONAL_BITS = 11
FIRMWARE_STARTUP_TIMEOUT = 5.0
REGISTER_NAMES = {
    0: "CONF",
    1: "ACC_X",
    2: "ACC_Y",
    3: "ACC_Z",
    4: "RATE_X",
    5: "RATE_Y",
    6: "RATE_Z",
    13: "TEMP",
    33: "SENSOR_STATUS",
    34: "ID_REG_0",
    35: "ID_REG_1",
    36: "ID_REG_2",
    37: "ID_REG_3",
    38: "ID_REG_4",
    39: "RESERVED",
    40: "MCU_ID_REG_0",
    41: "MCU_ID_REG_1",
    42: "MCU_ID_REG_2",
    43: "FWVER_REG",
}
MISO_STATUS_NAMES = ("OK", "soft error", "hard error", "invalid request")
SENSOR_REGISTERS = (1, 2, 3, 4, 5, 6, 13)


class UARTToolError(Exception):
    """An invalid or incomplete transaction with the K02 UART interface."""


def crc8(payload: bytes) -> int:
    """Return the K02 CRC-8 (poly 0x2F, init 0xFF, with leading zero byte)."""
    crc = 0xFF
    for byte in b"\x00" + payload:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x2F) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def exchange(connection, address: int, value: int = 0, *, write: bool = False):
    """Return (status, value) for status 0/1/2 (OK/soft/hard error).

    Raise UARTToolError for status 3 (invalid request) or an incomplete or
    corrupt response.
    """
    if not 0 <= address < 64:
        raise ValueError(f"register address must be in 0..63, got {address}")
    if not 0 <= value <= 0xFFFFFFFF:
        raise ValueError(f"register value must be in 0..0xFFFFFFFF, got {value}")

    request = bytes(
        [
            address | (0x80 if write else 0),
            (value >> 24) & 0xFF,
            (value >> 16) & 0xFF,
            (value >> 8) & 0xFF,
            value & 0xFF,
        ]
    )
    request += bytes([crc8(request)])
    if connection.write(request) != FRAME_SIZE:
        raise UARTToolError("UART write timed out before the full request was sent")

    response = connection.read(FRAME_SIZE)
    if len(response) != FRAME_SIZE:
        raise UARTToolError(
            f"incomplete UART response: received {len(response)} of {FRAME_SIZE} bytes"
        )
    if crc8(response[:5]) != response[5]:
        raise UARTToolError("response CRC is invalid")

    response_address = response[0] & 0x3F
    if response_address != address:
        raise UARTToolError(
            f"response register {response_address} does not match request {address}"
        )

    miso_status = response[0] >> 6
    if miso_status == 3:
        raise UARTToolError(f"firmware rejected the request for register {address}")
    register_value = int.from_bytes(response[1:5], byteorder="big", signed=False)
    return miso_status, register_value


def read_register(connection, address: int):
    """Read and return the status and value of one register."""
    return exchange(connection, address)


def write_register(connection, address: int, value: int):
    """Write a 32-bit value and return the response status and value."""
    return exchange(connection, address, value, write=True)


def wait_for_data_ready(connection, timeout: float) -> int:
    """Wait up to timeout seconds and return SENSOR_STATUS when DATA_READY is set."""
    deadline = time.monotonic() + timeout
    last_status = 0
    while True:
        _, last_status = read_register(connection, SENSOR_STATUS)
        if last_status & (1 << DATA_READY_BIT):
            return last_status
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise UARTToolError(
                f"DATA_READY did not assert within {timeout:g} s "
                f"(last SENSOR_STATUS=0x{last_status:08X})"
            )
        time.sleep(min(0.002, remaining))


def print_register_dump(connection) -> None:
    """Print registers 0 through FWVER_REG."""
    print("\nRegister dump (32-bit values in hexadecimal):")
    for address in range(FWVER_REG + 1):
        miso_status, value = read_register(connection, address)
        name = REGISTER_NAMES.get(address, "-")
        status_text = (
            "" if miso_status == 0 else f"  [MISO: {MISO_STATUS_NAMES[miso_status]}]"
        )
        print(f"{address:02d}  {name:<12}  0x{value:08X}{status_text}")


def choose_port(ports) -> str:
    """Prompt for a port from the supplied list and return its device name."""
    if not ports:
        raise UARTToolError("no serial ports are available")
    if not sys.stdin.isatty():
        raise UARTToolError("pass --port when selecting a serial port non-interactively")

    print("Available serial ports:")
    for index, port in enumerate(ports, start=1):
        description = port.description or "serial device"
        print(f"  {index}. {port.device} - {description}")
    while True:
        try:
            selection = input("Select the K02 UART port: ").strip()
        except EOFError as exc:
            raise UARTToolError("no port was selected") from exc
        try:
            index = int(selection)
        except ValueError:
            print("Enter the number shown beside the desired port.")
            continue
        if 1 <= index <= len(ports):
            return ports[index - 1].device
        print(f"Enter a number from 1 to {len(ports)}.")


def positive_int(text: str) -> int:
    """Parse a positive integer for an argparse option."""
    try:
        value = int(text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be an integer") from exc
    if value <= 0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return value


def positive_float(text: str) -> float:
    """Parse a finite positive float for an argparse option."""
    try:
        value = float(text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be a number") from exc
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("must be a finite number greater than zero")
    return value


def make_parser() -> argparse.ArgumentParser:
    """Build the command-line parser for the UART tool."""
    parser = argparse.ArgumentParser(
        description="Read K02 registers and stream sensor data over UART."
    )
    parser.add_argument(
        "--port",
        help="serial port, for example COM7 or /dev/ttyUSB0; prompts if omitted",
    )
    parser.add_argument(
        "--baud",
        type=positive_int,
        default=BAUD_RATE,
        help=f"UART baud rate (default: {BAUD_RATE})",
    )
    parser.add_argument(
        "--samples",
        type=positive_int,
        default=400,
        help="number of DATA_READY samples to print (default: 400)",
    )
    parser.add_argument(
        "--ready-timeout",
        type=positive_float,
        default=2.0,
        help="maximum wait for DATA_READY, in seconds (default: 2)",
    )
    parser.add_argument(
        "--no-register-dump",
        action="store_true",
        help="skip the initial register dump",
    )
    parser.add_argument(
        "--list-ports",
        action="store_true",
        help="list serial ports and exit",
    )
    return parser


def run(connection, args) -> bool:
    """Select 200 Hz, reinitialize the ASIC, and print sensor samples.

    Reading sensor data clears DATA_READY.
    """
    startup_deadline = time.monotonic() + FIRMWARE_STARTUP_TIMEOUT
    while True:
        miso_status, firmware_version = read_register(connection, FWVER_REG)
        if firmware_version != 0:
            break
        if time.monotonic() >= startup_deadline:
            raise UARTToolError(
                f"FWVER_REG remained zero for {FIRMWARE_STARTUP_TIMEOUT:g} s; "
                "no K02 firmware response was verified"
            )
        time.sleep(0.05)
    print(f"Firmware version: {firmware_version}")
    if miso_status:
        print(f"FWVER_REG MISO status: {MISO_STATUS_NAMES[miso_status]}")

    if not args.no_register_dump:
        print_register_dump(connection)

    _, written_conf = write_register(connection, CONF, 1 << ODR_BIT)
    if not written_conf & (1 << ODR_BIT):
        raise UARTToolError("firmware did not select 200 Hz ODR")
    print("Selected 200 Hz ODR.")
    # CONF changes reinitialize the ASIC asynchronously in the main loop.
    time.sleep(0.1)
    # Clear any ready flag left over from before the reinitialization.
    read_register(connection, SENSOR_REGISTERS[0])

    print("\nelapsed_s\tacc_x\tacc_y\tacc_z\trate_x\trate_y\trate_z\ttemp_c\tstatus")
    start = time.monotonic()
    for _ in range(args.samples):
        sensor_status = wait_for_data_ready(connection, args.ready_timeout)
        readings = []
        miso_statuses = set()
        for address in SENSOR_REGISTERS:
            response_status, raw_value = read_register(connection, address)
            miso_statuses.add(response_status)
            signed_value = (
                raw_value - 0x1_0000_0000
                if raw_value & 0x8000_0000
                else raw_value
            )
            readings.append(signed_value / (1 << FRACTIONAL_BITS))

        status_text = f"0x{sensor_status:08X}"
        non_ok_statuses = sorted(status for status in miso_statuses if status)
        if non_ok_statuses:
            labels = ",".join(MISO_STATUS_NAMES[status] for status in non_ok_statuses)
            status_text += f" ({labels})"
        print(
            f"{time.monotonic() - start:.4f}\t"
            + "\t".join(f"{reading:.4f}" for reading in readings)
            + f"\t{status_text}"
        )
        sys.stdout.flush()
    return True


def main() -> int:
    """Run the CLI using process arguments and return its exit status."""
    parser = make_parser()
    args = parser.parse_args()

    if args.list_ports:
        ports = sorted(list_ports.comports(), key=lambda port: port.device.lower())
        if not ports:
            print("No serial ports found.")
        else:
            for port in ports:
                description = port.description or "serial device"
                print(f"{port.device} - {description}")
        return 0

    try:
        if args.port:
            port_name = args.port
        else:
            ports = sorted(list_ports.comports(), key=lambda port: port.device.lower())
            port_name = choose_port(ports)
        print(f"Opening {port_name} at {args.baud} baud.")
        with serial.Serial(
            port=port_name,
            baudrate=args.baud,
            timeout=1.0,
            write_timeout=1.0,
        ) as connection:
            connection.reset_input_buffer()
            connection.reset_output_buffer()
            return 0 if run(connection, args) else 1
    except KeyboardInterrupt:
        print("\nStopped by user.")
        return 130
    except (UARTToolError, serial.SerialException, OSError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
