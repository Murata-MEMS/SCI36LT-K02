# SCI36LT-K02 Template Firmware

A template firmware for Murata SCI36LT-K02 featuring an integrated Cortex-M4 MCU. This baseline implementation accelerates customer development by providing sensor interfacing, dual communication protocols, and error handling.

## Overview

The Firmware implements a 6-DOF IMU interface (3-axis accelerometer + 3-axis gyroscope + temperature) with integrated communication protocols.

**Current Version:** FWVER 20261014

## Key Features

- **6-DOF Sensor Integration**: 3-axis accelerometer, 3-axis gyroscope, and temperature sensing
- **Dual Communication Interfaces**: SPI slave and UART support for host communication
- **Flexible Output Rates**: Configurable 100 Hz or 200 Hz data output
- **Robust Error Handling**: Per-axis saturation and error detection with status reporting
- **CRC Protection**: CRC8 validation on SPI/UART register frames
- **Optional USB Debug Interface**: USB CDC virtual COM port for telemetry and debugging (disabled by default)
- **Low Power Design**: CPU sleep mode between sensor samples
- **DMA-Driven I/O**: DMA-based SPI and UART transfers
- **ASIC Failure Handling**: Reports ASIC initialization failure and disables the ASIC SPI interface

## Hardware

- **MCU**: STM32L433RCY (Cortex-M4 @ 80 MHz, 256 KB Flash, 64 KB SRAM)
- **Sensor**: Murata SCI36LT-K02
- **Interfaces**:
  - SPI1 (Master to ASIC sensor)
  - SPI2 (Slave for external host)
  - UART3 (Serial communication @ 460,800 baud)
  - USB FS (optional debug output; disabled by default)
  - GPIO (data-ready signal, LDO enable, chip select)

### Host Connections

The pins below are STM32 GPIO names. Refer to the SCI36LT-K02 datasheet for module pin assignments and electrical requirements.

| Interface | Pins | Configuration |
| --- | --- | --- |
| SPI2 | PB12 NSS, PB13 SCK, PB14 MISO, PB15 MOSI | Mode 0, MSB first, active-low NSS |
| UART3 | PC10 TX, PC11 RX | 460,800 baud, 8 data bits, no parity, 1 stop bit |
| DATA_READY | PC1 | Active-high output |

## Software Architecture

### Core Modules

- **Main Application** (`main.c`): System initialization, task processing, and power management
- **Register Interface** (`reg_interface.c/h`): 64-register bank with read/write protection and CRC validation
- **ASIC Functions** (`AsicFunctions.c`): Sensor initialization, configuration, and data reading
- **Task Processor** (`task_processor.c`): Handles register modifications and triggers actions
- **Firmware Extension** (`firmware_extension.c/h`): Optional build-time integration boundary; no-op in the standard firmware build
- **String Functions** (`StrFunctions.c`): Utility functions for data formatting

### Communication Protocol

SPI and UART register transactions use 6-byte frames:

```
Request byte 0: [RW (1 bit) | RESERVED (1 bit) | ADDRESS (6 bits)]
Response byte 0: [MISO_STATUS (2 bits) | ADDRESS (6 bits)]
Bytes 1-4: [32-bit data (big-endian)]
Byte 5: [CRC8 checksum]
```

- **RW**: Read (0) or Write (1) operation
- **RESERVED**: Ignored by the firmware; send as 0
- **MISO_STATUS**: Response status (0=OK, 1=Soft Error, 2=Hard Error, 3=Invalid; response only)
- **ADDRESS**: Register address (0-63)
- **DATA**: 32-bit register value
- **CRC8**: Frame checksum. See [CRC Calculation](#crc-calculation).

UART sends the response after receiving the current request. SPI returns the response to the **previous** request. Ignore the first received SPI frame and send another valid request to retrieve the preceding response. Keep NSS low for each complete 6-byte frame, then high between frames, allowing time for the firmware to prepare the next response.

#### CRC Calculation

For requests and responses, use polynomial `0x2F`, initial value `0xFF`, no reflection, and final XOR `0x00`. Process a leading `0x00` byte followed by frame bytes 0-4, MSB first. Append the result as byte 5. The leading zero is used only for the calculation and is not transmitted.

For example, a read request for `FWVER_REG` (address 43, or `0x2B`) sets all four data bytes to `0x00`:

```text
Frame bytes 0-4:  2B 00 00 00 00
CRC input:       00 2B 00 00 00 00
CRC result:      DE
Transmitted:     2B 00 00 00 00 DE
```

### Register Map

| Address | Name | Access | Description |
|---------|------|--------|-------------|
| 0 | CONF | RW | Configuration (RST bit, ODR bit) |
| 1 | ACC_X | R | Accelerometer X data (fixed-point, 11 fractional bits) |
| 2 | ACC_Y | R | Accelerometer Y data (fixed-point, 11 fractional bits) |
| 3 | ACC_Z | R | Accelerometer Z data (fixed-point, 11 fractional bits) |
| 4 | RATE_X | R | Gyroscope X data (fixed-point, 11 fractional bits) |
| 5 | RATE_Y | R | Gyroscope Y data (fixed-point, 11 fractional bits) |
| 6 | RATE_Z | R | Gyroscope Z data (fixed-point, 11 fractional bits) |
| 13 | TEMP | R | Temperature data (fixed-point, 11 fractional bits) |
| 33 | SENSOR_STATUS | R | Error/saturation status bits |
| 34-36 | ID_REG_0-2 | R | ASIC serial number registers 1-3 |
| 37 | ID_REG_3 | R | ASIC component ID |
| 38 | ID_REG_4 | R | ASIC ID |
| 40-42 | MCU_ID_REG_0-2 | R | STM32 unique ID |
| 43 | FWVER_REG | R | Firmware version in `YYYYMMDD02` format. The suffix `02` identifies K02. |

Sensor data registers contain signed 32-bit fixed-point values. Divide by 2048 to obtain acceleration in m/s², angular rate in °/s, or temperature in °C.

### Sensor Status

`SENSOR_STATUS` uses the following bits. Bit positions and axis names are listed in matching order.

| Bits | Names | Meaning when set |
| --- | --- | --- |
| 0, 2, 4 | SAZ, SAY, SAX | Accelerometer Z, Y, X saturation |
| 1, 3, 5 | HAZ, HAY, HAX | Accelerometer Z, Y, X hard error |
| 6, 8, 10 | SGZ, SGY, SGX | Gyroscope Z, Y, X saturation |
| 7, 9, 11 | HGZ, HGY, HGX | Gyroscope Z, Y, X hard error |
| 13 | RES | Reset/startup in progress |
| 18 | CMS | Common ASIC error or ASIC initialization failure |
| 19 | ACRC | ASIC response CRC error |
| 20 | ACE | ASIC command error |
| 31 | DRY | New sensor data available |

### Sensor Output

- **Accelerometer:** ~±160 m/s² range; ASIC sensitivity 3200 LSB/(m/s²), firmware output 2048 LSB/(m/s²)
- **Gyroscope:** ~±490 °/s range; ASIC sensitivity 1060 LSB/(°/s), firmware output 2048 LSB/(°/s)
- **Temperature:** ASIC sensitivity 100 LSB/°C, firmware output 2048 LSB/°C
- **Output Data Rate:** 100 Hz or 200 Hz (configurable)
- **Low-Pass Filters:** 30 Hz at 100 Hz ODR and 68 Hz at 200 Hz ODR

## Getting Started

### Prerequisites

- STM32CubeIDE with its bundled ARM GCC toolchain
- ST-Link debugger/programmer
- STM32CubeMX only if changing the peripheral configuration

### Build Instructions

1. Import the project into STM32CubeIDE
2. Build using the `Debug` or `Release` configuration
3. Flash the firmware using ST-Link

### Initial Setup

Power on the device. The ASIC initializes automatically. Before writing configuration, wait for a nonzero `FWVER_REG` and for `SENSOR_STATUS.RES` to clear. Wait for `DATA_READY` before reading sensor data.

## Usage

### Basic Operation

1. **Configuration**: Set CONF bit 5 (`ODR`) to 0 for 100 Hz or 1 for 200 Hz. Set bit 0 (`RST`) to reinitialize the register bank and ASIC. Reset restores the default 100 Hz output rate. Changing ODR reinitializes the ASIC and temporarily invalidates sensor data.
2. **Data Reading**: Poll `ACC_X`, `ACC_Y`, `ACC_Z`, `RATE_X`, `RATE_Y`, `RATE_Z`, and `TEMP`
3. **Status Monitoring**: Check SENSOR_STATUS for errors and DATA_READY flag
4. **Debug Output**: If enabled, monitor the USB serial port for real-time telemetry

### UART Diagnostic

`Tools/SerialUART.py` dumps registers and streams sensor data over the serial connection. Requires Python 3.10+ and `pyserial`.

```powershell
python -m pip install pyserial
python Tools/SerialUART.py --list-ports
python Tools/SerialUART.py --port COM7
```

Replace `COM7` with the device's serial port, or omit `--port` to select one. Defaults: 460,800 baud, 400 samples, and 200 Hz ODR. The ODR remains set until reset or changed again. Use `--no-register-dump` to skip the dump.

### Optional USB Debug Output

USB debug output is disabled by default. In `Core/Inc/main.h`, set `DEBUG_PRINTS_ON` to `1` and `DEBUG_PRINTS_USE_UART` to `0` to enable the USB CDC virtual COM port. Set `DEBUG_PRINTS_ON` to `0` to disable debug output.

### Data Ready Signal

- **Pin**: PC1 (GPIO output)
- **Logic**: Asserted HIGH when averaged data is ready
- **Clear**: Reading any register in the sensor-data range (addresses 1-22) drives DATA_READY LOW and clears `DRY` (bit 31) in `SENSOR_STATUS` to 0
- **Usage**: External synchronization for data acquisition

## Validation

1. **Hardware Check**: Verify ASIC initialization through the sensor status register or, if enabled, USB debug output
2. **UART Check**: Use `Tools/SerialUART.py` to dump registers and stream sensor data
3. **SPI Check**: Use an external host tool to read registers and monitor real-time sensor data
4. **Error Handling**: Check status registers under various conditions
5. **CRC Validation**: Ensure all frames pass integrity checks


## Version History

### FWVER 20261014
- Improved ASIC error handling

### FWVER 20261013
- Added host SPI2 recovery after transfer errors or DMA rearm failures
- Added ASIC response CRC and command error reporting to the sensor status register
- Unified ASIC response parsing
- Fixed stale DRY and DATA_READY during reinitialization

### FWVER 20260916
- Added an internal pull-up to the host SPI chip-select input (SPI2 NSS) to prevent it from floating when the host does not drive it
- Configured otherwise unused GPIOs in analog mode during normal operation to disable floating digital input buffers

### FWVER 20260904
- Increased SPI2 TX DMA interrupt priority to avoid a SPI TX/RX deadlock
- Added an optional firmware extension boundary for internal-only functionality
- Updated UART pin pull configuration

### FWVER 20260505
- Added raw sensor axes support to the public template firmware
- Fixed sticky saturation status flags
- Removed the CFG_RST flash-reset input
- Disabled the ASIC SPI interface after ASIC initialization failure

### FWVER 20260402
- Added raw sensor axes support

### FWVER 20260401
- Initial release with SPI and UART interfaces
- Sensor data output implementation
- ODR and RST bits in CONF register
- SENSOR_STATUS register with error flags
- ID and version registers
- USB debug prints support

## Support and Documentation

- **SCI36LT-K02 Datasheet**: Contact Murata
- **STM32 Reference**: [STM32L433RCY datasheet](https://www.st.com/resource/en/datasheet/stm32l433cc.pdf)

## Licensing

This repository is distributed as source code only. Compiled firmware is not included.

Murata-owned original material is licensed under [BSD 3-Clause](LICENSE). Third-party components retain their own terms. The [third-party notices](THIRD_PARTY_NOTICES.md) identify the affected files and license texts. The ST components covered by [SLA0044](third_party_licenses/ST_SLA0044.txt) may be used only on or with ST processing units, and their terms do not permit applying an open-source license to those files.

## Contributing
This is an example. Use as guideline when developing your application on Murata's SCI36LT-K02 IMU. Provided as is, no warranties, no guarantees. 
