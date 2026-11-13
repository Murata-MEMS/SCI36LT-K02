# Third-party notices

This repository is distributed as source code only. Compiled firmware is not
included.

The root `LICENSE` applies only to original material owned by Murata Electronics
Oy. Third-party files, including modifications within them, retain their
applicable third-party terms. Keep the file-level notices and license files
listed below with any redistribution.

| Component or files | License | Notice location |
| --- | --- | --- |
| `USB_DEVICE/**`, `Middlewares/ST/STM32_USB_Device_Library/**`, ST-generated portions of `Core/Src/main.c` and `Core/Inc/main.h`, `Core/Src/stm32l4xx_hal_msp.c`, `Core/Src/stm32l4xx_it.c`, and `Core/Inc/stm32l4xx_it.h` | ST SLA0044 | Full terms in [`third_party_licenses/ST_SLA0044.txt`](third_party_licenses/ST_SLA0044.txt). The named Core and middleware files also carry ST license notices. |
| `Drivers/CMSIS/**` and `Drivers/CMSIS/Device/ST/STM32L4xx/**` | Apache-2.0 | Existing license copies in those directories. |
| `Drivers/STM32L4xx_HAL_Driver/**` and `Core/Inc/stm32l4xx_hal_conf.h` | BSD-3-Clause | Full ST terms in [`third_party_licenses/ST_BSD_3_CLAUSE.txt`](third_party_licenses/ST_BSD_3_CLAUSE.txt); copyright notices in the source files. |
| `Core/Startup/startup_stm32l433rcyx.s` and `Core/Src/system_stm32l4xx.c` | Apache-2.0 | Existing ST file notices; CMSIS license copy is in `Drivers/CMSIS/LICENSE.txt`. |
| `STM32L433RCYX_FLASH.ld`, `Core/Src/syscalls.c`, and `Core/Src/sysmem.c` | BSD-3-Clause | ST copyright notices in these files; full BSD terms in [`third_party_licenses/ST_BSD_3_CLAUSE.txt`](third_party_licenses/ST_BSD_3_CLAUSE.txt). |
| `pyserial`, used by `Tools/SerialUART.py` | BSD-3-Clause | External runtime dependency. Its source is not included in this repository. |
