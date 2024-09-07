# Description

This repository contains the **NEO-M8x** GPS module driver (based on the UART interface).

# Dependencies

The driver relies on:

* An external `types.h` header file defining the **standard C types** of the targeted MCU.
* The **embedded utility functions** defined in the [embedded-utils](https://github.com/Ludovic-Lesur/embedded-utils) repository.

Here is the versions compatibility table:

| **neom8x-driver** | **embedded-utils** |
|:---:|:---:|
| [sw1.1](https://github.com/Ludovic-Lesur/neom8x-driver/releases/tag/sw1.1) | >= [sw1.3](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw1.3) |
| [sw1.0](https://github.com/Ludovic-Lesur/neom8x-driver/releases/tag/sw1.0) | [sw1.0](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw1.0) to [sw1.2](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw1.2) |

# Compilation flags

| **Flag name** | **Value** | **Description** |
|:---:|:---:|:---:|
| `NEOM8X_DRIVER_DISABLE_FLAGS_FILE` | `defined` / `undefined` | Disable the `neom8x_driver_flags.h` header file inclusion when compilation flags are given in the project settings or by command line. |
| `NEOM8X_DRIVER_GPIO_ERROR_BASE_LAST` | `<value>` | Last error base of the low level GPIO driver. |
| `NEOM8X_DRIVER_UART_ERROR_BASE_LAST` | `<value>` | Last error base of the low level UART driver. |
| `NEOM8X_DRIVER_DELAY_ERROR_BASE_LAST` | `<value>` | Last error base of the low level delay driver. |
| `NEOM8X_DRIVER_GPS_DATA_TIME` | `defined` / `undefined` | Enable or disable the time acquisition feature. |
| `NEOM8X_DRIVER_GPS_DATA_POSITION` | `defined` / `undefined` | Enable or disable the position acquisition feature. |
| `NEOM8X_DRIVER_ALTITUDE_STABILITY_FILTER_MODE` | `0` / `1` / `2` | Altitude stability filter mode: `0` = disabled `1` = fixed `2` = dynamic.|
| `NEOM8X_DRIVER_ALTITUDE_STABILITY_THRESHOLD` | `<value>` | Altitude stability filter threshold (used when mode is `1`).
| `NEOM8X_DRIVER_VBCKP_CONTROL` | `defined` / `undefined` | Enable or disable the backup voltage pin control. |
| `NEOM8X_DRIVER_TIMEPULSE` | `defined` / `undefined` | Enable or disable the timepulse signal control. |
