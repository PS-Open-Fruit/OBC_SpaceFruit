# OBC_Peripheral_Testing
On Board Computer - A Project with DNA

## Project Overview
This repository contains STM32L496ZG microcontroller firmware implementations demonstrating various communication protocols and peripheral interfaces. The project showcases firmware development for embedded systems with focus on multiple communication standards commonly used in satellite and aerospace applications.

## Features

### Supported Communication Protocols

#### 1. **I2C Interface** (`l496zg_I2C`)
- Inter-Integrated Circuit (I2C) protocol implementation
- Two-wire serial communication for low-speed peripheral devices
- Ideal for sensor networks and multi-device communication on a single bus
- STM32L4xx HAL-based driver implementation

#### 2. **SPI Interface** (`l496zg_SPI`)
- Serial Peripheral Interface (SPI) protocol implementation
- High-speed synchronous serial communication
- Master-slave architecture for efficient data transfer
- Suitable for memory devices, ADCs, DACs, and display interfaces
- STM32L4xx HAL-based driver implementation

#### 3. **USB CDC** (`l496zg_usb_cdc`)
- USB Communications Device Class (CDC) implementation
- Virtual serial port over USB connectivity
- Real-time communication with host systems
- Includes USB Device Library middleware and device configuration
- Essential for debugging, firmware updates, and sensor data streaming

### Python Testing Suite (`python_usb_cdc`)
- Test scripts for USB CDC communication validation
- TX/RX test modules with CSV data logging
- UART serial communication testing utilities
- Data capture and analysis capabilities

## Project Structure

```
.
├── l496zg_I2C/              # I2C Protocol Implementation
│   ├── Core/                # Microcontroller core files
│   │   ├── Inc/             # Header files (main.h, HAL config)
│   │   ├── Src/             # Source files (main.c, interrupt handlers)
│   │   └── Startup/         # Startup assembly code
│   ├── Drivers/             # HAL & CMSIS drivers
│   │   ├── CMSIS/           # ARM Cortex Microcontroller Software Interface
│   │   └── STM32L4xx_HAL_Driver/  # STM32L4 Hardware Abstraction Layer
│   ├── Debug/               # Build output and compilation artifacts
│   └── *.ld                 # Linker scripts (FLASH/RAM)
│
├── l496zg_SPI/              # SPI Protocol Implementation
│   └── [Same structure as I2C]
│
├── l496zg_usb_cdc/          # USB CDC Implementation
│   ├── Core/                # Microcontroller core files
│   ├── Drivers/             # HAL & CMSIS drivers
│   ├── Middlewares/         # ST USB Device Library
│   ├── USB_DEVICE/          # USB CDC application layer
│   │   ├── App/             # USB device application (usb_device.c, usbd_cdc_if.c)
│   │   └── Target/          # USB target configuration
│   ├── Debug/               # Build output and compilation artifacts
│   └── *.ld                 # Linker scripts (FLASH/RAM)
│
└── python_usb_cdc/          # Testing & Validation Scripts
    ├── test_tx.py           # USB transmission test script
    ├── test_rx.py           # USB reception test script
    ├── uart.py              # UART serial utilities
    ├── test_tx.csv          # TX test data and results
    └── test_rx.csv          # RX test data and results
```

## Hardware Target
- **MCU:** STM32L496ZG (ARM Cortex-M4, 80 MHz, 1MB Flash, 128KB SRAM)
- **Board:** Compatible with STM32L496 Discovery or similar development boards
- **Interfaces:** I2C, SPI, USB Full-Speed

## Development Environment
- **IDE:** STM32CubeIDE or compatible
- **Build System:** ARM GCC Toolchain with Make
- **Debugger:** ST-Link V2/V3 compatible
- **STM32CubeMX:** Project configuration (.ioc files included)

## Getting Started

### Prerequisites
- STM32CubeIDE or GCC ARM Embedded Toolchain
- STM32L496 Development Board
- USB Cable for ST-Link debugger
- Python 3.x (for testing scripts)

### Building & Deploying
Each protocol implementation includes a `Debug/` directory with build artifacts:
- `makefile` - Project build configuration
- `sources.mk` - Source file definitions
- `*.list` - Compiled binary listing

### Testing
The `python_usb_cdc/` directory provides automated testing scripts for validating USB CDC communication with Python test harnesses.

## License
See [LICENSE](LICENSE) file for details.

## References
- [STM32L496 Datasheet](https://www.st.com/resource/en/datasheet/stm32l496zi.pdf)
- [STM32CubeL4 HAL Documentation](https://www.st.com/en/microcontrollers-microprocessors/stm32l4-series.html)
- [USB CDC Class Specification](https://www.usb.org/document-library/class-definitions-communication-devices-11)
