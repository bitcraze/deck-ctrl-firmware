# Deck Control Firmware

This firmware runs in a STM32C011 used for identification and control of the Crazyflie examption decks.
It replaces the One Wire memory currently used for identification purpose.
This firmware implements a special enumeration algorithm that allows to connect as many deck-control chip as needed on the same I2C bus.

## Features

The firmware implements the following capabilities:

- Deck identification (both what deck it is but also unique serial number)
- Coexist with multiple decks on the same I2C bus
- Reset MCU to reset state via I2C
- Bridge from I2C to various peripherals
  - [X] GPIO - Used to set GPIO direction and read/write GPIO pins
  - [ ] UART
  - [ ] SPI
  - [ ] ADC
  - [ ] PWM

## Architecture

### Memory Layout

| Address Range | Purpose | Access | POR State | Description |
|---------------|---------|---------|-----------|-------------|
| **Device Info** | | | | **Read-only firmware and product identification** |
| 0x0000-0x0001 | Magic number | R | 0xBCDC | Memory validation signature |
| 0x0002 | Major version | R | 0x00 | Deck control firmware major version |
| 0x0003 | Minor version | R | 0x01 | Deck control firmware minor version |
| 0x0004 | Vendor ID | R | 0xBC | Vendor identifier (0xBC = Bitcraze) |
| 0x0005 | Product ID | R | - | Board-specific product identifier |
| 0x0006 | Board revision | R | - | Hardware revision number |
| 0x0007-0x0014 | Product name | R | - | 14-byte product name string |
| **GPIO Control** | | | | **Pin direction and value control** |
| 0x1000-0x1001 | GPIO direction | R/W | 0x0000 | Pin direction (1=output, 0=input) |
| 0x1002-0x1003 | GPIO data | R/W | 0x0000 | Pin values (1=high, 0=low) |
| **I2C Configuration** | | | | **Address management** |
| 0x1800 | Address programming | W | 0x43 | New I2C address (takes effect after transaction) |
| **Device Identity** | | | | **Unique device identification** |
| 0x1900-0x190B | CPU unique ID | R | CPU ID | 12-byte unique device serial number |
| **Test Area** | | | | **Development and validation** |
| 0x1F00-0x1F0F | Scratchpad | R/W | 0x00 | 16-byte test memory area |

### GPIO Pin Mapping

The firmware controls 11 GPIO pins through memory-mapped registers. Each bit in the 16-bit GPIO registers corresponds to one pin:

**GPIOA pins**: PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA11, PA12 (bits 0-8)
**GPIOC pins**: PC14, PC15 (bits 9-10)

The GPIO control uses two 16-bit registers:
- **Direction register (0x1000-0x1001)**: Controls pin direction (1=output, 0=input)
- **Data register (0x1002-0x1003)**: Controls pin values for outputs, reads pin states for inputs

### I2C Protocol

All I2C addresses implement a 16-bit memory-mapped interface:
- **Read operation**: I2C write 2 bytes (address MSB, LSB), then restart and read data
- **Write operation**: I2C write 2 bytes (address MSB, LSB), then write data

## Multi-Device Discovery Algorithm

It's not feasible to give each deck a unique I2C address since the address space is limited. Although
it might be possible to give each type of deck a unique address, this would not scale well and also
block the possibility of using two of the same deck type at the same time. Instead the following has been
implemented:

At startup all decks have the same address, the discovery address. The host will try to read the serial
number from the default discovery address, while doing this all the decks will respond with their serial
at the same time. As the serial is being sent on the bus each I2C device will check so that the data matches
the data it sends, if not there will be an error state in the device. When this error state is reached
the device will change it's I2C address to a reset address. At the end of read only one unit will not
be in the error state, at this point the host will write to the device and set it's new I2C address. After
this the host will read the reset address and all the units which was previously in the error state will
jump back to the discovery address. This will continue until no devices are left on the discovery address.

The addresses are as follows:

- Reset to POR state: `0x41`
- Reset to discovery address: `0x42`
- Discovery address: `0x43`

### Host Perspective (Crazyflie)

Here's the flow of the discovery algorithm from the perspective of the user of the deck:

1. At POR (Power On Reset) all decks have the discovery address
2. Read 1 byte (at any address) from the ```Reset to POR state``` address to make sure the deck is
on the discovery address (you could be debugging and just resetting the host MCU and not toggling the power to the deck MCU)
3. Start the discovery procedure
    1. Read the serial number from the discovery address
    2. If no answer is received then the discovery is finished and you can exit
    3. If an answer is received then reprogram the I2C address of the deck making sure not to collide with other devices on the bus (by default this is ```Discovery address``` + 1)
    4. Read one byte (at any address) from the ```Reset to discovery address``` address, now all devices will jump back to the discovery address and you can start over from step 1
4. You now have a list of 0 or more serial numbers and I2C addresses of the decks on the bus which you can start using

### Firmware Perspective

From the deck firmware's perspective, the discovery process involves:
1. Starting with discovery address (0x43)
2. Monitoring I2C bus during serial number transmission
3. Detecting collisions by comparing transmitted vs. actual bus data
4. Switching to reset address (0x42) on collision detection
5. Returning to discovery address when reset command is received

## Development

### Prerequisites

- CMake 3.15+
- ARM GCC toolchain (`gcc-arm-none-eabi`)
- STM32CubeProgrammer (for flashing) or alternative tools like `probe-rs`/`pyocd`

### Building

The project uses CMake presets for different build configurations:

```bash
# Configure and build (Debug preset - default)
cmake --preset Debug
cmake --build build/Debug

# Other available presets: Release, RelWithDebInfo, MinSizeRel
cmake --preset Release
cmake --build build/Release
```

### Flashing

Using STM32CubeProgrammer:
```bash
STM32_Programmer_CLI --connect port=swd --download build/Debug/deck-control-firmware.elf -hardRst -rst
```

Alternative flashing tools:
```bash
# Using probe-rs
probe-rs download --chip stm32c011f6ux build/Debug/deck-control-firmware.elf

# Using pyocd
pyocd flash build/Debug/deck-control-firmware.elf --target stm32c011f6ux
```

### Debug Output

Debug printing is disabled by default. To enable:

1. Define `DBG_PRINT` macro in `Core/Inc/config.h`
2. Enable UART1 in STM32CubeMX and regenerate drivers
3. Debug output via UART1 (PA0/PA1) at 576000 baud

**Warning**: Debug prints in I2C ISR require reducing I2C speed to ~10kHz to prevent system overload.

### BOOT0 Configuration

For first-time programming, the BOOT0 pin must be enabled (factory default disables it):

```bash
# Display current option bytes
STM32_Programmer_CLI --connect port=swd -ob rdp=0x0 -ob displ

# Enable BOOT0 pin (set nBOOT_SEL=0)
STM32_Programmer_CLI --connect port=swd -ob rdp=0x0 -ob nBOOT_SEL=0
```

### VS Code Integration

The project includes VS Code configuration for STM32 extension:
- Build tasks in `.vscode/tasks.json`
- Debug configurations for ST-Link in `.vscode/launch.json`
- CMake integration with preset support

## Hardware Specifications

- **MCU**: STM32C011F6Ux (Cortex-M0+, 32KB Flash, 6KB RAM)
- **Development**: Generated with STM32CubeMX, uses STM32 HAL library
- **Communication**: I2C slave, optional UART debug output