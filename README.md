# mueb4-firmware

Firmware for 4th generation MUEB.

## Prerequisites

[STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)

## How to import the project

Open **STM32CubeIDE**

Use File->Open projects from file system

Specify "Import source"

Select **mueb4-firmware** folder, click Finish

### Download ST drivers

Help->Manage embedded software packages

Under **STM32F0**, select **STM32Cube MCU Package for STM32F0 Series** with the **1.11.x version**

### Generating code

Open **mueb4-firmware.ioc** from **STM32CubeIDE**

Project->Generate code(ALT+K)

**Code generator only generates/updates main.c and the peripheral files not main.cc file**

**Code generator modifies the optimization level to -O3, make sure to discard it!**

## Important hardware information

### Microcontroller

https://www.st.com/en/microcontrollers-microprocessors/stm32f030c8.html

| Part No     | Reference                   | Package | Flash     | RAM      | IO   | Freq   |
| :---------- | --------------------------- | ------- | --------- | -------- | ---- | ------ |
| STM32F030C8 | STM32F030C8T6/STM32F030C8Tx | LQFP48  | 64 kBytes | 8 kBytes | 39   | 48 MHz |

### WIZnet w5500 ethernet chip

https://www.wiznet.io/product-item/w5500/

https://github.com/Wiznet/ioLibrary_Driver

### Microchip 24AA02E48T-I/OT

https://www.microchip.com/wwwproducts/en/24AA02E48