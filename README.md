## Table of Contents

* [Introduction](#introduction)
* [Navigating the Repository](#navigating-the-repository)
* [Required Tools](#required-tools)
* [Code Examples List](#code-examples-list)
* [References](#references)

# Introduction
This repository contains examples and demos for PSoC 6 MCU family of devices, a single chip solution for the emerging IoT devices. PSoC 6 MCU bridges the gap between expensive, power hungry application processors and low‑performance microcontrollers (MCUs). The ultra‑low‑power, dual-core architecture of PSoC 6 MCU offers the processing performance needed by IoT devices, eliminating the tradeoffs between power and performance.

Cypress provides a wealth of data at [www.cypress.com](http://www.cypress.com/) to help you select the right PSoC device and effectively integrate it into your design. Visit our [PSoC 6 MCU](http://www.cypress.com/products/32-bit-arm-cortex-m4-psoc-6) webpage to explore more about PSoC 6 MCU family of device.
Feel free to explore through the code example source files and let us innovate together!

# Navigating the Repository

This repository provides examples that demonstrates how to develop PSoC 6 MCU based digital designs. PSoC 6 MCU provides wide digital resources like Serial Communication block (SCB) for UART, I2C, SPI based serial communication, TCPWM block for timer/counter/PWM based designs, UDBs for implementing digital functions like encoders, decoders, state machines etc. The examples listed out in this repository helps you understand how to use them and incorporate these features in your design.
If you are new to developing projects with PSoC 6 MCU, we recommend you to refer the [PSoC 6 Getting Started GitHub](https://github.com/cypresssemiconductorco/PSoC-6-MCU-Getting-Started) page which can help you familiarize with device features and guides you to create a simple PSoC 6 design with PSoC Creator IDE. For other block specific design please visit the following GitHub Pages:

For block level examples please refer to the following GitHub repositories:

#### 1. [Analog Designs](https://github.com/cypresssemiconductorco/PSoC-6-MCU-Analog-Designs)
#### 2. [BLE Connectivity Designs](https://github.com/cypresssemiconductorco/PSoC-6-MCU-BLE-Connectivity-Designs)
#### 3. [Audio Designs]
#### 4. [Device Related Designs]
#### 5. [System-Level Designs]
#### 6. [PSoC 6 Pioneer Kit Designs]

You can use these block level examples to guide you through the development of a system-level design using PSoC 6 MCU. All the code examples in this repository comes with well documented design guidelines to help you understand the design and how to develop it. The code examples and their associated documentation are in the Code Example folder in the repository.

# Required Tools

## Software
### Integrated Development Environment (IDE)
To use the code examples in this repository, please download and install
[PSoC Creator](http://www.cypress.com/products/psoc-creator)

## Hardware
### PSoC 6 MCU Development Kits
* [CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit](http://www.cypress.com/documentation/development-kitsboards/psoc-6-ble-pioneer-kit).

* [CY8CKIT-062-WiFi-BT PSoC 6 WiFi-BT Pioneer Kit](http://www.cypress.com/documentation/development-kitsboards/psoc-6-wifi-bt-pioneer-kit). 

**Note** Please refer to the code example documentation for selecting the appropriate kit for testing the project

## Code Examples List
### UART
#### 1. CE219656 - PSoC 6 MCU UART using Low Level APIs
This example demonstrates the UART transmit and receive operation in PSoC 6 MCU using low level APIs. This is done using polling, ISR, and DMA methods.
#### 2. CE221117 - PSoC 6 MCU UART using High Level APIs
This example demonstrates the serial communication block (SCB) based UART transmit and receive operation in PSoC® 6 MCU using high level APIs found in peripheral driver library (PDL).
#### 3. CE223001 - PSoC 6 MCU UART printf
This example demonstrates how to use the printf function with a Serial Communication Block (SCB) based UART in PSoC 6 MCU.
### SPI
#### 1. CE221120 - PSoC 6 MCU SPI Master
This example demonstrates the use of SPI SCB (Serial Communication Block) Component for PSoC 6 MCU in master mode. Four different subprojects show the use of Peripheral Driver Library (PDL) functions to communicate with an SPI slave.
#### 2. CE221121 - PSoC 6 MCU SPI Slave
This example demonstrates the use of PSoC 6 MCU Serial Communication Block (SCB) Component in SPI slave mode. Four projects show the use of Peripheral Driver Library (PDL) functions to receive data from an SPI master in different modes.
#### I2C
#### 1. CE220541 - PSoC 6 MCU SCB EZI2C
This code example demonstrates the implementation of an EZI2C Slave using the SCB Component on a PSoC 6 MCU. It also demonstrates how to control the color and intensity of an RGB LED using TCPWM Components.
#### 2. CE220818 - PSoC 6 MCU I2C Master
This example demonstrates the use of I2C SCB (Serial Control Block) Component for PSoC 6 MCU in master mode. Three different subprojects show the use of Peripheral Driver Library (PDL) functions to communicate with I2C and EzI2C slave.
#### 3. CE221119 – PSoC 6 MCU I2C Slave
This example demonstrates the operation of the PSoC 6 MCU Serial Control Block (SCB) in I2C Slave mode. Two projects show the use of Peripheral Driver Library (PDL) functions to receive data from an I2C Master in different modes.
### Smart IO
#### 1. CE219490 – PSoC 6 MCU Breathing LED using Smart IO 
This example uses a PWM and PSoC 6 MCU Smart IO Component to implement a breathing LED, where an LED gradually cycles through increasing and decreasing brightness levels. There is no CPU usage except for the initialization of PWM and Smart IO Components. This example also demonstrates how to use Smart IO to route the same signal through multiple I/O pins on the same port. This is demonstrated by inverting the signal using Smart IO and then routing the signal to another pin thus creating two breathing LEDs that are out of phase.
#### 2. CE219506 – PSoC 6 MCU Clock Buffer with Smart IO 
This code example demonstrates how the lookup tables (LUTs) in the Smart IO block can be used to buffer an external signal. It further shows how to use the buffered signal to drive a high load at the output. The PSoC 6 MCU device is put to Deep Sleep mode.
### TCPWM
#### 1. CE220169 – PSoC 6 MCU: Periodic Interrupt using TCPWM
This example demonstrates how to generate a periodic interrupt using the timer/counter pulse-width modulation (TCPWM) Component in Timer/Counter mode for PSoC 6 MCU devices.
#### 2. CE220290 - PSoC 6 MCU TCPWM Breathing LED
This code example demonstrates the implementation of an LED breathing effect using the TCPWM Component on a PSoC 6 MCU.
#### 3. CE220291 - PSoC 6 MCU TCPWM Square Wave
This code example demonstrates how to generate a square wave using the TCPWM Component on a PSoC 6 MCU.
#### 4. CE220692 – PSoC 6 MCU: Frequenc Measurement Using TCPWM
This code example demonstrates how to use the TCPWM Component in PSoC 6 MCU to measure the frequency of a periodic digital signal.
#### 5. CE220799 – PSoC 6 MCU: Direction Detection Using Quadrature Decoder
This code example demonstrates how to use the TCPWM Component in quadrature decoder mode in PSoC 6 MCU.
#### 6. CE221118 – PSoC 6 MCU TCPWM Event Counter
This example demonstrates the use of PSoC 6 MCU TCPWM to count the number of external events

## References
#### 1. PSoC 6 MCU
PSoC 6 bridges the gap between expensive, power hungry application processors and low‑performance microcontrollers (MCUs). The ultra‑low‑power PSoC 6 MCU architecture offers the processing performance needed by IoT devices, eliminating the tradeoffs between power and performance. The PSoC 6 MCU contains a dual‑core architecture, with both cores on a single chip. It has an Arm® Cortex®‑M4 for high‑performance tasks, and an Arm® Cortex®‑M0+ for low-power tasks, and with security built-in, your IoT system is protected.
To learn more on the device, please visit our [PSoC 6 MCU](http://www.cypress.com/products/32-bit-arm-cortex-m4-psoc-6) webpage.

####  2. PSoC 6 MCU Learning resource list
##### 2.1 PSoC 6 MCU Datasheets
Device datasheets list the features and electrical specifications of PSoC 6 families of devices: [PSoC 6 MCU Datasheets](http://www.cypress.com/search/all?f%5B0%5D=meta_type%3Atechnical_documents&f%5B1%5D=resource_meta_type%3A575&f%5B2%5D=field_related_products%3A114026)
##### 2.2 PSoC 6 MCU Application Notes
Application notes are available on the Cypress website to assist you with designing your PSoC application: [A list of PSoC 6 MCU ANs](http://www.cypress.com/psoc6an)
##### 2.3 PSoC 6 MCU Component Datasheets
PSoC Creator utilizes "components" as interfaces to functional Hardware (HW). Each component in PSoC Creator has an associated datasheet that describes the functionality, APIs, and electrical specifications for the HW. You can access component datasheets in PSoC Creator by right-clicking a component on the schematic page or by going through the component library listing. You can also access component datasheets from the Cypress website: [PSoC 6 Component Datasheets](http://www.cypress.com/documentation/component-datasheets)
##### 2.4 PSoC 6 MCU Technical Reference Manuals (TRM)
The TRM provides detailed descriptions of the internal architecture of PSoC 6 devices:[PSoC 6 MCU TRMs](http://www.cypress.com/psoc6trm)

## FAQ

### Technical Support
Need support for your design and development questions? Check out the [Cypress Developer Community 3.0](https://community.cypress.com/welcome).  

Interact with technical experts in the embedded design community and receive answers verified by Cypress' very best applications engineers. You'll also have access to robust technical documentation, active conversation threads, and rich multimedia content. 

You can also use the following support resources if you need quick assistance:
##### Self-help: [Technical Support](http://www.cypress.com/support)
##### Local Sales office locations: [Sales Office](http://www.cypress.com/about-us/sales-offices)
