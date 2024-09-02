# Oximeter Project on STM32G431KB NUCLEO 🚀

## Project Overview 📅

A few days ago, I started working on an oximeter project based on the STM32G431KB NUCLEO board. I chose this board because it belongs to the latest G4 series from STM32, which offers impressive performance features such as a maximum clock frequency of 170 MHz. Although the SRAM memory on this board is somewhat limited (only 16mB+6mB+10mB), making screen buffering a challenging task, the high clock speed compensates for this by providing robust processing capabilities. 🛠️

## Hardware Components 🔧

### Display 📺

For the display, I selected a 1.47" LCD IPS screen from Waveshare, which I connected via SPI1. This display provides a clear and vibrant interface for showing the oximeter readings, but its integration requires careful management of the limited SRAM on the STM32G431KB. Here below you have pinout map of LCD IPS SPI1 connection:
![Oximeter Project](https://i.imgur.com/qE09dE0.png)
![Oximeter Project](https://i.imgur.com/moJIyBc.png)
The gray cable (BL) is not connected because we do not use it.

### Sensor 🩺

No oximeter is complete without a reliable sensor to measure heart rate and blood oxygen saturation levels (SpO2). For this purpose, I chose the Fermion v2.0 MAX30102 sensor from DFRobot, which communicates via I2C. This sensor falls within a good price range and includes a built-in microcontroller that handles data processing algorithms and provides control registers for managing the voltage on the red and infrared LEDs.

Additionally, the MAX30102 sensor features an onboard FIFO (First In, First Out) buffer that must be managed carefully to prevent data staleness. Although the sensor supports UART communication, I opted for I2C due to its complexity, energy efficiency, and data integrity benefits.

## Features and Challenges ⚙️

- **Microcontroller:** STM32G431KB with 170 MHz clock speed.
- **Display:** 1.47" LCD IPS from Waveshare connected via SPI1.
- **Sensor:** Fermion v2.0 MAX30102 (I2C) for heart rate and SpO2 measurements.
- **Challenges:** Managing the limited SRAM for display buffering and ensuring timely data handling from the sensor's FIFO.

This project is still in the early stages, and I am excited to continue developing and overcoming the challenges associated with building an efficient and reliable oximeter using these components. Stay tuned for more updates as the project progresses! 😊

## Future Work 🛠️

- **Implementing Display Buffering:** Efficiently manage screen data with limited SRAM.
- **Sensor Data Management:** Optimize FIFO handling to ensure accurate and up-to-date readings.
- **Power Management:** Explore strategies for reducing power consumption while maintaining performance.

Feel free to follow along as I document my journey in developing this oximeter, and don't hesitate to share any feedback or suggestions! 🙌
