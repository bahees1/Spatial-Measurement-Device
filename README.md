# Spatial-Measurement-Device

## Overview
This project provides a quick and lightweight way to capture a 3D representation of small indoor spaces, such as hallways or rooms. By using a VL53L1X Time-of-Flight (ToF) sensor attached to a stepper motor, we can sweep the sensor through 360° and measure distances at set intervals. These measurements are then transferred to a MSP432E401Y microcontroller for initial processing and sent via UART to a Python script, which calculates 3D coordinates and plots them with Open3D.

## How It’s Made
Sensor & Motor Setup

1. A VL53L1X ToF sensor is mounted on a 28BYJ-48 stepper motor.
The stepper motor rotates in fixed increments (e.g., 22.5° per step), allowing multiple distance measurements per rotation.
Microcontroller Coordination

2. The MSP432E401Y microcontroller uses I2C to communicate with the ToF sensor, retrieving distance data at each angle.
UART at 115200 bps is then used to send these distance readings to a connected PC.
Data Processing & Visualization

3. A Python script reads the streamed distance data in real time.
The script converts each distance and angle into (x, y, z) coordinates.
Open3D generates a point-cloud visualization, giving you a 3D map of the measured space.

## How I Optimized the Project
* Angle Step Size & Number of Frames
By allowing the user to configure the angle step size (e.g., 16 measurements per rotation) and the total number of scans, the system can quickly take coarse measurements or produce more detailed 3D maps if time permits.

* Trigonometric Calculations in Python
Offloading trigonometric and coordinate computations from the microcontroller to the Python script reduces load on the microcontroller. This not only simplifies the firmware but also speeds up the scanning process.

* Efficient Data Transfer
Using a 115200 bps UART rate provides a balance between reliable communication and speed, ensuring the measurement data is transmitted efficiently without overwhelming the microcontroller or the PC’s serial buffer.

* Modular Code
Separating sensor initialization, stepper control, and data transmission into distinct modules in the microcontroller code makes the system easier to maintain, debug, and upgrade.
