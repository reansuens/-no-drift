# Project Documentation

## Overview
This project implements advanced motion control algorithms suitable for various applications, including robotics and sensor fusion.

## Features
- Real-time motion control
- Sensor fusion capabilities
- Customizable PID controller
- Support for various filtering algorithms

## Hardware Specifications
- Microcontroller: Saeed xiao C3
- Sensors: MPU6050, quadrature encoders. 
- Power Supply: Two 3.7v Lithium batteries in serial (7.4v) 

## Algorithm Details
### Complementary Filter
The Complementary Filter is a simple and efficient algorithm for sensor fusion that combines accelerometer and gyroscope data to provide stable orientation estimates.



### Madgwick Filter
The Madgwick filter is an advanced sensor fusion algorithm that uses quaternion representation to derive orientation from accelerometer, gyroscope, and magnetometer data, optimizing for real-time performance.

### Kalman Filter
The Kalman Filter is a statistical algorithm that provides optimal estimates by utilizing a series of measurements over time, suitable for noisy sensory data.

## PID Controller Implementation
The PID controller adjusts the control outputs based on the proportional, integral, and derivative errors to achieve desired motion control objectives.

## Motion Control
This section covers the control logic used to manage motors and actuators based on sensor feedback, ensuring precise movement.

## Testing Guide
- Describe the testing procedures to validate the hardware and software functionality.
- Include unit tests and integration tests to ensure reliability.

## Build Instructions
Follow these steps to build the project:
1. Clone the repository.
2. Install necessary dependencies.
3. Upload the code to the microcontroller using `cargo espflash flash --monitor` 
4. Configure the hardware as per the specifications.

## Comparison Table of Filter Types
| Filter Type   | Best Use Case                | Limitations                     |
|---------------|-----------------------------|----------------------------------|
| Complementary | Basic motion tracking       | Limited to specific conditions  |
| Madgwick      | Complex environment tracking| Requires tuning                 |
| Kalman        | Optimal state estimation    | Computationally intensive       |
