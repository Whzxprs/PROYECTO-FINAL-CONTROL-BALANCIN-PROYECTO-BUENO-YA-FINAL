# Two-Wheeled Inverted Pendulum Robot

This repository contains the implementation of a control algorithm for a two-wheeled inverted pendulum robot, inspired by the concepts and experiments presented in the book *Automatic Control with Experiments, 2nd Edition*. The project demonstrates advanced control techniques using the ESP32-S3 microcontroller and provides a platform for learning and experimenting with dynamic systems.

## Features

- Real-time control of a two-wheeled robot using a control algorithm.
- Integration of sensors and motor drivers for precise control and stability.
- Developed using the ESP-IDF framework for ESP32-S3, showcasing its capabilities in embedded systems.
- Expandable for remote control, data logging, and wireless updates via Wi-Fi or Bluetooth.

## Materials

To build the robot, you will need the following components:

- **ESP32-S3 microcontroller**: Main controller for processing and executing control algorithms.
- **MCP3008 ADC**: Analog-to-digital converter for reading analog signals from sensors.
- **MPU6050 IMU (6 DoF accelerometer and gyroscope)**: For measuring the robot's tilt and angular velocity.
- **TB6612FQN Driver IC**: Dual DC motor driver for controlling the robot's wheels.
- **DC Motors with encoders**: For precise movement and feedback.
- **Chassis**: A sturdy structure to hold all components.
- **Wheels**: Compatible with the motors to provide stable movement.
- **11.1V LiPo battery**: To power the system.

## Prerequisites

Before starting, ensure you have the following:

1. **Development Environment**:
   - Install ESP-IDF following the [official setup guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/).
   - A computer with support for the ESP-IDF toolchain.
   - A USB cable for flashing and debugging.

2. **Tools**:
   - Soldering iron and solder.
   - Multimeter for testing connections.
   - Screwdrivers and assembly tools for the chassis.

3. **Programming Knowledge**:
   - Familiarity with C programming.
   - Understanding of control systems and algorithms.

## Getting Started

1. Clone this repository:
   ```bash
   git clone https://github.com/yourusername/two-wheeled-pendulum.git
   cd two-wheeled-pendulum
   ```

2. Build the firmware:
    ```bash
    idf.py build
    ```

3. Flash the firmware:
    ```bash
    idf.py flash
    ```

4. Monitor the output:
    ```bash
    idf.py monitor
    ```

## Applications

- **Educational**: Demonstrates control system principles in a practical and engaging way.
- **Research**: Serves as a platform for experimenting with advanced control algorithms.
- **Robotics**: Can be expanded for use in autonomous navigation or remote control systems.

## Future Improvements

- Add wireless control via Bluetooth or Wi-Fi.
- Improve the following sensor technique to achieve higher speeds on the track.
- Enhance the control algorithm with adaptive or predictive techniques.

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.

## Acknowledgments

- Concepts and inspiration drawn from *Automatic Control with Experiments, 2nd Edition*.
- Community contributions and resources for ESP-IDF and robotics projects.
