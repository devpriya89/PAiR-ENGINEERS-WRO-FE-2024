# WRO 2024 Future Engineers

This repository contains all the necessary code, documentation, and resources for our WRO 2024 Future Engineers project. The project utilizes an RC crawler 4x4, controlled using two Arduino Nano boards and a Raspberry Pi 5, with additional components including a LiPo battery and motors.

## Project Overview

The project is a self-navigating car capable of mapping a track using ultrasonic sensors and controlling its movement through the gathered data. The system uses:
- RC crawler toy 4x4 as the chassis
- 2x small DC motors for movement control
- 1x LiPo battery (2200 mAh)
- 2x Arduino Nano for sensor and motor control
- 1x Raspberry Pi 5 for data processing
- 1x MG995 servo for steering

## Repository Structure

- **Hardware/**: Contains information and specifications of the hardware components used in the project.
- **Arduino/**: Contains Arduino code for the Nano boards.
  - `Nano1_Motor_Control/` – Controls the motors of the RC crawler.
  - `Nano2_Sensor_Control/` – Handles sensor data processing.
- **RaspberryPi/**: Contains Python scripts for Raspberry Pi.
  - `Pi5_Scripts/` – Includes camera processing and data mapping scripts.
- **Documentation/**: Detailed documentation and setup instructions for the project.
- **Images/**: Visual resources including the car setup and circuit diagrams.

## Installation and Setup

### Raspberry Pi 5 Dependencies

To run the Python scripts on the Raspberry Pi, you need to install the following libraries:

1. Update and upgrade the Raspberry Pi:
    ```bash
    sudo apt-get update
    sudo apt-get upgrade
    ```

2. Install required Python libraries:
    - `curses`: For terminal-based user interfaces
    - `opencv-python`: For real-time video processing
    - `pyserial`: For serial communication with Arduino
    - `random`: Included in Python's standard library, no need to install separately

    Use the following command to install the libraries:

    ```bash
    pip3 install opencv-python pyserial
    ```

3. Install `curses` library:
    ```bash
    sudo apt-get install python3-curses
    ```

### Arduino Setup

1. **Install Arduino IDE**: You need the Arduino IDE to upload the code to your Nano boards. You can download it from [here](https://www.arduino.cc/en/software).

2. **Connect and Upload**: 
   - Connect each Arduino Nano to your computer using a USB cable.
   - Upload `motor_control.ino` to `Nano1`.
   - Upload `sensor_data.ino` to `Nano2`.

### Running the Project

1. **Connect all Components**: Refer to `Documentation/Setup_Instructions.md` for detailed wiring diagrams and connection setups.

2. **Start Raspberry Pi Script**:
    ```bash
    python3 RaspberryPi/Pi5_Scripts/data_mapping.py
    ```

3. **Control Motors and Sensors**: The Arduino boards will handle motor and sensor control based on commands received from the Raspberry Pi.

## How It Works

- The RC crawler's motors are controlled by `Nano1` based on the data from sensors processed by `Nano2`.
- `Nano2` collects data from ultrasonic sensors, which is then sent to the Raspberry Pi for processing.
- The Raspberry Pi processes the sensor data and makes decisions for autonomous navigation.

## Block Diagram

![Block Diagram](Documentation/Block_Diagram.png)

## License

This project is licensed under the MIT License. See the `LICENSE` file for more details.

## Authors

- [Your Name] - Team Lead
- [Team Members]

## Acknowledgments

- Special thanks to Shikha Singh, our mentor, for guiding us throughout the project.
