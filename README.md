# WRO 2024 Future Engineers by PAiR Engineers.

This repository contains all the necessary code, documentation, and resources for our WRO 2024 Future Engineers project. The project utilizes an RC crawler 4x4, controlled using two Arduino Nano boards and a Raspberry Pi 5, with additional components including a LiPo battery, motors, ultrasonic sensors, and an MPU6050 sensor.

## Project Overview

The project is a self-navigating car capable of mapping a track using ultrasonic sensors and controlling its movement through the gathered data. The system includes:

- **Chassis**: RC crawler toy 4x4
- **Motors**: 2x small DC motors for movement control
- **Battery**: 1x LiPo battery (2200 mAh)
- **Microcontrollers**: 2x Arduino Nano (Input Nano and Output Nano)
- **Processor**: 1x Raspberry Pi 5 for data processing
- **Sensors**:
  - 4x Ultrasonic sensors for distance measurement
  - 1x MPU6050 sensor for motion detection and stability control
- **Steering**: 1x MG995 servo for steering

## Repository Structure

- **schemes/**: Contains information and specifications of the hardware components used in the project.
- **src/**: Contains Arduino code for the Nano boards.
  - `OUTPUT_NANO/` – Controls the motors of the RC crawler.
  - `INPUT_NANO/` – Handles sensor data processing from the ultrasonic sensors and MPU6050.
  - `Main_R1/` – Open Challenge.
  - `Main_R2/` – Includes camera processing and data mapping script For obstacle Round.
- **Documentation/**: Detailed documentation and setup instructions for the project.
- **t-photos/**: Team Photos.
- **v-photos/**: Vehicle/Robot Photos.

## Installation and Setup

### Raspberry Pi 5 Dependencies

To run the Python scripts on the Raspberry Pi, install the following dependencies:

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

1. **Install Arduino IDE**: Download the Arduino IDE from [here](https://www.arduino.cc/en/software).

2. **Connect and Upload**: 

   - Connect each Arduino Nano to your computer using a USB cable.
   - Upload `OUTPU_NANO.ino` to **Output Nano**.
   - Upload `INPUT.ino` to **Input Nano**.

### Running the Project

1. **Connect all Components**: Refer to `Documentation/Setup_Instructions.md` for detailed wiring diagrams and connection setups.


3. **Control Motors and Sensors**: The Arduino boards will handle motor and sensor control based on commands received from the Raspberry Pi.

## How It Works

- The RC crawler's motors are controlled by **Output Nano** based on data from sensors processed by **Input Nano**.
- **Input Nano** collects data from four ultrasonic sensors (left, right, front, back) and the MPU6050 sensor, which is then sent to the Raspberry Pi for processing.
- The Raspberry Pi processes the sensor data and makes decisions for autonomous navigation.


## License

This project is licensed under the MIT License. See the `LICENSE` file for more details.

## Authors

- **PAiR ENGINEERS** - Team 

## Acknowledgments

- Special thanks to **Shikha Singh**, our mentor, for guiding us throughout the project.
