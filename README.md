# TeleManus Project - leap_ik_control and servo_control Packages


## Overview

This project, TeleManus, uses a Leap Motion Controller to manipulate a 6-DOF robotic arm. The system calculates inverse kinematics (IK) based on hand movements and sends commands to control servos via an STM32 microcontroller. The setup includes:

- **leap_ik_control**: Captures hand data from the Leap Motion, calculates IK, and publishes servo angles.
- **servo_control**: Subscribes to servo angle commands and sends them to the STM32 to control the robotic arm.



## System Requirements

- **Raspberry Pi 4** with Ubuntu 22.04
- **[STM32 Microcontroller (ROS Expansion Board)](https://www.amazon.com/dp/B0CZHPCVLX/ref=twister_B0BX52PQRT?_encoding=UTF8&th=1)**
- **Laptop** (optional) for IK calculations and wireless communication with Raspberry Pi

## Installation Instructions

### Step 1: Install Prerequisites

1. Install system dependencies:

```
sudo apt update && sudo apt install -y python3-pip libusb-1.0-0-dev 
```

2. Install ROS 2 (Humble) for Ubuntu 22.04:
```
sudo apt install ros-humble-desktop
```

3. Set up Python dependencies from `requirements.txt`:
```
pip install -r requirements.txt
```

4. **STM32 Setup**: Follow the [STM32 Setup Instructions](https://github.com/TheTacoBytes/STM32-ROS-ExpansionBoard) to configure the STM32 microcontroller for use with Python.


### Step 2: Install Leap Motion SDK

Follow these instructions from the Ultraleap documentation for Linux setup:  
[Ultraleap Documentation for Linux](https://docs.ultraleap.com/linux/)

#### Quick steps:

1. Download and install the Leap Motion SDK for Linux.
```
wget https://developer.leapmotion.com/releases/Leap_Motion_Developer_Kit_4.1.0.tgz
```

2. Extract the downloaded package and navigate into the directory:
```
tar -xvf Leap_Motion_Developer_Kit_4.1.0.tgz
cd Leap_Motion_Developer_Kit_4.1.0
```

3. Install the Leap Daemon to start capturing hand-tracking data:
```
sudo cp lib/libLeap.so /usr/lib/

sudo ./install.sh
```


### Step 3: Install Ultraleap Python Bindings

To control the Leap Motion from Python, install the bindings from the GitHub repository:
```
pip install git+https://github.com/ultraleap/leapc-python-bindings.git@2341c6c3db5dc08236bb52890d8c43a224139694#egg=leap&subdirectory=leapc-python-api
```


### Step 4: Run the TeleManus Nodes

1. Start ROS and source the environment:
```
source /opt/ros/humble/setup.bash
```

2. Run the `leap_ik_control` package to calculate IK and publish servo commands:
```
ros2 run leap_ik_control leap_ik
```

3. Run the `servo_control` package to receive the commands and control the STM32:
```
ros2 run servo_control servo_control
```


## Usage Notes

- Make sure the STM32 is connected to the Raspberry Pi 4 and configured for UART communication.
- The `leap_ik_control` node uses Leap Motion data to compute IK and publishes the joint angles for the robotic arm.
- The `servo_control` node listens for these commands and forwards them to the STM32 to drive the servos.



## Project Structure

- **leap_ik_control**: Calculates and publishes servo angles from Leap Motion hand-tracking data.
- **servo_control**: Receives angle commands and communicates with the STM32 to move the servos.

## License

This project is licensed under the Apache-2.0 License.

