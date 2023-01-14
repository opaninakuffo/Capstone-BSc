# Usage
This folder contains the code pertaining to the Capstone Project titled Improved Teleoperation of Dual Robotic Manipulators using Vision Based Control.

The folder contains two main files and some supporting files: 
- The OperatorSubsystemSoftware.py file, written in Python, is the main program running on the opertors computer. In order to work, it requires a Leap Motion Controller as well the computer being connected via Bluetooth to the two HC-05 modules on the Robotic Manipulators Subsystem. 
- The RoboticManipulatorsSubsystem.c, queues.h and queues.c files are written in C programming language and must be flashed to an FRDMKL25Z board using the MCUXpresso IDE.

## Notice
The **Report Documentation** which can be found in the root directory, contains in-depth knowledge on how to connect and operate the interface.
