# Improved Teleoperation of Dual Robotic Manipulators using Vision Based Control
This capstone project presents the culmination of my BSc. Computer Engineering degree from Ashesi University. 

In depth report and documentation can be found in the root directory titled *Report Documentation*

## Demo
A demo of this project can be found [here](https://www.youtube.com/shorts/dp0F9bslgc4)

## Abstract 
The advancement of manual control of dual robotic manipulators has considerably lagged
behind the progress of the single robotic manipulator. The widespread solutions for dual
robotic manipulators consist mainly of joysticks and wearable technology, which are often
expensive and complex to operate or uncomfortable to use. In order to bridge the divide
between single and dual manipulators and provide alternatives to currently existing
solutions, this study proposes an improved way of teleoperating dual robotic manipulators
through vision-based control. By leveraging the Leap Motion Controller as the visual input
sensor, the FRDM KL25Z board as the microcontroller unit, and Bluetooth as the
transmission media, a simple real-time solution for visually controlling two 4-DOF robotic
manipulators was designed, prototyped, and tested. Experimental results showed that the
proposed solution is accurate, relatively cheaper, easy to use and understand, and more than
capable of competing with the currently existing solutions.

## Design
![Solution](https://i.ibb.co/ZczfWQT/High-Level-Diagram-2.jpg)

## Implementation
To create a single system capable of interfacing and remotely connecting the Leap
Motion Controller, the FRDMKL25Z microcontroller, and the dual robotic manipulators,
software for the two main subsystems have to be developed: 
- The operator's subsystem software component (Developed in Python using Leap Motion SDK)
- The robotic manipulators' subsystem software component (Developed in C using MCUXpresso IDE)
