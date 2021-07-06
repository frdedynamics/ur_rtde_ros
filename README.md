# ur_rtde_ros

Controlling and receiving data from an UR robot using the [Real-Time Data Exchange (RTDE)](https://www.universal-robots.com/how-tos-and-faqs/how-to/ur-how-tos/real-time-data-exchange-rtde-guide-22229/). 

Python binding (pybind11) is used in this package.

The main python version is python3 for this package. Some extra settings needed if python2 is wanted to be used. See more: https://sdurobotics.gitlab.io/ur_rtde/index.html



## Installation

Follow the instructions to install **Universal Robots RTDE C++ InterfaceUniversal Robots RTDE C++ Interface** from this website https://sdurobotics.gitlab.io/ur_rtde/index.html .

Install and build `ur_rtde` .



## Extra steps

To make use of our ROS package to see the the built `ur_rtde` libraries, modify the CMakelist.txt adding `find_package(ur_rtde REQUIRED PATH your-ur-rtde-path)`
