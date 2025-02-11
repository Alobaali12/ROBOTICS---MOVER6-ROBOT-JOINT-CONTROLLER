# A Joint Controller for a MOVER6 6 robot written in C++ & MATLAB

# System Architecture
![Picture1](https://github.com/user-attachments/assets/dcf0b8d6-2417-47a5-af5d-9f5cd0c495f8)

# MATLAB:
% Authors: Ali Mohamed and Elion Selko
% Code Description: This script takes a user defined end effector translation and
% rotation defined in X, Y, Z, A, B, C which determines the pose of end effector frame with
% respect to robot base frame.
% The end effector moves linearly between the initial point and the
% final point.
% The URDF file is equipped with joint limits, which ik solver needs to satisfy
% whenever an Inverse Kinematics problem is solved, otherwise the code
% breaks and throws an error.
% The algorithm is also equipped with a customized inverse kinematics
% solver which the user can use if they uncomment lines [90-94] and [100-106]
% Additionally the user may establish a communication with ROS topics and
% publish joint conifgurations by uncommenting lines [40-46] and [129-135]

# C++: 
Authors: Ali Mohamed & Elion Selko
This C++ code implements a motion controller for the MOVER6 robot.
It accepts an N x 6 vector of waypoint joint configurations, provided either via a terminal window or
MATLAB.
The code reconstructs the N x 6 waypoint configuration matrix, sequentially calculates the required joint
velocities, and publishes them to the joint jog topic to actuate the robot.
The implementation follows an Object-Oriented Programming (OOP) architecture with distinct `Robot` and
`Joint` classes, each encapsulating their respective attributes and methods.
Various operational flags can be toggled, such as selecting between relative or trapezoidal velocity motion
profiles.
