# FRC-2020

![Build](https://github.com/FRC3476/FRC-2020/workflows/Build/badge.svg)

FRC 2020 robot code

Code for the robot we drove is in the 2020-robot branch. The master branch has code for a v2 of our robot that was designed, but not built.

The code that we have written is located in src/main/java/frc/

The subsystem folder has all the code that controls the robot
  - Each class controls a diffrent part of the robot
  - All classes in this folder follow the singleton programming practices.

The robot folder has the main class and the constants class
  - The Robot.java takes in controller inputs and calls subsystem classes to control the robot 

The Utility folder contains code that is reused alot and does not directly control the robot
