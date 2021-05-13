# iCub-applications

## Responsible:
<img src="https://user-images.githubusercontent.com/43743081/89022636-a17e9e00-d322-11ea-9abd-92cda85d3705.jpeg" width="180">

This repo will contain the applications needed to work on the torque control on iCub robot.

## Background
Torque control has been implemented on legged robots showing excellent performances. However, the implementation is challenging for those humanoid robots that have been originally built to be position controlled sicne they do not provide joint torque sensors and present a large friction.
For this reason in iCub this control strategy has been implemented thanks to the estimation of joint torques and external forces from whole-body model at multi-joint level. The current approach is shown in the following diagram.

<img src="https://github.com/loc2/element_joint-torque-control/blob/master/doc/iCub_LowLevelControl.jpg" width="800">

## Objectives
The objective of the element is to devel a controller architecture that allows to achieve torque based control for the iCub3 motors exploiting the low-level current control, instead of the low-level PWM control.

## Outcomes
The possible outcomes can be summarized in the following list:
* **A new motor velocity estimation to be implemented on the 2foc board (running at 20 khz)**
* **A C++ software that gives the possibility to acquire datasets for the motor parameters identification**
* **A C++ software that implements the fixed base TSID to test the torque control on the leg motors** :heavy_check_mark:
* **Improved tracking of cartesian positions using the robot dynamics when the motors are torque controlled**

## Tasks
The following epic-tasks can been identified:
* **Integrate a new velocity estimation on the 2foc board and with the walking of iCub3**
* **Implements fixed base TSID**
* **Enable torque control via current control on iCub3 legs**
* **Identify iCub3 legs motors parameters**
* **Improve cartesian position tracking of iCub3 leg motors**
