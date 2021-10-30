Descriptions of Demo 1
---------------------------------------------------------------------------------------------------------------

Ben Geisler
Jiuzou Zhang
Robert Schmidt
Louie Heilweil

This document contains descriptions of the code and tasks used in to create Demo 1.

---------------------------------------------------------------------------------------------------------------
Localization
--
The main task for localization this week was to produce code that allowed the angular and radial positions and 
angular and radial velocities to be recorded and used to give parameters to the control system.

---------------------------------------------------------------------------------------------------------------
Simulation and Control
--
 The main task for simulation and control is to model the positional and angular positional behavior using MATLAB
 and simulink models and then set up certain P/PD controllers for the closed loop situations. The secondary objecive is to form up an algorithm that not only move   the drone to the desired position but can also adjust the errors 
 that accumulates along the way of the operation.
 
 The final two outer-loop controllers design is shown as follows, one for angular velocity and one for translational velocity:
 
 <img width="1387" alt="image" src="https://user-images.githubusercontent.com/91347870/139518162-8fc1f452-2fff-4c61-be63-bdbf5a7858fa.png">


---------------------------------------------------------------------------------------------------------------
Computer Vision
--
The idea behind computer vision was to be able to recognize blue painter's tape and assign the angle of the tape 
and the camera to an Export Variable that system integration outputs to an LCD scren.

---------------------------------------------------------------------------------------------------------------
System Integration
--
The system integration is responsible for fetching data from the computer vision side and transmit them to the LCD screen and display the readings of the angles.
