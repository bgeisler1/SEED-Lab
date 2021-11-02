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
angular and radial velocities to be recorded and used to give parameters to the control system. Much of this 
code had been written during the course of Assignment 2, so it was mainly a matter of integrating it with the
physical control system (i.e. helping the step experiments be performed, aid in the simulation portion as well
as with fine-tuning the robot). The code written to obtain the radial and angular position is contained within
the file "GetRobotCoords.ino" in the Localization folder. The code added included the velocity and position 
calculation code, as well as the pin assignments and robot driving. The instantaneous angular and forward 
velocitiy code is shown below:

    instanteousAngularVelocity = RADII/100 * (angular_velocity_R - angular_velocity_L)/(DISTANCE/100);
    instanteousForwardVelocity = RADII/100 * (angular_velocity_R + angular_velocity_L);
    
The position and angular position code contributed is below:

    currentRho = RADII/100 * (absOfangularPos_R + angularPos_L)/2;
    currentPhi = RADII/100 * (absOfangularPos_R - angularPos_L)/(DISTANCE/100);
    
The majority of the work on Demo 1 was spent assisting Simulation and Control in writing the larger part of the
code and help with simulating the robot. This included helping determine the logic for the control, as well as
helping perform the step experiments and fine tuning the robot, including building a Finite State Machine to 
control which state the robot is in.

---------------------------------------------------------------------------------------------------------------
Simulation and Control
--
The main task for simulation and control is to model the positional and angular positional behavior using MATLAB and simulink models and then set up certain P/PD controllers for the closed loop situations. The secondary objecive is to form up an algorithm that not only moves the drone to the desired position but can also adjust the errors that accumulates along the way of the operation.

The final two outer-loop controllers design is shown as follows, one for angular velocity and one for translational velocity:
![image](https://user-images.githubusercontent.com/91347867/139537859-25ad0891-de43-4ec8-80cd-b12680695392.png)


---------------------------------------------------------------------------------------------------------------
Computer Vision
--
The idea behind computer vision was to be able to recognize blue painter's tape and assign the angle of the tape and the camera to an Export Variable that system integration outputs to an LCD screen.

---------------------------------------------------------------------------------------------------------------
System Integration
--
The system integration is responsible for fetching data from the computer vision side and transmit them to the LCD screen and display the readings of the angles.
