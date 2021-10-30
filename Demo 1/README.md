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

---------------------------------------------------------------------------------------------------------------
Computer Vision
--

---------------------------------------------------------------------------------------------------------------
System Integration
