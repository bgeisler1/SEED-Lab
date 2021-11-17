Descriptions of Demo 2
---------------------------------------------------------------------------------------------------------------

Ben Geisler
Jiuzou Zhang
Robert Schmidt
Louie Heilweil

This document contains descriptions of the code and tasks used in to create Demo 2.

---------------------------------------------------------------------------------------------------------------
Localization
--

---------------------------------------------------------------------------------------------------------------
Simulation and Control
--
The main task for simulation and control is to create a finite state machine to iterate through the states of turning and moving 
forward by using the transmitted values of angles of theta and phi to account for the errors and adjust the robot to the desired
position. As our computer vision takes a relatively long time to read in and analyze the input video footage. A state of waiting 5
seconds is needed to prevent the robot of going over and not seeing the tape. The moving distance is also of a set distance of 20-30
cm instead of directing driving towards the identified marker in order to make infinitesimally closer approach to the center of the tape.

The general logic of the FSM, as shown in these two key states, is shown below:



---------------------------------------------------------------------------------------------------------------
Computer Vision
--

---------------------------------------------------------------------------------------------------------------
System Integration
--

