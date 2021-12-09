Descriptions of Final Demo
---------------------------------------------------------------------------------------------------------------

Ben Geisler
Jiuzou Zhang
Robert Schmidt
Louie Heilweil

This document contains descriptions of the code and tasks used in to create the Final Demo.
Note that the final code used for the demo is called Final_Code.ino, which is in the Arduino Code folder.

---------------------------------------------------------------------------------------------------------------
Localization
--
For this demo, the main localization focus was to determine how to make the ninety degree turn. After demo 2,
the robot could make it through the course until the sharper turns. The first step was to increase the angle of
the camera so that it would be less likely to miss a turn. This made it so that it moved less distance each time 
it processed the camera image updating. This made it so that it could make one of the sharper turns without losing sight
of the tape. The next step was to make it so that the robot would turn when it lost sight of the tape (at the 90 degree
turn), which is further discussed in simulation and control.


---------------------------------------------------------------------------------------------------------------
Simulation and Control
--
The main task for simulation and control is to update the state machine so that once the robot detects the ninety degree turn,or loses its sight
it can proceed to the dedeicated turn state and complete the task after a custom delay of 2s (which is used to make sure it is not picking up new values). Additionally, some fine tuning of the controller parameters
is also done to make sure the calculated distance and the actual moving distance are as close as possible. Some customer scaling
factors were also added to adpt to different battery voltages in case the robot over or undershoots in the turn state.


---------------------------------------------------------------------------------------------------------------
Computer Vision
--
The Computer Vision subsystem is responsible for recognizing blue painters tape and isolating the tape in its environment. The center of the tape, and the angle of the tape with  respect to the camera is found and assigned to variables that are exported to the control subsystem. The code is also capable of finding when the tape is at the vision limit of the screen.

---------------------------------------------------------------------------------------------------------------
System Integration
--
The system integration is responsible for 
