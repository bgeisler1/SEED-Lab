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
cm instead of directing driving towards the identified marker. This can make infinitesimally closer approaches to the center of the tape.

The general logic of the FSM, as shown in these two key states, is shown below:

SEARCH STATE(WAITING FOR CV SIGNALS) - TURN CALBIRATION -- TURN STATE (CALL TURN CONTROLLER) -- FORWARD CALIBRATION -- FORWARD STATE (CALL FORWARD CONTROL))



---------------------------------------------------------------------------------------------------------------
Computer Vision
--

---------------------------------------------------------------------------------------------------------------
System Integration
--
The system integration is responsible for transmitted the real-time parameters from the computer vision side to arduino using I2C byte transmission. The transmission
parameters are 1. If detected signal 2. The horizontal angle of Phi 3. The sign (positive or negativeO of Phi) 4. The vertical angle of Theta 5.The sign (positive or negativeO of Theta. As we did not use the block transmission function, the parameters were sent one by one using a case switch statment that reads in values in a sequential order. The binary numbers of Phi and Theta, if negative, will then be converted back to their positive values with a negative sign using 2's complements.

The system intgration will also inform the state machine if it needs to switch states by update the static variable inside the state machine, albeit the transmitted values themselves are ephemeral. The static numerical vairables of transmittedPhi and transmittedTheta also get updates in the same pattern.
