Descriptions of Demo 2
---------------------------------------------------------------------------------------------------------------

Ben Geisler
Jiuzou Zhang
Robert Schmidt
Louie Heilweil

This document contains descriptions of the code and tasks used in to create Demo 2.
Note that the final code used for the demo is called Final_Code.ino which is in the Simulation and Control file

---------------------------------------------------------------------------------------------------------------
Localization
--
For this demo, the main localization focus was verifying that the robot would navigate correctly when given the values from computer vision.
It was important to communicate with computer vision for this portion of the project so we could determine what values to send over from
the Raspberry Pi to the Arduino. We determined that we would need a vertical angle from camera to the tape, which was based on the height of the robot,
as well as the horizontal angle from the camera, so that we could determine how much the robot needed to turn. A boolean was also to be
sent, so that the robot would know when the tape was within view, and would stop searching. The vertical angle was used to determine how far away
the tape was, and the horizontal angle was used to determine the angle the robot would have to turn. The calculations given the vertical angle (theta) are
shown below, where ALPHA is the angle of the camera, and H is the height of the robot:

beta = (PI/2 - (ALPHA*PI/180) - (theta*PI/180) );
rho = H * (tan(beta)) - 0.09;
  
If the angle was positive, it would move clockwise, and if it was negative, it would turn counterclockwise. Calculating these essentially gave 
us the inputs we needed to move to any point of blue tape, using the control system from Demo 1 and the state machine for this demo. 


---------------------------------------------------------------------------------------------------------------
Simulation and Control
--
The main task for simulation and control is to create a finite state machine to iterate through the states of turning and moving 
forward by using the transmitted values of angles of theta and phi to account for the errors and adjust the robot to the desired
position. As our computer vision takes a relatively long time to read in and analyze the input video footage. A state of waiting 5
seconds is needed to prevent the robot of going over and not seeing the tape. The moving distance is also of a set distance of 20-30
cm instead of directing driving towards the identified marker. This can make infinitesimally closer approaches to the center of the tape and drive along it.

The general logic of the FSM, as shown in these two key states, is shown below:

SEARCH STATE(WAITING FOR CV SIGNALS) - TURN CALBIRATION -- TURN STATE (CALL TURN CONTROLLER) -- FORWARD CALIBRATION -- FORWARD STATE (CALL FORWARD CONTROL))



---------------------------------------------------------------------------------------------------------------
Computer Vision
--
The Computer Vision subsystem is responsible for obtaining both the horizontal and vertical angles as well, as whether or not they are negitive, rounding the angles to the nearest integer for system intergration purposes, and also deciding whether or not the end of the tape is at the bottom of the vision of the camera. The program is supposed to recognize blue painters tape and recognize these values.

---------------------------------------------------------------------------------------------------------------
System Integration
--
The system integration is responsible for transmitted the real-time parameters from the computer vision side to arduino using I2C byte transmission. The transmission
parameters are 1. If detected signal 2. The horizontal angle of Phi 3. The sign (positive or negativeO of Phi) 4. The vertical angle of Theta 5.The sign (positive or negativeO of Theta. As we did not use the block transmission function, the parameters were sent one by one using a case switch statment that reads in values in a sequential order. The binary numbers of Phi and Theta, if negative, will then be converted back to their positive values with a negative sign using 2's complements.

The system intgration will also inform the state machine if it needs to switch states by update the static variable inside the state machine, albeit the transmitted values themselves are ephemeral. The static numerical vairables of transmittedPhi and transmittedTheta also get updates in the same pattern.
