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

  case TURN:
    // Temporarily set desiredRho to be 0 as we do not want the drone to move forward for now... 
    desiredRho = 0;
    // Call the turn control function block
    turnControl();
     // These if's will determine if the drone will keep moving forward or correct the angular position if it went off due to friction along the way
      if (abs(positionError) <= 0.05){
          maneuveringState = CALIBRATION_FORWARD;
          }
      else{
            maneuveringState = TURN;
        }
    break;
    
  case CALIBRATION_FORWARD:
/***  IF LOGIC TO CALIBRATE PARAMETERS ***/
    maneuveringState = DRIVE_FORWARD;
  break;
  case DRIVE_FORWARD:
    desiredPhi = currentPhi;
    forwardControl();
  break;
  }
}



---------------------------------------------------------------------------------------------------------------
Computer Vision
--

---------------------------------------------------------------------------------------------------------------
System Integration
--

