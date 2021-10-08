GROUP 9
-
Robert Schmidt, Charles Heilweil, Jiuzou Zhang, Ben Geisler
-
Mini Project
-
This folder contains the code that was used to complete the Mini Project. There is some preliminary code in the "Prelim Code" folder,
but the final codes for the Raspberry Pi and Arduino each have their own file. Below are descriptions of each of the roles.

------------------------------------------------------------------------------------------------------------------------
Description of Computer Vision: 

Computer vision was relatively simple. In assignment two the center of a yellow hexagon was found, so this concept was extrapolated to also deciding if that location was found in certain quadrants. This was achieved by adding case statements that decided which quadrant the yellow hexagon was in given the current location of the center of the hexagon. The resulting quadrant was assigned to a variable as an integer (1-4) and system integration made sure that this information tied to the variable was transferred to the Arduino. 

------------------------------------------------------------------------------------------------------------------------
Description and Modeling of Simulation and Control: 

Open-Loop Response of Angular Velocity: 
The design of the control system begins with the data collection from the motor using the step response of motor with respect to its angular velocity with a fixed sampling time of 10ms as defined in the Arduino code (Code commented out in the loop function). Then we applied the method specified in Lecture 15 of EENG 307 to model the transfer function of the angular velocity. As the actual transfer function was oscillating around 12, we assumed the K value to be 12. Using linear interpolation at the regime of data points between 0.64K, the 1/σ value could be found, which turned out to be 1/0.0885 = 11.301 and then the entire transfer function could be written as 12*11.301/(1+11.301) = 135.616/(1+11.301). 

------------------------------------------------------------------------------------------------------------------------
Description of System Integration: 

System Integration was very straight forward this week. In assignment two I dealt with learning how to communicate with I2C and Serial. I chose to stick with I2C communication as I thought it would be better for the rest of the project assignments to come. Basically, once computer vision had any values that it needed to give to the Arduino, I would send that information from one location to another. After sending this information I also needed to get the LCD screen to populate the different quadrants that we were seeing. The hardest part about the system integration portion of this code was figuring out where and how I would be able to add my code to my group’s codes.   

------------------------------------------------------------------------------------------------------------------------
Description of Localization: 

The localization portion of the project included reading the encoder and obtaining velocities to get the step response of the motor. Using the encoder library, the encoder counts were read, and the angular position was calculated using the relationship between total number of counts for one full rotation. Localization also included much of the hardware setup to drive the motor. Using the oscilloscope, the formula relating the written value (between 0 and 255) to the pulse width was derived. This equation is as follows: PulseWidth = AnalogWriteVal*.008 
