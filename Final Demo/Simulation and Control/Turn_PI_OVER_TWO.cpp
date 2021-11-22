/* 
Arduino Code Demo 1 - Group 9
Ben Geisler, Jiuzou Zhang, Robert Schmidt 

The basic setup of this code is to convert desired distances/angles into the corresponding speeds using the outer loop PD controller, which will set specified
speeds and translate the values into voltage readings and then PWM waves using the inner loop P controller. The outer and inner loop controllers were modeled
using the simulink models and transfer function identifications by setting up the step experiments with decoupling effects. But nonetheless the speeds were
scaled down using a custom percentage factor in order to reduce the translational and rotational inertiæ. The case statement also helps to set the order of rotation
and moving forwards. Sometimes the two controllers of distance and angle need to run simulateneously but at other times they need to be decoupled for more accuracy.
*/

#include <Encoder.h> // The encoder library comes in handy for this project 
#include <Wire.h> 
#define SLAVE_ADDRESS 0x04 
  
// Define some PINs for the encoder, the first set is for the right encoder
#define ENCODER_PIN_A 2 
#define ENCODER_PIN_B 5 

#define ENCODER_PIN_C 3
#define ENCODER_PIN_D 6

// Define some PWM PINs for the motor 
#define PWM_OUTPUT_PIN_R 9
#define PWM_OUTPUT_PIN_L 10

// Define some constants for the PID Controller  
#define PWM_BOUND 255 
#define DELAY 5 
#define RADII 7.3  // This specifies the radius of the wheel in centimeters
#define DISTANCE 24.45
#define PWM_WAVES_PER_VOLT  36.43  // This value is given by 255/SUPPLY_VOLTAGE

//Controller values for velocity control
#define KpRho 16.94
#define KiRho 333.657977806759
#define KpPhi 4.22293708172003

//Controller values for position control
#define KpDistance 3.492088875086
#define KdDistance 1.88165512919462

#define KpPosition 16.180293597136
#define KdPosition 2.63548543922997

// Define the physical quantities

#define H 0.16
#define ALPHA 27

//States for FSM
//#define DRIVE_FORWARD 0
//#define TURN 1

//FSM state value
int state;



// Define some parameters for the loop
int number = 0; 
int imReady = 1;
int jiuzouNumber;
int benNumber;
int robertNumber;
int benNegative;
int jiuzouNegative;
float totDist =0;
bool markersFound = 0;
float exportVariable = 0;
int transmissionState = 0;
float phi = 0;
float theta = 0;
float beta = 0;
double rho = 0;

// Encoder variables 
float motorPos_L = 0; 
float motorPos_R = 0;
float angularPos_L = 0;
float angularPos_R = 0; 
int N = 3200; //Counts per revolution 

//Set position here 
float currentPosition_L = 0; 
float currentPosition_R = 0;

float angular_velocity_L = 0; 
float angular_velocity_R = 0;
float recorded_position_L = 0; 
float recorded_position_R = 0; 

//Test parameters
float instanteousForwardVelocity = 0;
float instanteousAngularVelocity = 0;
float currentTheta = 0;
float errorOfTheta = 0;
float storedErrorOfTheta = 0;
float D = 0;
float desiredRhoDot = 0;
float desiredPhiDot = 0;

//Desired values
float desiredRho = 3;
float desiredPhi = 0;

float absOfangularPos_R = 0;
float currentRho = 0;
float currentPhi = 0;
float currentRhoDot = 0;
float currentPhiDot = 0;
float errorOfRhoDot = 0;
float errorOfPhiDot = 0;
float deltaV = 0;
float sigmaV = 0;
float Va1 = 0;
float Va2 = 0;
int controlSignal_L = 0;
int controlSignal_R = 0;
double integrator1 = 0;
double storedIntegrator1 = 0;

float distanceError = 0;
float distanceDerivativeError = 0;
float positionError = 0;
float positionDerivativeError = 0;
float storedDistanceError = 0;
float storedPositionError = 0;

//Error to check how close wheel is to desired position 
float error = 0; 

//Sampling time for loop 
float samplingTime = 0;
float storedTime = 0; 
float currentTime = 0; 
float integrator = 0; 
float storedIntegrator = 0; 
int controlSignal = 0; 
float time_now = 0; 
float recorded_time = 0; 

// Constants obtained from MATLAB simulations 
const float Kp = 0.643661597696071; 
const float Ki = 0.0749866625772769;  

// These are the enumerated states for the state machine
enum maneuvering {SEARCH_TAPE,CALIBRATION_TURN,TURN,DRIVE_FORWARD,CALIBRATION_FORWARD,WAIT,WAIT_TURN,TURN_PI_OVER_TWO};
// Set the default state to SEARCH_TAPE
maneuvering maneuveringState = SEARCH_TAPE;
int transmittedCVInstructions = 0;
int wentOverCounter = 0;
float transmittedRho = (3 * 0.3048) - 0.03;
float transmittedPhi = (-PI/2) * 1.07;

// Setup encoder - pins 2 (interrupt) and 5 (no interrupt) 
Encoder motorEncR(ENCODER_PIN_A, ENCODER_PIN_B); 
Encoder motorEncL(ENCODER_PIN_C, ENCODER_PIN_D);

void setup() { 
  
  pinMode(13, OUTPUT); 
// start serial for output 
  Serial.begin(9600); 
   // initialize i2c as slave Robert
  Wire.begin(SLAVE_ADDRESS);
  // define callbacks for i2c communication Robert
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData); 
  desiredRho = (desiredRho*0.3048) - 0.03;
  desiredPhi = desiredPhi * 1.07;
//Set the pins as outputs 
  pinMode(4, OUTPUT); 
  pinMode(7, OUTPUT); 
  pinMode(8, OUTPUT); 
  pinMode(PWM_OUTPUT_PIN_R, OUTPUT); 
  pinMode(PWM_OUTPUT_PIN_L, OUTPUT); 
  digitalWrite(7, HIGH); 
  digitalWrite(8, HIGH); 
  //Pin 12 is input 
  pinMode(12, INPUT); 
 //Pin 4 write high for motor drive 
  digitalWrite(4, HIGH); 
  state = TURN;

} 

void loop() { 
  
  time_now = millis(); 

  while(millis() < time_now + DELAY){ 
    
    //wait approx. [period] ms 
    motorPos_L = motorEncR.read(); 
    motorPos_R = motorEncL.read();

    angularPos_L = 2 * PI * (double)motorPos_L / (double)N;
    angularPos_R = 2 * PI * (double)motorPos_R / (double)N;


    //Calculate angular velocities
    angular_velocity_R = (angularPos_R - recorded_position_R)/(time_now/1000 - recorded_time/1000); 
    angular_velocity_L = (angularPos_L - recorded_position_L)/(time_now/1000 - recorded_time/1000);

    angular_velocity_R = - angular_velocity_R;
    
    instanteousAngularVelocity = RADII/100 * (angular_velocity_R - angular_velocity_L)/(DISTANCE/100);
    instanteousForwardVelocity = RADII/100 * (angular_velocity_R + angular_velocity_L);
    } 
  

  recorded_position_L = angularPos_L;
  recorded_position_R = angularPos_R;
  recorded_time = time_now;

 
  mathematics();
  //receiveData(1);
  maneuvering();
  
  Serial.print(currentRho);
  Serial.print("\t");
  Serial.print(currentPhi);
  Serial.print("\t");
  Serial.println(maneuveringState);
} 

void mathematics()
{

  if (imReady == 1) {
  if (benNegative != 0)
  {
    benNumber = benNumber - 256;
  }

  if (jiuzouNegative != 0)

  {
    jiuzouNumber = jiuzouNumber - 256; 
  }

  if ((abs(benNumber) <= 90) && (abs(jiuzouNumber) <= 90))
  {
    phi = benNumber*PI/180;
    theta = jiuzouNumber;
    transmittedCVInstructions = robertNumber;
  }

  if (phi == 0 && theta == 0)
  {
    rho = 0;
  }
  else {
    beta = (PI/2 - (ALPHA*PI/180) - (theta*PI/180) );
    rho = H * (tan(beta)) - 0.09;
  }
  beta = (PI/2 - (ALPHA*PI/180) - (theta*PI/180));
  }
}


void maneuvering()
{
  switch(maneuveringState)
    {
    case SEARCH_TAPE:
    if (transmittedCVInstructions != 0)
    {
      maneuveringState = CALIBRATION_TURN;
      // Initialize othe parameters
    }
    else // Hard code the rotational velocity
    {
      desiredRho  = 0;
      turnControl();
      if (abs(positionError) < 0.05)
      {
      Serial.print("  Stopping turning...    ");
      analogWrite(PWM_OUTPUT_PIN_R, 0);
      analogWrite(PWM_OUTPUT_PIN_L, 0);
      maneuveringState = WAIT;
      }
    }
    break;

    case WAIT:
    desiredPhi  = currentPhi + PI/6;
    while(millis() < time_now + 6000 ){     
      //wait approx. [period] ms 
    }  
    maneuveringState = SEARCH_TAPE;
    break;

    case WAIT_TURN:
    while(millis() < time_now + 6000 ){     
      //wait approx. [period] ms 
    }  
    if (transmittedCVInstructions == 1)
    {
      maneuveringState = CALIBRATION_TURN;
    }
    else if (transmittedCVInstructions == 0 && wentOverCounter < 3)
    {
      maneuveringState = TURN_PI_OVER_TWO;
      wentOverCounter++;
    }
    else
    {
      maneuveringState = CALIBRATION_TURN;
    }
    break;

    case CALIBRATION_TURN:
    // A state where the rotational maneuvering has theorectically stopped but we have to want wait it actually stops
    currentPhiDot = RADII/100 * (angular_velocity_R - angular_velocity_L)/(DISTANCE/100);
    if (currentPhiDot == 0)
    {
        // Reset the parameters for angular positions, have a fresh restart
        /*angularPos_R  = 0;
        recorded_position_R = 0;  
        angularPos_L  = 0;
        recorded_position_L = 0;
        positionError = 0;
        distanceError = 0;
        errorOfRhoDot = 0;
        errorOfPhiDot = 0; */
        transmittedPhi = phi;
        transmittedRho = rho;
        desiredPhi = currentPhi+ (-0.60*transmittedPhi);
        maneuveringState = TURN;
    } else
    {
      // Do nothing otherwise, as we have to make sure it completely stops
      analogWrite(PWM_OUTPUT_PIN_R, 0);
      analogWrite(PWM_OUTPUT_PIN_L, 0);
    }
    break;
    
  case TURN:
    // Temporarily set desiredRho to be 0 as we do not want the drone to move forward for now... 
    desiredRho = 0;
    Serial.println(transmittedPhi);
    //desiredPhi = currentPhi + phi;
    /*
    Serial.print(time_now/1000);
    Serial.print("\t");
    Serial.print(distanceError);
    Serial.print("\t");
    Serial.println(positionError);
    */
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
      // This specific state will calibrate and set the parameters for the forward state (only once)
    if (transmittedRho >= 0.20)
    {
      // The desired value of Rho is set to be 20cm for incremental movements
      desiredRho = currentRho + 0.20;
    }
    else 
    {
      // If the transmittedRho is within 20cm, just move that desired distance
      transmittedRho = rho;
      desiredRho = currentRho + transmittedRho;
    }
    maneuveringState = DRIVE_FORWARD;
  break;
  
  case DRIVE_FORWARD:
    desiredPhi = currentPhi;
    forwardControl();
  break;
  // Hard code the PI/2 turns in case the CV failed to pick up new signals    
  case TURN_PI_OVER_TWO:
      desiredPhi = currentPhi - PI/2;
      turnControl();
      if (abs(positionError) <= 0.05){
          maneuveringState = WAIT_TURN;
          }
      else{
          maneuveringState = TURN_PI_OVER_TWO;
        }
  break;    
  }
}

void turnControl ()  // This turn controller is decoupled and will only turn upon request
{
  currentTime = millis();
  samplingTime = currentTime - storedTime;
  absOfangularPos_R = - angularPos_R;
  
  currentRho = RADII/100 * (absOfangularPos_R + angularPos_L)/2;
  currentPhi = RADII/100 * (absOfangularPos_R - angularPos_L)/(DISTANCE/100);
  //Checks errors for rho, which is part of the outer control loop
  distanceError = desiredRho - currentRho;
  // This line and the line below specifies the derivative part (D) of the outer controller
  distanceDerivativeError = (distanceError - storedDistanceError)/(samplingTime/1000);
  desiredRhoDot = KpDistance * distanceError + distanceDerivativeError * KdDistance;


  //Checks errors for phi, which is part of the outer control loop
  positionError = desiredPhi - currentPhi;
  // This line and the line below specifies the derivative part (D) of the outer controller
  positionDerivativeError = (positionError - storedPositionError)/(samplingTime/1000);
  desiredPhiDot = KpPosition * positionError + positionDerivativeError * KdPosition;

  //Calculations for velocities, which are all done in standard SUI units. No cm's or inches
  currentRhoDot = RADII/100 * (angular_velocity_R + angular_velocity_L)/2;
  currentPhiDot = RADII/100 * (angular_velocity_R - angular_velocity_L)/(DISTANCE/100);
  
  errorOfRhoDot = desiredRhoDot - currentRhoDot;
  errorOfPhiDot = desiredPhiDot - currentPhiDot;
  
 // The theoretically-usable I portion of the controller, which wa not used in this case
  integrator =  storedIntegrator + errorOfRhoDot * (samplingTime/1000);
  integrator1 = storedIntegrator1 + errorOfPhiDot * (samplingTime/1000);
  
  // These calculations derive the sum voltage and the differential voltage, which will then be translated
  deltaV = KpPhi * errorOfPhiDot;
  // In order to decouple the effects, the drone will not be moving forward when turning
  sigmaV = 0;
  
   // The aforementioned voltages will be translated into the voltages on each motor, denoted as Va1 and Va2
  Va1 = (sigmaV + deltaV)/2;
  Va2 = (sigmaV - deltaV)/2;
  // The voltages will then be translated into the actual PWM waves using the pwm_waves_per_volt constant
  controlSignal_R = Va1 * PWM_WAVES_PER_VOLT;
  controlSignal_L = Va2 * PWM_WAVES_PER_VOLT;
      
  // These are the constraints for the pwm wave readings, which should never exceed 255 or -255
  if (abs(controlSignal_R) > PWM_BOUND) 
  { 
    controlSignal_R = constrain(controlSignal_R, -1, 1) * PWM_BOUND; 
    errorOfRhoDot = constrain (errorOfRhoDot, -1, 1) * min(abs(errorOfRhoDot), PWM_BOUND / KpRho); 
  } 
  if (abs(controlSignal_L) > PWM_BOUND)
  {
    controlSignal_L = constrain(controlSignal_L, -1, 1) * PWM_BOUND;
    errorOfPhiDot = constrain (errorOfPhiDot, -1, 1) * min(abs(errorOfPhiDot), PWM_BOUND / KpPhi);
  }
 // Take the absolute value of the control signals (in pwm waves), if they are not already
  controlSignal_R = abs(controlSignal_R);
  controlSignal_L = abs(controlSignal_L);
  
      //Serial.print(state);
      // The tolerance is specified is in t
      if (abs(positionError) < 0.05)
      {
      Serial.print("  Stopping turning...    ");
      analogWrite(PWM_OUTPUT_PIN_R, 0);
      analogWrite(PWM_OUTPUT_PIN_L, 0);
      }
      else if (positionError >= 0)
      {
      Serial.print("  Turning...   ");
      digitalWrite(7, HIGH); //+
      digitalWrite(8, LOW); // -
      analogWrite(PWM_OUTPUT_PIN_R, controlSignal_R*0.30);
      analogWrite(PWM_OUTPUT_PIN_L, controlSignal_L*0.30);
      //analogWrite(PWM_OUTPUT_PIN_R, controlSignal_R);
      //analogWrite(PWM_OUTPUT_PIN_L, controlSignal_L);
      } else 
      {
      Serial.print("  Turning in reverse   ");
      digitalWrite(7, LOW); //+
      digitalWrite(8, HIGH); // -
      analogWrite(PWM_OUTPUT_PIN_R, controlSignal_R*0.30);
      analogWrite(PWM_OUTPUT_PIN_L, controlSignal_L*0.30);
      }
  storedTime = currentTime;
  storedIntegrator = integrator;
  storedIntegrator1 = integrator1;
}

void forwardControl()  // This controller is not decoupled and will move forward and adjust the angle along the way
{
  currentTime = millis();
  samplingTime = currentTime - storedTime;
  absOfangularPos_R = - angularPos_R;

  currentRho = RADII/100 * (absOfangularPos_R + angularPos_L)/2;
  currentPhi = RADII/100 * (absOfangularPos_R - angularPos_L)/(DISTANCE/100);

  //Checks errors for rho, which is part of the outer control loop
  distanceError = desiredRho - currentRho;
  // This line and the line below specifies the derivative part (D) of the outer controller
  distanceDerivativeError = (distanceError - storedDistanceError)/(samplingTime/1000);
  desiredRhoDot = KpDistance * distanceError + distanceDerivativeError * KdDistance;

  //Checks errors for phi, which is part of the outer control loop
  positionError = desiredPhi - currentPhi;
  // This line and the line below specifies the derivative part (D) of the outer controller
  positionDerivativeError = (positionError - storedPositionError)/(samplingTime/1000);
  desiredPhiDot = KpPosition * positionError + positionDerivativeError * KdPosition;

  //Calculations for velocities, which are all done in standard SUI units. No cm's or inches
  currentRhoDot = RADII/100 * (angular_velocity_R + angular_velocity_L)/2;
  currentPhiDot = RADII/100 * (angular_velocity_R - angular_velocity_L)/(DISTANCE/100);
  
  errorOfRhoDot = desiredRhoDot - currentRhoDot;
  errorOfPhiDot = desiredPhiDot - currentPhiDot;
  
 // The theoretically-usable I portion of the controller, which wa not used in this case
  integrator =  storedIntegrator + errorOfRhoDot * (samplingTime/1000);
  integrator1 = storedIntegrator1 + errorOfPhiDot * (samplingTime/1000);

 // sum and delta will be calculated accordingly
  sigmaV = KpRho * errorOfRhoDot;
  deltaV = KpPhi * errorOfPhiDot;
  // The aforementioned voltages will be translated into the voltages on each motor, denoted as Va1 and Va2
  Va1 = (sigmaV + deltaV)/2;
  Va2 = (sigmaV - deltaV)/2;
  // The voltages will then be translated into the actual PWM waves using the pwm_waves_per_volt constant
  controlSignal_R = Va1 * PWM_WAVES_PER_VOLT;
  controlSignal_L = Va2 * PWM_WAVES_PER_VOLT;
  
  if (abs(controlSignal_R) > PWM_BOUND) 
  { 
    controlSignal_R = constrain(controlSignal_R, -1, 1) * PWM_BOUND; 
    errorOfRhoDot = constrain (errorOfRhoDot, -1, 1) * min(abs(errorOfRhoDot), PWM_BOUND / KpRho); 
  } 
  if (abs(controlSignal_L) > PWM_BOUND)
  {
    controlSignal_L = constrain(controlSignal_L, -1, 1) * PWM_BOUND;
    errorOfPhiDot = constrain (errorOfPhiDot, -1, 1) * min(abs(errorOfPhiDot), PWM_BOUND / KpPhi);
  }
 
  controlSignal_R = abs(controlSignal_R);
  controlSignal_L = abs(controlSignal_L);
      Serial.print(state);
      if (abs(distanceError) < 0.01)
      {
      Serial.print("  stopped    ");
      analogWrite(PWM_OUTPUT_PIN_R, 0);
      analogWrite(PWM_OUTPUT_PIN_L, 0);
      maneuveringState = WAIT_TURN; 
      }
      else if (distanceError >= 0)
      {
      Serial.print("  Running high    ");
      digitalWrite(7, HIGH); //+
      digitalWrite(8, HIGH); // -
      analogWrite(PWM_OUTPUT_PIN_R, controlSignal_R*0.30);
      analogWrite(PWM_OUTPUT_PIN_L, controlSignal_L*0.30);
      } else 
      {
      Serial.print("  Running low    ");
      digitalWrite(7, LOW); //+
      digitalWrite(8, LOW); // -
      analogWrite(PWM_OUTPUT_PIN_R, 40);
      analogWrite(PWM_OUTPUT_PIN_L, 40);
      }
  storedTime = currentTime;
  storedIntegrator = integrator;
  storedIntegrator1 = integrator1;
}

// The case switch statement inside this specific ISR will transmit these data one by one/
void receiveData(int byteCount)
{ //Serial.println("In receiveData() ");
  switch (transmissionState){
  case 0:
  while (Wire.available()) {
    robertNumber = Wire.read();
    }
  transmissionState++;
  break;
  case 1:
    while(Wire.available()){
    benNumber = Wire.read();
    }
  transmissionState++;
  break;
  case 2:
    while(Wire.available()){
    benNegative = Wire.read();
    }
  transmissionState++;
  break;
  case 3:
    while(Wire.available()){
    jiuzouNumber = Wire.read();
    }
  transmissionState++;
  break;
  case 4:
    while(Wire.available()){
    jiuzouNegative = Wire.read();
      }
  transmissionState = 0; // Reset the state machine to case 0 once the sequence is completed
  break;
  }
}
// callback for sending data
void sendData() {
  Wire.write(imReady);
}
