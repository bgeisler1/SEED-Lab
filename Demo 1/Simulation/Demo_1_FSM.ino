/* 
Arduino Code Demo 1 - Group 9
Ben Geisler, Jiuzou Zhang, Robert Schmidt 
This code takes inputs from the Pi in the form of a digit, converts it into an angular position, and turns the wheel using a proportional-integral controller. 
This code uses the encoder library to read the position of the encoder, so the controller can determine the current position in relation to the actual position. 
The controller is used to control the speed and position. This also keeps external disturbances from causing error in the location. 
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

//States for FSM
#define DRIVE_FORWARD 0
#define TURN 1

//FSM state value
int state;

// Define some parameters for the loop
int number = 0; 

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
float desiredRho = 2;
float desiredPhi = 2*PI;

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
 
// Setup encoder - pins 2 (interrupt) and 5 (no interrupt) 
Encoder motorEncR(ENCODER_PIN_A, ENCODER_PIN_B); 
Encoder motorEncL(ENCODER_PIN_C, ENCODER_PIN_D);

void setup() { 
  pinMode(13, OUTPUT); 
// start serial for output 
  Serial.begin(9600);  
  desiredRho = (desiredRho*0.3048) - 0.02;
  desiredPhi = desiredPhi*1.05;
  
//Set the pins as outputs 
  pinMode(4, OUTPUT); 
  pinMode(7, OUTPUT); 
  pinMode(8, OUTPUT); 
  digitalWrite(7, HIGH); 
  digitalWrite(8, HIGH); 
  pinMode(PWM_OUTPUT_PIN_R, OUTPUT); 
  pinMode(PWM_OUTPUT_PIN_L, OUTPUT); 
  //Pin 12 is input 
  pinMode(12, INPUT); 
 //Pin 4 write high for motor drive 
  digitalWrite(4, HIGH); 
  state = TURN;

} 

void loop() { 
  
  //desiredPosition = number * PI / 2;
  
  //digitalWrite(7, HIGH); //+
  //digitalWrite(8, LOW); // -

  //analogWrite(PWM_OUTPUT_PIN_R, 0.5*PWM_WAVES_PER_VOLT);
  //analogWrite(PWM_OUTPUT_PIN_L, 0.5*PWM_WAVES_PER_VOLT);
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

  
 //Run the controller – turns wheel to specified position 
  PIDController(); 
} 

void PIDController()
{
  currentTime = millis();
  samplingTime = currentTime - storedTime;

  absOfangularPos_R = abs(angularPos_R);

  
  currentRho = RADII/100 * (absOfangularPos_R + angularPos_L)/2;
  currentPhi = RADII/100 * (absOfangularPos_R - angularPos_L)/(DISTANCE/100);

  //Checks errors for rho
  distanceError = desiredRho - currentRho;
  distanceDerivativeError = (distanceError - storedDistanceError)/(samplingTime/1000);
  desiredRhoDot = KpDistance * distanceError + distanceDerivativeError * KdDistance;

  //Checks errors for phi
  positionError = desiredPhi - currentPhi;
  positionDerivativeError = (positionError - storedPositionError)/(samplingTime/1000);
  desiredPhiDot = KpPosition * positionError + positionDerivativeError * KdPosition;

  
  currentRhoDot = RADII/100 * (angular_velocity_R + angular_velocity_L)/2;
  currentPhiDot = RADII/100 * (angular_velocity_R - angular_velocity_L)/(DISTANCE/100);
  
  errorOfRhoDot = desiredRhoDot - currentRhoDot;
  errorOfPhiDot = desiredPhiDot - currentPhiDot;

  integrator =  storedIntegrator + errorOfRhoDot * (samplingTime/1000);
  integrator1 = storedIntegrator1 + errorOfPhiDot * (samplingTime/1000);
  
  sigmaV = KpRho * errorOfRhoDot;
  deltaV = KpPhi * errorOfPhiDot;
  
  Va1 = (sigmaV + deltaV)/2;
  Va2 = (sigmaV - deltaV)/2;
  
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
  
  
  /*
  if (controlSignal_R > 0)
  {
    digitalWrite (8, LOW);
  } else
  {
    digitalWrite (8, HIGH);
  }
  */
 
  controlSignal_R = abs(controlSignal_R);
  controlSignal_L = abs(controlSignal_L);

  
//Controls the distance so it adjusts when it overshoots

  switch(state){
    case TURN:
      digitalWrite(7, HIGH); //+
      digitalWrite(8, LOW); // -
    
      
      if (abs(positionError) < 0.05)
      {
      analogWrite(PWM_OUTPUT_PIN_R, 0);
      analogWrite(PWM_OUTPUT_PIN_L, 0); 
      state = DRIVE_FORWARD;
      }
      else if (positionError >= 0)
      {
      digitalWrite(7, HIGH); //+
      digitalWrite(8, LOW); // -
      analogWrite(PWM_OUTPUT_PIN_R, controlSignal_R*0.30);
      analogWrite(PWM_OUTPUT_PIN_L, controlSignal_L*0.30);
      } else 
      {
      digitalWrite(7, LOW); //+
      digitalWrite(8, HIGH); // -
      analogWrite(PWM_OUTPUT_PIN_R, 40);
      analogWrite(PWM_OUTPUT_PIN_L, 40);
      }
      break;
      
    case DRIVE_FORWARD:
      if (abs(distanceError) < 0.01)
      {
      analogWrite(PWM_OUTPUT_PIN_R, 0);
      analogWrite(PWM_OUTPUT_PIN_L, 0); 
      }
      else if (distanceError >= 0)
      {
      digitalWrite(7, HIGH); //+
      digitalWrite(8, HIGH); // -
      analogWrite(PWM_OUTPUT_PIN_R, controlSignal_R*0.30);
      analogWrite(PWM_OUTPUT_PIN_L, controlSignal_L*0.30);
      } else 
      {
      digitalWrite(7, LOW); //+
      digitalWrite(8, LOW); // -
      analogWrite(PWM_OUTPUT_PIN_R, 40);
      analogWrite(PWM_OUTPUT_PIN_L, 40);
      }
      break;
      
    default:
    
      break;
  }



  Serial.print(currentTime/1000);
  Serial.print("\t"); 
  Serial.print(positionError); 
  Serial.print("\t"); 
  Serial.print(currentPhi);
  Serial.print("\t"); 
  Serial.println(distanceError);
  
  //Serial.print("\t"); 
  //Serial.println(); 
  storedTime = currentTime;
  storedIntegrator = integrator;
  storedIntegrator1 = integrator1;
}
