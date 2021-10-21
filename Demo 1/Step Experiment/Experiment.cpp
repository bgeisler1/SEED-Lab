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

// Define some parameters for the loop
int number = 0; 
int state = 0; 

// Define some constants for the PID Controller  
#define PWM_BOUND 255 
#define DELAY 10 
#define RADII 7.3  // This specifies the radius of the wheel in centimeters
#define DISTANCE 24.45


// Encoder variables 
float motorPos_R = 0; 
float motorPos_L = 0;
float angularPos_R = 0;
float angularPos_L = 0; 
int N = 3200; //Counts per revolution 

// PWM variables 
int pwmSpeed = 0; 
//Set position here 
float desiredPosition = PI; 
float currentPosition_R = 0; 
float currentPosition_L = 0;

float angular_velocity_R = 0; 
float angular_velocity_L = 0;
float recorded_position_R = 0; 
float recorded_position_L = 0; 

float instanteousForwardVelocity = 0;

//Error to check how close wheel is to desirec position 
float error = 0; 

//Sampling time for loop 
float samplingTime = 0;
float storedTime = 0; 
float currentTime; 
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
  
  // initialize i2c as slave 
  Wire.begin(SLAVE_ADDRESS); 
  // define callbacks for i2c communication 
  Wire.onReceive(receiveData); 
  Wire.onRequest(sendData); 

//Set the pins as outputs 
  pinMode(4, OUTPUT); 
  pinMode(7, OUTPUT); 
  pinMode(8, OUTPUT); 
  pinMode(PWM_OUTPUT_PIN_R, OUTPUT); 
  pinMode(PWM_OUTPUT_PIN_L, OUTPUT); 
  //Pin 12 is input 
  pinMode(12, INPUT); 
 //Pin 4 write high for motor drive 
  digitalWrite(4, HIGH); 
} 

void loop() { 

  //delay(100); 
  // callback for received data 
  //Calculate position based on the input quadrant 
  
  //desiredPosition = number * PI / 2;

  //Find current angular position using encoder library 

  analogWrite(PWM_OUTPUT_PIN_R, 26);
  analogWrite(PWM_OUTPUT_PIN_L, 26);

  digitalWrite(7, HIGH);
  digitalWrite(8, HIGH);

  time_now = millis(); 


  while(millis() < time_now + DELAY){ 
    
    //wait approx. [period] ms 
    motorPos_R = motorEncR.read(); 
    motorPos_L = motorEncL.read();

    angularPos_R = 2 * PI * (double)motorPos_R / (double)N;
    angularPos_L = 2 * PI * (double)motorPos_L / (double)N;

    //Calculate angular velocities
    angular_velocity_R = (angularPos_R - recorded_position_R)/(time_now/1000 - recorded_time/1000); 
    angular_velocity_L = (angularPos_L - recorded_position_L)/(time_now/1000 - recorded_time/1000);

    instanteousForwardVelocity = RADII/100 * (angularPos_R + angularPos_L)/2;
    } 
  

  recorded_position_L = angularPos_L;
  recorded_position_R = angularPos_R;

  recorded_time = time_now;


  Serial.print((time_now+ DELAY)/1000); 
  Serial.print("\t"); 
  Serial.println(instanteousForwardVelocity)



 //The block below calculates the angular position and velocity of the wheel, which was used for step response simulation 
  /*analogWrite(PWM_OUTPUT_PIN, PWM_BOUND); 
    time_now = millis(); 

//Forced sampling time to get more accurate data 
    while(millis() < time_now + DELAY){ 
        //wait approx. [period] ms 
        motorPos = motorEnc.read(); 
        angularPos = 2*PI*(double)motorPos/(double)N; 

//Calculate angular velocity 
        angular_velocity = (angularPos - recorded_position)/(time_now/1000 - recorded_time/1000); 
    } 
    recorded_time = time_now; 
    recorded_position = angularPos; 
  
//Prints to serial monitor 
    Serial.print((time_now+ DELAY)/1000); 
    Serial.print("\t"); 
    Serial.println(angular_velocity); */ 

//Serial.println(angularPos); 
  //Mechanism to reset position to zero - https://www.pjrc.com/teensy/td_libs_Encoder.html#optimize 
  

  /* Looks for anything in the serial monitor, if there is something, write zero to encoder 
  if (Serial.available()) { 
    Serial.read(); 
    Serial.println("Reset position to zero"); 
    motorEnc.write(0); 
  } */
  
 //Run the controller – turns wheel to specified position 
  //PIDController(); 
} 
  


void PIDController()

{
  currentTime = milis();
  samplingTime = currentTime - storedTime;

  currentTheta = RADII * (currentPosition_R-currentPosition_L)/DISTANCE;
  errorOfTheta = desiredPosition - currentTheta;

// The parameters for the differential coefficient is hereby defined
  if (samplingTime < 0)
  {
    Kd = 0;
  }
  else
  {
    Kd = (thetaError - storedthetaError);
  }
}

storedErrorOfTheta = errorOfTheta;

// Note that rhoDot denotes the instantenous forward velocity and phiDot denotes the rotational velcoity

rhoDot = RADII * (angularPos_R + angularPos_L)/2;
phiDot = RADII * (angularPos_R - angularPos_L)/DISTANCE;













 //Controller implementation 
/*void PIDController() { 
  
 //Takes current time 
  currentTime = millis(); 
  samplingTime = currentTime - storedTime; 
 //Gets current position 
  currentPosition = angularPos; 
  error = desiredPosition - currentPosition; 
  
  
  integrator = storedIntegrator + error * (samplingTime); 
  controlSignal = Kp * error + Ki * integrator; 
  
  if (abs(controlSignal) > PWM_BOUND) 
  { 
    controlSignal = constrain(controlSignal, -1, 1) * PWM_BOUND; 
    error = constrain (error, -1, 1) * min(abs(error), PWM_BOUND / Kp); 
    integrator = (controlSignal - Kp * error) / Ki; 
  } 
  
  controlSignal = abs(controlSignal); 
  
  if  (abs(error) < 0.015) 
  { 
    controlSignal = 0; 
    analogWrite(PWM_OUTPUT_PIN_R, controlSignal); 
  } 
  else if (error > 0) 
  { 
    digitalWrite (7, LOW); 
    analogWrite(PWM_OUTPUT_PIN_R, controlSignal); 
  } else if (error < 0) 
  { 
    digitalWrite (7, HIGH); 
    analogWrite(PWM_OUTPUT_PIN_R, controlSignal); 
  } 
 
  Serial.println(integrator); 
  //Serial.print("\t"); 
  //Serial.println(currentPosition); 
  storedIntegrator = integrator; 
  storedTime = currentTime; 
} */
  
// Function for callback for received data 
void receiveData(int byteCount) 
{  
#loop responsible for putting data into variable number. 
  while (Wire.available()) { 
    number = Wire.read(); 
    Serial.print(number); 
    Serial.print('\t'); 
  } 
} 
// callback for sending data 
void sendData() { 
   
  Wire.write(number); 
} 
