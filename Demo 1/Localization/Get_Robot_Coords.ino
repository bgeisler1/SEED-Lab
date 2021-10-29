//Localization arduino code - Demo 1
//Written by Ben Geisler
//Gets velocity and position coordinates of robot. 
//Prints the position coordinates (x, y, phi) to serial monitor

#include <Encoder.h>

//Global variables
int N = 3200;
double SAMPLE_TIME = 10;
double t = 0;

//Position variables
long motorPosL;
double angularPosL = 0;
long motorPosR;
double angularPosR = 0;
double newPosL = 0;
double newAngPosL = 0;
double angVelocityL = 0;
double newPosR = 0;
double newAngPosR = 0;
double angVelocityR = 0;


//wheel radius (r)/separation (b)
double r = 7.3;//Inches
double b = 24.45;//Inches

//Pin assignments for encoder
const int PIN_A_RIGHT = 2;
const int PIN_A_LEFT = 5;
const int PIN_B_RIGHT = 3;
const int PIN_B_LEFT = 6;

//x-y-phi position variables
double x_new = 0;
double x_old = 0;
double y_new = 0;
double y_old = 0;
double phi_new = 0;
double phi_old = 0;

//velocities
double v_L;
double v_r;
double v_r_old;
double v_L_old;

Encoder motorEncL(PIN_A_LEFT,PIN_A_RIGHT);
Encoder motorEncR(PIN_B_LEFT,PIN_B_RIGHT);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(12, INPUT);

  //take current time
  t_old = (double)millis()/1000;

}

void loop() { 

  //Current time
  t = millis();

  //Take sample
  //
  //Read position
  motorPosL = motorEncL.read();
  motorPosR = motorEncR.read();
  
  //Find current angular position
  angularPosR = 2*PI*(double)motorPos/(double)N;
  angularPosL = 2*PI*(double)motorPos/(double)N;

  //Wait sampling time
  while(millis()< t + SAMPLE_TIME){
    //WAIT
  }

  //Get the new positions
  newPosR = motorEncR.read();
  newPosL = motorEncL.read();
  
  //New angular positions
  newAngPosR = 2*PI*(double)newPosR/(double)N;
  newAngPosL = 2*PI*(double)newPosL/(double)N;

  //Angular velocities
  angVelocityR = (newAngPosR - angularPosR)/(SAMPLE TIME/1000);
  angVelocityL = (newAngPosL - angularPosL)/(SAMPLE TIME/1000);
  
  //Velocity right
  v_r = r*angVelocityR;
  //Velocity left
  v_L = r*angVelocityL;

  //Get the new values of phi, x, y
  phi_new = phi_old + (SAMPLE_TIME)*(r/b)*(v_L - v_r);
  x_new = x_old + (SAMPLE_TIME)*cos(phi_old)*(v_L + v_r)/2;
  y_new = y_old + (SAMPLE_TIME)*sin(phi_old)*(v_L + v_r)/2;

  //Update old values
  x_old = x_new;
  y_old = y_new;
  phi_old = phi_new;
  v_r_old = v_r;

  //Prints the positions as they update
  Serial.print(x_new);
  Serial.print("\t");
  Serial.print(y_new);
  Serial.print("\t");
  Serial.println(phi_new);

}
