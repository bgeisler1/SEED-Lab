enum maneuvering {SEARCH_TAPE,CALIBRATION,TURNING,FORWARD,CIRCLE,WAIT};

maneuvering maneuveringState = SEARCH_TAPE;

void maneuvering ()
{
  switch(maneuveringState)
    
    case SEARCH_TAPE:
    if (receivedCVInstructions != 0)
    {
      maneuveringState = CALIBRATION;
      // Initialize othe parameters
    }
    else // Hard code the rotational velocity
    {
      desiredRhoDot  = 0;
      desiredPhiDot  = 0.1;
      velocityControl();
    }
    break;
    case CALIBRATION:
    // A state where the rotational maneuvering has theorectically stopped but we have to want wait it actually stops
    currentPhiDot = RADII/100 * (angular_velocity_R - angular_velocity_L)/(DISTANCE/100);
    if (currentPhiDot == 0)
    {
    // Reset the parameters for angular positions, have a fresh restart
    angularPos_R  = 0;
    recorded_position_R = 0;  
    angularPos_L  = 0;
    recorded_position_L = 0;
    } else
    {
      // Do nothing otherwise, as we have to make sure it completely stops
    }
    break;
  case TURNING:
  
    break;
}

void velocityControl()
{
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
  
      // The tolerance is specified is in t
      if (abs(errorOfPhiDot) < 0.05)
      {
      Serial.print("  Stopping adjusting angles...    ");
      analogWrite(PWM_OUTPUT_PIN_R, 0);
      analogWrite(PWM_OUTPUT_PIN_L, 0); 
      
      }
      else if (errorOfPhiDot >= 0)
      {
      Serial.print("  Turning...   ");
      digitalWrite(7, HIGH); //+
      digitalWrite(8, LOW); // -
      analogWrite(PWM_OUTPUT_PIN_R, controlSignal_R*0.30);
      analogWrite(PWM_OUTPUT_PIN_L, controlSignal_L*0.30);
      } else 
      {
      Serial.print("  Turning in reverse   ");
      digitalWrite(7, LOW); //+
      digitalWrite(8, HIGH); // -
      analogWrite(PWM_OUTPUT_PIN_R, controlSignal_R*0.30);
      analogWrite(PWM_OUTPUT_PIN_L, controlSignal_R*0.30);
      }
  
      if (abs(errorOfRhoDot) < 0.001)
      {
      Serial.print("  stopped    ");
      analogWrite(PWM_OUTPUT_PIN_R, 0);
      analogWrite(PWM_OUTPUT_PIN_L, 0); 
      }
      else if (errorOfRhoDot >= 0)
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

void turnControl ()
{
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
  
 // These if's will determine if the drone will keep moving forward or correct the angular position if it went off due to friction along the way
      if (abs(positionError) <= 0.05){
          state = DRIVE_FORWARD;
          }
      else{
            state = TURN;
        }
      Serial.print(state);
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
      } else 
      {
      Serial.print("  Turning in reverse   ");
      digitalWrite(7, LOW); //+
      digitalWrite(8, HIGH); // -
      analogWrite(PWM_OUTPUT_PIN_R, controlSignal_R*0.30);
      analogWrite(PWM_OUTPUT_PIN_L, controlSignal_R*0.30);
      }
}

void forwardControl
{
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
      if (abs(distanceError) < 0.001)
      {
      Serial.print("  stopped    ");
      analogWrite(PWM_OUTPUT_PIN_R, 0);
      analogWrite(PWM_OUTPUT_PIN_L, 0); 
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
}

