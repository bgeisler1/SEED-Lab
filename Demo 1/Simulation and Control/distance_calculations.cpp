// define constants
# ALPHA  (17/180) * PI
# H 0.17

loop()
{
 *********************************
 /* Stuff from the previous codes
 *********************************
 
 */
  
thetaRadians = theta/180 * PI;
phiRadians = phi/180 * PI;
totalRho = H * (tan(ALPHA) + tan(PI/2 - thetaRadians + ALPHA));

// If sensed a change, update the desiredValues in the functions
if (theta != storedTheta || phi != storedPhi)
{
desiredRho = totalRho;
desiredPhi = phiRadians;
}

storedTheta = theta;
storedPhi = phi;
}
