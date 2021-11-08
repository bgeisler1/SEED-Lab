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
      
    }
}
