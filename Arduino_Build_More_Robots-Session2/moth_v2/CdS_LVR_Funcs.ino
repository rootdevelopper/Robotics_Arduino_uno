/*********************************************
 * 
 * CdS_LVR_Funcs.ino
 * CdS Light Variable Resistor functions for 
 * "Build More Robot" Series - session 2
 * 
 *********************************************/

// CdS LVR functions
uint16_t evaluate_LVR_sensor(uint8_t sensor_side)        // sensor_side must have value of 1 or 2
{
  const uint8_t R = 1;                 // right
  const uint8_t L = 2;                 // left
  const uint16_t LVR_OFFSET =  0;      // single point offset (could be used to improve match somewhat)
  
  // variables
  int16_t light_val = 0;               // LSR reading

  if (sensor_side == R) {
     light_val = analogRead(PIN_LSR_L) + LVR_OFFSET;  // offset would be applied to only one side to match the other side
  }
  
  if (sensor_side == L) {
     light_val = analogRead(PIN_LSR_R);
  }

  return(light_val);
    
  
}

uint8_t moth(int16_t L_light_val, int16_t R_light_val, int16_t deadband)   // move to the light
{
  // variables
  const uint8_t FWD = 0;  // forward
  const uint8_t R = 1;            // right
  const uint8_t L = 2;            // left
  
  uint8_t direct = FWD;
  
  if ((R_light_val - L_light_val) < deadband) {
    direct = L;
  }
  // right CdS sensor sees more light (lower resistance)
  else if ((L_light_val - R_light_val) < deadband) {
    direct = R;
  }
    
  return(direct); 
}

uint8_t cockroach(int16_t L_light_val, int16_t R_light_val, int16_t deadband)    // seek the darkness
{
  // variables
  const uint8_t FWD = 0;  // forward
  const uint8_t R = 1;            // right
  const uint8_t L = 2;            // left
  
  uint8_t direct = FWD;
  
  if ((R_light_val - L_light_val) > deadband) {
    direct = L;
  }
  // right CdS sensor sees more light (lower resistance)
  else if ((L_light_val - R_light_val) > deadband) {
    direct = R;
  }
  
  return(direct);
}

uint16_t proportional_response_LVR (uint16_t L_LVR_value, uint16_t R_LVR_value) 
{
  const float KP = 2;
  uint16_t response = KP * abs(L_LVR_value - R_LVR_value);
  
  if (response > 100) {    // set max value
    response = 100;
  }
  
  return (response);
  
}

