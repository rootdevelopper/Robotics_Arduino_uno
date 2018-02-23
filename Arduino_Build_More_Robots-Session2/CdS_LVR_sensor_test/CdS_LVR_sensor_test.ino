/***********************************************************************
 * 
 *     -- CdS LVR Sensor Test --
 *     
 * code for "Build More Robots" tutorial series by DPRG
 * 
 * Program writes the CdS LvR sensor values to the serial port.
 * 
 * 
 ***********************************************************************/
// IR Obstacle Collision Detection Module

#define PIN_LSR_L A0          // Left CdS LSR sensor
#define PIN_LSR_R A5          // Right CdS LSR sensor

void setup() 
{
    digitalWrite(PIN_LSR_L, INPUT_PULLUP);  // set pullup on analog pin 0 
    digitalWrite(PIN_LSR_R, INPUT_PULLUP);  // set pullup on analog pin 0 
    Serial.begin(9600);
}

void loop() 
{
    int R_light_val = 0;       // Right LSR reading
    int L_light_val = 0;       // Left LSR reading

    L_light_val = analogRead(PIN_LSR_L);
    Serial.print("LSR_values: L ");
    Serial.print(L_light_val);
    delay(200);
    R_light_val = analogRead(PIN_LSR_R);
    Serial.print(", R ");
    Serial.println(R_light_val);
    delay(200);
}
