/***********************************************************************
 * 
 *     -- HC-SR04 Ultrasonic Sensor Test --
 *     
 * code for "Build More Robots" tutorial series by DPRG
 * 
 * Program writes the HC-SR04 object detection distance values 
 * the serial port.
 * 
 * 
 ***********************************************************************/

#include <NewPing.h>

// name the pins for the ultrasonic sensor
#define PIN_TRIGGER  8  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define PIN_ECHO     9  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm. (optismist)

// declare instance of NewPing for the sonar
NewPing sonar(PIN_TRIGGER, PIN_ECHO, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

void setup()
{
    Serial.begin(9600);     // Initialize serial port
}

 
void loop()
{
   float duration = sonar.ping();       // measure the echo pulse duration (timeout determined by MAX_DISTANCE)
   float distance= duration/58.0;         // convert time of echo pulse to dist in cm
   Serial.print("F distance: ");
   Serial.print(distance);
   Serial.println(" cm");
   
   delay (100);
}

 
