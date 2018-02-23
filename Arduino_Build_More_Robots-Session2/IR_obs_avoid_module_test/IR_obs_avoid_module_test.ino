/***********************************************************************
 * 
 *     -- IR Obstacle Avoidance Sensor Module Test --
 *     
 * code for "Build More Robots" tutorial series by DPRG
 * 
 * Program writes the module's value to the serial port and turns on
 * the LED connected to pin 13 of the Uno when an obstacle is detected.
 * 
 * 
 ***********************************************************************/
// IR Obstacle Collision Detection Module

#define PIN_LED  13           // onboard Uno LED
#define PIN_IR_SIGNAL 7       // IR module's signal pin

void setup() 
{
    Serial.begin(9600);
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_IR_SIGNAL, INPUT);
}

void loop() 
{
    int obstacle = HIGH;  // HIGH means no obstacle found
    
    obstacle = digitalRead(PIN_IR_SIGNAL);
    if ( obstacle == LOW) {
        Serial.println("OBSTACLE");
        digitalWrite(PIN_LED, HIGH);
    }
    else {
        Serial.println("clear");
        digitalWrite(PIN_LED, LOW);
    }
    delay(200);
}
