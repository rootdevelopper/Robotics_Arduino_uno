/******************************************************************
 *    
 *      --  Servo Test --
 *    
 *  code for "Build More Robots"  
 *  
 *  Program Starts with the servo looking forward for 3 sec, 
 *  then turns the servo Left 3 sec, then turns the servo
 *  Right 3 sec. Seqeunce repeat.
 *    
 *******************************************************************/
#include <Servo.h>         // uses timer1 on Arduino Uno (timer5 on Arduino Mega)

// define servo pin
#define PIN_SERVO  11    // Define sense pin of servo

// declare instance of Servo for the pan servo
Servo pan_servo;        // Set up the pan_servo
 
void setup() {
  Serial.begin(9600);     // Initialize serial port

  // attach pan servo to pin 11
  pan_servo.attach(PIN_SERVO); 

}

void loop() {
  // turn servo forward
  pan_servo.write(90);
  Serial.println("servo facing forward");
  delay(3000);            
  // turn servo left
  pan_servo.write(177);
  Serial.println("servo facing left");
  delay(3000);
  //turn servo right
  pan_servo.write(5);
  Serial.println("servo facing right");
  delay(3000);
}
