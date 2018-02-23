/*******************************************************************************

    --  Obstacle Avoidance  --
	
 Code for "Build More Robots" - session 1

  PGM: Obstacle_Avoidnance_simple.ino

 Detects when an object is in front of the robot and attempts to avoid it. 
 Its algorithm for avoiding an obstacle is:
   1. Use the ultrasonic sensor to measure distance to any object in front
      of the robot.
   2. If an object is determined to be too close, stop the robot and 
      use the ultrasonic sensor to look to the right and left of the robot.
   3. The distances of the right and left measurements are used to determine
      the direction to turn the robot.
   4. If both right and left directions appear to be blocked the robot backs up.
      If a clear direction exists the robot turns in that direction.
  
 Revisions:
     20180207 Doug Paradis   v0.0 

 notes:
     1. Remove ENA and ENB jumpers on L298 Motor 
        Controller connect to Pins D5 an D6
     2. Program uses NewPing library
 
*******************************************************************************/
#include <Servo.h>         // uses timer1 on Arduino Uno (timer5 on Arduino Mega)
#include <NewPing.h>

// define servo pin
#define PIN_SERVO  11    // Define sense pin of servo

// The motor wires should be RED to front of robot, 
//BLACK towards back of robot on both sides
// define pins for the motor driver board
#define PIN_IN1_L_MTR  A1  // Define A1 (15) Pin
#define PIN_IN2_L_MTR  A2  // Define A2 (16) Pin
#define PIN_IN3_R_MTR  A3  // Define A3 (17) Pin
#define PIN_IN4_R_MTR  A4  // Define A4 (18) Pin
#define PIN_ENA_L_PWM  6   //Define  D6 Pin
#define PIN_ENB_R_PWM  5   //Define  D5 Pin

// name the pins for the ultrasonic sensor
#define PIN_TRIGGER  8  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define PIN_ECHO     9  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

// declare instance of NewPing for the sonar
NewPing sonar(PIN_TRIGGER, PIN_ECHO, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

// declare instance of Servo for the pan servo
Servo pan_servo;        // Set up the pan_servo


void setup()
{
    Serial.begin(57600);     // Initialize serial port
  
    // set pin types for motor driver board
    pinMode(PIN_IN1_L_MTR,OUTPUT); // Define pin 14 for output (IN1)
    pinMode(PIN_IN2_L_MTR,OUTPUT); // Define pin 15 for output (IN2)
    pinMode(PIN_IN3_R_MTR,OUTPUT); // Define pin 16 for output (IN3) 
    pinMode(PIN_IN4_R_MTR,OUTPUT); // Define pin 17 for output (IN4)

   // In Arduino you don't need to use pinMode on pins that will be using PWM
   // pinMode(PIN_ENA_L_PWM,  OUTPUT); // Define pin 5 for output (PWM on ENA)  
   // pinMode(PIN_ENB_R_PWM,  OUTPUT); // Define pin 6 for output (PWM on ENB)
 
    // attach pan servo to pin 11
    pan_servo.attach(PIN_SERVO);   
}

 
void loop()
{

   	// variables
   	// setup speeds for different directions
   	unsigned char fwd_spd = 100;      // fwd spd
   	unsigned char R_spd = 150;        // turning right spd
   	unsigned char L_spd = 150;        // turning left spd
   	unsigned char rev_spd = 200;      // bkup spd
   	
   	// directions - must match directions in detection()
   	const unsigned char GO_FWD = 0;  // forward
   	const unsigned char GO_R = 1;    // right
   	const unsigned char GO_L = 2;    // left
   	const unsigned char GO_REV = 3;  // backwards
   	
   	unsigned char direction = 0;       // initial direction is forward
    
    pan_servo.write(90);               // set servo to look forward

    direction = detection();           // determine if object in front of robot
    
   // move based on direction determined by location of obstacle
   if(direction == GO_REV) {             // go backwards
     back(rev_spd, 200);               
     turnL(L_spd, 200);              // go slightly left
     Serial.println(" Reverse ");   
   }
   if(direction == GO_R) {             // go right
     back(rev_spd, 100); 
     turnR(R_spd, 600);                 
     Serial.println(" Right ");   
   }
   if(direction == GO_L) {             // go left 
     back(rev_spd, 100);      
     turnL(L_spd, 600);                  
     Serial.println(" Left ");       
   }  
   if(direction == GO_FWD) {            // go forward
    advance(fwd_spd, 50);                
    Serial.println(" Advance ");     
   }
}

 //-------------- functions -------------------------------------

// direction functions  
void advance(int spd, int tm_mov)     // go
{
       // right mtr fwd
       digitalWrite(PIN_IN3_R_MTR,HIGH);        
       digitalWrite(PIN_IN4_R_MTR,LOW);         
       analogWrite(PIN_ENB_R_PWM,spd);
       // left mtr fwd      
       digitalWrite(PIN_IN1_L_MTR,HIGH);        
       digitalWrite(PIN_IN2_L_MTR,LOW);         
       analogWrite(PIN_ENA_L_PWM,spd);
       delay(tm_mov); 
}
    
void turnR(int spd, int tm_mov)        //right
{
       //right mtr fwd 
       digitalWrite(PIN_IN3_R_MTR,HIGH);  
       digitalWrite(PIN_IN4_R_MTR,LOW);
       analogWrite(PIN_ENB_R_PWM,spd);
       // left mtr backwards
       digitalWrite(PIN_IN1_L_MTR,LOW);
       digitalWrite(PIN_IN2_L_MTR,HIGH);  
       analogWrite(PIN_ENA_L_PWM,spd);
       delay(tm_mov);
}
    
void turnL(int spd, int tm_mov)        //left
{
       // right mtr backwards
       digitalWrite(PIN_IN3_R_MTR,LOW);
       digitalWrite(PIN_IN4_R_MTR,HIGH);   
       analogWrite(PIN_ENB_R_PWM,spd);
       // left mtr fwd
       digitalWrite(PIN_IN1_L_MTR,HIGH);   
       digitalWrite(PIN_IN2_L_MTR,LOW);
       analogWrite(PIN_ENA_L_PWM,spd);
       delay(tm_mov);
} 
       
void back(int spd, int tm_mov)          //back
{
       // right mtr backwards
       digitalWrite(PIN_IN3_R_MTR,LOW);  
       digitalWrite(PIN_IN4_R_MTR,HIGH);
       analogWrite(PIN_ENB_R_PWM, spd);
       // left mtr backwards
       digitalWrite(PIN_IN1_L_MTR,LOW);  
       digitalWrite(PIN_IN2_L_MTR,HIGH);
       analogWrite(PIN_ENA_L_PWM, spd); 
       delay(tm_mov);                   // motor run time
   
}

void stop(int tm_mov)         //stop
{
     // both mtrs stopped 
     digitalWrite(PIN_IN3_R_MTR,LOW);
     digitalWrite(PIN_IN4_R_MTR,LOW);
     digitalWrite(PIN_IN1_L_MTR,LOW);
     digitalWrite(PIN_IN2_L_MTR,LOW);
     delay(tm_mov);
}

// ultrasonic functions
float meas_dist()            // measure distance from ultrasonic sensor
{
      float duration = sonar.ping();       // measure the echo pulse duration (timeout determined by MAX_DISTANCE)
      float distance= duration/58.0;         // convert time of echo pulse to dist in cm
      return (distance);
}
  
// distance / servo functions
float meas_us_fwd()            // Measure the distance ahead
{
	pan_servo.write(90);

	float distance= meas_dist();

	Serial.print("F distance:");
	 Serial.println(distance);
	
	return (distance);
}

float meas_us_L()            // Measure the distance on the left
{
	pan_servo.write(177);

	float distance= meas_dist();

	Serial.print("L distance:");
	Serial.println(distance);

	return (distance);
}

float meas_us_R()          // Measure the distance on the right
{
	pan_servo.write(5);

	float distance= meas_dist();

	Serial.print("R distance:");
	Serial.println(distance);

	return (distance);
}

// object avoidance function    
unsigned char detection()          // detect obstacles           
{      
      float dist = 0.0;
	  unsigned char direct = 0;
	  	
      const unsigned char FWD = 0;  // forward
      const unsigned char R = 1;    // right
      const unsigned char L = 2;    // left
      const unsigned char REV = 3;  // backwards

      const unsigned int SERVO_DELAY_TM = 200;

      
      dist = meas_us_fwd();                   // read dist in front of robot
	             
     if((dist < 25) && (dist > 0)) {            // if obj is in front of robot
        stop(200);        
        float Ldist = meas_us_L();              // Read the left distance
        delay(SERVO_DELAY_TM);                  // Wait for servo motor to stabilize 
        float Rdist = meas_us_R();              // Read the right distance  
        delay(SERVO_DELAY_TM);                  // Wait for servo motor to stabilize  
        
        if(Ldist > Rdist) {                     
           direct = L;                          // turn Left
		   
        }
        if(Ldist <= Rdist) {                    // turn Right
           direct = R;                      
        } 
        if (Ldist < 15 && Rdist < 15) {         // backup
           direct = REV;             
        }          
     }
     else {                                     // go forward    
        direct = FWD;            
     }

     if((dist < 10) && (dist > 0)) {          // if obj is really close
        stop(200);                       
        direct = REV;
     } 
     
	 return (direct);
     
     
} 

   



