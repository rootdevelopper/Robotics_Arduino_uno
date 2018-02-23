/*******************************************************

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
 
 ********************************************************/
#include <Servo.h>         // uses timer1 on Arduino Uno (timer5 on Arduino Mega) 
#include <NewPing.h>

// define servo pin
#define PIN_SERVO  11    // Define sense pin of servo

// define pins for the motor driver board
#define PIN_IN1_L_MTR  A1  // Define A1 (15) Pin
#define PIN_IN2_L_MTR  A2  // Define A2 (16) Pin
#define PIN_IN3_R_MTR  A3  // Define A3 (17) Pin
#define PIN_IN4_R_MTR  A4  // Define A4 (18) Pin
#define PIN_ENA_L_PWM  5   //Define  D5 Pin
#define PIN_ENB_R_PWM  6   //Define  D6 Pin

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
    // The motor wires should be RED to front of robot, 
    //BLACK towards back of robot on both sides
    // set pin types for motor driver board
    pinMode(PIN_IN1_L_MTR,OUTPUT); // Define 14 pin for the output (IN1)
    pinMode(PIN_IN2_L_MTR,OUTPUT); // Define 15 pin for the output (IN2)
    pinMode(PIN_IN3_R_MTR,OUTPUT); // Define 16 pin for the output (IN3) 
    pinMode(PIN_IN4_R_MTR,OUTPUT); // Define 17 pin for the output (IN4)
   // In Arduino you don't need to use pinMode on pins that will be used for PWM
   // pinMode(PIN_ENA_L_PWM,  OUTPUT);  // Define 5 pin for PWM output 
   // pinMode(PIN_ENB_R_PWM,  OUTPUT); // Define 6 pin for PWM output
 
   // attach pan servo to pin 11
   pan_servo.attach(PIN_SERVO);   
}

 
void loop()
{
	// variables
	// setup speeds for different directions
	// uint8_t fwd_spd = 100;      // fwd spd
    // Forward speeds are split by motor to allow adjustment to correct drift
    // only works (kinda...) for one forward speed and one floor surface
    uint8_t fwd_L_mtr_spd = 99;      // fwd spd for L motor  
	uint8_t fwd_R_mtr_spd = 100;      // fwd spd for R motor
	uint8_t R_spd = 150;        // turning right spd
	uint8_t L_spd = 150;        // turning left spd
	uint8_t rev_spd = 200;      // bkup spd 
	
  // directions - must match directions in detection()
  const uint8_t GO_FWD = 0;  // forward
  const uint8_t GO_R = 1;    // right
  const uint8_t GO_L = 2;    // left
  const uint8_t GO_REV = 3;  // backwards
	
	uint8_t direction = 0;       // initial direction is forward
	 
    static uint32_t old_loop_tm = 0;  // debug

	// debug
    uint32_t loop_tm = millis();
    uint32_t delta_loop_tm = loop_tm - old_loop_tm;
    //Serial.print("delta_loop_tm: ");
   // Serial.println(delta_loop_tm);
    old_loop_tm = loop_tm;
    
    pan_servo.write(90);               // set servo to look forward
	
    direction = detection();        // determine if object in front of robot


	switch (direction) {
		case GO_FWD:                         // Forward
			advance(fwd_L_mtr_spd, fwd_R_mtr_spd);
			delay(50);
			Serial.println(" Advance ");
		break;
		case GO_R:                        // Right
			back(rev_spd); 
			delay(100);
			turnR(R_spd); 
			delay(600);
			Serial.println(" Right ");
		break;
		case GO_L:                        // Left
			back(rev_spd); 
			delay(100);
			turnL(L_spd); 
			delay(600);
			Serial.println(" Left ");
		break;
		case GO_REV:                        // Reverse
			back(rev_spd);
			delay(200);
			turnL(L_spd);              // go slightly left
			delay(200);
			Serial.println(" Reverse ");
		break;
		default :
			stop();
	}
	
}

//-------------- functions -------------------------------------

// direction functions 
void advance(uint8_t Lspd, uint8_t Rspd)     // go
    {
       // both motors fwd
       set_motors(Lspd,Rspd);
       
    }
    
void turnR(uint8_t spd)        //right
    {
       // left mtr backwards, right mtr fwd 
       set_motors(-spd,spd);
	}
    
void turnL(uint8_t spd)        //left
    {
       // left mtr fwd, right mtr backwards
       set_motors(spd, -spd);
    } 
       
void back(uint8_t spd)          //back
    {
       // both mtrs backwards
       set_motors(-spd,-spd); 
    }

void stop()         //stop
{
     // both mtrs stoped 
     set_motors(0,0);
}


// ultrasonic functions
float meas_dist()            // measure distance from ultrasonic sensor
{
      //float distance = pulseIn(echoPin, HIGH,60000);  // measure the echo pulse duration (timeout in 60 mSec)
      float duration = sonar.ping();  // measure the echo pulse duration (timeout in 60 mSec)
      float distance= duration/58;                    // convert time of echo pulse to dist in cm
      return (distance);
}

// distance / servo functions
float meas_us_fwd()            // Measure the distance ahead
{
      static uint32_t old_ping_tm = 0;
      pan_servo.write(90);
      float distance= meas_dist(); 
      //Serial.print("F distance:");      
     // Serial.println(distance);    

	  // debug
      //Serial.print("fwd_delta_ping_tm: ");
      uint32_t ping_tm = millis();
      uint32_t delta_ping_tm = ping_tm - old_ping_tm;
      //Serial.println(delta_ping_tm);
      old_ping_tm = ping_tm;
	  
      return (distance);     
}  
    
float meas_us_L()            // Measure the distance on the left 
{
      pan_servo.write(5);
      //delay(delay_time);
      float distance= meas_dist();      
      //Serial.print("L distance:");     
      //Serial.println(distance);        
      return (distance);             
}  

float meas_us_R()   // Measure the distance on the right 
{
      pan_servo.write(177);
      //delay(delay_time);
      float distance= meas_dist();      
      //Serial.print("R distance:");       
      //Serial.println(distance);        
      return (distance);              
}  

// object avoidance function    
uint8_t detection()                     
{      
      float dist = 0.0;
	  uint8_t direct = 0;
	  	
      const uint8_t FWD = 0;  // forward
      const uint8_t R = 1;    // right
      const uint8_t L = 2;    // left
      const uint8_t REV = 3;  // backwards

      const uint16_t SERVO_DELAY_TM = 200;

      
      dist = meas_us_fwd();                   // read dist in front of robot
	  
/*
     if((dist < 10) && (dist > 0)) {          // if obj is really close
        stop();                       
        direct = REV;
     } 
*/	           
     if((dist < 25) && (dist > 0)) {            // if obj is in front of robot
        stop();        
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
        stop();                       
        direct = REV;
     } 
     
	 return (direct);
     
} 

// motor functions
void set_motors(int16_t Lmtr_spd, int16_t Rmtr_spd)
{
   set_Lmtr_speed(Lmtr_spd);
   set_Rmtr_speed(Rmtr_spd); 
   Serial.print("Lmtr_spd: ");
   Serial.println(Lmtr_spd);
}

void set_Lmtr_speed(int16_t speed)
{
  uint8_t direct = 0;    // forward
  if (speed < 0){
    direct = 1;          // reverse
  }
  switch (direct) {
    case 0:
      digitalWrite(PIN_IN1_L_MTR, HIGH);
      digitalWrite(PIN_IN2_L_MTR, LOW);   
      analogWrite(PIN_ENA_L_PWM, speed);
      break;
    case 1:
      digitalWrite(PIN_IN1_L_MTR, LOW);
      digitalWrite(PIN_IN2_L_MTR, HIGH);   
      analogWrite(PIN_ENA_L_PWM, -speed);
      break;
    default :
      digitalWrite(PIN_IN1_L_MTR, LOW);
      digitalWrite(PIN_IN2_L_MTR, LOW);
  }
}

void set_Rmtr_speed(int16_t speed)
{
  uint8_t direct = 0;    // forward
  if (speed < 0){
    direct = 1;          // reverse
  }
  switch (direct) {
    case 0:
      digitalWrite(PIN_IN3_R_MTR, HIGH);
      digitalWrite(PIN_IN4_R_MTR, LOW);   
      analogWrite(PIN_ENB_R_PWM, speed);
      break;
    case 1:
      digitalWrite(PIN_IN3_R_MTR, LOW);
      digitalWrite(PIN_IN4_R_MTR, HIGH);   
      analogWrite(PIN_ENB_R_PWM, -speed);
      break;
    default :
      digitalWrite(PIN_IN3_R_MTR,LOW);
      digitalWrite(PIN_IN4_R_MTR,LOW);
  }
}
