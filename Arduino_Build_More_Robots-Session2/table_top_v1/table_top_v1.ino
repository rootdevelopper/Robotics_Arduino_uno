/*******************************************************

    --  Table Top  --
	
 Code for "Build More Robots" - session 2

  PGM: table_top_v1.ino

 Uses two IR obstacle avoidance sensors with comparator outputs to monitor
 the table top. When the sensor overhangs the table, the output goes low. While
 over the table top the sensor goes high indicating an obstacle. 
 
 The output of the two IR sensors are connected to INT0 and INT1 and an
 interrupt service routine (ISR) is performed when the interrupts are triggered.
 
 Ultrasonic Obstacle Avoidance Sensor:
 Detects when an object is in front of the robot and attempts to avoid it. 
 Its algorithm for avoiding an obstacle is:
   1. The ultrasonic sensor is positioned 15 degrees from forward to the right
      and the distance is measured of any object in right front of the robot.
   2. Th ultrasonic sensor is positioned 15 degrees from forward to the left
      and the distance is measured of any object in left front of the robot.
   3. These two distance readings are used to determine if the robot should 
      turn left or right or backup.	

An arbitration function is used to determine if the robot should move based on the
LVR measurements or the obstacle avoidance measurements.

Startup routine included to allow the robot to be placed on floor before the 
wheels start to turn.
  
 Revisions:
     20180212 Doug Paradis   v0.0 

 notes:
     1. Remove ENA and ENB jumpers on L298 Motor 
        Controller connect to Pins D5 an D6
     2. Program uses NewPing library
	 3. IR drop sensors are connected to left: D7, right: D4.
 
 ********************************************************/
#include <Servo.h>         // uses timer1 on Arduino Uno (timer5 on Arduino Mega) 
#include <NewPing.h>

// define servo pin
#define PIN_SERVO  11    // Define signal pin of servo


// The motor wires should be RED to front of robot, BLACK towards back of robot on both sides
// define pins for the motor driver board
#define PIN_IN1_L_MTR  A1  // Define A1 (15) Pin
#define PIN_IN2_L_MTR  A2  // Define A2 (16) Pin
#define PIN_IN3_R_MTR  A3  // Define A3 (17) Pin
#define PIN_IN4_R_MTR  A4  // Define A4 (18) Pin
#define PIN_ENA_L_PWM  5   //Define  D5 Pin
#define PIN_ENB_R_PWM  6   //Define  D6 Pin

// Define drop sensor pins
#define PIN_L_DROP_SEN 7
#define PIN_R_DROP_SEN 4


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
    Serial.begin(115200);     // Initialize serial port
	
    // set pin types for motor driver board
    pinMode(PIN_IN1_L_MTR,OUTPUT); // Define 14 pin for the output (IN1)
    pinMode(PIN_IN2_L_MTR,OUTPUT); // Define 15 pin for the output (IN2)
    pinMode(PIN_IN3_R_MTR,OUTPUT); // Define 16 pin for the output (IN3) 
    pinMode(PIN_IN4_R_MTR,OUTPUT); // Define 17 pin for the output (IN4)
   // In Arduino you don't need to use pinMode on pins that will be used for PWM
   // pinMode(PIN_ENA_L_PWM,  OUTPUT);  // Define 5 pin for PWM output 
   // pinMode(PIN_ENB_R_PWM,  OUTPUT); // Define 6 pin for PWM output
   
   // set pins for drop sensors
   pinMode(PIN_L_DROP_SEN,INPUT);
   pinMode(PIN_R_DROP_SEN,INPUT);
   
 
   // attach pan servo to pin 11
   pan_servo.attach(PIN_SERVO);   
   
   // start the motors after 3 sec delay
   delay (3000);
}

 
void loop()
{
	// variables
	// setup speeds for different directions
	// uint8_t fwd_spd = 100;      // fwd spd
    // Forward speeds are split by motor to allow adjustment to correct drift
    // only works (kinda...) for one forward speed and one floor surface
  uint8_t fwd_L_mtr_spd = 89;      // fwd spd for L motor  
	uint8_t fwd_R_mtr_spd = 90;      // fwd spd for R motor
	uint8_t R_spd = 150;        // turning right spd
	uint8_t L_spd = 150;        // turning left spd
	uint8_t rev_spd = 200;      // bkup spd 
	
	// directions - must match directions in detection()
	const uint8_t GO_FWD = 0;  // forward
	const uint8_t GO_R = 1;    // right
	const uint8_t GO_L = 2;    // left
	const uint8_t GO_REV = 3;  // backwards
	
	uint8_t direction = 0;       // initial direction is forward
	
	
	direction = GO_FWD;
	
	if ((digitalRead(PIN_L_DROP_SEN) == HIGH) || (digitalRead(PIN_R_DROP_SEN) == HIGH )) {
		direction = GO_L;
	}
	
	
	switch (direction) {
		case GO_FWD:                         // Forward
			advance(fwd_L_mtr_spd, fwd_R_mtr_spd);
			//delay(50);
		break;
		case GO_R:                        // Right
			back(rev_spd); 
			delay(100);
			turnR(R_spd); 
			delay(600);
		break;
		case GO_L:                        // Left
			back(rev_spd); 
			delay(300);          //delay(100);
			turnL(L_spd); 
			delay(600);
		break;
		case GO_REV:                        // Reverse
			back(rev_spd);
			delay(200);
			turnL(L_spd);              // go slightly left
			delay(200);
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
      float duration = sonar.ping();  // measure the echo pulse duration (timeout in 60 mSec)
      float distance= duration/58.0;                    // convert time of echo pulse to dist in cm
      return (distance);
}

// distance / servo functions
float meas_us_fwd()            // Measure the distance ahead
{
      pan_servo.write(90);
      float distance= meas_dist(); 
  
      return (distance);     
}  
    
float meas_us_L()            // Measure the distance on the left 
{
      pan_servo.write(177);
      float distance= meas_dist();      
    
      return (distance);             
}  

float meas_us_R()   // Measure the distance on the right 
{
      pan_servo.write(5);
      float distance= meas_dist();      
   
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
	  
	  direct = FWD;

      
      dist = meas_us_fwd();                   // read dist in front of robot
           
     if((dist < 25) && (dist > 0)) {            // if obj is in front of robot
        stop(); 
		       
        float Ldist = meas_us_L();              // Read the left distance
        delay(SERVO_DELAY_TM);                  // Wait for servo motor to stabilize 
		
        float Rdist = meas_us_R();              // Read the right distance;  
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



