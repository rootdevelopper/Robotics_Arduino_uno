/*******************************************************************************

    --  Motor Test  --
	
 Code for "Build More Robots" 

*******************************************************************************/

// define pins for the motor driver board
#define PIN_IN1_L_MTR  A1  // Define A1 (15) Pin
#define PIN_IN2_L_MTR  A2  // Define A2 (16) Pin
#define PIN_IN3_R_MTR  A3  // Define A3 (17) Pin
#define PIN_IN4_R_MTR  A4  // Define A4 (18) Pin
#define PIN_EN0_L_PWM  5   //Define  D5 Pin
#define PIN_EN1_R_PWM  6   //Define  D6 Pin

void setup()
{
    Serial.begin(9600);     // Initialize serial port
  
    // set pin types for motor driver board
    pinMode(PIN_IN1_L_MTR,OUTPUT); // Define pin 14 for output (IN1)
    pinMode(PIN_IN2_L_MTR,OUTPUT); // Define pin 15 for output (IN2)
    pinMode(PIN_IN3_R_MTR,OUTPUT); // Define pin 16 for output (IN3) 
    pinMode(PIN_IN4_R_MTR,OUTPUT); // Define pin 17 for output (IN4)

   // In Arduino you don't need to use pinMode on pins that will be using PWM
   // pinMode(PIN_EN0_L_PWM,  OUTPUT); // Define pin 5 for output (PWM on EN0)  
   // pinMode(PIN_EN1_R_PWM,  OUTPUT); // Define pin 6 for output (PWM on EN1)
 }

 
void loop()
{

    // stop both motors
    stopMotors();
    Serial.println("stop both motors");
    delay(5000);
    // run left motor only
    leftMotor(150);
    Serial.println("run left motor only");
    delay(5000);
    // run right motor only
    rightMotor(150);
    Serial.println("run right motor only");
    delay(5000);
}

 //-------------- functions -------------------------------------
// stop both motors
void stopMotors(void)
{
  // stop left motor
  digitalWrite(PIN_IN1_L_MTR,LOW);        
  digitalWrite(PIN_IN2_L_MTR,LOW);         
  analogWrite(PIN_EN0_L_PWM,0);  
  // stop right motor
  digitalWrite(PIN_IN3_R_MTR,LOW);        
  digitalWrite(PIN_IN4_R_MTR,LOW);         
  analogWrite(PIN_EN1_R_PWM,0);  
}
// run left motor only
void leftMotor(int spd)
{
  // run left motor forward
  digitalWrite(PIN_IN1_L_MTR,HIGH);        
  digitalWrite(PIN_IN2_L_MTR,LOW);         
  analogWrite(PIN_EN0_L_PWM,spd);  
  // stop right motor
  digitalWrite(PIN_IN3_R_MTR,LOW);        
  digitalWrite(PIN_IN4_R_MTR,LOW);         
  analogWrite(PIN_EN1_R_PWM,0);  
}
// run right motor only
void rightMotor(int spd)
{
  // stop left motor
  digitalWrite(PIN_IN1_L_MTR,LOW);        
  digitalWrite(PIN_IN2_L_MTR,LOW);         
  analogWrite(PIN_EN0_L_PWM,0);  
  // run right motor forward
  digitalWrite(PIN_IN3_R_MTR,HIGH);        
  digitalWrite(PIN_IN4_R_MTR,LOW);         
  analogWrite(PIN_EN1_R_PWM,spd);  
}

