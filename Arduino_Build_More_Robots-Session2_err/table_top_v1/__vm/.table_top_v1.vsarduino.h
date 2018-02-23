/* 
	Editor: http://www.visualmicro.com
			visual micro and the arduino ide ignore this code during compilation. this code is automatically maintained by visualmicro, manual changes to this file will be overwritten
			the contents of the Visual Micro sketch sub folder can be deleted prior to publishing a project
			all non-arduino files created by visual micro and all visual studio project or solution files can be freely deleted and are not required to compile a sketch (do not delete your own code!).
			note: debugger breakpoints are stored in '.sln' or '.asln' files, knowledge of last uploaded breakpoints is stored in the upload.vmps.xml file. Both files are required to continue a previous debug session without needing to compile and upload again
	
	Hardware: Arduino/Genuino Uno, Platform=avr, Package=arduino
*/

#define __AVR_ATmega328p__
#define __AVR_ATmega328P__
#define ARDUINO 10608
#define ARDUINO_MAIN
#define F_CPU 16000000L
#define __AVR__
#define F_CPU 16000000L
#define ARDUINO 10608
#define ARDUINO_AVR_UNO
#define ARDUINO_ARCH_AVR
//
//
void advance(uint8_t Lspd, uint8_t Rspd);
void turnR(uint8_t spd);
void turnL(uint8_t spd);
void back(uint8_t spd);
void stop();
float meas_dist();
float meas_us_fwd();
float meas_us_L();
float meas_us_R();
uint8_t detection();
void set_motors(int16_t Lmtr_spd, int16_t Rmtr_spd);
void set_Lmtr_speed(int16_t speed);
void set_Rmtr_speed(int16_t speed);
uint16_t evaluate_LVR_sensor(uint8_t sensor_side);
uint8_t moth(int16_t L_light_val, int16_t R_light_val, int16_t deadband);
uint8_t cockroach(int16_t L_light_val, int16_t R_light_val, int16_t deadband);
uint16_t proportional_response_LVR (uint16_t L_LVR_value, uint16_t R_LVR_value);

#include "pins_arduino.h" 
#include "arduino.h"
#include "table_top_v1.ino"
