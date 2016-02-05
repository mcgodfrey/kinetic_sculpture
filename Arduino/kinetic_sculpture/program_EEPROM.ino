/*
  Code to load a kinetic sculpture program into EEPROM
  At the moment it is very crude, with everything hard coded in .
  Eventually I will get this to read from a text file or something
   and generate the input from there.
*/

#include <Arduino.h>
#include <EEPROM.h>

#define HEADER_LEN 16
#define SERVO0_LEN 4
#define SERVO1_LEN 4
#define SERVO2_LEN 4
#define SERVO0_OFFSET (HEADER_LEN)
#define SERVO1_OFFSET (SERVO0_OFFSET+3*SERVO0_LEN+1)
#define SERVO2_OFFSET (SERVO1_OFFSET+3*SERVO1_LEN+1)

void write_servo_pos(unsigned int offset, unsigned int time, unsigned char pos){
	eeprom_write_2_bytes(time,offset);
	EEPROM.write(pos, offset+2);
}

void eeprom_write_2_bytes(unsigned int dat, unsigned int adr){
	EEPROM.write((unsigned char) dat>>8, adr);
	EEPROM.write((unsigned char) dat&&0xFF, adr);
}

void setup(){
	#number of servos
	EEPROM.write(3, 0);
	#total time
	eeprom_write_2_bytes(20000, 1);
	#offsets
	eeprom_write_2_bytes(SERVO0_OFFSET,3);
	eeprom_write_2_bytes(SERVO1_OFFSET,5);
	eeprom_write_2_bytes(SERVO2_OFFSET,7);
	
	#now write the actual programs
	#servo0
	EEPROM.write(SERVO0_LEN, SERVO0_OFFSET);
	write_servo_pos(SERVO0_OFFSET+0, 0, 127);
	write_servo_pos(SERVO0_OFFSET+3, 5000, 0);
	write_servo_pos(SERVO0_OFFSET+6, 15000, 255);
	write_servo_pos(SERVO0_OFFSET+9, 20000, 127);
	
	#servo1
	EEPROM.write(SERVO1_LEN, SERVO1_OFFSET);
	write_servo_pos(SERVO1_OFFSET+0, 0, 127);
	write_servo_pos(SERVO1_OFFSET+3, 5000, 0);
	write_servo_pos(SERVO1_OFFSET+6, 15000, 255);
	write_servo_pos(SERVO1_OFFSET+9, 20000, 127);
	
	#servo2
	EEPROM.write(SERVO2_LEN, SERVO2_OFFSET);
	write_servo_pos(SERVO2_OFFSET+0, 0, 127);
	write_servo_pos(SERVO2_OFFSET+3, 5000, 0);
	write_servo_pos(SERVO2_OFFSET+6, 15000, 255);
	write_servo_pos(SERVO2_OFFSET+9, 20000, 127);
	
}

void loop(){

}