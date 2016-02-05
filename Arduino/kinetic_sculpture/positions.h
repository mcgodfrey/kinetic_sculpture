#ifndef POSITIONS_H_
#define POSITIONS_H_

#include <EEPROM.h>
#include <SPI.h>

/* Calculates the required servo position at a particular time*/
unsigned char calc_servo_pos(servo_no, t);

/* Send the servo position to the FPGA over SPI*/
void write_servo_pos(unsigned char servo_no, unsigned char pos);

/* Returns the total running time (im ms) for this program */
unsigned int get_prog_time();

/* returns the number of servos used in this program */
unsigned char get_num_servos();

/* returns the address offset for the time/position data for servo number "servo_num" in memory */
unsigned int get_offset(unsigned char servo_no);

/*Get number of program points for this ball. 0-255 */
unsigned char get_num_points(unsigned int offset);

/*returns the timestamp for point "i" for a servo */
unsigned int get_time(unsigned int offset, unsigned char i);

/*returns the position for point "i" for a servo */
unsigned char get_pos(unsigned int offset, unsigned char i);

/* performs linear interpolation */
unsigned char interp_pos(t0, t1, p0, p1, t);

#endif /* POSITIONS_H_ */
