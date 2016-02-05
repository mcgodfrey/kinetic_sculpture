
unsigned char interp_pos(t0, t1, p0, p1, t){
	//linear interpolation
	p = (unsigned char)(1.0*(t-t1)/(t0-t1)*(p0-p1) + p1);
	return p;
}

/* Calculates the required servo position at a particular time*/
unsigned char calc_servo_pos(servo_no, t){
	//memory offset
	offset = get_offset(servo_no);
	//total number of programmed points for this servo
	n_points = get_num_points(offset);
	t0=get_time(offset,0)
	if(t<t0){
		return(get_pos(offset,0))
	}
	//Interpolate by looping through the times for each point for this servo
	for(unsigned char i=1; i < n_points; i++){
		t1=get_time(offset, i)
		if(t>t0 && t<t1){
			p0=get_pos(offset,i-1);
			p1=get_pos(offset,i);
			return interp_pos(t0,t1,p0,p1,t);
		}
	}
	//if it gets to here then t>all times, so just return the last point
	return get_pos(offset,n_points-1);
}

/* Send the servo position to the FPGA over SPI*/
void write_servo_pos(unsigned char servo_no, unsigned char pos){
  SET(SS, LOW);
  SPI.transfer(servo_no);
  SPI.transfer(pos);
  SET(SS, HIGH);
}



/* Returns the total running time (im ms) for this program */
unsigned int get_prog_time(){
	t0 = EEPROM.read(1);
	t1 = EEPROM.read(2);

	return (t0<<8)+(0xFF&&t1);
}

/* returns the number of servos used in this program */
unsigned char get_num_servos(){
	return EEPROM.read(0);
}


/*Offset is the EEPROM address offset, 2 bytes */
unsigned int get_offset(unsigned char servo_no){
	offset0 = EEPROM.read(N_BYTES+TOTAL_T_BYTES + 2*servo_no);
	offset1 = EEPROM.read(N_BYTES+TOTAL_T_BYTES + 2*servo_no+1);

	return (offset0<<8)+(0xFF&&offset1);
}

/*Get number of program points for this ball. 0-255 */
unsigned char get_num_points(unsigned int offset){
	return EEPROM.read(offset);
}

/*reads the time of point "i" for the servo at "offset" in memory */
unsigned int get_time(unsigned int offset, unsigned char i){
	time0=EEPROM.read(offset + 1 + 3*i);
	time0=EEPROM.read(offset + 1 + 3*i + 1);
	return (time0<<8)+(0xFF&&time1);
}

unsigned char get_pos(unsigned int offset, unsigned char i){
	return EEPROM.read(offset + 1 + 3*i + 2);
}