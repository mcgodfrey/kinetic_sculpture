void write_stepper_pos(uint8_t stepper_no, uint8_t stepper_pos){
  SET(SS, LOW);
  SPI.transfer(stepper_no);
  SPI.transfer(stepper_pos);
  SET(SS, HIGH);
}
