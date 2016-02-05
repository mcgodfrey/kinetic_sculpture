#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_


typedef enum {
	INIT, PAUSE, UPDATE_POS, TRIGGER
} progState_t;

progState_t progState = INIT;

//update interval (ms)
int update_interval = 100;


#define LED_REG 0x00

//Trigger is pin 1 of the avr_flags
#define TRIGGER_PORT PORTB
#define TRIGGER_DDR DDRB
#define TRIGGER_PIN PINB
#define TRIGGER_MASK 0x10

#endif // DEFINTIONS_H_
