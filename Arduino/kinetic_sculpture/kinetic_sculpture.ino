/*
 * Arduino control code for kinetic sculpture clone
 * Based on the mojo base project for initialising the FPGA
 * 
 * Once all the initialisation is done, the arduino is used to control
 *  the FPGA. I has a programmed sequence for each of the balls, and it
 *  loops through, updating the positions of all the balls every x ms.
 *  Once it has sent all the new positions to the FPGA it waits until
 *  it's interval timer expires and then sends a trigger to get the FPGA
 *  to actually update all the positions.
 * 
*/
#include "hardware.h"
#include "ring_buffer.h"
#include <SPI.h>
#include "flash.h"
#include "registers.h"

typedef enum {
  IDLE,
  READ_SIZE,
  WRITE_TO_FLASH,
  WRITE_TO_FPGA,
  VERIFY_FLASH,
  LOAD_FROM_FLASH
} 
loaderState_t;

typedef enum {
  WAIT, START_LOAD, LOAD, SERVICE
} 
taskState_t;

//update interval (ms)
int interval = 100;

//Trigger is pin 1 of the avr_flags
#define TRIGGER_PORT PORTB
#define TRIGGER_DDR DDRB
#define TRIGGER_PIN PINB
#define TRIGGER_MASK 0x10

uint8_t loadBuffer[800];
RingBuffer_t serialBuffer;
volatile taskState_t taskState = SERVICE;

program prog_code;
prog_code.all_servos[0].n=4;
prog_code.all_servos[0].positions[0]=127;
prog_code.all_servos[0].positions[1]=0;
prog_code.all_servos[0].positions[2]=255;
prog_code.all_servos[0].positions[3]=127;
prog_code.all_servos[0].times[0]=0;
prog_code.all_servos[0].times[1]=5000;
prog_code.all_servos[0].times[2]=15000;
prog_code.all_servos[0].times[3]=20000;


unsigned char interp_pos(t, times, positions, n){
	if (t == 0) {
		return(positions[0]);
	}
	int i;
	for (i = 0; i < n, i++) {
		if (times[i] > t) {
			break;
		}
	}
	//linear interpolation
	y_lin = (unsigned char)(1.0*(t - times[i])/(times[i-1] - times[i])*(positions[i-1]-positions[i]) + positions[i]);
	return y_lin;
}


unsigned char calc_servo_pos(prog_code,servo_no, t){
	times = prog_code.all_servos[servo_no].times;
	positions = prog_code.all_servos[servo_no].positions;
	n = prog_code.all_servos[servo_no].n; 
	
	return (interp_pos(t, times, positions, n));
}


/* This is where you should add your own code! Feel free to edit anything here. 
 This function will work just like the Arduino loop() function in that it will
 be called over and over. You should try to not delay too long in the loop to 
 allow the Mojo to enter loading mode when requested. */
void userLoop() {
	//for each stepper at next time
	//  calculate new position
	//  write_stepper_pos(servo_no, stepper_pos);
	//wait for interval timer to expire
	//pulse the update pin to get the fpga to update positions
	
	static unsigned char servo_num = 0;
	static unsigned int interval_no = 0;
	static unsigned int prev_time = milis();
	if (servo_num < NUM_SERVOS) {
		servo_pos = calculate_servo_pos(servo_no, interval_no*interval);
		write_servo_pos(servo_num, servo_pos);
		servo_num++;
	} else {
		unsigned int current_time = millis();
		if (curr_time > prev_time+interval) {
			SET(TRIGGER,HIGH);	//pulse the TRIGGER pin to send
			SET(TRIGGER,LOW);		//new positions to the servos
			prev_time = curr_time;
			servo_num = 0;
			interval_no++;
		}
	}
}


void userInit() {
	OUT(TRIGGER);			// TRIGGER is an output
	SET(TRIGGER,LOW);		// active HIGH
}






/* this is used to undo any setup you did in initPostLoad */
void disablePostLoad() {
  UCSR1B = 0; // disable serial port
  SPI.end();  // disable SPI
  SET(CCLK, LOW);
  OUT(PROGRAM);
  SET(PROGRAM, LOW); // reset the FPGA
  IN(INIT);
  SET(INIT, HIGH); // pullup on INIT
}

/* Here you can do some setup before entering the userLoop loop */
void initPostLoad() {
  Serial.flush();

  // These buffers are used by the demo ADC/Serial->USB code to prevent dropped samples
  RingBuffer_InitBuffer(&serialBuffer, loadBuffer, 256);
  
  FLAGS_DDR &= ~FLAGS_MASK; // make inputs
  FLAGS_PORT &= ~FLAGS_MASK; // no pull ups

  // Again, the Arduino libraries didn't offer the functionality we wanted
  // so we access the serial port directly. This sets up an interrupt
  // that is used with our own buffer to capture serial input from the FPGA
  UBRR1 = 1; // 0.125 M Baud

  UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
  UCSR1A = (1 << U2X1);
  UCSR1B = (1 << TXEN1) | (1 << RXEN1) | (1 << RXCIE1);
  UCSR1B = 0; // disable serial port

  // Setup all the SPI pins
  SET(CS_FLASH, HIGH);
  OUT(SS);
  SET(SS, HIGH);
  SPI_Setup(); // enable the SPI Port

  DDRD |= (1 << 3);
  DDRD &= ~(1 << 2);
  PORTD |= (1 << 2);

  // This pin is used to signal the serial buffer is almost full
  OUT(TX_BUSY);
  SET(TX_BUSY, LOW);

  // set progam as an input so that it's possible to use a JTAG programmer with the Mojo
  IN(PROGRAM);

  // the FPGA looks for CCLK to be high to know the AVR is ready for data
  SET(CCLK, HIGH);
  userInit();
}

void setup() {
  /* Disable clock division */
  clock_prescale_set(clock_div_1);

  OUT(CS_FLASH);
  SET(CS_FLASH, HIGH);
  OUT(CCLK);
  OUT(PROGRAM);

  /* Disable digital inputs on analog pins */
  DIDR0 = 0xF3;
  DIDR2 = 0x03;

  Serial.begin(115200); // Baud rate does nothing

  sei(); // enable interrupts

  loadFromFlash(); // load on power up
  initPostLoad();
}

void loop() {
  static loaderState_t state = IDLE;
  static int8_t destination;
  static int8_t verify;
  static uint32_t byteCount;
  static uint32_t transferSize;

  int16_t w;
  uint8_t bt;
  uint8_t buffIdx;

  switch (taskState) {
  case WAIT:
    break;
  case START_LOAD: // command to enter loader mode
    disablePostLoad(); // setup peripherals
    taskState = LOAD; // enter loader mode
    state = IDLE; // in idle state
    break;
  case LOAD:
    w = Serial.read();
    bt = (uint8_t) w;
    if (w >= 0) { // if we have data
      switch (state) {
      case IDLE: // in IDLE we are waiting for a command from the PC
        byteCount = 0;
        transferSize = 0;
        if (bt == 'F') { // write to flash
          destination = 0; // flash
          verify = 0; // don't verify
          state = READ_SIZE;
          Serial.write('R'); // signal we are ready
        }
        if (bt == 'V') { // write to flash and verify
          destination = 0; // flash
          verify = 1; // verify
          state = READ_SIZE;
          Serial.write('R'); // signal we are ready
        }
        if (bt == 'R') { // write to RAM
          destination = 1; // ram
          state = READ_SIZE;
          Serial.write('R'); // signal we are ready
        }
        if (bt == 'E') { //erase
          eraseFlash();
          Serial.write('D'); // signal we are done
        }
        break;
      case READ_SIZE: // we need to read in how many bytes the config data is
        transferSize |= ((uint32_t) bt << (byteCount++ * 8));
        if (byteCount > 3) {
          byteCount = 0;
          if (destination) {
            state = WRITE_TO_FPGA;
            initLoad(); // get the FPGA read for a load
            startLoad(); // start the load
          } 
          else {
            state = WRITE_TO_FLASH;
            eraseFlash();
          }
          Serial.write('O'); // signal the size was read
        }
        break;
      case WRITE_TO_FLASH:
        // we can only use the batch write for even addresses
        // so address 5 is written as a single byte
        if (byteCount == 0)
          writeByteFlash(5, bt);

        buffIdx = (byteCount++ - 1) % 256;
        loadBuffer[buffIdx] = bt;

        if (buffIdx == 255 && byteCount != 0)
          writeFlash(byteCount + 5 - 256, loadBuffer, 256); // write blocks of 256 bytes at a time for speed

        if (byteCount == transferSize) { // the last block to write

            if (buffIdx != 255) // finish the partial block write
            writeFlash(byteCount + 5 - (buffIdx + 1), loadBuffer,
            buffIdx + 1);

          delayMicroseconds(50); // these are necciary to get reliable writes
          uint32_t size = byteCount + 5;
          for (uint8_t k = 0; k < 4; k++) {
            writeByteFlash(k + 1, (size >> (k * 8)) & 0xFF); // write the size of the config data to the flash
            delayMicroseconds(50);
          }
          delayMicroseconds(50);
          writeByteFlash(0, 0xAA); // 0xAA is used to signal the flash has valid data
          Serial.write('D'); // signal we are done
          Serial.flush(); // make sure it sends
          if (verify) {
            state = VERIFY_FLASH;
          } 
          else {
            state = LOAD_FROM_FLASH;
          }
        }
        break;
      case WRITE_TO_FPGA:
        sendByte(bt); // just send the byte!
        if (++byteCount == transferSize) { // if we are done
          sendExtraClocks(); // send some extra clocks to make sure the FPGA is happy
          state = IDLE;
          taskState = SERVICE; // enter user mode
          initPostLoad();
          Serial.write('D'); // signal we are done
          Serial.flush();
        }
        break;
      case VERIFY_FLASH:
        if (bt == 'S') {
          byteCount += 5;
          for (uint32_t k = 0; k < byteCount; k += 256) { // dump all the flash data
            uint16_t s;
            if (k + 256 <= byteCount) {
              s = 256;
            } 
            else {
              s = byteCount - k;
            }
            readFlash(loadBuffer, k, s); // read blocks of 256
            uint16_t br = Serial.write((uint8_t*) loadBuffer, s); // dump them to the serial port
            k -= (256 - br); // if all the bytes weren't sent, resend them next round
            delay(10); // needed to prevent errors in some computers running Windows (give it time to process the data?)
          }
          state = LOAD_FROM_FLASH;
        }
        break;
      case LOAD_FROM_FLASH:
        if (bt == 'L') {
          loadFromFlash(); // load 'er up!
          Serial.write('D'); // loading done
          Serial.flush();
          state = IDLE;
          taskState = SERVICE;
          initPostLoad();
        }
        break;
      }
    }

    break;
  case SERVICE:
    userLoop(); // loop the user code
    break;
  } 
}

/* This is called when any control lines on the serial port are changed. 
 It requires a modification to the Arduino core code to work.         
 
 This looks for 5 pulses on the DTR line within 250ms. Checking for 5 
 makes sure that false triggers won't happen when the serial port is opened. */
void lineStateEvent(unsigned char linestate)
{
  static unsigned long start = 0; 
  static uint8_t falling = 0;
  if (!(linestate & LINESTATE_DTR)) {
    if ((millis() - start) < 250) {
      if (++falling >= 5)
        taskState = START_LOAD;
    } 
    else {
      start = millis();
      falling = 1;
    }
  }
}

void serialRXEnable() {
  UCSR1B |= (1 << RXEN1);
}

void serialRXDisable() {
  UCSR1B &= ~(1 << RXEN1);
}

static inline void Serial_SendByte(const char DataByte)
{
  while (!(UCSR1A & (1 << UDRE1)));
  UDR1 = DataByte;
}

/* This function handles all the serial to USB work. It works
 much the same way as the ADC task, but it just forwards data
 from one port to the other instead of the ADC to the FPGA. */
void uartTask() {
  if (Serial) { // does the data have somewhere to go?
    uint16_t ct = RingBuffer_GetCount(&serialBuffer);
    if (ct > 0) { // is there data to send?
      if (serialBuffer.Out + ct <= serialBuffer.End) { // does it loop in our buffer?
        ct = Serial.write(serialBuffer.Out, ct); // dump all the date
        serialBuffer.Out += ct;
        if (serialBuffer.Out == serialBuffer.End)
          serialBuffer.Out = serialBuffer.Start; // loop the buffer
      } 
      else { // it looped the ring buffer
        uint8_t* loopend = serialBuffer.Out + ct;
        uint16_t ct2 = loopend - serialBuffer.End;
        uint16_t ct1 = ct - ct2;
        uint16_t ct1s = Serial.write(serialBuffer.Out, ct1); // dump first block
        if (ct1s == ct1) {
          ct2 = Serial.write(serialBuffer.Start, ct2); // dump second block
          serialBuffer.Out = serialBuffer.Start + ct2; // update the pointers
          ct = ct1+ct2;
        } 
        else {
          ct = ct1s;
          serialBuffer.Out += ct;
        }
      }

      uint_reg_t CurrentGlobalInt = GetGlobalInterruptMask();
      GlobalInterruptDisable();

      serialBuffer.Count -= ct; // update the count

        SetGlobalInterruptMask(CurrentGlobalInt);

    }

    if (RingBuffer_GetCount(&serialBuffer) < 250) {
      SET(TX_BUSY, LOW); // re-enable the serial port
      serialRXEnable();
    }

    int16_t w;
    while ((w = Serial.read()) >= 0) {
      Serial_SendByte(w);
    }

  }
}

ISR(USART1_RX_vect) { // new serial data!
  *(serialBuffer.In) = UDR1;

  if (++serialBuffer.In == serialBuffer.End)
    serialBuffer.In = serialBuffer.Start;

  serialBuffer.Count++;

  sei();

  if (serialBuffer.Count >= 250) { // are we almost out of space?
    SET(TX_BUSY, HIGH); // signal we can't take any more
    if (serialBuffer.Count > 254) 
      serialRXDisable(); // if our flag is ignored disable the serial port so it doesn't clog things up
  }
}




























