/*
	Servo motor control demo module
	Causes connected servos to swing back and forth with
	   different periods, individually adjustable.
*/
module mojo_top(
    // 50MHz clock input
    input clk,
    // Input from reset button (active low)
    input rst_n,
    // cclk input from AVR, high when AVR is ready
    input cclk,
    // Outputs to the 8 onboard LEDs
    output[7:0]led,
    // AVR SPI connections
    output spi_miso,
    input spi_ss,
    input spi_mosi,
    input spi_sck,
    // AVR ADC channel select
    output [3:0] spi_channel,
    // Serial connections
    input avr_tx, // AVR Tx => FPGA Rx
    output avr_rx, // AVR Rx => FPGA Tx
    input avr_rx_busy, // AVR Rx buffer full
	 
    //outputs to the servos
	 output [3:0] servo_out
    );

wire rst = ~rst_n; // make reset active high

// these signals should be high-z when not used
assign spi_miso = 1'bz;
assign avr_rx = 1'bz;
assign spi_channel = 4'bzzzz;

//set the onboard LEDs to a pattern for debugging.
assign led = 8'b01010101;


localparam NUM_SERVOS = 4;

//Pos is a 32 bit array (8 bits per servo)
//It would probably be better to use a 2d array.
wire [(NUM_SERVOS*8)-1:0] pos;


genvar i;
//Generate a serparate up/down counter for each servo
//Each one moves at a different speed (different CTR_LEN)
generate
  for (i = 0; i < NUM_SERVOS; i=i+1) begin: counter_gen_loop
    counter #(.CTR_LEN(26+i)) pos_counter (
      .clk(clk),
      .rst(rst),
      .value(pos[(i+1)*8-1:i*8])
    );
  end
endgenerate


//loop to generate the servo controllers, 1 per servo.
//each one has its position connected to the corresponding
//   up/down counter generated above.
generate
  for (i = 0; i < NUM_SERVOS; i=i+1) begin: servo_gen_loop
    servo_controller servo_sweep (
      .clk(clk),
      .rst(rst),
      .position(pos[(i+1)*8-1:i*8]),
      .servo(servo_out[i])
    );
  end
endgenerate

endmodule