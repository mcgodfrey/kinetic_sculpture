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
	// AVR pins
	input trigger,
	output [2:0] avr_flags,
	// Serial connections
	input avr_tx, // AVR Tx => FPGA Rx
	output avr_rx, // AVR Rx => FPGA Tx
	input avr_rx_busy, // AVR Rx buffer full
	
	//outputs to the servos
	output [0:0] servo_out
	);

	wire rst = ~rst_n; // make reset active high

	wire avr_ready; // high when the AVR is ready

	assign avr_flags[2:0] = avr_ready ? 3'b000 : 3'bzzz; // no flags are in use, keep high-z when avr_ready is low
  
  	localparam NUM_SERVOS = 1;
	
	wire new_pos;
	wire [5:0] servo_num;
	wire [7:0] servo_pos;
	
	wire [8*NUM_SERVOS-1:0] pos_buffer, output_buffer;
	
	
	//loop through and create all the servo controller modules
	genvar i;
	generate
		for (i = 0; i < NUM_SERVOS; i = i+1) begin: servo_gen_loop
			servo_controller controller (
				.clk(clk),
				.rst(rst),
				.position(output_buffer[(i<<4)+7:i<<4]),
				.servo(servo_out[i])
			);
		end
	endgenerate
	
	//if any new positions arrive from the ave (over SPI) then 
	//save them into the pos_buffer
	buffer_ctrl #(.NUM_SERVOS(NUM_SERVOS)) buffer_ctrl (
		.clk(clk),
		.rst(rst),
		.servo_num(servo_num),
		.servo_pos(servo_pos),
		.new_pos(new_pos),
		.pos_buffer(pos_buffer)
	);	
	
	//latches the value of pos_buffer into output_buffer whenever we
	//get a trigger from the avr. also toggles an led
	output_latch output_latch (
		.rst(rst),
		.trigger(trigger),
		.pos_buffer(pos_buffer),
		.output_buffer(output_buffer)
	);
	
	//handles SPI comms with the avr
	avr_interface #(.CLK_RATE(50000000), .SERIAL_BAUD_RATE(500000)) avr_interface (
		.clk(clk),
		.rst(rst),
		.cclk(cclk),
		.ready(avr_ready),
		.spi_miso(spi_miso),
		.spi_mosi(spi_mosi),
		.spi_sck(spi_sck),
		.spi_ss(spi_ss),
		.tx(avr_rx),
		.rx(avr_tx),
		.rx_data(),
		.new_rx_data(),
		.tx_data(8'h00),
		.new_tx_data(1'b0),
		.tx_busy(),
		.tx_block(avr_rx_busy),
		.servo_num(servo_num),
		.servo_pos(servo_pos),
		.new_pos(new_pos)
	);
	
	

endmodule