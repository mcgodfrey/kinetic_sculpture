module output_latch #(parameter NUM_SERVOS=1) (
	input rst,
	input trigger,
	input [8*NUM_SERVOS-1:0] pos_buffer,
	output reg [8*NUM_SERVOS-1:0] output_buffer
	);
	
	//latches the pos_buffer to the output buffer whenever
	//we receive a trigger from the avr
	//if rst is high then output is default (127)
	always @(trigger, rst) begin
		if (rst) begin
			output_buffer = {NUM_SERVOS{8'd127}};
		end else if (trigger) begin
			output_buffer = pos_buffer;
		end
	end
	
endmodule